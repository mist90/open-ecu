/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include "../include/critical_section.hpp"
#include <algorithm>
#include <stdio.h>
#include <cmath>

// Speed measurement configuration
#define SPEED_TIMEOUT_US          500000  ///< Timeout for speed measurement (1 second)
#define BLDC_NUM_PHASES           3        ///< Number of phases in BLDC motor

// Platform specific time function
uint32_t time_us(void);

namespace libecu {

BldcController::BldcController(
    PwmInterface& pwm_interface,
    HallInterface& hall_interface,
    const MotorControlParams& params,
    AdcInterface* adc_interface) noexcept
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , adc_interface_(adc_interface)
    , hall_monitor_(pwm_interface_.getFrequency())
    , motor_pll_(pwm_interface_.getFrequency(), params.max_speed_rps * params.num_poles * BLDC_NUM_PHASES, params.useInverseCommTable)
    , pid_speed_controller_()
    , six_step_(pwm_interface)
    , foc_(pwm_interface, pwm_interface.getFrequency())
    , algorithm_(params.algorithm)
    , params_(params)
    , dmode_(DriveMode::NEUTRAL)
    , initialized_(false)
    , hall_update_pending_(false)
    , open_loop_step_(0)
    , open_loop_last_step_time_us_(0)
    , last_pid_update_time_us_(0)
    , duty_command_(0.0f)
    , filtered_target_speed_(0.0f)
    , filtered_measured_speed_(0.0f)
    , filtered_measured_current_(0.0f)
    , limited_target_speed_(0.0f)

{
    // Initialize status
    status_.current_speed_rps = 0.0f;
    status_.target_speed_rps = 0.0f;
    status_.duty_cycle = 0.0f;
    status_.target_current = 0.0f;
    status_.measured_current = 0.0f;
    status_.measured_current_filtered = 0.0f;
    status_.bus_voltage = 0.0f;
    status_.temperature_c = TEMPERATURE_INVALID_C;  // nothing sampled yet
    status_.pll_angle = 0.0f;
    status_.current_pid_saturation = 0;
    status_.target_position = 0xFF;
    status_.measured_position = 0xFF;
    status_.is_running = false;
    status_.control_mode = ControlMode::CLOSED_LOOP_VELOCITY;
    status_.electric_mode = ElectricMode::CURRENT_MODE;
    status_.i_d = 0.0f;
    status_.i_q = 0.0f;
    status_.v_d = 0.0f;
    status_.v_q = 0.0f;
    status_.angle_e_rad = 0.0f;
    status_.duty_u = 0.0f;
    status_.duty_v = 0.0f;
    status_.duty_w = 0.0f;

    if (algorithm_ != DriveAlgorithm::FOC) {
        algorithm_ = DriveAlgorithm::SIX_STEP;
    }
    status_.algorithm = algorithm_;

    // PID controller settings
    params_.pid_voltage_mode.sample_time_s = 1.0f / float(pwm_interface_.getFrequency());
    params_.pid_voltage_mode.min_output = 0.0f;
    params_.pid_voltage_mode.max_output = 1.0f;

    params_.pid_current_mode.sample_time_s = 1.0f / float(pwm_interface_.getFrequency());
    params_.pid_current_mode.min_output = params_.min_current;
    params_.pid_current_mode.max_output = params_.max_current;

    params_.pid_current_regulator.sample_time_s = 1.0f / float(pwm_interface_.getFrequency());
    // Output limits come from the caller. With a feed-forward attached the PI
    // has to be able to trim *downwards*, so min_output must be allowed to go
    // negative; fall back to [0,1] when the caller left them unset.
    if (params_.pid_current_regulator.max_output <= params_.pid_current_regulator.min_output) {
        params_.pid_current_regulator.min_output = 0.0f;
        params_.pid_current_regulator.max_output = 1.0f;
    }

    pid_speed_controller_.setParameters(params_.pid_current_mode);

    // Hand each algorithm its own configuration. Both are configured whether
    // or not they are selected, so switching with AT+ALGO never lands on an
    // algorithm running default gains.
    SixStepParams six_step_params;
    six_step_params.max_duty_cycle = params_.max_duty_cycle;
    six_step_params.blanking_cycles = params_.current_blanking_cycles;
    six_step_.setParams(six_step_params);
    six_step_.setCurrentPi(params_.pid_current_regulator);

    // MotorControlParams is a plain struct the caller fills field by field, so
    // a field added later can easily arrive uninitialised from an older call
    // site. These fall back rather than driving the bridge with garbage.
    //
    // The thermal limit is one of those late fields, and an indeterminate one
    // is worse than a wrong one: a tiny or negative value trips the drive into
    // NEUTRAL at room temperature, a huge one removes the protection.
    if (!(params_.max_temperature_c > 0.0f)) {  // catches NaN too
        params_.max_temperature_c = 100.0f;
    }

    FocParams foc_params = params_.foc;
    if (foc_params.max_duty_cycle <= 0.0f || foc_params.max_duty_cycle > 1.0f) {
        foc_params.max_duty_cycle = params_.max_duty_cycle;
    }
    if (foc_params.max_modulation <= 0.0f || foc_params.max_modulation > 1.0f) {
        foc_params.max_modulation = 0.95f;
    }
    params_.foc = foc_params;
    foc_.setParams(foc_params);
    foc_.setCurrentPiGains(params_.foc_current_regulator.kp, params_.foc_current_regulator.ki);

    motor_pll_.setUsePLL(true);
}

MotorAlgorithm& BldcController::algorithm() noexcept
{
    return (algorithm_ == DriveAlgorithm::FOC)
         ? static_cast<MotorAlgorithm&>(foc_)
         : static_cast<MotorAlgorithm&>(six_step_);
}

const MotorAlgorithm& BldcController::algorithm() const noexcept
{
    return (algorithm_ == DriveAlgorithm::FOC)
         ? static_cast<const MotorAlgorithm&>(foc_)
         : static_cast<const MotorAlgorithm&>(six_step_);
}

bool BldcController::initialize() noexcept
{
    pwm_interface_.setNeutral();

    // Reset PID controller
    pid_speed_controller_.reset();
    six_step_.reset();
    foc_.reset();

    initialized_ = true;
    return true;
}

void BldcController::update() noexcept
{
    // Speed feedback, signed **relative to the commanded direction**: positive
    // when the rotor turns the way dmode_ asks, negative when it turns against
    // it. The PLL itself reports signed electrical speed in the Hall sequence's
    // own frame - negative in REVERSE, because the sequence descends - so the
    // direction sign converts one to the other.
    //
    // This used to be std::abs(). That fixed a real problem (the raw signed
    // value cannot reach a magnitude setpoint in REVERSE, which pinned the loop
    // at max_current) but replaced it with a worse one: a magnitude is blind to
    // direction. At SPD=0 *any* rotation then gives error = 0 - |v| < 0, the
    // PID winds its output down to min_current, and FOC turns that into
    // negative Iq - which brakes a forward-turning rotor but *accelerates* a
    // backward-turning one. Nudge the shaft the wrong way by hand and the motor
    // ran away to its no-load ceiling: measured 8.61 RPS, duty 0.95, demand
    // pinned at the -6 A floor.
    //
    // Six-step masked this for as long as it was the only algorithm: its
    // current PI has min_output = 0 and its duty is clamped non-negative, so it
    // cannot produce reverse torque at all and a negative demand merely coasts.
    // FOC can, so the latent sign error became a runaway.
    //
    // Signed feedback fixes all four quadrants: turning against the command now
    // yields a *positive* error and therefore corrective torque, while turning
    // with it behaves exactly as before.
    const float direction = (dmode_ == DriveMode::REVERSE) ? -1.0f : 1.0f;
    float speed_rps = direction * motor_pll_.getSpeedStepsSec()
                    / (params_.num_poles * BLDC_NUM_PHASES);

    float alpha = params_.measured_speed_lpf_alpha;
    if (alpha > 0.0f && alpha < 1.0f) {
        filtered_measured_speed_ = alpha * speed_rps + (1.0f - alpha) * filtered_measured_speed_;
    } else {
        filtered_measured_speed_ = speed_rps;
    }

    // ---- Thermal protection -----------------------------------------------
    // Sampled here rather than in the PWM ISR on purpose. The NTC is a regular
    // ADC conversion started on demand - microseconds of polling, and it can be
    // aborted and restarted by the injected trigger - which has no business in
    // a 20 kHz loop that already uses a third of its period. 100 Hz is several
    // orders of magnitude faster than any thermal mass on this board.
    //
    // Runs whether or not the motor is running: a drive that has just been
    // stopped hot is exactly when the limit matters, and it is what stops the
    // bridge being re-armed into a fault.
    if (adc_interface_) {
        const float temperature_c = adc_interface_->readTemperature();
        {
            CriticalSection cs;
            status_.temperature_c = temperature_c;
        }

        // NEUTRAL below, not inside the critical section above: setDriveMode()
        // takes one of its own and CriticalSection does not nest - its
        // destructor re-enables interrupts unconditionally.
        //
        // TEMPERATURE_INVALID_C fails this comparison, so an open sensor
        // coasts rather than trips. See MotorControlParams::max_temperature_c.
        if (temperature_c > params_.max_temperature_c && dmode_ != DriveMode::NEUTRAL) {
            setDriveMode(DriveMode::NEUTRAL);
        }
    }

    MotorStatus status;
    {
        CriticalSection cs;
        status_.current_speed_rps = filtered_measured_speed_;
        status = status_;
    }

    if (status.is_running) {
        switch (status.control_mode) {
            case ControlMode::OPEN_LOOP: {
                uint32_t current_time_us = time_us();
                uint32_t step_interval_us = calculateOpenLoopStepInterval(status.target_speed_rps);
                if (step_interval_us > 0 && (current_time_us - open_loop_last_step_time_us_) >= step_interval_us) {
                    open_loop_step_ = (open_loop_step_ + 1) % 6;
                    open_loop_last_step_time_us_ = current_time_us;
                }
                motor_pll_.updateHall(open_loop_step_);
                break;
            }

            case ControlMode::CLOSED_LOOP_VELOCITY: {
                // Velocity control with speed PID
                // Calculate real time difference since last PID update
                uint32_t current_time_us = time_us();
                float dt;

                if (last_pid_update_time_us_ == 0) {
                    // First PID update - use nominal period
                    dt = 1.0f / params_.control_frequency;
                } else {
                    // Calculate actual time difference in seconds
                    uint32_t dt_us = current_time_us - last_pid_update_time_us_;
                    dt = static_cast<float>(dt_us) / 1000000.0f;
                }

                // Update timestamp for next iteration
                last_pid_update_time_us_ = current_time_us;

                // Apply acceleration limiting (slew rate limiter on target_speed)
                float limited_target = applyAccelerationLimit(
                    status.target_speed_rps,
                    dt
                );

                // Run speed PID (already configured for correct mode)
                float pid_output = pid_speed_controller_.update(
                    limited_target,
                    status.current_speed_rps,
                    dt,
                    status.current_pid_saturation
                );

                // Electric mode determines how to use PID output
                if (status.electric_mode == ElectricMode::VOLTAGE_MODE) {
                    // VOLTAGE_MODE: PID outputs the duty command directly
                    {
                        CriticalSection cs;
                        duty_command_ = pid_output;
                    }
                } else {
                    // CURRENT_MODE: PID outputs target current
                    // Store for inner current loop (runs in pwmInterruptHandler)
                    {
                        CriticalSection cs;
                        status_.target_current = pid_output;
                    }

                    // duty_cycle is calculated by pwmInterruptHandler() (atomic read)
                }
                break;
            }

            case ControlMode::CLOSED_LOOP_TORQUE:
                // Torque control: fixed duty cycle or current + Hall sensor commutation
                // No speed PID - user sets fixed torque/current command
                break;
        }
    }
}

void BldcController::setTargetSpeed(float speed_rps) noexcept
{
    CriticalSection cs;

    if (dmode_ == DriveMode::NEUTRAL)
        return;
    if (speed_rps < 0.0f)
        return;
    // Clamp to maximum speed
    status_.target_speed_rps = std::min(std::abs(speed_rps), params_.max_speed_rps);
}

void BldcController::setDutyCycle(float duty_cycle) noexcept
{
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    if (duty_cycle < 0.0f)
        return;
    // duty_command_ is the request; status_.duty_cycle is what the algorithm
    // reports back. They are the same number in six-step voltage mode but not
    // in FOC, where the reported value is the modulation index of the applied
    // voltage vector - feeding that back as the command would make it decay.
    duty_command_ = std::max(0.0f, std::min(duty_cycle, params_.max_duty_cycle));
    status_.duty_cycle = duty_command_;
}

void BldcController::setCurrent(float current_a) noexcept
{
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    status_.target_current = std::max(params_.min_current, std::min(current_a, params_.max_current));
}

void BldcController::setControlMode(ControlMode mode) noexcept
{
    CriticalSection cs;
    if (status_.control_mode != mode) {
        status_.control_mode = mode;

        // Reset PID when switching to velocity control
        if (mode == ControlMode::CLOSED_LOOP_VELOCITY) {
            pid_speed_controller_.reset();
            last_pid_update_time_us_ = 0;  // Reset timing for fresh start
        }
    }
}

void BldcController::setElectricMode(ElectricMode mode) noexcept
{
    CriticalSection cs;
    status_.electric_mode = mode;

    // Reconfigure PID controller with appropriate parameters for the mode
    if (mode == ElectricMode::VOLTAGE_MODE) {
        // VOLTAGE_MODE: PID outputs duty cycle (0.0-1.0)
        pid_speed_controller_.setParameters(params_.pid_voltage_mode);
    } else {
        // CURRENT_MODE: PID outputs current (Amperes)
        pid_speed_controller_.setParameters(params_.pid_current_mode);
    }
    pid_speed_controller_.reset();

    // The inner loop changes shape underneath the outer one; a stale
    // integrator from the other mode is meaningless.
    algorithm().reset();
}

void BldcController::setAlgorithm(DriveAlgorithm new_algorithm) noexcept
{
    CriticalSection cs;
    if (algorithm_ == new_algorithm) {
        return;
    }

    // Hand the bridge over. The outgoing algorithm's integrators are cleared
    // so it cannot resume from a state that belonged to a different
    // modulation scheme, and the incoming one establishes its switching
    // configuration - six-step re-commutates from scratch, FOC puts all three
    // phases in UP and raises its one and only COM event.
    algorithm().reset();
    algorithm_ = new_algorithm;
    status_.algorithm = algorithm_;
    algorithm().onEnter();

    // The speed loop's anti-windup input came from the old inner loop.
    status_.current_pid_saturation = 0;
    pid_speed_controller_.reset();
}

DriveAlgorithm BldcController::getAlgorithm() const noexcept
{
    CriticalSection cs;
    return algorithm_;
}

void BldcController::setFocCurrentPi(float kp, float ki) noexcept
{
    CriticalSection cs;
    params_.foc_current_regulator.kp = kp;
    params_.foc_current_regulator.ki = ki;
    foc_.setCurrentPiGains(kp, ki);
}

void BldcController::getFocCurrentPi(float& kp, float& ki) const noexcept
{
    CriticalSection cs;
    foc_.getCurrentPiGains(kp, ki);
}

void BldcController::setFocAngleOffsetDeg(float degrees_e) noexcept
{
    CriticalSection cs;
    const float rad = degrees_e * (FOC_PI / 180.0f);
    params_.foc.angle_offset_rad = rad;
    foc_.setAngleOffsetRad(rad);
}

float BldcController::getFocAngleOffsetDeg() const noexcept
{
    CriticalSection cs;
    return foc_.getAngleOffsetRad() * (180.0f / FOC_PI);
}

void BldcController::setDriveMode(DriveMode mode) noexcept
{
    CriticalSection cs;
    dmode_ = mode;
    if (dmode_ == DriveMode::NEUTRAL) {
        // High-Z the bridge *before* releasing the pins, so the switches are
        // commanded off rather than merely abandoned.
        pwm_interface_.setNeutral();
        pwm_interface_.enable(false);
        status_.target_speed_rps = 0.0f;
        status_.duty_cycle = 0.0f;
        duty_command_ = 0.0f;
        status_.target_current = 0.0f;
        pid_speed_controller_.reset();
        algorithm().reset();
        filtered_measured_current_ = 0.0f;
    } else {
        pwm_interface_.enable(true);
        // Must follow enable(): the PWM driver ignores channel-state writes
        // while the outputs are off, so FOC's all-phases-UP configuration
        // would be dropped if onEnter() ran first.
        algorithm().onEnter();
    }
}

void BldcController::setSpeedPid(float kp, float ki, float kd) noexcept
{
    PidParameters p = pid_speed_controller_.getParameters();
    p.kp = kp;
    p.ki = ki;
    p.kd = kd;
    pid_speed_controller_.setParameters(p);
    pid_speed_controller_.reset();
}

void BldcController::setCurrentPid(float kp, float ki, float kd) noexcept
{
    CriticalSection cs;
    PidParameters p = params_.pid_current_regulator;
    p.kp = kp;
    p.ki = ki;
    p.kd = kd;
    params_.pid_current_regulator = p;
    six_step_.setCurrentPi(p);
}

HallMonitor::Info BldcController::getHallInfo() const noexcept {
    CriticalSection cs;
    return hall_monitor_.getInfo();
}

void BldcController::clearHallFault() noexcept {
    CriticalSection cs;
    hall_monitor_.clearFault();
}

void BldcController::setHallMonitorParams(const HallMonitorParams& params) noexcept {
    CriticalSection cs;
    hall_monitor_.setParameters(params);
}

MotorPLL::PllInfo BldcController::getPllInfo() const noexcept {
    CriticalSection cs;
    return motor_pll_.getInfo();
}

void BldcController::setPllGains(float kp, float ki) noexcept {
    CriticalSection cs;
    motor_pll_.setGains(kp, ki);
}

DriveMode BldcController::getDriveMode() const noexcept {
    CriticalSection cs;
    return dmode_;
}

void BldcController::getSpeedPidGains(float& kp, float& ki, float& kd) const noexcept {
    CriticalSection cs;
    const PidParameters& p = pid_speed_controller_.getParameters();
    kp = p.kp;
    ki = p.ki;
    kd = p.kd;
}

void BldcController::getCurrentPidGains(float& kp, float& ki, float& kd) const noexcept {
    CriticalSection cs;
    const PidParameters& p = six_step_.getCurrentPi();
    kp = p.kp;
    ki = p.ki;
    kd = p.kd;
}

void BldcController::getPllBaseGains(float& kp, float& ki) const noexcept {
    CriticalSection cs;
    motor_pll_.getBaseGains(kp, ki);
}

void BldcController::start() noexcept
{
    status_.is_running = true;

    CriticalSection cs;
    open_loop_last_step_time_us_ = time_us();
    uint8_t current_hall = hall_interface_.getPosition();
    status_.measured_position = current_hall;
    motor_pll_.updateHall(current_hall);

    // The inverter follows the *drive mode*, not the run flag.
    //
    // This used to call enable(true) unconditionally. dmode_ is NEUTRAL at
    // construction and gets there through the initialiser list, never through
    // setDriveMode(), so nothing had ever called enable(false) - which left all
    // six switches live from boot while the drive mode reported NEUTRAL. The
    // only way to silence it was to cycle the mode to something non-NEUTRAL and
    // back, because setDriveMode() is the sole caller of enable(false).
    //
    // Latent in six-step, where CCR=0 leaves the low side statically on: a DC
    // short across the windings, wrong but inaudible. FOC made it obvious -
    // onEnter() puts all three phases at 50 % duty, so the bridge modulates at
    // 20 kHz and can be heard.
    if (dmode_ == DriveMode::NEUTRAL) {
        pwm_interface_.enable(false);
    } else {
        pwm_interface_.enable(true);
        // After enable(), for the reason given in setDriveMode()
        algorithm().onEnter();
    }

    // Reset PID timing and target speed filters
    last_pid_update_time_us_ = 0;
    filtered_target_speed_ = 0.0f;
    filtered_measured_speed_ = 0.0f;
    limited_target_speed_ = 0.0f;
}

void BldcController::stop() noexcept
{
    CriticalSection cs;
    status_.is_running = false;
    status_.duty_cycle = 0.0f;
    duty_command_ = 0.0f;
    // Zero the torque command too. The PWM ISR keeps running after stop() and
    // recomputes the duty from the current loop every cycle, so clearing only
    // the duty left the drive producing exactly as much torque as before.
    status_.target_current = 0.0f;
    open_loop_step_ = 0;
    algorithm().reset();
    pwm_interface_.updateDutyCycle(0.0f);

    status_.current_speed_rps = 0.0f;

    // Reset PID timing
    last_pid_update_time_us_ = 0;
}

MotorStatus BldcController::getStatus() const noexcept
{
    return status_;
}

float BldcController::applyAccelerationLimit(float target_speed, float dt) noexcept
{
    // Stage 1: LPF to smooth noisy analog input
    float alpha = params_.target_speed_lpf_alpha;
    if (alpha > 0.0f && alpha < 1.0f) {
        filtered_target_speed_ = alpha * target_speed + (1.0f - alpha) * filtered_target_speed_;
    } else {
        filtered_target_speed_ = target_speed;
    }

    // Stage 2: Slew rate limiter
    if (params_.acceleration_rate == 0.0f) {
        limited_target_speed_ = filtered_target_speed_;
        return limited_target_speed_;
    }

    float speed_diff = filtered_target_speed_ - limited_target_speed_;
    float max_change = params_.acceleration_rate * dt;

    if (std::abs(speed_diff) <= max_change) {
        limited_target_speed_ = filtered_target_speed_;
    } else if (speed_diff > 0.0f) {
        limited_target_speed_ += max_change;
    } else {
        limited_target_speed_ -= max_change;
    }

    return limited_target_speed_;
}

uint32_t BldcController::calculateOpenLoopStepInterval(float speed_rps) noexcept
{
    // step_interval_us = 1,000,000 / (speed_rps / 60 * num_poles * 6)
    // = 1,000,000 * 60 / (speed_rps * num_poles * 6)
    // = 10,000,000 / (speed_rps * num_poles)
    if (speed_rps <= 0.0f) {
        return 0;
    }
    return static_cast<uint32_t>(10000000.0f / (speed_rps * params_.num_poles * BLDC_NUM_PHASES));
}

void BldcController::hallSensorInterruptHandler() noexcept
{
    if (status_.control_mode == ControlMode::OPEN_LOOP)
        return;

    // Deferred Hall update: set flag, process in PWM ISR.
    // Hall signal may bounce (1→0→1) due to EMI from power cables.
    // PWM ISR re-reads stable state 50μs later, hiding bounce from PLL.
    hall_update_pending_ = true;
}

void BldcController::pwmInterruptHandler() noexcept {
    // Check if current controller and ADC are available
    if (!adc_interface_) {
        return;
    }

    // One snapshot of everything, taken once, handed to whichever algorithm is
    // selected. Neither algorithm reaches into hardware for measurements, so
    // they cannot disagree about what this PWM period looked like.
    MotorAlgorithmInput in;

    // ADC already converted all injected channels — reading registers adds no
    // latency. All three shunts are read even in six-step, which uses one of
    // them: the cost is two extra register reads and two conversions, and it
    // is what lets the two algorithms share a single input path.
    adc_interface_->readAllCurrents(in.i_u, in.i_v, in.i_w);
    in.bus_voltage = adc_interface_->readBusVoltage();

    // Hall health time base. Reads no hardware; it only decays the monitor's
    // accumulators and ages a standing illegal code, both of which have to run
    // on the PWM clock rather than on Hall events - those arrive at the
    // commutation rate, which is exactly what is unreliable when a line breaks.
    hall_monitor_.tick((dmode_ != DriveMode::NEUTRAL) && status_.is_running);

    // Process deferred Hall update (debounced re-read from stable GPIO state)
    if (hall_update_pending_) {
        hall_update_pending_ = false;
        // One read, shared by the health monitor and the PLL, so they can never
        // disagree and the lines are never sampled mid-transition.
        uint8_t hall_state = hall_interface_.getPosition();
        hall_monitor_.onPosition(hall_state,
                                 (dmode_ != DriveMode::NEUTRAL) && status_.is_running);
        if (hall_state <= 5) {
            {
                CriticalSection cs;
                status_.measured_position = hall_state;
            }
            {
                CriticalSection cs;
                motor_pll_.updateHall(hall_state);
            }
        }
    }

    // Read shared data atomically (avoid torn reads from SysTick interrupt)
    {
        CriticalSection cs;
        in.electric_mode = status_.electric_mode;
        in.target_current = status_.target_current;
        in.duty_command = duty_command_;
        motor_pll_.updateTick();
        in.step = motor_pll_.getNextHall(dmode_);
        in.angle_steps = motor_pll_.getAngle();
        in.speed_steps_s = motor_pll_.getSpeedStepsSec();
    }
    in.drive_mode = dmode_;

    // Hall health. The monitor latches, so unlike the old PLL rule this does
    // not re-assert every cycle; it has to be cleared explicitly once the
    // wiring is dealt with.
    if (hall_monitor_.isFaulted() && dmode_ != DriveMode::NEUTRAL) {
        setDriveMode(DriveMode::NEUTRAL);
    }

    if (in.bus_voltage > params_.max_voltage)
        setDriveMode(DriveMode::NEUTRAL);

    // ---- Algorithm selection ----------------------------------------------
    // A plain switch rather than a virtual call through MotorAlgorithm: the
    // target of each branch is known at compile time, so the compiler can
    // inline it, and at 20 kHz in a `-O0` build every avoided indirection is
    // worth having. The base class still exists for the non-ISR paths
    // (reset/onEnter), where clarity is worth more than cycles.
    MotorAlgorithmOutput out{};
    if (dmode_ == DriveMode::NEUTRAL) {
        // Bridge disabled - there is nothing to regulate. Running the
        // algorithm anyway lets the d/q integrators random-walk on ADC noise
        // until they hit their clamp, which then publishes as duty 0.95 while
        // the drive reports NEUTRAL. Functionally harmless, because onEnter()
        // clears them when the drive re-arms, but a thoroughly misleading
        // number to put in telemetry - and indistinguishable, from the
        // outside, from the bridge actually being at 95 %.
        out.angle_e_rad = in.angle_steps * FOC_RAD_PER_STEP;
    } else {
        switch (algorithm_) {
            case DriveAlgorithm::FOC:
                out = foc_.update(in);
                break;

            case DriveAlgorithm::SIX_STEP:
            default:
                out = six_step_.update(in);
                break;
        }
    }

    {
        CriticalSection cs;
        // Report a filtered current: the raw value is a single ADC sample per
        // PWM cycle, and sampling that at 100 Hz aliases the chopping ripple
        // into the telemetry. Control and +OSC keep the raw value.
        const float i_alpha = params_.measured_current_lpf_alpha;
        if (i_alpha > 0.0f && i_alpha < 1.0f) {
            filtered_measured_current_ += i_alpha * (out.measured_current - filtered_measured_current_);
        } else {
            filtered_measured_current_ = out.measured_current;
        }
        status_.measured_current = out.measured_current;
        status_.measured_current_filtered = filtered_measured_current_;
        status_.duty_cycle = out.duty;
        status_.duty_u = out.duty_u;
        status_.duty_v = out.duty_v;
        status_.duty_w = out.duty_w;
        status_.bus_voltage = in.bus_voltage;
        status_.pll_angle = in.angle_steps;
        status_.current_pid_saturation = out.saturation;
        status_.target_position = in.step;
        status_.i_d = out.i_d;
        status_.i_q = out.i_q;
        status_.v_d = out.v_d;
        status_.v_q = out.v_q;
        status_.angle_e_rad = out.angle_e_rad;
    }
}

} // namespace libecu
