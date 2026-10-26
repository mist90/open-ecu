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

namespace libecu {

BldcController::BldcController(
    PwmInterface& pwm_interface,
    HallInterface& hall_interface,
    const MotorControlParams& params,
    AdcInterface* adc_interface) noexcept
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , adc_interface_(adc_interface)
    , hall_monitor_()
    , motor_pll_(params.max_speed_rps * params.num_poles * BLDC_NUM_PHASES, params.useInverseCommTable)
    , open_loop_()
    , pi_speed_controller_()
    , six_step_(pwm_interface)
    , foc_(pwm_interface)
    , algorithm_(params.algorithm)
    , params_(params)
    , dmode_(DriveMode::NEUTRAL)
    , initialized_(false)
    , hall_update_pending_(false)
    , duty_command_{}
    , filtered_target_speed_{}
    , filtered_measured_speed_{}
    , filtered_measured_current_{}
    , limited_target_speed_{}
    , acceleration_step_{}

{
    // Initialize status
    status_.temperature_c = number_v<Temperature, -273>;  // nothing sampled yet
    status_.current_pi_saturation = 0;
    status_.target_position = 0xFF;
    status_.measured_position = 0xFF;
    status_.is_running = false;
    status_.control_mode = ControlMode::CLOSED_LOOP_VELOCITY;
    status_.electric_mode = ElectricMode::CURRENT_MODE;

    if (algorithm_ != DriveAlgorithm::FOC) {
        algorithm_ = DriveAlgorithm::SIX_STEP;
    }
    status_.algorithm = algorithm_;

    // Outer PI uses ActiveTiming::SPEED_HZ; the inner PI uses CURRENT_HZ.
    params_.pi_voltage_mode.min_output = 0.0f;
    params_.pi_voltage_mode.max_output = 1.0f;

    params_.pi_current_mode.min_output = params_.min_current;
    params_.pi_current_mode.max_output = params_.max_current;
    // Output limits come from the caller. With a feed-forward attached the PI
    // has to be able to trim *downwards*, so min_output must be allowed to go
    // negative; fall back to [0,1] when the caller left them unset.
    if (params_.pi_current_regulator.max_output <= params_.pi_current_regulator.min_output) {
        params_.pi_current_regulator.min_output = 0.0f;
        params_.pi_current_regulator.max_output = 1.0f;
    }

    pi_speed_controller_.setParameters(params_.pi_current_mode);

    // Hand each algorithm its own configuration. Both are configured whether
    // or not they are selected, so switching with AT+ALGO never lands on an
    // algorithm running default gains.
    SixStepParams six_step_params;
    six_step_params.max_duty_cycle = params_.max_duty_cycle;
    six_step_params.blanking_cycles = params_.current_blanking_cycles;
    six_step_.setParams(six_step_params);
    six_step_.setCurrentPi(params_.pi_current_regulator);

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
    Profile::PllSpeed max_electrical_speed{};
    Speed acceleration{};
    parameters_valid_ =
        params.num_poles > 0 &&
        tryNumber(params.max_speed_rps * params.num_poles * BLDC_NUM_PHASES, max_electrical_speed) &&
        tryNumber(params_.max_duty_cycle, limits_.max_duty) &&
        tryNumber(params_.max_current, limits_.max_current) &&
        tryNumber(params_.min_current, limits_.min_current) &&
        tryNumber(params_.max_voltage, limits_.max_voltage) &&
        tryNumber(params_.max_temperature_c, limits_.max_temperature) &&
        tryNumber(params_.max_speed_rps, limits_.max_speed) &&
        tryNumber(params_.target_speed_lpf_alpha, limits_.target_alpha) &&
        tryNumber(params_.measured_speed_lpf_alpha, limits_.speed_alpha) &&
        tryNumber(params_.measured_current_lpf_alpha, limits_.current_alpha) &&
        tryNumber(params_.acceleration_rate, acceleration) &&
        open_loop_.setSpeed(limits_.max_speed, params_.num_poles) &&
        foc_.setParams(foc_params) &&
        foc_.setCurrentPiGains(params_.foc_current_regulator.kp, params_.foc_current_regulator.ki);
    acceleration_step_ = divideByInteger<Accumulator>(acceleration, ActiveTiming::SPEED_HZ);
    acceleration_enabled_ = acceleration > Speed{};
    open_loop_.setSpeed(Speed{}, params_.num_poles);

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
    if (!parameters_valid_ || !pi_speed_controller_.isConfigured() || !six_step_.isCurrentPiConfigured())
        return false;

    // Reset PI controller
    pi_speed_controller_.reset();
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
    // PI winds its output down to min_current, and FOC turns that into
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
    Speed speed_rps = divideByInteger<Speed>(motor_pll_.getNativeSpeedStepsSec(),
                                            params_.num_poles * BLDC_NUM_PHASES);
    if (dmode_ == DriveMode::REVERSE) speed_rps = -speed_rps;

    const Fraction alpha = limits_.speed_alpha;
    if (alpha > Fraction{} && alpha < number_v<Fraction, 1>) {
        filtered_measured_speed_ += multiplyTo<Speed>(alpha, speed_rps - filtered_measured_speed_);
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
        const Temperature temperature_c = adc_interface_->readNativeTemperature();
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
        if (temperature_c > limits_.max_temperature && dmode_ != DriveMode::NEUTRAL) {
            setDriveMode(DriveMode::NEUTRAL);
        }
    }

    NativeMotorStatus status;
    {
        CriticalSection cs;
        status_.current_speed_rps = filtered_measured_speed_;
        status = status_;
    }

    if (status.algorithm == DriveAlgorithm::FOC) {
        const Duty modulation = FocAlgorithm::Core::modulationMagnitude(status.v_d, status.v_q, status.bus_voltage);
        CriticalSection cs;
        if (algorithm_ == DriveAlgorithm::FOC)
            status_.duty_cycle = dmode_ == DriveMode::NEUTRAL ? Duty{} : modulation;
    }

    // A parked drive still reaches here every tick, but NEUTRAL has already
    // high-Z'd the bridge, so the velocity loop has nothing to act on - and
    // running it anyway is what put a jerk into the next start.
    //
    // On release the setpoint drops to zero instantly while the rotor coasts
    // on, and the slew limiter walks its output down at acceleration_rate, so
    // the loop sees seconds of large positive error that no amount of output
    // can answer with the bridge off. It integrates to its ceiling and then
    // *stays* there, because once the ramp and the rotor both reach zero the
    // error is zero and nothing unwinds it. setDriveMode() resets the PI on
    // the way into NEUTRAL, but that reset happens once and the following
    // ticks simply wound it up again. Measured on MOTOR_1 releasing from
    // 5 RPS: parked target current settled at 3.2 A and held, and the next
    // FORWARD applied it as a step - 3.1 A into a stationary rotor inside
    // 40 ms. Releasing from 12 RPS reached the 9.4 A the throttle logs show.
    if (dmode_ == DriveMode::NEUTRAL) {
        holdVelocityLoop();
    } else if (status.is_running) {
        switch (status.control_mode) {
            case ControlMode::OPEN_LOOP: {
                // The field angle advances on the PWM clock. The slow loop
                // reports physical Hall speed, just as it does in closed loop.
                break;
            }

            case ControlMode::CLOSED_LOOP_VELOCITY: {
                const Speed limited_target = applyAccelerationLimit(status.target_speed_rps);
                using PiSpeed = ActivePiProfile::SpeedPiTypes::Input;
                const auto pi_output = pi_speed_controller_.updateNative(
                    numberCast<PiSpeed>(limited_target),
                    numberCast<PiSpeed>(status.current_speed_rps),
                    status.current_pi_saturation
                );

                // Electric mode determines how to use PI output
                if (status.electric_mode == ElectricMode::VOLTAGE_MODE) {
                    // VOLTAGE_MODE: PI outputs the duty command directly
                    {
                        CriticalSection cs;
                        duty_command_ = numberCast<Duty>(pi_output);
                    }
                } else {
                    // CURRENT_MODE: PI outputs target current
                    // Store for inner current loop (runs in pwmInterruptHandler)
                    {
                        CriticalSection cs;
                        status_.target_current = numberCast<Current>(pi_output);
                    }

                    // duty_cycle is calculated by pwmInterruptHandler() (atomic read)
                }
                break;
            }

            case ControlMode::CLOSED_LOOP_TORQUE:
                // Torque control: fixed duty cycle or current + Hall sensor commutation
                // User sets a fixed torque/current command
                break;
        }
    }
}

void BldcController::setTargetSpeed(float speed_rps) noexcept
{
    Speed target{};
    if (!tryNumber(speed_rps, target)) return;
    setNativeTargetSpeed(target);
}

void BldcController::setNativeTargetSpeed(Speed target) noexcept {
    CriticalSection cs;

    if (dmode_ == DriveMode::NEUTRAL)
        return;
    if (target < Speed{})
        return;
    // Clamp to maximum speed
    const Speed limited = std::min(target, limits_.max_speed);
    if (!open_loop_.setSpeed(limited, params_.num_poles)) return;
    status_.target_speed_rps = limited;
}

void BldcController::setDutyCycle(float duty_cycle) noexcept
{
    Duty target{};
    if (!tryNumber(duty_cycle, target)) return;
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    if (duty_cycle < 0.0f)
        return;
    // duty_command_ is the request; status_.duty_cycle is what the algorithm
    // reports back. They are the same number in six-step voltage mode but not
    // in FOC, where the reported value is the modulation index of the applied
    // voltage vector - feeding that back as the command would make it decay.
    duty_command_ = numericClamp(target, Duty{}, limits_.max_duty);
    status_.duty_cycle = duty_command_;
}

void BldcController::setCurrent(float current_a) noexcept
{
    Current target{};
    if (!tryNumber(current_a, target)) return;
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    status_.target_current = numericClamp(target, limits_.min_current, limits_.max_current);
}

void BldcController::setControlMode(ControlMode mode) noexcept
{
    CriticalSection cs;
    if (status_.control_mode != mode) {
        const bool from_open_loop = status_.control_mode == ControlMode::OPEN_LOOP;
        status_.control_mode = mode;
        if (from_open_loop) {
            // Hall tracking continues during open loop. Refresh a stationary
            // sensor without discarding a valid estimate of a moving rotor.
            synchronizeHallPosition();
            algorithm().reset();
        } else if (mode == ControlMode::OPEN_LOOP) {
            open_loop_.seed(motor_pll_.getNativeAngle());
        }

        // Reset PI when switching to velocity control
        if (mode == ControlMode::CLOSED_LOOP_VELOCITY) {
            pi_speed_controller_.reset();
        }
    }
}

bool BldcController::setElectricMode(ElectricMode mode) noexcept
{
    CriticalSection cs;
    const PiParameters& tuning = mode == ElectricMode::VOLTAGE_MODE
                               ? params_.pi_voltage_mode : params_.pi_current_mode;
    if (!pi_speed_controller_.setParameters(tuning)) return false;
    status_.electric_mode = mode;
    pi_speed_controller_.reset();

    // The inner loop changes shape underneath the outer one; a stale
    // integrator from the other mode is meaningless.
    algorithm().reset();
    return true;
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
    status_.current_pi_saturation = 0;
    pi_speed_controller_.reset();
}

DriveAlgorithm BldcController::getAlgorithm() const noexcept
{
    CriticalSection cs;
    return algorithm_;
}

bool BldcController::setFocCurrentPi(float kp, float ki) noexcept
{
    CriticalSection cs;
    if (!foc_.setCurrentPiGains(kp, ki)) return false;
    params_.foc_current_regulator.kp = kp;
    params_.foc_current_regulator.ki = ki;
    return true;
}

void BldcController::getFocCurrentPi(float& kp, float& ki) const noexcept
{
    CriticalSection cs;
    foc_.getCurrentPiGains(kp, ki);
}

bool BldcController::setFocAngleOffsetDeg(float degrees_e) noexcept
{
    CriticalSection cs;
    const float rad = degrees_e * (FOC_PI / 180.0f);
    if (!foc_.setAngleOffsetRad(rad)) return false;
    params_.foc.angle_offset_rad = rad;
    return true;
}

float BldcController::getFocAngleOffsetDeg() const noexcept
{
    CriticalSection cs;
    return foc_.getAngleOffsetRad() * (180.0f / FOC_PI);
}

void BldcController::setDriveMode(DriveMode mode) noexcept
{
    CriticalSection cs;
    const bool was_neutral = dmode_ == DriveMode::NEUTRAL;
    dmode_ = mode;
    if (dmode_ == DriveMode::NEUTRAL) {
        // High-Z the bridge *before* releasing the pins, so the switches are
        // commanded off rather than merely abandoned.
        pwm_interface_.setNeutral();
        pwm_interface_.enable(false);
        status_.target_speed_rps = Speed{};
        open_loop_.setSpeed(Speed{}, params_.num_poles);
        status_.duty_cycle = Duty{};
        duty_command_ = Duty{};
        status_.target_current = Current{};
        pi_speed_controller_.reset();
        algorithm().reset();
        filtered_measured_current_ = Current{};
    } else {
        if (was_neutral) {
            synchronizeHallPosition();
            if (status_.control_mode == ControlMode::OPEN_LOOP)
                open_loop_.seed(motor_pll_.getNativeAngle());
        }
        pwm_interface_.enable(true);
        // Must follow enable(): the PWM driver ignores channel-state writes
        // while the outputs are off, so FOC's all-phases-UP configuration
        // would be dropped if onEnter() ran first.
        algorithm().onEnter();
    }
}

void BldcController::setAccelerationRate(float rate_rps2) noexcept
{
    Speed rate{};
    if (!tryNumber(rate_rps2, rate) || rate < Speed{}) return;
    CriticalSection cs;
    params_.acceleration_rate = rate_rps2;
    acceleration_step_ = divideByInteger<Accumulator>(rate, ActiveTiming::SPEED_HZ);
    acceleration_enabled_ = rate > Speed{};
    // limited_target_speed_ is deliberately left alone: it is where the ramp
    // has got to so far, and resetting it to the raw target would turn a change
    // of rate limit into the very step the limiter exists to prevent.
}

float BldcController::getAccelerationRate() const noexcept
{
    CriticalSection cs;
    return params_.acceleration_rate;
}

bool BldcController::setSpeedPi(float kp, float ki) noexcept
{
    CriticalSection cs;
    PiParameters p = pi_speed_controller_.getParameters();
    p.kp = kp;
    p.ki = ki;
    if (!pi_speed_controller_.setParameters(p)) return false;
    pi_speed_controller_.reset();
    if (status_.electric_mode == ElectricMode::VOLTAGE_MODE) params_.pi_voltage_mode = p;
    else params_.pi_current_mode = p;
    return true;
}

bool BldcController::setCurrentPi(float kp, float ki) noexcept
{
    CriticalSection cs;
    PiParameters p = params_.pi_current_regulator;
    p.kp = kp;
    p.ki = ki;
    if (!six_step_.setCurrentPi(p)) return false;
    params_.pi_current_regulator = p;
    return true;
}

HallMonitor::Info BldcController::getHallInfo() const noexcept {
    CriticalSection cs;
    return hall_monitor_.getInfo();
}

void BldcController::clearHallFault() noexcept {
    CriticalSection cs;
    hall_monitor_.clearFault();
}

bool BldcController::setHallMonitorParams(const HallMonitorParams& params) noexcept {
    CriticalSection cs;
    return hall_monitor_.setParameters(params);
}

MotorPLL::PllInfo BldcController::getPllInfo() const noexcept {
    CriticalSection cs;
    return motor_pll_.getInfo();
}

bool BldcController::setPllGains(float kp, float ki) noexcept {
    CriticalSection cs;
    return motor_pll_.setGains(kp, ki);
}

DriveMode BldcController::getDriveMode() const noexcept {
    CriticalSection cs;
    return dmode_;
}

void BldcController::getSpeedPiGains(float& kp, float& ki) const noexcept {
    CriticalSection cs;
    const PiParameters& p = pi_speed_controller_.getParameters();
    kp = p.kp;
    ki = p.ki;
}

void BldcController::getCurrentPiGains(float& kp, float& ki) const noexcept {
    CriticalSection cs;
    const PiParameters& p = six_step_.getCurrentPi();
    kp = p.kp;
    ki = p.ki;
}

void BldcController::getPllBaseGains(float& kp, float& ki) const noexcept {
    CriticalSection cs;
    motor_pll_.getBaseGains(kp, ki);
}

void BldcController::start() noexcept
{
    status_.is_running = true;

    CriticalSection cs;
    synchronizeHallPosition(true);
    open_loop_.seed(motor_pll_.getNativeAngle());

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

    // Reset target speed filters
    filtered_target_speed_ = Speed{};
    filtered_measured_speed_ = Speed{};
    limited_target_speed_ = Accumulator{};
}

void BldcController::stop() noexcept
{
    CriticalSection cs;
    status_.is_running = false;
    status_.duty_cycle = Duty{};
    duty_command_ = Duty{};
    // Zero the torque command too. The PWM ISR keeps running after stop() and
    // recomputes the duty from the current loop every cycle, so clearing only
    // the duty left the drive producing exactly as much torque as before.
    status_.target_current = Current{};
    algorithm().reset();
    pwm_interface_.updateNativeDutyCycle(Duty{});

    status_.current_speed_rps = Speed{};
}

MotorStatus BldcController::getStatus() const noexcept
{
    NativeMotorStatus snapshot;
    {
        CriticalSection cs;
        snapshot = status_;
    }
    return motorStatusCast<FloatProfile>(snapshot);
}

void BldcController::synchronizeHallPosition(bool discard_tracking) noexcept
{
    const uint8_t hall = hall_interface_.getPosition();
    status_.measured_position = hall;
    const bool changed = motor_pll_.getHallStateRaw() != hall;
    if (hall <= 5) motor_pll_.updateHall(hall);
    // Preserve a real PLL that is already tracking a coasting rotor.
    if (discard_tracking || changed || hall > 5) motor_pll_.reset();
    // Also feed the Hall monitor through the normal debounced PWM path, and
    // retry an invalid sample without requiring another physical edge.
    hall_update_pending_ = true;
}

void BldcController::holdVelocityLoop() noexcept
{
    pi_speed_controller_.reset();
    filtered_target_speed_ = Speed{};
    // Park the ramp at the *measured* speed rather than at zero, so that
    // re-engaging starts from where the rotor actually is - bumpless whether
    // it is standing still or still coasting.
    limited_target_speed_ = numberCast<Accumulator>(filtered_measured_speed_);
}

BldcController::Speed BldcController::applyAccelerationLimit(Speed target_speed) noexcept
{
    // Stage 1: LPF to smooth noisy analog input
    const Fraction alpha = limits_.target_alpha;
    if (alpha > Fraction{} && alpha < number_v<Fraction, 1>) {
        filtered_target_speed_ += multiplyTo<Speed>(alpha, target_speed - filtered_target_speed_);
    } else {
        filtered_target_speed_ = target_speed;
    }

    // Stage 2: Slew rate limiter
    if (!acceleration_enabled_) {
        limited_target_speed_ = numberCast<Accumulator>(filtered_target_speed_);
        return filtered_target_speed_;
    }

    const Accumulator target = numberCast<Accumulator>(filtered_target_speed_);
    const Accumulator speed_diff = target - limited_target_speed_;
    const Accumulator max_change = acceleration_step_;

    if (numericAbs(speed_diff) <= max_change) {
        limited_target_speed_ = target;
    } else if (speed_diff > Accumulator{}) {
        limited_target_speed_ += max_change;
    } else {
        limited_target_speed_ -= max_change;
    }

    return numberCast<Speed>(limited_target_speed_);
}

void BldcController::hallSensorInterruptHandler() noexcept
{
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
    MotorAlgorithmInputT<Profile> in{};

    // ADC already converted all injected channels — reading registers adds no
    // latency. All three shunts are read even in six-step, which uses one of
    // them: the cost is two extra register reads and two conversions, and it
    // is what lets the two algorithms share a single input path.
    adc_interface_->readNativeCurrents(in.i_u, in.i_v, in.i_w);
    in.bus_voltage = adc_interface_->readNativeBusVoltage();

    // Hall health time base. Reads no hardware; it only decays the monitor's
    // accumulators and ages a standing illegal code, both of which have to run
    // on the PWM clock rather than on Hall events - those arrive at the
    // commutation rate, which is exactly what is unreliable when a line breaks.
    const bool hall_controls_drive = status_.control_mode != ControlMode::OPEN_LOOP;
    const bool hall_guard_active = hall_controls_drive && dmode_ != DriveMode::NEUTRAL && status_.is_running;
    hall_monitor_.tick(hall_guard_active);

    // Process deferred Hall update (debounced re-read from stable GPIO state)
    if (hall_update_pending_) {
        hall_update_pending_ = false;
        // One read, shared by the health monitor and the PLL, so they can never
        // disagree and the lines are never sampled mid-transition.
        uint8_t hall_state = hall_interface_.getPosition();
        hall_monitor_.onPosition(hall_state, hall_guard_active);
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
    Profile::Angle measured_angle{};
    {
        CriticalSection cs;
        in.electric_mode = status_.electric_mode;
        in.target_current = status_.target_current;
        in.duty_command = duty_command_;
        motor_pll_.updateTick();
        measured_angle = motor_pll_.getNativeAngle();
        if (status_.control_mode == ControlMode::OPEN_LOOP) {
            if (status_.is_running) open_loop_.advance(dmode_);
            in.step = open_loop_.step(dmode_, params_.useInverseCommTable);
            in.angle_steps = open_loop_.angle();
            in.speed_steps_s = status_.is_running ? open_loop_.speed(dmode_) : Profile::PllSpeed{};
        } else {
            in.step = motor_pll_.getNextHall(dmode_);
            in.angle_steps = measured_angle;
            in.speed_steps_s = motor_pll_.getNativeSpeedStepsSec();
        }
    }
    in.drive_mode = dmode_;

    // Hall health. The monitor latches, so unlike the old PLL rule this does
    // not re-assert every cycle; it has to be cleared explicitly once the
    // wiring is dealt with.
    if (hall_controls_drive && hall_monitor_.isFaulted() && dmode_ != DriveMode::NEUTRAL) {
        setDriveMode(DriveMode::NEUTRAL);
    }

    if (in.bus_voltage > limits_.max_voltage)
        setDriveMode(DriveMode::NEUTRAL);

    // ---- Algorithm selection ----------------------------------------------
    // A plain switch rather than a virtual call through MotorAlgorithm: the
    // target of each branch is known at compile time, so the compiler can
    // inline it, and at 20 kHz in a `-O0` build every avoided indirection is
    // worth having. The base class still exists for the non-ISR paths
    // (reset/onEnter), where clarity is worth more than cycles.
    const MotorAlgorithmOutputT<Profile> out = [&]() -> MotorAlgorithmOutputT<Profile> {
        if (dmode_ == DriveMode::NEUTRAL) {
            // Do not run the current regulators against an inactive bridge.
            MotorAlgorithmOutputT<Profile> parked{};
            parked.angle_e_rad = multiplyTo<Profile::Angle>(in.angle_steps, FocAlgorithm::Core::RAD_PER_STEP);
            return parked;
        }
        // Construct only the selected result: no redundant clearing/copying
        // of a second fixed-point output structure on every current-loop tick.
        switch (algorithm_) {
            case DriveAlgorithm::FOC:
                return foc_.updateNative(in);

            case DriveAlgorithm::SIX_STEP:
            default:
                return six_step_.updateNative(in);
        }
    }();

    {
        CriticalSection cs;
        // Report a filtered current: the raw value is a single ADC sample per
        // PWM cycle, and sampling that at 100 Hz aliases the chopping ripple
        // into the telemetry. Control and +OSC keep the raw value.
        const Fraction i_alpha = limits_.current_alpha;
        if (i_alpha > Fraction{} && i_alpha < number_v<Fraction, 1>) {
            filtered_measured_current_ += multiplyTo<Current>(i_alpha,
                                                             out.measured_current - filtered_measured_current_);
        } else {
            filtered_measured_current_ = out.measured_current;
        }
        status_.measured_current = out.measured_current;
        status_.measured_current_filtered = filtered_measured_current_;
        if (algorithm_ != DriveAlgorithm::FOC) status_.duty_cycle = out.duty;
        status_.duty_u = out.duty_u;
        status_.duty_v = out.duty_v;
        status_.duty_w = out.duty_w;
        status_.bus_voltage = in.bus_voltage;
        status_.pll_angle = measured_angle;
        status_.current_pi_saturation = out.saturation;
        status_.target_position = in.step;
        status_.i_d = out.i_d;
        status_.i_q = out.i_q;
        status_.v_d = out.v_d;
        status_.v_q = out.v_q;
        status_.angle_e_rad = out.angle_e_rad;
    }
}

} // namespace libecu
