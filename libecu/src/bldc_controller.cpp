/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include "../include/critical_section.hpp"
#include "../include/motor_config.hpp"
#include <algorithm>
#include <stdio.h>
#include <cmath>

// Speed measurement configuration
#define SPEED_TIMEOUT_US          500000  ///< Timeout for speed measurement (1 second)
#define BLDC_NUM_PHASES           3        ///< Number of phases in BLDC motor

namespace libecu {
namespace {
constexpr bool rolling_limit = motor_config::rolling_integral.amps_per_rps > 0;
constexpr bool traction_limit = motor_config::traction_gain_a_per_rps > 0;
}

LIBECU_SLOW_PATH BldcController::BldcController(
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

    // Speed PI integration and the 100 Hz ramp/measurement have separate clocks.
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
    if constexpr (traction_limit) {
        Speed traction_rate{};
        parameters_valid_ &= tryNumber(params_.traction_acceleration_rps2, traction_rate) && traction_rate >= Speed{};
        traction_acceleration_step_ = divideByInteger<Speed>(traction_rate, ActiveTiming::SPEED_HZ);
    }
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

LIBECU_SLOW_PATH bool BldcController::initialize() noexcept
{
    speed_pi_ready_ = false;
    cancelPwmInterruptHandler();
#if defined(LIBECU_NATIVE_CARRIER_ONLY) && LIBECU_NATIVE_CARRIER_ONLY
    {
        CriticalSection lock; // Timer acquisition is already running during controller construction.
        pwm_interface_.parkExclusive();
    }
#else
    pwm_interface_.setNeutral();
#endif
    if (!parameters_valid_ || !pi_speed_controller_.isConfigured() || !six_step_.isCurrentPiConfigured())
        return false;

    // Reset PI controller
    pi_speed_controller_.reset();
    six_step_.reset();
    foc_.reset();

    initialized_ = true;
    return true;
}

LIBECU_SLOW_PATH void BldcController::update() noexcept
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
    const auto speed_steps_s = motor_pll_.getNativeSpeedStepsSec();
    Speed speed_rps = divideByInteger<Speed>(speed_steps_s, params_.num_poles * BLDC_NUM_PHASES);
    if (dmode_ == DriveMode::REVERSE) speed_rps = -speed_rps;

    const Fraction alpha = limits_.speed_alpha;
    if (alpha > Fraction{} && alpha < number_v<Fraction, 1>) {
        filtered_measured_speed_ += multiplyTo<Speed>(alpha, speed_rps - filtered_measured_speed_);
    } else {
        filtered_measured_speed_ = speed_rps;
    }

    // ---- Thermal protection -----------------------------------------------
    // Convert the last complete auxiliary snapshot at the slow-loop rate.
    // Acquisition is owned by each board's sampling pipeline; this read never
    // starts a conversion or waits for it. The board enforces readiness/freshness.
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

        // Only the shared temperature needs publication. Parking is a separate transaction.
        //
        // TEMPERATURE_INVALID_C fails this comparison, so an open sensor
        // coasts rather than trips. See MotorControlParams::max_temperature_c.
        if (temperature_c > limits_.max_temperature && dmode_ != DriveMode::NEUTRAL) {
            setDriveMode(DriveMode::NEUTRAL);
        }
    }

#if LIBECU_FOC_DUTY_TELEMETRY
    struct {
        Profile::Voltage v_d, v_q, bus_voltage;
        DriveAlgorithm algorithm;
    } modulation_snapshot;
#endif
    {
        CriticalSection cs;
        status_.current_speed_rps = filtered_measured_speed_;
#if LIBECU_FOC_DUTY_TELEMETRY
        modulation_snapshot = {status_.v_d, status_.v_q, status_.bus_voltage, algorithm_};
#endif
    }

#if LIBECU_FOC_DUTY_TELEMETRY
    if (modulation_snapshot.algorithm == DriveAlgorithm::FOC) {
        const Duty modulation = FocAlgorithm::Core::modulationMagnitude(
            modulation_snapshot.v_d, modulation_snapshot.v_q, modulation_snapshot.bus_voltage);
        CriticalSection cs;
        if (algorithm_ == DriveAlgorithm::FOC)
            status_.duty_cycle = dmode_ == DriveMode::NEUTRAL ? Duty{} : modulation;
    }
#endif

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
    // ticks simply wound it up again. In a bench run, releasing from
    // 5 RPS: parked target current settled at 3.2 A and held, and the next
    // FORWARD applied it as a step - 3.1 A into a stationary rotor inside
    // 40 ms. Releasing from 12 RPS reached the 9.4 A the throttle logs show.
    if (dmode_ == DriveMode::NEUTRAL) {
        holdVelocityLoop();
        return;
    }
    if (!status_.is_running || status_.control_mode != ControlMode::CLOSED_LOOP_VELOCITY) return;

    // Release residual braking/traction integral at standstill; a moving rotor still needs braking.
    constexpr auto stopped_steps_s = number_v<Profile::PllSpeed, 1, 10>;
    const Speed target = status_.target_speed_rps;
    if (target == Speed{} && numericAbs(speed_steps_s) <= stopped_steps_s &&
        numericAbs(filtered_measured_speed_) <=
            divideByInteger<Speed>(stopped_steps_s, params_.num_poles * BLDC_NUM_PHASES)) {
        speed_pi_ready_ = false;
        pi_speed_controller_.reset();
        if constexpr (traction_limit) traction_acceleration_.reset();
        filtered_target_speed_ = Speed{};
        limited_target_speed_ = Accumulator{};
        publishSpeedPiOutput({});
        return;
    }

    speed_pi_frame_.target = numberCast<SpeedPi::Input>(applyAccelerationLimit(target));
    speed_pi_frame_.measured = numberCast<SpeedPi::Input>(filtered_measured_speed_);
    if ((rolling_limit || traction_limit) && status_.electric_mode == ElectricMode::CURRENT_MODE) {
        if constexpr (rolling_limit)
            speed_pi_frame_.integral_ceiling = numberCast<SpeedPi::Output>(
                rollingIntegralCeiling<Profile, motor_config::rolling_integral>(filtered_measured_speed_, limits_.max_current));
        else
            speed_pi_frame_.integral_ceiling = pi_speed_controller_.native().integralCeiling();
        if constexpr (traction_limit) {
            // Earn acceleration authority only once per new measured-speed sample.
            constexpr auto gain = constant<Speed>(motor_config::traction_gain_a_per_rps);
            speed_pi_frame_.traction_ceiling = numberCast<SpeedPi::Output>(traction_acceleration_.ceiling(
                filtered_measured_speed_, traction_acceleration_step_,
                numberCast<Current>(pi_speed_controller_.native().getOutput()), limits_.max_current, gain));
        }
    }
    speed_pi_ready_ = true;
    updateSpeedPi(); // The slow tick includes exactly one PI step.
}

LIBECU_SLOW_PATH void BldcController::updateSpeedPi() noexcept {
    if (!speed_pi_ready_) return;
    if (dmode_ == DriveMode::NEUTRAL) {
        // A carrier park interrupted the slow preparation before it marked the frame ready.
        holdVelocityLoop();
        if constexpr (traction_limit) traction_acceleration_.reset();
        return;
    }
    const auto& frame = speed_pi_frame_;
    SpeedPi::Output output;
    // Saturation comes from the live current loop, not the last 100 Hz measurement.
    const int saturation = status_.current_pi_saturation;
    if ((rolling_limit || traction_limit) && status_.electric_mode == ElectricMode::CURRENT_MODE) {
        if constexpr (traction_limit)
            output = pi_speed_controller_.updateNativeCapped(frame.target, frame.measured, saturation,
                                                             frame.integral_ceiling, frame.traction_ceiling);
        else
            output = pi_speed_controller_.updateNativeCapped(frame.target, frame.measured, saturation,
                                                             frame.integral_ceiling);
    } else {
        output = pi_speed_controller_.updateNative(frame.target, frame.measured, saturation);
    }
    publishSpeedPiOutput(output);
}

LIBECU_SLOW_PATH void BldcController::publishSpeedPiOutput(SpeedPi::Output output) noexcept {
    // SysTick cannot re-enable the bridge. PWM can park it during the unmasked PI.
    if (status_.electric_mode == ElectricMode::VOLTAGE_MODE) {
        const Duty duty = numberCast<Duty>(output);
        __asm__ volatile("" : : "m"(duty) : "memory");
        CriticalSection lock;
        if (dmode_ != DriveMode::NEUTRAL) {
            duty_command_ = duty;
            return;
        }
    } else {
        const Current current = numberCast<Current>(output);
        __asm__ volatile("" : : "m"(current) : "memory");
        CriticalSection lock;
        if (dmode_ != DriveMode::NEUTRAL) {
            status_.target_current = current;
            return;
        }
    }
    // Discard integrator state calculated across a park, as well as the old demand.
    holdVelocityLoop();
    if constexpr (traction_limit) traction_acceleration_.reset();
}

void BldcController::setTargetSpeed(float speed_rps) noexcept
{
    Speed target{};
    if (!tryNumber(speed_rps, target)) return;
    setNativeTargetSpeed(target);
}

void BldcController::setNativeTargetSpeed(Speed target) noexcept {
    PreparedTargetSpeed prepared;
    if (prepareNativeTargetSpeed(target, prepared)) commitNativeTargetSpeed(prepared);
}

LIBECU_SLOW_PATH
bool BldcController::prepareNativeTargetSpeed(Speed target, PreparedTargetSpeed& prepared) const noexcept {
    prepared = PreparedTargetSpeed{};
    if (target < Speed{}) return false;
    // Clamp to maximum speed
    const Speed limited = std::min(target, limits_.max_speed);
    if (!OpenLoopPhase<Profile, ActiveTiming::CURRENT_HZ>::prepareSpeed(limited, params_.num_poles, prepared.rate_))
        return false;
    prepared.limited_speed_ = limited;
    prepared.owner_ = this;
    return true;
}

bool BldcController::commitNativeTargetSpeed(const PreparedTargetSpeed& prepared) noexcept {
    CriticalSection cs;
    return commitNativeTargetSpeed(prepared, cs);
}

bool BldcController::commitNativeTargetSpeed(const PreparedTargetSpeed& prepared,
                                            const CriticalSection&) noexcept {
    if (prepared.owner_ != this || dmode_ == DriveMode::NEUTRAL) return false;
    open_loop_.applyPreparedRate(prepared.rate_);
    // Positive commands go through the next slow ramp sample. On release, don't keep integrating
    // the old positive request before that sample; an identical lease refresh does nothing.
    if (prepared.limited_speed_ == Speed{} && status_.target_speed_rps != Speed{}) speed_pi_ready_ = false;
    status_.target_speed_rps = prepared.limited_speed_;
    return true;
}

void BldcController::setDutyCycle(float duty_cycle) noexcept
{
    Duty target{};
    if (!tryNumber(duty_cycle, target) || duty_cycle < 0.0f) return;
    const Duty duty = numericClamp(target, Duty{}, limits_.max_duty);
    __asm__ volatile("" : : "m"(duty) : "memory");
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    // duty_command_ is the request; status_.duty_cycle is what the algorithm
    // reports back. They are the same number in six-step voltage mode but not
    // in FOC, where the reported value is the modulation index of the applied
    // voltage vector - feeding that back as the command would make it decay.
    duty_command_ = duty;
    status_.duty_cycle = duty;
}

void BldcController::setCurrent(float current_a) noexcept
{
    Current target{};
    if (!tryNumber(current_a, target)) return;
    const Current current = numericClamp(target, limits_.min_current, limits_.max_current);
    __asm__ volatile("" : : "m"(current) : "memory");
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    status_.target_current = current;
}

void BldcController::setControlMode(ControlMode mode) noexcept
{
    CriticalSection cs;
    if (status_.control_mode != mode) {
        speed_pi_ready_ = false;
        if constexpr (traction_limit) traction_acceleration_.reset();
        cancelPwmInterruptHandler();
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
    // Tuning setters share the foreground command owner. Neither control IRQ
    // writes this configuration. A mode switch always starts with a reset PI.
    const PiParameters& tuning = mode == ElectricMode::VOLTAGE_MODE
                               ? params_.pi_voltage_mode : params_.pi_current_mode;
    const auto native = SpeedPi::prepareTuning(tuning);
    if (!native) return false;
    __asm__ volatile("" : : "m"(*native) : "memory");
    CriticalSection cs;
    pi_speed_controller_.applyTuning<true>(tuning, *native);
    speed_pi_ready_ = false;
    cancelPwmInterruptHandler();
    status_.electric_mode = mode;
    if constexpr (traction_limit) traction_acceleration_.reset();

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
    speed_pi_ready_ = false;
    cancelPwmInterruptHandler();

    // Hand the bridge over. The outgoing algorithm's integrators are cleared
    // so it cannot resume from a state that belonged to a different
    // modulation scheme, and the incoming one establishes its switching
    // configuration - six-step re-commutates from scratch, FOC puts all three
    // phases in UP and raises its one and only COM event.
    algorithm().reset();
    algorithm_ = new_algorithm;
    if constexpr (traction_limit) traction_acceleration_.reset();
    status_.algorithm = algorithm_;
#if !LIBECU_FOC_DUTY_TELEMETRY
    if (algorithm_ == DriveAlgorithm::FOC) status_.duty_cycle = Duty{};
#endif
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
    FocAlgorithm::PreparedGains gains;
    if (!FocAlgorithm::prepareCurrentPiGains(kp, ki, gains)) return false;
    __asm__ volatile("" : : "m"(gains) : "memory");
    CriticalSection cs;
    foc_.applyCurrentPiGains(gains);
    params_.foc_current_regulator.kp = kp;
    params_.foc_current_regulator.ki = ki;
    return true;
}

void BldcController::getFocCurrentPi(float& kp, float& ki) const noexcept
{
    FocAlgorithm::Core::Gain p{}, i{};
    {
        CriticalSection cs;
        foc_.getNativeCurrentPiGains(p, i);
    }
    kp = numberCast<float>(p);
    ki = numberCast<float>(i);
}

bool BldcController::setFocAngleOffsetDeg(float degrees_e) noexcept
{
    const float rad = degrees_e * (FOC_PI / 180.0f);
    FocAlgorithm::Core::Angle angle{};
    if (!tryNumber(rad, angle)) return false;
    __asm__ volatile("" : : "m"(angle), "m"(rad) : "memory");
    CriticalSection cs;
    foc_.applyAngleOffset(rad, angle);
    params_.foc.angle_offset_rad = rad;
    return true;
}

float BldcController::getFocAngleOffsetDeg() const noexcept
{
    float rad;
    {
        CriticalSection cs;
        rad = foc_.getAngleOffsetRad();
    }
    return rad * (180.0f / FOC_PI);
}

void BldcController::clearParkedCommands() noexcept {
    speed_pi_ready_ = false;
    status_.target_speed_rps = Speed{};
    open_loop_.applyPreparedRate({}); // Zero DDS increment/rate, retaining the running phase.
    status_.duty_cycle = Duty{};
    duty_command_ = Duty{};
    status_.target_current = Current{};
    pi_speed_controller_.reset();
    algorithm().reset();
    filtered_measured_current_ = Current{};
}

[[gnu::cold, gnu::noinline]] void BldcController::parkFromCarrier() noexcept {
    if (traction_limit && dmode_ != DriveMode::NEUTRAL) traction_acceleration_.reset();
    dmode_ = DriveMode::NEUTRAL;
    pwm_interface_.parkExclusive(); // Gate off first; a preempting break can only reinforce the stop.
    clearParkedCommands();
}

LIBECU_HOT_LOOP void BldcController::setDriveMode(DriveMode mode) noexcept
{
    CriticalSection cs;
    const bool was_neutral = dmode_ == DriveMode::NEUTRAL;
    if (dmode_ != mode) {
        speed_pi_ready_ = false;
        if constexpr (traction_limit) traction_acceleration_.reset();
    }
    dmode_ = mode;
    if (dmode_ == DriveMode::NEUTRAL) {
        pwm_interface_.parkExclusive();
        clearParkedCommands();
    } else {
        cancelPwmInterruptHandler();
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
    const Accumulator step = divideByInteger<Accumulator>(rate, ActiveTiming::SPEED_HZ);
    __asm__ volatile("" : : "m"(step) : "memory");
    CriticalSection cs;
    params_.acceleration_rate = rate_rps2;
    acceleration_step_ = step;
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

LIBECU_SLOW_PATH bool BldcController::setSpeedPi(float kp, float ki) noexcept
{
    using Pi = decltype(pi_speed_controller_);
    using Kernel = Pi::Kernel;
    Pi::Gains gains;
    if (!Pi::prepareGains(kp, ki, gains)) return false;
    __asm__ volatile("" : : "m"(gains) : "memory");

    const auto& pi = pi_speed_controller_.native();
    const auto active = [&] {
        return dmode_ != DriveMode::NEUTRAL && status_.is_running &&
               status_.control_mode == ControlMode::CLOSED_LOOP_VELOCITY;
    };
    const auto publish = [&](const Kernel::Accumulator* integral) {
        pi_speed_controller_.applyGains(kp, ki, gains, integral);
        auto& tuning = status_.electric_mode == ElectricMode::VOLTAGE_MODE
                     ? params_.pi_voltage_mode : params_.pi_current_mode;
        tuning.kp = kp;
        tuning.ki = ki;
    };

    Kernel::OperatingPoint point;
    Pi::Output floor, ceiling;
    ElectricMode mode;
    DriveMode direction;
    Speed rolling_speed{};
    {
        CriticalSection lock;
        if (!pi_speed_controller_.isConfigured()) return false;
        const auto old_kp = pi.proportionalGain();
        const bool same = gains.kp == old_kp && gains.ki == pi.integralGain();
        if (!same && active() && (pi.isSaturated() || status_.current_pi_saturation)) return false;
        if (gains.kp == old_kp || !active()) {
            publish(nullptr); // In particular, a no-op or Ki-only change keeps every fractional bit of I.
            return true;
        }
        direction = dmode_;
        mode = status_.electric_mode;
        point = pi.operatingPoint();
        floor = pi.integralFloor();
        ceiling = pi.integralCeiling();
        if constexpr (rolling_limit) rolling_speed = status_.current_speed_rps;
    }
    if (rolling_limit && mode == ElectricMode::CURRENT_MODE) {
        const auto limit = rollingIntegralCeiling<Profile, motor_config::rolling_integral>(rolling_speed, limits_.max_current);
        ceiling = std::clamp(numberCast<Pi::Output>(limit), floor, ceiling);
    }
    Kernel::Accumulator integral;
    if (!Kernel::rebaseIntegral(point, gains.kp, floor, ceiling, integral)) return false;
    __asm__ volatile("" : : "m"(integral) : "memory");
    // The PI runs in SysTick. Only commit against a still-valid operating
    // point: a tick or a higher-priority park can occur during preparation.
    CriticalSection lock;
    const auto& now = pi.operatingPoint();
    if (!active() || dmode_ != direction || status_.electric_mode != mode ||
        pi.isSaturated() || status_.current_pi_saturation ||
        now.error != point.error || now.output != point.output) return false;
    if (rolling_limit && mode == ElectricMode::CURRENT_MODE && status_.current_speed_rps != rolling_speed) return false;
    publish(&integral);
    return true;
}

bool BldcController::setCurrentPi(float kp, float ki) noexcept
{
    using Pi = SixStepAlgorithm::CurrentPi;
    if (six_step_.isCurrentPiConfigured()) {
        Pi::Gains gains;
        if (!Pi::prepareGains(kp, ki, gains)) return false;
        __asm__ volatile("" : : "m"(gains) : "memory");
        CriticalSection cs;
        six_step_.resetCurrentPiGains(kp, ki, gains);
        params_.pi_current_regulator.kp = kp;
        params_.pi_current_regulator.ki = ki;
        return true;
    }
    // Startup may have rejected the original gains. Revalidate the rails and
    // Kb too before making a previously unconfigured regulator usable.
    PiParameters p = params_.pi_current_regulator;
    p.kp = kp;
    p.ki = ki;
    const auto native = Pi::prepareTuning(p);
    if (!native) return false;
    __asm__ volatile("" : : "m"(*native) : "memory");
    CriticalSection cs;
    six_step_.resetCurrentPi(p, *native);
    params_.pi_current_regulator.kp = kp;
    params_.pi_current_regulator.ki = ki;
    return true;
}

HallMonitor::Info BldcController::getHallInfo() const noexcept {
    auto snapshot = [this] {
        CriticalSection cs;
        return hall_monitor_.snapshot();
    }();
    // An interrupt-mask barrier alone does not order pure soft-float/divide
    // helpers. Consume the raw snapshot strictly after PRIMASK restoration.
    __asm__ volatile("" : "+m"(snapshot) : : "memory");
    return HallMonitor::describe(snapshot);
}

HallFault BldcController::getHallFault() const noexcept {
    // The volatile scalar latch is atomic on all supported Cortex-M targets.
    // Callers checking a larger transaction hold their own snapshot mask.
    return hall_monitor_.faultCause();
}

void BldcController::clearHallFault() noexcept {
    CriticalSection cs;
    hall_monitor_.clearFault();
}

bool BldcController::setHallMonitorParams(const HallMonitorParams& params) noexcept {
    HallMonitor::PreparedParameters prepared;
    if (!HallMonitor::prepareParameters(params, prepared)) return false;
    // Materialize the whole bundle before masking, including under LTO: pure
    // coefficient arithmetic must not be sunk into the publication section.
    __asm__ volatile("" : : "m"(prepared) : "memory");
    CriticalSection cs;
    hall_monitor_.applyPrepared(prepared);
    return true;
}

MotorPLL::PllInfo BldcController::getPllInfo() const noexcept {
    auto state = [this] {
        CriticalSection cs;
        return motor_pll_.nativeInfo();
    }();
    __asm__ volatile("" : "+m"(state) : : "memory");
    return MotorPLL::describe(state);
}

bool BldcController::setPllGains(float kp, float ki) noexcept {
    MotorPLL::PreparedGains prepared;
    if (!MotorPLL::prepareGains(kp, ki, prepared)) return false;
    __asm__ volatile("" : : "m"(prepared) : "memory"); // Keep coefficient calculation before the mask under LTO.
    CriticalSection cs;
    motor_pll_.applyPrepared(prepared);
    return true;
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
    MotorPLL::Core::Gain p, i;
    {
        CriticalSection cs;
        motor_pll_.getNativeBaseGains(p, i);
    }
    kp = numberCast<float>(p);
    ki = numberCast<float>(i);
}

void BldcController::start() noexcept
{
    CriticalSection cs;
    speed_pi_ready_ = false;
    status_.is_running = true;
    cancelPwmInterruptHandler();
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
    if constexpr (traction_limit) traction_acceleration_.reset();
    filtered_target_speed_ = Speed{};
    filtered_measured_speed_ = Speed{};
    limited_target_speed_ = Accumulator{};
}

void BldcController::stop() noexcept
{
    CriticalSection cs;
    speed_pi_ready_ = false;
    cancelPwmInterruptHandler();
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
    speed_pi_ready_ = false;
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

LIBECU_HOT_LOOP void BldcController::hallSensorInterruptHandler() noexcept
{
    // Coalesce Hall events until the next PWM frame. This does not
    // guarantee a minimum settling time: an edge can arrive immediately before it.
    // Input pulse rejection belongs to the board's Hall acquisition path.
    hall_update_pending_ = true;
}

#if defined(LIBECU_PWM_DEADLINE_ONLY) && LIBECU_PWM_DEADLINE_ONLY
[[gnu::always_inline]] inline
#else
LIBECU_HOT_LOOP
#endif
void BldcController::observeHall(bool guard_active, uint8_t position) noexcept {
    // Age Hall health once per PWM frame even without a new edge.
    hall_monitor_.tick(guard_active);
    if (hall_update_pending_) {
        hall_update_pending_ = false;
        // Share one decoded sample between the health monitor and the PLL.
        // This alone does not qualify a raw GPIO transition as stable.
        const uint8_t hall_state = position == HALL_NOT_SAMPLED ? hall_interface_.getPosition() : position;
        hall_monitor_.onPosition(hall_state, guard_active);
        if (hall_state <= 5) {
            status_.measured_position = hall_state;
            motor_pll_.updateHall(hall_state);
        } else {
            // Recheck an illegal GPIO code on the next PWM tick.
            // A brief glitch must not look continuously illegal merely because
            // no further filtered Hall edge arrives during the hold timer.
            hall_update_pending_ = true;
        }
    }
}

LIBECU_HOT_LOOP void BldcController::pwmInterruptHandler() noexcept {
    preparePwmInterruptHandler();
    completePwmInterruptHandler();
}

void BldcController::preparePwmInterruptHandler() noexcept { preparePwmFrame<false>(); }
LIBECU_DEADLINE_CODE("prepare_drive") LIBECU_HOT_LOOP
void BldcController::prepareOwnedCarrier(uint8_t position) noexcept { preparePwmFrame<true>(position); }

template<bool Owned>
#if defined(LIBECU_PWM_DEADLINE_ONLY) && LIBECU_PWM_DEADLINE_ONLY
[[gnu::always_inline]] inline
#else
LIBECU_HOT_LOOP
#endif
void BldcController::preparePwmFrame(uint8_t position) noexcept {
    if (!adc_interface_ || pwm_prepared_) return;
    // Only the serialized carrier can consume this frame. A higher-priority
    // break can cancel it even in updateTick(); never set the flag again on return.
    pwm_prepared_ = true;

    prepared_hall_controls_drive_ = status_.control_mode != ControlMode::OPEN_LOOP;
    const bool hall_guard_active = prepared_hall_controls_drive_ &&
                                   dmode_ != DriveMode::NEUTRAL && status_.is_running;
    observeHall(hall_guard_active, position);

    recordLoopStage(LoopStage::Hall);
    // Keep the command snapshot coherent with this tick's PLL and angle.
    auto& in = prepared_pwm_frame_.input;
    if constexpr (!Owned) {
        // Portable split callers can change a command between callbacks.
        // The serialized owner instead reads the original status fields in FOC.
        in.electric_mode = status_.electric_mode;
        in.target_current = status_.target_current;
    }
    in.duty_command = duty_command_;
    in.drive_mode = dmode_;
    motor_pll_.updateTick();
    prepared_pwm_measured_angle_ = motor_pll_.getNativeAngle();
    if (status_.control_mode == ControlMode::OPEN_LOOP) {
        if (status_.is_running) open_loop_.advance(in.drive_mode);
        in.step = open_loop_.step(in.drive_mode, params_.useInverseCommTable);
        in.angle_steps = open_loop_.angle();
        in.speed_steps_s = status_.is_running ? open_loop_.speed(in.drive_mode) : Profile::PllSpeed{};
    } else {
        in.step = motor_pll_.getNextHall(in.drive_mode, prepared_pwm_measured_angle_);
        in.angle_steps = prepared_pwm_measured_angle_;
        in.speed_steps_s = motor_pll_.getNativeSpeedStepsSec();
    }
    recordLoopStage(LoopStage::Pll);
}

LIBECU_HOT_LOOP void BldcController::completePwmInterruptHandler() noexcept {
    if (!pwm_prepared_) return;
    completePwmFrom(*adc_interface_, pwm_interface_);
}
inline void BldcController::filterPublishedCurrent() noexcept {
#if defined(LIBECU_RAW_CURRENT_TELEMETRY) && LIBECU_RAW_CURRENT_TELEMETRY
    // Preserve the legacy status/wire field; publish this frame's current.
    // The FOC d/q feedback filters are independent and remain in the control core.
    status_.measured_current_filtered = status_.measured_current;
#else
    const Fraction alpha = limits_.current_alpha;
    const Current measured = status_.measured_current;
    filtered_measured_current_ = alpha > Fraction{} && alpha < number_v<Fraction, 1>
        ? CurrentFeedbackFilter<Profile>::advance(filtered_measured_current_, measured, alpha) : measured;
    status_.measured_current_filtered = filtered_measured_current_;
#endif
}
LIBECU_HOT_LOOP void BldcController::finishOwnedStatus() noexcept {
    const auto& in = prepared_pwm_frame_.input;
    status_.bus_voltage = in.bus_voltage;
    status_.pll_angle = prepared_pwm_measured_angle_;
    status_.target_position = in.step;
    filterPublishedCurrent();
    recordLoopStage(LoopStage::Status);
}
LIBECU_HOT_LOOP void BldcController::publishPwmStatus() noexcept {
    const auto& in = prepared_pwm_frame_.input;
    const auto& out = prepared_pwm_frame_.output;
    {
        CriticalSection cs;
        // Report a filtered current: the raw value is one current-frame value per
        // PWM cycle, and sampling that at 100 Hz aliases the chopping ripple
        // into the telemetry. Control and +OSC keep the raw value.
        status_.measured_current = out.measured_current;
        filterPublishedCurrent();
        if (algorithm_ != DriveAlgorithm::FOC) status_.duty_cycle = out.duty;
#if !LIBECU_FOC_DUTY_TELEMETRY
        else status_.duty_cycle = Duty{};
#endif
        status_.duty_u = out.duty_u;
        status_.duty_v = out.duty_v;
        status_.duty_w = out.duty_w;
        status_.bus_voltage = in.bus_voltage;
        status_.pll_angle = prepared_pwm_measured_angle_;
        status_.current_pi_saturation = out.saturation;
        status_.target_position = in.step;
        status_.i_d = out.i_d;
        status_.i_q = out.i_q;
        status_.v_d = out.v_d;
        status_.v_q = out.v_q;
        status_.angle_e_rad = out.angle_e_rad;
    }
    recordLoopStage(LoopStage::Status);
}

} // namespace libecu
