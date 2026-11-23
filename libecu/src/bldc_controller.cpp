/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include "../include/critical_section.hpp"
#include "../include/motor_config.hpp"
#include <algorithm>
#include <cmath>

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
    , io_(params.max_speed_rps * params.num_poles * BLDC_NUM_PHASES, params.useInverseCommTable)
    , six_step_(pwm_interface)
    , foc_(pwm_interface)
    , params_(params)
{
    io_.in.algorithm = params.algorithm == DriveAlgorithm::FOC ? DriveAlgorithm::FOC : DriveAlgorithm::SIX_STEP;

    // Speed PI integration and the 100 Hz ramp/measurement have separate clocks.
    params_.speed_pi.min_output = params_.min_current;
    params_.speed_pi.max_output = params_.max_current;
    // Output limits come from the caller. With a feed-forward attached the PI
    // has to be able to trim *downwards*, so min_output must be allowed to go
    // negative; fall back to [0,1] when the caller left them unset.
    if (params_.pi_current_regulator.max_output <= params_.pi_current_regulator.min_output) {
        params_.pi_current_regulator.min_output = 0.0f;
        params_.pi_current_regulator.max_output = 1.0f;
    }

    pi_speed_controller_.setParameters(params_.speed_pi);

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
        MotorField::prepareSpeed(limits_.max_speed, params_.num_poles, io_.in.field_rate) &&
        foc_.setParams(foc_params) &&
        foc_.setCurrentPiGains(params_.foc_current_regulator.kp, params_.foc_current_regulator.ki);
    acceleration_step_ = divideByInteger<Accumulator>(acceleration, ActiveTiming::SPEED_HZ);
    acceleration_enabled_ = acceleration > Speed{};
    if constexpr (traction_limit) {
        Speed traction_rate{};
        parameters_valid_ &= tryNumber(params_.traction_acceleration_rps2, traction_rate) && traction_rate >= Speed{};
        traction_acceleration_step_ = divideByInteger<Speed>(traction_rate, ActiveTiming::SPEED_HZ);
    }
    io_.in.field_rate = {};

    io_.out.pll.setUsePLL(true);
}

MotorAlgorithm& BldcController::algorithm() noexcept
{
    return (io_.in.algorithm == DriveAlgorithm::FOC)
         ? static_cast<MotorAlgorithm&>(foc_)
         : static_cast<MotorAlgorithm&>(six_step_);
}

LIBECU_SLOW_PATH bool BldcController::initialize() noexcept
{
    speed_pi_ready_ = false;
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

    return true;
}

LIBECU_SLOW_PATH void BldcController::update() noexcept
{
    // Speed feedback, signed **relative to the commanded direction**: positive
    // when the rotor turns the way io_.in.drive_mode asks, negative when it turns against
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
    const auto speed_steps_s = io_.out.pll.speed_steps_s;
    Speed speed_rps = divideByInteger<Speed>(speed_steps_s, params_.num_poles * BLDC_NUM_PHASES);
    if (io_.in.drive_mode == DriveMode::REVERSE) speed_rps = -speed_rps;

    const Fraction alpha = limits_.speed_alpha;
    const Speed previous_speed = io_.in.current_speed_rps;
    if (alpha > Fraction{} && alpha < number_v<Fraction, 1>) {
        speed_rps = previous_speed + multiplyTo<Speed>(alpha, speed_rps - previous_speed);
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
            io_.in.temperature_c = temperature_c;
        }

        // Only the shared temperature needs publication. Parking is a separate transaction.
        //
        // TEMPERATURE_INVALID_C fails this comparison, so an open sensor
        // coasts rather than trips. See MotorControlParams::max_temperature_c.
        if (temperature_c > limits_.max_temperature && io_.in.drive_mode != DriveMode::NEUTRAL) {
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
        io_.in.current_speed_rps = speed_rps;
#if LIBECU_FOC_DUTY_TELEMETRY
        const auto& e = io_.out.electrical;
        modulation_snapshot = {e.v_d, e.v_q, e.bus_voltage, io_.in.algorithm};
#endif
    }

#if LIBECU_FOC_DUTY_TELEMETRY
    if (modulation_snapshot.algorithm == DriveAlgorithm::FOC) {
        const Duty modulation = FocAlgorithm::Core::modulationMagnitude(
            modulation_snapshot.v_d, modulation_snapshot.v_q, modulation_snapshot.bus_voltage);
        CriticalSection cs;
        if (io_.in.algorithm == DriveAlgorithm::FOC)
            io_.in.foc_modulation = io_.in.drive_mode == DriveMode::NEUTRAL ? Duty{} : modulation;
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
    if (io_.in.drive_mode == DriveMode::NEUTRAL) {
        holdVelocityLoop();
        return;
    }
    if (!io_.in.is_running || io_.in.control_mode != ControlMode::CLOSED_LOOP_VELOCITY) return;

    // Release residual braking/traction integral at standstill; a moving rotor still needs braking.
    constexpr auto stopped_steps_s = number_v<Profile::PllSpeed, 1, 10>;
    const Speed target = io_.in.target_speed_rps;
    if (target == Speed{} && numericAbs(speed_steps_s) <= stopped_steps_s &&
        numericAbs(io_.in.current_speed_rps) <=
            divideByInteger<Speed>(stopped_steps_s, params_.num_poles * BLDC_NUM_PHASES)) {
        speed_pi_ready_ = false;
        pi_speed_controller_.reset();
        if constexpr (traction_limit) traction_acceleration_.reset();
        filtered_target_speed_ = Speed{};
        limited_target_speed_ = Accumulator{};
        publishSpeedPiOutput({});
        return;
    }

    applyAccelerationLimit(target);
    if constexpr (rolling_limit || traction_limit) {
        if constexpr (rolling_limit)
            speed_pi_limits_.integral_ceiling = numberCast<SpeedPi::Output>(
                rollingIntegralCeiling<Profile, motor_config::rolling_integral>(io_.in.current_speed_rps, limits_.max_current));
        else
            speed_pi_limits_.integral_ceiling = pi_speed_controller_.native().integralCeiling();
        if constexpr (traction_limit) {
            // Earn acceleration authority only once per new measured-speed sample.
            constexpr auto gain = constant<Speed>(motor_config::traction_gain_a_per_rps);
            speed_pi_limits_.traction_ceiling = numberCast<SpeedPi::Output>(traction_acceleration_.ceiling(
                io_.in.current_speed_rps, traction_acceleration_step_,
                numberCast<Current>(pi_speed_controller_.native().getOutput()), limits_.max_current, gain));
        }
    }
    speed_pi_ready_ = true;
    updateSpeedPi(); // The slow tick includes exactly one PI step.
}

LIBECU_SLOW_PATH void BldcController::updateSpeedPi() noexcept {
    if (!speed_pi_ready_) return;
    if (io_.in.drive_mode == DriveMode::NEUTRAL) {
        // A carrier park interrupted the slow preparation before it marked the frame ready.
        holdVelocityLoop();
        if constexpr (traction_limit) traction_acceleration_.reset();
        return;
    }
    const auto target = numberCast<SpeedPi::Input>(numberCast<Speed>(limited_target_speed_));
    const auto measured = numberCast<SpeedPi::Input>(io_.in.current_speed_rps);
    SpeedPi::Output output;
    // Saturation comes from the live current loop, not the last 100 Hz measurement.
    const int saturation = io_.out.electrical.current_pi_saturation;
    if constexpr (rolling_limit || traction_limit) {
        if constexpr (traction_limit)
            output = pi_speed_controller_.updateNativeCapped(target, measured, saturation,
                                                             speed_pi_limits_.integral_ceiling, speed_pi_limits_.traction_ceiling);
        else
            output = pi_speed_controller_.updateNativeCapped(target, measured, saturation,
                                                             speed_pi_limits_.integral_ceiling);
    } else {
        output = pi_speed_controller_.updateNative(target, measured, saturation);
    }
    publishSpeedPiOutput(output);
}

LIBECU_SLOW_PATH void BldcController::publishSpeedPiOutput(SpeedPi::Output output) noexcept {
    // SysTick cannot re-enable the bridge. PWM can park it during the unmasked PI.
    const Current current = numberCast<Current>(output);
    __asm__ volatile("" : : "m"(current) : "memory");
    {
        CriticalSection lock;
        if (io_.in.drive_mode != DriveMode::NEUTRAL) {
            io_.in.target_current = current;
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
    if (prepared.owner_ != this || io_.in.drive_mode == DriveMode::NEUTRAL) return false;
    io_.in.field_rate = prepared.rate_;
    // Positive commands go through the next slow ramp sample. On release, don't keep integrating
    // the old positive request before that sample; an identical lease refresh does nothing.
    if (prepared.limited_speed_ == Speed{} && io_.in.target_speed_rps != Speed{}) speed_pi_ready_ = false;
    io_.in.target_speed_rps = prepared.limited_speed_;
    return true;
}

void BldcController::setCurrent(float current_a) noexcept
{
    Current target{};
    if (!tryNumber(current_a, target)) return;
    const Current current = numericClamp(target, limits_.min_current, limits_.max_current);
    __asm__ volatile("" : : "m"(current) : "memory");
    CriticalSection cs;
    if (io_.in.drive_mode == DriveMode::NEUTRAL)
        return;
    io_.in.target_current = current;
}

void BldcController::setControlMode(ControlMode mode) noexcept
{
    CriticalSection cs;
    if (io_.in.control_mode != mode) {
        speed_pi_ready_ = false;
        if constexpr (traction_limit) traction_acceleration_.reset();
        const bool from_open_loop = io_.in.control_mode == ControlMode::OPEN_LOOP;
        io_.in.control_mode = mode;
        if (from_open_loop) {
            // Hall tracking continues during open loop. Refresh a stationary
            // sensor without discarding a valid estimate of a moving rotor.
            synchronizeHallPosition();
            algorithm().reset();
        } else if (mode == ControlMode::OPEN_LOOP) {
            io_.out.field.seed(io_.out.pll.angle_steps);
        }

        // Reset PI when switching to velocity control
        if (mode == ControlMode::CLOSED_LOOP_VELOCITY) {
            pi_speed_controller_.reset();
        }
    }
}

void BldcController::setAlgorithm(DriveAlgorithm new_algorithm) noexcept
{
    CriticalSection cs;
    if (io_.in.algorithm == new_algorithm) {
        return;
    }
    speed_pi_ready_ = false;

    // Hand the bridge over. The outgoing algorithm's integrators are cleared
    // so it cannot resume from a state that belonged to a different
    // modulation scheme, and the incoming one establishes its switching
    // configuration - six-step re-commutates from scratch, FOC puts all three
    // phases in UP and raises its one and only COM event.
    algorithm().reset();
    io_.in.algorithm = new_algorithm;
    if constexpr (traction_limit) traction_acceleration_.reset();
#if LIBECU_FOC_DUTY_TELEMETRY
    io_.in.foc_modulation = Duty{};
#endif
    algorithm().onEnter();

    // The speed loop's anti-windup input came from the old inner loop.
    io_.out.electrical.current_pi_saturation = 0;
    pi_speed_controller_.reset();
}

DriveAlgorithm BldcController::getAlgorithm() const noexcept
{
    CriticalSection cs;
    return io_.in.algorithm;
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
    io_.in.target_speed_rps = Speed{};
    io_.in.field_rate = {}; // Zero DDS increment/rate, retaining the running phase.
    io_.in.target_current = Current{};
    pi_speed_controller_.reset();
    algorithm().reset();
#if LIBECU_FOC_DUTY_TELEMETRY
    io_.in.foc_modulation = {};
#endif
#if !defined(LIBECU_RAW_CURRENT_TELEMETRY) || !LIBECU_RAW_CURRENT_TELEMETRY
    io_.out.measured_current_filtered = {};
#endif
}

[[gnu::cold, gnu::noinline]] void BldcController::parkFromCarrier() noexcept {
    if (traction_limit && io_.in.drive_mode != DriveMode::NEUTRAL) traction_acceleration_.reset();
    io_.in.drive_mode = DriveMode::NEUTRAL;
    pwm_interface_.parkExclusive(); // Gate off first; a preempting break can only reinforce the stop.
    clearParkedCommands();
}

LIBECU_HOT_LOOP void BldcController::setDriveMode(DriveMode mode) noexcept
{
    CriticalSection cs;
    const bool was_neutral = io_.in.drive_mode == DriveMode::NEUTRAL;
    if (io_.in.drive_mode != mode) {
        speed_pi_ready_ = false;
        if constexpr (traction_limit) traction_acceleration_.reset();
    }
    io_.in.drive_mode = mode;
    if (io_.in.drive_mode == DriveMode::NEUTRAL) {
        pwm_interface_.parkExclusive();
        clearParkedCommands();
    } else {
        if (was_neutral) {
            synchronizeHallPosition();
            if (io_.in.control_mode == ControlMode::OPEN_LOOP)
                io_.out.field.seed(io_.out.pll.angle_steps);
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
        return io_.in.drive_mode != DriveMode::NEUTRAL && io_.in.is_running &&
               io_.in.control_mode == ControlMode::CLOSED_LOOP_VELOCITY;
    };
    const auto publish = [&](const Kernel::Accumulator* integral) {
        pi_speed_controller_.applyGains(kp, ki, gains, integral);
        params_.speed_pi.kp = kp;
        params_.speed_pi.ki = ki;
    };

    Kernel::OperatingPoint point;
    Pi::Output floor, ceiling;
    DriveMode direction;
    Speed rolling_speed{};
    {
        CriticalSection lock;
        if (!pi_speed_controller_.isConfigured()) return false;
        const auto old_kp = pi.proportionalGain();
        const bool same = gains.kp == old_kp && gains.ki == pi.integralGain();
        if (!same && active() && (pi.isSaturated() || io_.out.electrical.current_pi_saturation)) return false;
        if (gains.kp == old_kp || !active()) {
            publish(nullptr); // In particular, a no-op or Ki-only change keeps every fractional bit of I.
            return true;
        }
        direction = io_.in.drive_mode;
        point = pi.operatingPoint();
        floor = pi.integralFloor();
        ceiling = pi.integralCeiling();
        if constexpr (rolling_limit) rolling_speed = io_.in.current_speed_rps;
    }
    if constexpr (rolling_limit) {
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
    if (!active() || io_.in.drive_mode != direction ||
        pi.isSaturated() || io_.out.electrical.current_pi_saturation ||
        now.error != point.error || now.output != point.output) return false;
    if (rolling_limit && io_.in.current_speed_rps != rolling_speed) return false;
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
        return io_.out.hall.snapshot();
    }();
    // An interrupt-mask barrier alone does not order pure soft-float/divide
    // helpers. Consume the raw snapshot strictly after PRIMASK restoration.
    __asm__ volatile("" : "+m"(snapshot) : : "memory");
    return HallMonitor::describe(snapshot);
}

HallFault BldcController::getHallFault() const noexcept {
    // The volatile scalar latch is atomic on all supported Cortex-M targets.
    // Callers checking a larger transaction hold their own snapshot mask.
    return io_.out.hall.faultCause();
}

void BldcController::clearHallFault() noexcept {
    CriticalSection cs;
    io_.out.hall.clearFault();
}

bool BldcController::setHallMonitorParams(const HallMonitorParams& params) noexcept {
    HallMonitor::PreparedParameters prepared;
    if (!HallMonitor::prepareParameters(params, prepared)) return false;
    // Materialize the whole bundle before masking, including under LTO: pure
    // coefficient arithmetic must not be sunk into the publication section.
    __asm__ volatile("" : : "m"(prepared) : "memory");
    CriticalSection cs;
    io_.out.hall.applyPrepared(prepared);
    return true;
}

MotorPLL::PllInfo BldcController::getPllInfo() const noexcept {
    auto state = [this] {
        CriticalSection cs;
        return io_.out.pll.nativeInfo();
    }();
    __asm__ volatile("" : "+m"(state) : : "memory");
    return MotorPLL::describe(state);
}

bool BldcController::setPllGains(float kp, float ki) noexcept {
    MotorPLL::PreparedGains prepared;
    if (!MotorPLL::prepareGains(kp, ki, prepared)) return false;
    __asm__ volatile("" : : "m"(prepared) : "memory"); // Keep coefficient calculation before the mask under LTO.
    CriticalSection cs;
    io_.out.pll.applyPrepared(prepared);
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
        io_.out.pll.getNativeBaseGains(p, i);
    }
    kp = numberCast<float>(p);
    ki = numberCast<float>(i);
}

void BldcController::start() noexcept
{
    CriticalSection cs;
    speed_pi_ready_ = false;
    io_.in.is_running = true;
    synchronizeHallPosition(true);
    io_.out.field.seed(io_.out.pll.angle_steps);

    // The inverter follows the *drive mode*, not the run flag.
    //
    // This used to call enable(true) unconditionally. io_.in.drive_mode is NEUTRAL at
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
    if (io_.in.drive_mode == DriveMode::NEUTRAL) {
        pwm_interface_.enable(false);
    } else {
        pwm_interface_.enable(true);
        // After enable(), for the reason given in setDriveMode()
        algorithm().onEnter();
    }

    // Reset target speed filters
    if constexpr (traction_limit) traction_acceleration_.reset();
    filtered_target_speed_ = Speed{};
    io_.in.current_speed_rps = Speed{};
    limited_target_speed_ = Accumulator{};
}

void BldcController::stop() noexcept
{
    CriticalSection cs;
    speed_pi_ready_ = false;
    io_.in.is_running = false;
    io_.in.target_current = Current{};
    algorithm().reset();
    pwm_interface_.updateNativeDutyCycle(Duty{});

    io_.in.current_speed_rps = Speed{};
}

void BldcController::synchronizeHallPosition(bool discard_tracking) noexcept
{
    const uint8_t hall = hall_interface_.getPosition();
    io_.out.measured_position = hall;
    const bool changed = io_.out.pll.getHallStateRaw() != hall;
    if (hall <= 5) io_.out.pll.updateHall(hall);
    // Preserve a real PLL that is already tracking a coasting rotor.
    if (discard_tracking || changed || hall > 5) io_.out.pll.reset();
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
    limited_target_speed_ = numberCast<Accumulator>(io_.in.current_speed_rps);
}

void BldcController::applyAccelerationLimit(Speed target_speed) noexcept
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
        return;
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
    io_.out.hall.tick(guard_active);
    if (hall_update_pending_) {
        hall_update_pending_ = false;
        // Share one decoded sample between the health monitor and the PLL.
        // This alone does not qualify a raw GPIO transition as stable.
        const uint8_t hall_state = position == HALL_NOT_SAMPLED ? hall_interface_.getPosition() : position;
        io_.out.hall.onPosition(hall_state, guard_active);
        if (hall_state <= 5) {
            io_.out.measured_position = hall_state;
            io_.out.pll.updateHall(hall_state);
        } else {
            // Recheck an illegal GPIO code on the next PWM tick.
            // A brief glitch must not look continuously illegal merely because
            // no further filtered Hall edge arrives during the hold timer.
            hall_update_pending_ = true;
        }
    }
}

LIBECU_HOT_LOOP bool BldcController::pwmInterruptHandler() noexcept {
    return adc_interface_ && pwmInterruptHandler(*adc_interface_, pwm_interface_);
}

LIBECU_DEADLINE_CODE("track_rotor") LIBECU_HOT_LOOP
void BldcController::trackRotor(uint8_t position) noexcept {
    const auto& in = io_.in;
    auto& out = io_.out;
    const bool open = in.control_mode == ControlMode::OPEN_LOOP;
    observeHall(!open && in.drive_mode != DriveMode::NEUTRAL && in.is_running, position);
    recordLoopStage(LoopStage::Hall);
    out.pll.updateTick();
    if (open) {
        if (in.is_running) out.field.advance(in.drive_mode, in.field_rate);
        out.electrical.target_position = out.field.step(in.drive_mode, params_.useInverseCommTable);
    } else {
        out.electrical.target_position = out.pll.getNextHall(in.drive_mode);
    }
    recordLoopStage(LoopStage::Pll);
}

} // namespace libecu
