/**
 * @file six_step_algorithm.cpp
 * @brief Implementation of trapezoidal six-step commutation
 */

#include "../include/six_step_algorithm.hpp"
#include "../include/foc_math.hpp"

#include <algorithm>

namespace libecu {

// Six-step commutation table. Row k energises one phase high and one low; the
// resulting stator field sits at (k*60 - 30) electrical degrees, which is where
// the FOC angle offset in foc_algorithm.hpp is derived from.
const CommutationStep SixStepAlgorithm::COMMUTATION_TABLE[6] = {
    {PwmState::UP,   PwmState::DOWN, PwmState::OFF},  // 0
    {PwmState::UP,   PwmState::OFF,  PwmState::DOWN}, // 1
    {PwmState::OFF,  PwmState::UP,   PwmState::DOWN}, // 2
    {PwmState::DOWN, PwmState::UP,   PwmState::OFF},  // 3
    {PwmState::DOWN, PwmState::OFF,  PwmState::UP},   // 4
    {PwmState::OFF,  PwmState::DOWN, PwmState::UP},   // 5
};

SixStepAlgorithm::SixStepAlgorithm(PwmInterface& pwm_interface) noexcept
    : pwm_interface_(pwm_interface)
    , current_pi_()
    , params_{0.95f, 6}
    , current_step_(0xFF)
    , cached_phase_u_state_(PwmState::OFF)
    , cached_phase_v_state_(PwmState::OFF)
    , cached_phase_w_state_(PwmState::OFF)
    , cycles_since_commutation_(255)
    , last_duty_{}
    , last_saturation_(0)
{
}

void SixStepAlgorithm::setParams(const SixStepParams& params) noexcept
{
    params_ = params;
    max_duty_ = numberCast<Duty>(params.max_duty_cycle);
}

bool SixStepAlgorithm::setCurrentPi(const PiParameters& params) noexcept
{
    if (!current_pi_.setParameters(params)) return false;
    current_pi_.reset();
    return true;
}

const PiParameters& SixStepAlgorithm::getCurrentPi() const noexcept
{
    return current_pi_.getParameters();
}

void SixStepAlgorithm::reset() noexcept
{
    current_pi_.reset();
    current_step_ = 0xFF;
    cycles_since_commutation_ = 255;
    last_duty_ = Duty{};
    last_saturation_ = 0;
}

void SixStepAlgorithm::onEnter() noexcept
{
    // Forget the applied step so the next update() re-commutates unconditionally.
    // Coming from FOC the bridge has all three phases UP, which is not a row of
    // the table and must not be left in place.
    reset();
}

MotorAlgorithmOutput SixStepAlgorithm::update(const MotorAlgorithmInput& in) noexcept
{
    return motorOutputCast<MotorAlgorithmOutput>(updateNative(motorInputCast<Input>(in)));
}

SixStepAlgorithm::Output SixStepAlgorithm::updateNative(const Input& in) noexcept
{
    Output out{};

    // Read the shunt *before* this cycle's commutation: the sample belongs to
    // the period that just ended, under the switching states still cached.
    const Current measured_current = currentFromActivePhase(in);

    const bool commutating = (in.step <= 5) && (in.step != current_step_);

    Duty duty;
    int8_t saturation;

    if (in.electric_mode == ElectricMode::VOLTAGE_MODE) {
        duty = numericClamp(in.duty_command, Duty{}, max_duty_);
        saturation = 0;
    } else {
        const bool blanked = params_.blanking_cycles > 0 &&
                             cycles_since_commutation_ < params_.blanking_cycles;
        if (blanked) {
            // Hold the last duty and leave the PI untouched, so neither the
            // proportional term nor the integrator sees the freewheeling sample.
            duty = last_duty_;
            saturation = last_saturation_;
        } else {
            // The PI gains come from the motor's electrical model (see
            // current_loop_tuning.hpp), so the loop bandwidth is set by physics
            // rather than by hand-tuning.
            using PiCurrent = ActivePiProfile::CurrentPiTypes::Input;
            const Duty duty_raw = numberCast<Duty>(current_pi_.updateNative(
                numberCast<PiCurrent>(in.target_current), numberCast<PiCurrent>(measured_current)));
            duty = numericClamp(duty_raw, Duty{}, max_duty_);

            // Saturation is reported from the *clamped* duty, not from the
            // regulator's own limits: max_duty_cycle is applied here, outside
            // the PI, so the PI reports itself unsaturated while the bridge is
            // in fact wide open. The sign tells the speed loop which way it is
            // stuck; see PiController.
            saturation = static_cast<int8_t>(current_pi_.saturationSign());
            if (saturation == 0) {
                saturation = (duty_raw > max_duty_) ? 1 : (duty_raw < Duty{}) ? -1 : 0;
            }
        }
    }

    if (cycles_since_commutation_ < 255) {
        cycles_since_commutation_++;
    }

    if (commutating) {
        current_step_ = in.step;
        cycles_since_commutation_ = 0;
        applyCommutationStep(COMMUTATION_TABLE[in.step], duty);
    } else {
        // Modulation depth only. All three phases carry the same duty; the
        // switching state set at the last commutation decides which of them
        // actually modulates it.
        pwm_interface_.updateNativeDutyCycle(duty);
    }

    last_duty_ = duty;
    last_saturation_ = saturation;

    out.duty_u = duty;
    out.duty_v = duty;
    out.duty_w = duty;
    out.duty = duty;
    out.measured_current = measured_current;
    out.v_q = multiplyTo<Profile::Voltage>(duty, in.bus_voltage);
    out.angle_e_rad = multiplyTo<Profile::Angle>(in.angle_steps,
        number_v<Profile::Angle, 10471975511966LL, 10000000000000LL>);
    out.saturation = saturation;
    return out;
}

void SixStepAlgorithm::applyCommutationStep(const CommutationStep& step, Duty duty_cycle) noexcept
{
    cached_phase_u_state_ = step.phase_u;
    cached_phase_v_state_ = step.phase_v;
    cached_phase_w_state_ = step.phase_w;

    // Queued, not applied: CCPC preloads OCxM/CCxE/CCxNE until the COM event.
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, step.phase_u);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, step.phase_v);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, step.phase_w);

    // Duty travels on its own path: preloaded against the timer's update event
    // while the switching states above are preloaded against the COM event
    // below. Both are atomic within themselves, and neither has to wait for the
    // other, because no phase state is encoded in a compare register.
    pwm_interface_.updateNativeDutyCycle(duty_cycle);

    // Apply all three channel states atomically (TIM_EGR_COMG on STM32)
    pwm_interface_.apply();
}

SixStepAlgorithm::Current SixStepAlgorithm::currentFromActivePhase(const Input& in) const noexcept
{
    if (cached_phase_u_state_ == PwmState::DOWN) {
        return in.i_u;
    } else if (cached_phase_v_state_ == PwmState::DOWN) {
        return in.i_v;
    } else if (cached_phase_w_state_ == PwmState::DOWN) {
        return in.i_w;
    }
    // No DOWN phase (all OFF, or FOC left the bridge in all-UP): nothing on a
    // shunt that means anything.
    return Current{};
}

PwmState SixStepAlgorithm::getPhaseState(PwmChannel channel) const noexcept
{
    switch (channel) {
        case PwmChannel::PHASE_U: return cached_phase_u_state_;
        case PwmChannel::PHASE_V: return cached_phase_v_state_;
        case PwmChannel::PHASE_W: return cached_phase_w_state_;
        default:                  return PwmState::OFF;
    }
}

} // namespace libecu
