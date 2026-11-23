/**
 * @file six_step_algorithm.cpp
 * @brief Implementation of trapezoidal six-step commutation
 */

#include "../include/six_step_algorithm.hpp"

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
    , current_step_(0xFF)
    , cycles_since_commutation_(255)
    , last_duty_{}
    , last_saturation_(0)
{
}

void SixStepAlgorithm::setParams(const SixStepParams& params) noexcept
{
    max_duty_ = numberCast<Duty>(params.max_duty_cycle);
    blanking_cycles_ = params.blanking_cycles;
}

bool SixStepAlgorithm::setCurrentPi(const PiParameters& params) noexcept
{
    const auto native = CurrentPi::prepareTuning(params);
    if (!native) return false;
    resetCurrentPi(params, *native);
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

SixStepAlgorithm::Output SixStepAlgorithm::updateNative(const Input& sample) noexcept
{
    MotorInput in{};
    MotorElectricalOutput out{};
    sample.unpack(in, out);
    const bool commutated = updateElectrical(in, out, sample.angle_steps);
    pwm_interface_.updateNativeDutyCycle(out.duty_cycle);
    if (commutated) pwm_interface_.apply();
    return Output::fromCarrier(out);
}

void SixStepAlgorithm::updateCarrier(const MotorInput& in, MotorElectricalOutput& out,
                                     Profile::Angle angle_steps) noexcept
{
    const bool commutated = updateElectrical(in, out, angle_steps);
    pwm_interface_.writeCarrierDuty(out.duty_cycle, out.duty_cycle, out.duty_cycle);
    if (commutated) pwm_interface_.applyCarrierStates();
}

LIBECU_HOT_LOOP bool SixStepAlgorithm::updateElectrical(
        const MotorInput& in, MotorElectricalOutput& out, Profile::Angle angle_steps) noexcept {
    // Read the shunt *before* this cycle's commutation: the sample belongs to
    // the period that just ended, under the previously applied row.
    const Current measured_current = currentFromActivePhase(out);

    Duty duty;
    int8_t saturation;

    if (cycles_since_commutation_ < blanking_cycles_) {
        // Hold the last duty and leave the PI untouched, so neither the
        // proportional term nor the integrator sees the freewheeling sample.
        duty = last_duty_;
        saturation = last_saturation_;
    } else {
        using PiCurrent = ActivePiProfile::CurrentPiTypes::Input;
        const Duty duty_raw = numberCast<Duty>(current_pi_.updateNative(
            numberCast<PiCurrent>(in.target_current), numberCast<PiCurrent>(measured_current)));
        duty = numericClamp(duty_raw, Duty{}, max_duty_);

        // The bridge's duty ceiling may be below the PI's own upper rail.
        // Report either clamp to the speed loop for anti-windup.
        saturation = static_cast<int8_t>(current_pi_.saturationSign());
        if (saturation == 0) {
            saturation = (duty_raw > max_duty_) ? 1 : (duty_raw < Duty{}) ? -1 : 0;
        }
    }

    last_duty_ = duty;
    last_saturation_ = saturation;

    out.duty_u = duty;
    out.duty_v = duty;
    out.duty_w = duty;
    out.duty_cycle = duty;
    out.measured_current = measured_current;
    out.i_d = Current{};
    out.i_q = Current{};
    out.v_d = Profile::Voltage{};
    out.v_q = multiplyTo<Profile::Voltage>(duty, out.bus_voltage);
    out.angle_e_rad = multiplyTo<Profile::Angle>(angle_steps,
        number_v<Profile::Angle, 10471975511966LL, 10000000000000LL>);
    out.current_pi_saturation = saturation;

    const uint8_t step = out.target_position;
    if (step <= 5 && step != current_step_) {
        current_step_ = step;
        cycles_since_commutation_ = 0;
        queueCommutationStep(COMMUTATION_TABLE[step]);
        return true;
    }
    if (cycles_since_commutation_ < 255) ++cycles_since_commutation_;
    return false;
}

void SixStepAlgorithm::queueCommutationStep(const CommutationStep& step) noexcept
{
    // Queued, not applied: CCPC preloads OCxM/CCxE/CCxNE until the COM event.
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, step.phase_u);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, step.phase_v);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, step.phase_w);
}

SixStepAlgorithm::Current SixStepAlgorithm::currentFromActivePhase(const MotorElectricalOutput& sampled) const noexcept
{
    // After park or FOC there is no preceding six-step DOWN phase.
    if (current_step_ >= 6) return {};
    const auto& step = COMMUTATION_TABLE[current_step_];
    if (step.phase_u == PwmState::DOWN) return sampled.i_u;
    if (step.phase_v == PwmState::DOWN) return sampled.i_v;
    return sampled.i_w; // Every commutation row has exactly one DOWN phase.
}

PwmState SixStepAlgorithm::getPhaseState(PwmChannel channel) const noexcept
{
    if (current_step_ >= 6) return PwmState::OFF;
    const auto& step = COMMUTATION_TABLE[current_step_];
    switch (channel) {
        case PwmChannel::PHASE_U: return step.phase_u;
        case PwmChannel::PHASE_V: return step.phase_v;
        case PwmChannel::PHASE_W: return step.phase_w;
        default:                  return PwmState::OFF;
    }
}

} // namespace libecu
