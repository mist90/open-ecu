/**
 * @file motor_pll.cpp
 * @brief Float-facing boundary of the profile-parametric Hall PLL
 */
#include "../include/motor_pll.hpp"

namespace libecu {

MotorPLL::MotorPLL(float max_electrical_speed, bool inverse) noexcept
    : core_(numberCast<Core::Speed>(max_electrical_speed), inverse) {}

void MotorPLL::updateHall(uint8_t hall) noexcept { core_.updateHall(hall); }
void MotorPLL::updateTick() noexcept { core_.updateTick(); }
uint8_t MotorPLL::getNextHall(const volatile DriveMode& mode) noexcept { return core_.getNextHall(mode); }
void MotorPLL::setUsePLL(bool use) noexcept { core_.setUsePLL(use); }
bool MotorPLL::isUsingPLL() const noexcept { return core_.isUsingPLL(); }
void MotorPLL::reset() noexcept { core_.reset(); }
float MotorPLL::getAngle() const noexcept { return numberCast<float>(core_.getAngle()); }
float MotorPLL::getSpeedStepsSec() const noexcept { return numberCast<float>(core_.getSpeedStepsSec()); }

MotorPLL::PllInfo MotorPLL::getInfo() const noexcept {
    const auto state = core_.getInfo();
    return {state.use_pll, state.hall_state_raw, numberCast<float>(state.angle),
            numberCast<float>(state.angle_per_second), numberCast<float>(state.pll_integral),
            numberCast<float>(state.kp), numberCast<float>(state.ki), state.is_sync};
}

bool MotorPLL::setGains(float kp, float ki) noexcept {
    Core::Gain proportional{}, integral{};
    Core::StepGain step{};
    if (!tryNumber(kp, proportional) || !tryNumber(ki, integral) ||
        !tryNumber(numberCast<float>(integral) / ActiveTiming::CURRENT_HZ, step)) return false;
    core_.setGains(proportional, integral);
    return true;
}

void MotorPLL::getBaseGains(float& kp, float& ki) const noexcept {
    const auto state = core_.getInfo();
    kp = numberCast<float>(state.kp);
    ki = numberCast<float>(state.ki);
}

} // namespace libecu
