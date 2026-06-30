/**
 * @file commutation_controller.cpp
 * @brief Implementation of 6-step commutation algorithm with complementary PWM
 */

#include "../include/algorithms/commutation_controller.hpp"

namespace libecu {

// 6-step commutation table
const CommutationStep CommutationController::COMMUTATION_TABLE[6] = {
    {PwmState::UP,  PwmState::DOWN, PwmState::OFF},  // 0
    {PwmState::UP,  PwmState::OFF,  PwmState::DOWN}, // 1
    {PwmState::OFF, PwmState::UP,   PwmState::DOWN}, // 2
    {PwmState::DOWN, PwmState::UP,  PwmState::OFF},  // 3
    {PwmState::DOWN, PwmState::OFF, PwmState::UP},   // 4
    {PwmState::OFF, PwmState::DOWN, PwmState::UP},   // 5
};

CommutationController::CommutationController(PwmInterface& pwm_interface, HallInterface& hall_interface, uint8_t num_poles) noexcept
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , current_step_(0)
    , num_poles_(num_poles)
{
}

uint8_t CommutationController::getCurrentPosition() noexcept
{
    return hall_interface_.getPosition();
}

bool CommutationController::update(uint8_t position, float duty_cycle) noexcept
{
    if (position > 5) {
        return false;
    }

    applyCommutationStep(COMMUTATION_TABLE[position], duty_cycle);

    return true;
}

void CommutationController::applyCommutationStep(const CommutationStep& step, float duty_cycle) noexcept
{
    // Cache phase states for fast access
    cached_phase_u_state_ = step.phase_u;
    cached_phase_v_state_ = step.phase_v;
    cached_phase_w_state_ = step.phase_w;

    // Queue PWM states — changes are preloaded, not applied until apply() is called
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, step.phase_u, duty_cycle);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, step.phase_v, duty_cycle);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, step.phase_w, duty_cycle);

    // Apply all three channel states atomically (TIM_EGR_COMG on STM32)
    pwm_interface_.apply();
}

void CommutationController::updateDutyCycle(float duty_cycle) noexcept
{
    // Update duty cycle without changing phase states
    // Uses cached phase states from last commutation update

    // Apply duty cycle to existing phase states (no state change)
    if (cached_phase_u_state_ == PwmState::UP)
        pwm_interface_.updateDutyCycle(PwmChannel::PHASE_U, duty_cycle);
    else if (cached_phase_v_state_ == PwmState::UP)
        pwm_interface_.updateDutyCycle(PwmChannel::PHASE_V, duty_cycle);
    else
        pwm_interface_.updateDutyCycle(PwmChannel::PHASE_W, duty_cycle);
}

PwmState CommutationController::getPhaseState(PwmChannel channel) const noexcept
{
    switch (channel) {
        case PwmChannel::PHASE_U:
            return cached_phase_u_state_;
        case PwmChannel::PHASE_V:
            return cached_phase_v_state_;
        case PwmChannel::PHASE_W:
            return cached_phase_w_state_;
        default:
            return PwmState::OFF;
    }
}

} // namespace libecu
