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

CommutationController::CommutationController(PwmInterface& pwm_interface, HallInterface& hall_interface, uint8_t num_poles)
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , current_step_(0)
    , is_running_(false)
    , num_poles_(num_poles)
{
}

bool CommutationController::initialize(uint32_t pwm_frequency)
{
    // Initialize PWM with dead-time (100ns is typical for power MOSFETs)
    if (!pwm_interface_.initialize(pwm_frequency, 100)) {
        return false;
    }
    
    // Initialize all phases to OFF state
    pwm_interface_.setState(PwmChannel::PHASE_U, PwmState::OFF);
    pwm_interface_.setState(PwmChannel::PHASE_V, PwmState::OFF);
    pwm_interface_.setState(PwmChannel::PHASE_W, PwmState::OFF);
    
    // Set all phases to neutral (50% duty cycle)
    pwm_interface_.setNeutral();
    
    return true;
}

uint8_t CommutationController::getCurrentPosition()
{
    return hall_interface_.getPosition();
}

bool CommutationController::update(uint8_t position, float duty_cycle)
{
    if (position > 5) {
        return false;
    }
    
    applyCommutationStep(COMMUTATION_TABLE[position], duty_cycle);
    
    is_running_ = true;
    return true;
}

void CommutationController::emergencyStop()
{
    pwm_interface_.emergencyStop();
    is_running_ = false;
    current_step_ = 0;
}

void CommutationController::applyCommutationStep(const CommutationStep& step, float duty_cycle)
{
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 0.9f;

    // Cache phase states for fast access
    cached_phase_u_state_ = step.phase_u;
    cached_phase_v_state_ = step.phase_v;
    cached_phase_w_state_ = step.phase_w;

    // Apply PWM states with direct duty_cycle control
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, step.phase_u, duty_cycle);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, step.phase_v, duty_cycle);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, step.phase_w, duty_cycle);
}

void CommutationController::updateDutyCycle(float duty_cycle)
{
    // Update duty cycle without changing phase states
    // Uses cached phase states from last commutation update

    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 0.9f;

    // Apply duty cycle to existing phase states (no state change)
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, cached_phase_u_state_, duty_cycle);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, cached_phase_v_state_, duty_cycle);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, cached_phase_w_state_, duty_cycle);
}

PwmState CommutationController::getCachedPhaseState(PwmChannel channel) const
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
