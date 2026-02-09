/**
 * @file commutation_controller.cpp
 * @brief Implementation of 6-step commutation algorithm with complementary PWM
 */

#include "../include/algorithms/commutation_controller.hpp"

namespace libecu {

// 6-step commutation table for counter-clockwise rotation
// UP = high-side active, DOWN = low-side active, OFF = high impedance
const CommutationStep CommutationController::COMMUTATION_TABLE_CCW[6] = {
    {PwmState::OFF, PwmState::DOWN, PwmState::UP},   // 0 -> 5
    {PwmState::UP,  PwmState::DOWN, PwmState::OFF},  // 1 -> 0
    {PwmState::UP,  PwmState::OFF,  PwmState::DOWN}, // 2 -> 1
    {PwmState::OFF, PwmState::UP,   PwmState::DOWN}, // 3 -> 2
    {PwmState::DOWN, PwmState::UP,  PwmState::OFF},  // 4 -> 3
    {PwmState::DOWN, PwmState::OFF, PwmState::UP},   // 5 -> 4
};

// 6-step commutation table for clockwise rotation
const CommutationStep CommutationController::COMMUTATION_TABLE_CW[6] = {
    {PwmState::UP,  PwmState::OFF,  PwmState::DOWN}, // 0 -> 1
    {PwmState::OFF, PwmState::UP,   PwmState::DOWN}, // 1 -> 2
    {PwmState::DOWN, PwmState::UP,  PwmState::OFF},  // 2 -> 3
    {PwmState::DOWN, PwmState::OFF, PwmState::UP},   // 3 -> 4
    {PwmState::OFF, PwmState::DOWN, PwmState::UP},   // 4 -> 5
    {PwmState::UP,  PwmState::DOWN, PwmState::OFF},  // 5 -> 0
};

CommutationController::CommutationController(PwmInterface& pwm_interface, HallInterface& hall_interface, uint8_t num_poles)
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , current_position_(MotorPosition::INVALID)
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
    // Read current Hall sensor state
    HallState hall_state = hall_interface_.readState();
    
    // Check if Hall state is valid
    if (!hall_interface_.isValidState(hall_state)) {
        return 0xFF; // Invalid position
    }
    
    // Get motor position from Hall state
    libecu::MotorPosition pos = hall_interface_.getPosition(hall_state);
    if (pos == MotorPosition::INVALID) {
        return 0xFF; // Invalid position
    }
    
    // Convert MotorPosition to 0-5 range
    // POSITION_1=1 -> 0, POSITION_2=2 -> 1, ..., POSITION_6=6 -> 5
    if (pos >= MotorPosition::POSITION_1 && 
        pos <= MotorPosition::POSITION_6) {
        return static_cast<uint8_t>(pos) - 1;
    }
    
    return 0xFF; // Invalid position
}

bool CommutationController::update(uint8_t position, float duty_cycle, RotationDirection direction)
{
    if (position > 5) {
        return false;
    }
    
    const CommutationStep* table = (direction == RotationDirection::CLOCKWISE) ? 
                                   COMMUTATION_TABLE_CW : COMMUTATION_TABLE_CCW;
    
    applyCommutationStep(table[position], duty_cycle);
    
    is_running_ = true;
    return true;
}

void CommutationController::emergencyStop()
{
    pwm_interface_.emergencyStop();
    is_running_ = false;
    current_step_ = 0;
    current_position_ = MotorPosition::INVALID;
}

void CommutationController::applyCommutationStep(const CommutationStep& step, float duty_cycle)
{
    // duty_cycle 0.0-1.0 where:
    // 0.0 = minimum torque (or maximum negative) or decreasing current (for current mode only)
    // 0.5 = neutral (no net torque/current)
    // 1.0 = maximum torque (or maximum positive) or increasing current
    // For UP state: high-side switches at duty_cycle
    // For DOWN state: low-side switches at duty_cycle
    // For OFF state: both switches disabled (high-Z)

    // Clamp duty_cycle to valid range
    if (duty_cycle < -0.9f) duty_cycle = -0.9f;
    if (duty_cycle > 0.9f) duty_cycle = 0.9f;

    // Convert duty_cycle to 0.5-1.0 range around neutral point
    // duty_cycle==0.5 is neutral (no net voltage/current)
    // >0.5 - positive torque/current
    // <0.5 - negative torque/current
    duty_cycle = 0.5f + duty_cycle * 0.5f;

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

    // Clamp duty_cycle to valid range
    if (duty_cycle < -0.9f) duty_cycle = -0.9f;
    if (duty_cycle > 0.9f) duty_cycle = 0.9f;

    // Convert duty_cycle to 0.5-1.0 range around neutral point
    duty_cycle = 0.5f + duty_cycle * 0.5f;

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
