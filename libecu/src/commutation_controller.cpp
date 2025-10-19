/**
 * @file commutation_controller.cpp
 * @brief Implementation of 6-step commutation algorithm with complementary PWM
 */

#include "../include/algorithms/commutation_controller.hpp"

namespace libecu {

// 6-step commutation table for clockwise rotation
// UP = 50% + delta (high-side dominant), DOWN = 50% - delta (low-side dominant)
const CommutationStep CommutationController::COMMUTATION_TABLE_CW[6] = {
    {PwmState::UP,  PwmState::DOWN, PwmState::OFF}, // Step 1: U+V- (U high, V low)
    {PwmState::UP,  PwmState::OFF,  PwmState::DOWN}, // Step 2: U+W- (U high, W low)  
    {PwmState::OFF, PwmState::UP,   PwmState::DOWN}, // Step 3: V+W- (V high, W low)
    {PwmState::DOWN, PwmState::UP,  PwmState::OFF}, // Step 4: V+U- (V high, U low)
    {PwmState::DOWN, PwmState::OFF, PwmState::UP},  // Step 5: W+U- (W high, U low)
    {PwmState::OFF, PwmState::DOWN, PwmState::UP}   // Step 6: W+V- (W high, V low)
};

// 6-step commutation table for counter-clockwise rotation
const CommutationStep CommutationController::COMMUTATION_TABLE_CCW[6] = {
    {PwmState::UP,  PwmState::OFF,  PwmState::DOWN}, // Step 1: U+W- (U high, W low)
    {PwmState::UP,  PwmState::DOWN, PwmState::OFF}, // Step 2: U+V- (U high, V low)
    {PwmState::OFF, PwmState::DOWN, PwmState::UP},  // Step 3: W+V- (W high, V low)
    {PwmState::DOWN, PwmState::OFF, PwmState::UP},  // Step 4: W+U- (W high, U low)
    {PwmState::DOWN, PwmState::UP,  PwmState::OFF}, // Step 5: V+U- (V high, U low)
    {PwmState::OFF, PwmState::UP,   PwmState::DOWN}  // Step 6: V+W- (V high, W low)
};

CommutationController::CommutationController(PwmInterface& pwm_interface, HallInterface& hall_interface)
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , current_position_(MotorPosition::INVALID)
    , current_step_(0)
    , is_running_(false)
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

bool CommutationController::update(float duty_cycle, RotationDirection direction)
{
    // Read current Hall sensor state
    HallState hall_state = hall_interface_.readState();
    
    // Check if Hall state is valid
    if (!hall_interface_.isValidState(hall_state)) {
        return false;
    }
    
    // Get motor position from Hall state
    current_position_ = hall_interface_.getPosition(hall_state);
    if (current_position_ == MotorPosition::INVALID) {
        return false;
    }
    
    // Get commutation step from position
    current_step_ = getStepFromPosition(current_position_, direction);
    
    // Select appropriate commutation table
    const CommutationStep* table = (direction == RotationDirection::CLOCKWISE) ? 
                                   COMMUTATION_TABLE_CW : COMMUTATION_TABLE_CCW;
    
    // Apply commutation step
    applyCommutationStep(table[current_step_], duty_cycle);
    
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

uint8_t CommutationController::getStepFromPosition(MotorPosition position, RotationDirection direction)
{
    // Convert motor position to commutation step (0-5)
    switch (position) {
        case MotorPosition::POSITION_1: return (direction == RotationDirection::CLOCKWISE) ? 0 : 5;
        case MotorPosition::POSITION_2: return (direction == RotationDirection::CLOCKWISE) ? 1 : 4;
        case MotorPosition::POSITION_3: return (direction == RotationDirection::CLOCKWISE) ? 2 : 3;
        case MotorPosition::POSITION_4: return (direction == RotationDirection::CLOCKWISE) ? 3 : 2;
        case MotorPosition::POSITION_5: return (direction == RotationDirection::CLOCKWISE) ? 4 : 1;
        case MotorPosition::POSITION_6: return (direction == RotationDirection::CLOCKWISE) ? 5 : 0;
        default: return 0;
    }
}

//#include <iostream>
#include <stdio.h>

void CommutationController::applyCommutationStep(const CommutationStep& step, float duty_cycle)
{
    // duty_cycle 0.0-1.0 where:
    // 0.0 = 0V (neutral, delta = 0)
    // 1.0 = maximum voltage (delta = 0.5)
    // For UP state: actual_duty = 0.5 + delta = 0.5 + duty_cycle/2
    // For DOWN state: actual_duty = 0.5 - delta = 0.5 - duty_cycle/2
    
    // Clamp duty_cycle to valid range
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    
    // Calculate delta: duty_cycle 0.0-1.0 maps to delta 0.0-0.5
    float delta = duty_cycle * 0.5f;
    
    // Apply PWM states with delta modulation using new setChannelState method
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, step.phase_u, delta);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, step.phase_v, delta);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, step.phase_w, delta);
    printf("Phase U: %s\n", ((step.phase_u == PwmState::OFF)? "OFF" : (step.phase_u == PwmState::UP)? "UP" : "DOWN"));
    printf("Phase V: %s\n", ((step.phase_v == PwmState::OFF)? "OFF" : (step.phase_v == PwmState::UP)? "UP" : "DOWN"));
    printf("Phase W: %s\n\n", ((step.phase_w == PwmState::OFF)? "OFF" : (step.phase_w == PwmState::UP)? "UP" : "DOWN"));
}

} // namespace libecu
