/**
 * @file commutation_controller.cpp
 * @brief Implementation of 6-step commutation algorithm with complementary PWM
 */

#include "../include/algorithms/commutation_controller.hpp"

extern "C" {
uint32_t HAL_GetTick(void);
}

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

CommutationController::CommutationController(PwmInterface& pwm_interface, HallInterface& hall_interface, uint8_t num_poles)
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , current_position_(MotorPosition::INVALID)
    , current_step_(0)
    , is_running_(false)
    , num_poles_(num_poles)
    , last_step_time_us_(0)
    , step_interval_us_(0)
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

bool CommutationController::update(float duty_cycle, RotationDirection direction)
{
    // Get current position (0-5 range)
    uint8_t position = getCurrentPosition();
    
    // Check if position is valid
    if (position == 0xFF) {
        return false;
    }
    
    // Select appropriate commutation table
    const CommutationStep* table = (direction == RotationDirection::CLOCKWISE) ? 
                                   COMMUTATION_TABLE_CW : COMMUTATION_TABLE_CCW;
    
    // Apply commutation step
    applyCommutationStep(table[position], duty_cycle);
    
    is_running_ = true;
    return true;
}

bool CommutationController::updateOpenLoop(float duty_cycle, float target_speed_rpm, RotationDirection direction)
{
    // If speed is zero, stop the motor
    if (target_speed_rpm <= 0.0f) {
        // Set all phases to OFF
        pwm_interface_.setState(PwmChannel::PHASE_U, PwmState::OFF);
        pwm_interface_.setState(PwmChannel::PHASE_V, PwmState::OFF);
        pwm_interface_.setState(PwmChannel::PHASE_W, PwmState::OFF);
        is_running_ = false;
        return true;
    }
    
    // Calculate step interval based on target speed
    step_interval_us_ = calculateStepInterval(target_speed_rpm);
    
    // Get current time
    uint32_t current_time_us = getCurrentTimeUs();
    
    // Check if it's time to advance to the next step
    if (is_running_ && (current_time_us - last_step_time_us_) >= step_interval_us_) {
        // Increment step (0-5, wrap around)
        if (direction == RotationDirection::CLOCKWISE) {
            current_step_ = (current_step_ + 1) % 6;
        } else {
            current_step_ = (current_step_ == 0) ? 5 : current_step_ - 1;
        }
        last_step_time_us_ = current_time_us;
    } else if (!is_running_) {
        // Initialize timing on first run
        last_step_time_us_ = current_time_us;
        is_running_ = true;
    }

    // Select appropriate commutation table
    const CommutationStep* table = (direction == RotationDirection::CLOCKWISE) ? 
                                   COMMUTATION_TABLE_CW : COMMUTATION_TABLE_CCW;
    
    // Apply commutation step
    applyCommutationStep(table[current_step_], duty_cycle);
    
    return true;
}

void CommutationController::emergencyStop()
{
    pwm_interface_.emergencyStop();
    is_running_ = false;
    current_step_ = 0;
    current_position_ = MotorPosition::INVALID;
    last_step_time_us_ = 0;
    step_interval_us_ = 0;
}

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
}

uint32_t CommutationController::calculateStepInterval(float speed_rpm)
{
    // Calculate step interval in microseconds
    // Formula: step_interval_us = 1,000,000 / (speed_rpm * 3 * num_poles)
    // where 3 = number of phases, num_poles = number of pole pairs
    if (speed_rpm <= 0.0f) {
        return 0;
    }
    
    return static_cast<uint32_t>(1000000.0f / (speed_rpm * 3.0f * num_poles_));
}

uint32_t CommutationController::getCurrentTimeUs()
{
    // Use HAL_GetTick() which returns milliseconds, convert to microseconds
    // Note: This provides 1ms resolution which is adequate for motor control
    // For higher precision, a dedicated timer would be used in production
    return HAL_GetTick() * 1000;
}

} // namespace libecu
