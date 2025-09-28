/**
 * @file commutation_controller.cpp
 * @brief Implementation of 6-step commutation algorithm
 */

#include "../include/algorithms/commutation_controller.hpp"

namespace libecu {

// 6-step commutation table for clockwise rotation
const CommutationStep CommutationController::COMMUTATION_TABLE_CW[6] = {
    {PwmState::HIGH_SIDE, PwmState::LOW_SIDE,  PwmState::OFF},       // Step 1: U+V-
    {PwmState::HIGH_SIDE, PwmState::OFF,       PwmState::LOW_SIDE},  // Step 2: U+W-
    {PwmState::OFF,       PwmState::HIGH_SIDE, PwmState::LOW_SIDE},  // Step 3: V+W-
    {PwmState::LOW_SIDE,  PwmState::HIGH_SIDE, PwmState::OFF},       // Step 4: V+U-
    {PwmState::LOW_SIDE,  PwmState::OFF,       PwmState::HIGH_SIDE}, // Step 5: W+U-
    {PwmState::OFF,       PwmState::LOW_SIDE,  PwmState::HIGH_SIDE}  // Step 6: W+V-
};

// 6-step commutation table for counter-clockwise rotation
const CommutationStep CommutationController::COMMUTATION_TABLE_CCW[6] = {
    {PwmState::HIGH_SIDE, PwmState::OFF,       PwmState::LOW_SIDE},  // Step 1: U+W-
    {PwmState::HIGH_SIDE, PwmState::LOW_SIDE,  PwmState::OFF},       // Step 2: U+V-
    {PwmState::OFF,       PwmState::LOW_SIDE,  PwmState::HIGH_SIDE}, // Step 3: W+V-
    {PwmState::LOW_SIDE,  PwmState::OFF,       PwmState::HIGH_SIDE}, // Step 4: W+U-
    {PwmState::LOW_SIDE,  PwmState::HIGH_SIDE, PwmState::OFF},       // Step 5: V+U-
    {PwmState::OFF,       PwmState::HIGH_SIDE, PwmState::LOW_SIDE}   // Step 6: V+W-
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
    if (!pwm_interface_.initialize(pwm_frequency)) {
        return false;
    }
    
    if (!hall_interface_.initialize()) {
        return false;
    }
    
    // Read initial Hall state
    HallState initial_state = hall_interface_.readState();
    current_position_ = hall_interface_.getPosition(initial_state);
    
    return hall_interface_.isValidState(initial_state);
}

bool CommutationController::update(float duty_cycle, RotationDirection direction)
{
    // Read current Hall sensor state
    HallState hall_state = hall_interface_.readState();
    
    // Check if Hall state is valid
    if (!hall_interface_.isValidState(hall_state)) {
        emergencyStop();
        return false;
    }
    
    // Get current motor position
    current_position_ = hall_interface_.getPosition(hall_state);
    
    // Get commutation step
    current_step_ = getStepFromPosition(current_position_, direction);
    
    // Select commutation table based on direction
    const CommutationStep* table = (direction == RotationDirection::CLOCKWISE) ? 
                                   COMMUTATION_TABLE_CW : COMMUTATION_TABLE_CCW;
    
    // Apply commutation step
    applyCommutationStep(table[current_step_], duty_cycle);
    
    is_running_ = (duty_cycle > 0.0f);
    
    return true;
}

void CommutationController::emergencyStop()
{
    pwm_interface_.emergencyStop();
    is_running_ = false;
}

uint8_t CommutationController::getStepFromPosition(MotorPosition position, RotationDirection direction)
{
    // Convert position to step index (0-5)
    uint8_t step = static_cast<uint8_t>(position) - 1;
    
    // Ensure step is in valid range
    if (step >= 6) {
        step = 0;
    }
    
    return step;
}

void CommutationController::applyCommutationStep(const CommutationStep& step, float duty_cycle)
{
    // Set PWM states for each phase
    pwm_interface_.setState(PwmChannel::PHASE_U, step.phase_u);
    pwm_interface_.setState(PwmChannel::PHASE_V, step.phase_v);
    pwm_interface_.setState(PwmChannel::PHASE_W, step.phase_w);
    
    // Set duty cycle for active phases
    if (step.phase_u == PwmState::HIGH_SIDE) {
        pwm_interface_.setDutyCycle(PwmChannel::PHASE_U, duty_cycle);
    }
    if (step.phase_v == PwmState::HIGH_SIDE) {
        pwm_interface_.setDutyCycle(PwmChannel::PHASE_V, duty_cycle);
    }
    if (step.phase_w == PwmState::HIGH_SIDE) {
        pwm_interface_.setDutyCycle(PwmChannel::PHASE_W, duty_cycle);
    }
}

} // namespace libecu