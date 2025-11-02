/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include <algorithm>
#include <stdio.h>

// Speed measurement configuration
#define SPEED_WINDOW_MIN_MS       10    ///< Minimum measurement window (ms)
#define SPEED_WINDOW_MAX_MS       5000  ///< Maximum measurement window (ms)
#define SPEED_MIN_STEPS           4     ///< Minimum steps required for valid speed measurement
#define SPEED_STEPS_PER_REVOLUTION 24   ///< Total steps per mechanical revolution (6 * num_poles / 2)

extern "C" {
uint32_t HAL_GetTick(void);
}

namespace libecu {

BldcController::BldcController(
    PwmInterface& pwm_interface,
    HallInterface& hall_interface,
    CommutationController& commutation_controller,
    PidController& pid_controller,
    SafetyMonitor& safety_monitor,
    const MotorControlParams& params)
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , commutation_controller_(commutation_controller)
    , pid_controller_(pid_controller)
    , safety_monitor_(safety_monitor)
    , params_(params)
    , direction_(RotationDirection::CLOCKWISE)
    , initialized_(false)
    , speed_first_position_(0)
    , speed_last_position_(0)
    , speed_first_time_ms_(0)
    , speed_last_time_ms_(0)
    , speed_step_count_(0)
    , speed_window_min_ms_(SPEED_WINDOW_MIN_MS)
    , speed_measurement_active_(false)
{
    // Initialize status
    status_.current_speed_rpm = 0.0f;
    status_.target_speed_rpm = 0.0f;
    status_.duty_cycle = 0.0f;
    status_.position = MotorPosition::INVALID;
    status_.active_fault = SafetyFault::NONE;
    status_.is_running = false;
    status_.mode = ControlMode::OPEN_LOOP;
}

bool BldcController::initialize()
{
    // Initialize commutation controller (includes PWM and Hall sensor)
    if (!commutation_controller_.initialize()) {
        return false;
    }
    
    // Reset PID controller
    pid_controller_.reset();
    
    // Reset safety monitor
    safety_monitor_.resetFaultCounters();
    
    initialized_ = true;
    return true;
}

void BldcController::update(const SafetyData& safety_data)
{
    if (!initialized_) {
        return;
    }
    
    // Update safety monitoring
    SafetyFault fault = safety_monitor_.update(safety_data);
    status_.active_fault = fault;
    
    // Handle safety faults
    if (fault != SafetyFault::NONE) {
        handleSafetyFault(fault);
        return;
    }
    
    // Calculate current motor speed
    status_.current_speed_rpm = calculateSpeed();
    
    // Update position status (convert from internal current_position_)
    // This is updated by calculateSpeed() internally
    if (speed_measurement_active_) {
        // Convert 0-5 range to MotorPosition enum
        if (speed_last_position_ <= 5) {
            status_.position = static_cast<MotorPosition>(speed_last_position_ + 1);
        } else {
            status_.position = MotorPosition::INVALID;
        }
    }
    
    // Control loop based on mode
    float target_duty_cycle = 0.0f;
    
    if (status_.is_running) {
        switch (status_.mode) {
            case ControlMode::OPEN_LOOP:
                target_duty_cycle = status_.duty_cycle;
                break;
                
            case ControlMode::CLOSED_LOOP:
                // Apply acceleration limiting
                float dt = 1.0f / params_.control_frequency;
                float limited_target = applyAccelerationLimit(
                    status_.target_speed_rpm, 
                    status_.current_speed_rpm, 
                    dt
                );
                
                // PID control
                target_duty_cycle = pid_controller_.update(
                    limited_target, 
                    status_.current_speed_rpm, 
                    dt
                );
                
                // Clamp to maximum duty cycle
                target_duty_cycle = std::min(target_duty_cycle, params_.max_duty_cycle);
                break;
        }
    }
    
    // Update commutation based on control mode
    status_.duty_cycle = target_duty_cycle;
    
    if (status_.mode == ControlMode::OPEN_LOOP) {
        // Use target speed for open-loop control
        commutation_controller_.updateOpenLoop(target_duty_cycle, status_.target_speed_rpm, direction_);
    } else {
        // Use Hall sensor feedback for closed-loop control
        commutation_controller_.update(target_duty_cycle, direction_);
    }
}

void BldcController::setTargetSpeed(float speed_rpm)
{
    // Clamp to maximum speed
    status_.target_speed_rpm = std::min(std::abs(speed_rpm), params_.max_speed_rpm);
    
    // Set direction based on sign
    direction_ = (speed_rpm >= 0.0f) ? RotationDirection::CLOCKWISE : RotationDirection::COUNTER_CLOCKWISE;
}

void BldcController::setDutyCycle(float duty_cycle)
{
    status_.duty_cycle = std::max(0.0f, std::min(duty_cycle, params_.max_duty_cycle));
}

void BldcController::setControlMode(ControlMode mode)
{
    if (status_.mode != mode) {
        status_.mode = mode;
        
        // Reset PID when switching to closed loop
        if (mode == ControlMode::CLOSED_LOOP) {
            pid_controller_.reset();
        }
    }
}

void BldcController::setDirection(RotationDirection direction)
{
    direction_ = direction;
}

void BldcController::start()
{
    if (status_.active_fault == SafetyFault::NONE) {
        status_.is_running = true;
        pwm_interface_.enable(true);
        
        // Reset speed measurement on motor start
        speed_measurement_active_ = false;
        speed_step_count_ = 0;
        speed_window_min_ms_ = SPEED_WINDOW_MIN_MS;
    }
}

void BldcController::stop()
{
    status_.is_running = false;
    status_.duty_cycle = 0.0f;
    commutation_controller_.update(0.0f, direction_);
    
    // Reset speed measurement on motor stop
    speed_measurement_active_ = false;
    speed_step_count_ = 0;
    status_.current_speed_rpm = 0.0f;
}

void BldcController::emergencyStop()
{
    status_.is_running = false;
    status_.duty_cycle = 0.0f;
    commutation_controller_.emergencyStop();
    safety_monitor_.setEmergencyStop(true);
}

MotorStatus BldcController::getStatus() const
{
    return status_;
}

void BldcController::clearFault(SafetyFault fault)
{
    safety_monitor_.clearFault(fault);
    
    // Clear emergency stop if that was the fault
    if (fault == SafetyFault::EMERGENCY_STOP) {
        safety_monitor_.setEmergencyStop(false);
    }
}

float BldcController::calculateSpeed()
{
    /**
     * Adaptive speed measurement algorithm for BLDC motors
     * 
     * Algorithm overview:
     * 1. Track rotor position changes over time (0..5 cyclic from Hall sensors)
     * 2. Use adaptive time window to handle wide speed range (slow to fast)
     * 3. Accumulate steps in current direction
     * 4. When window reaches minimum duration AND minimum steps accumulated:
     *    - Calculate speed from steps/time
     *    - Reset window to minimum
     * 5. If not enough steps in minimum window:
     *    - Double the window size (up to maximum)
     * 6. If maximum window reached with insufficient steps:
     *    - Motor is stopped (speed = 0)
     * 7. Handle direction reversals:
     *    - Subtract reversed steps from accumulator
     *    - Reduce window size
     *    - If reversed past starting point, flip measurement direction
     */
    
    // Get current position (0..5 cyclic) from CommutationController
    uint8_t current_position = commutation_controller_.getCurrentPosition();
    
    // Check if position is valid
    if (current_position == 0xFF) {
        // Invalid position, return last known speed
        return status_.current_speed_rpm;
    }
    
    uint32_t current_time_ms = HAL_GetTick();
    
    // Initialize measurement on first call
    if (!speed_measurement_active_) {
        speed_first_position_ = current_position;
        speed_last_position_ = current_position;
        speed_first_time_ms_ = current_time_ms;
        speed_last_time_ms_ = current_time_ms;
        speed_step_count_ = 0;
        speed_window_min_ms_ = SPEED_WINDOW_MIN_MS;
        speed_measurement_active_ = true;
        return 0.0f;
    }
    
    // Check if position has changed
    if (current_position != speed_last_position_) {
        // Calculate step delta (handling cyclic 0..5 wrap-around)
        int8_t delta = 0;
        int8_t diff_forward = (current_position - speed_last_position_ + 6) % 6;
        int8_t diff_backward = (speed_last_position_ - current_position + 6) % 6;
        
        // Determine direction of movement (choose shorter path)
        if (diff_forward <= diff_backward) {
            delta = diff_forward;  // Forward movement
        } else {
            delta = -diff_backward; // Backward movement
        }
        
        // Check for direction reversal
        if ((speed_step_count_ > 0 && delta < 0) || (speed_step_count_ < 0 && delta > 0)) {
            // Direction reversed: subtract accumulated steps
            speed_step_count_ += delta;
            
            // Reduce window (halve it, but keep at minimum)
            speed_window_min_ms_ = (speed_window_min_ms_ > SPEED_WINDOW_MIN_MS * 2) ? 
                                    speed_window_min_ms_ / 2 : SPEED_WINDOW_MIN_MS;
            
            // Check if we've gone backwards past the start
            if ((speed_step_count_ > 0 && delta < 0 && 
                 (current_time_ms < speed_first_time_ms_ || 
                  current_position == speed_first_position_)) ||
                (speed_step_count_ < 0 && delta > 0 && 
                 (current_time_ms < speed_first_time_ms_ || 
                  current_position == speed_first_position_))) {
                // Last step went before first step - reverse measurement direction
                speed_first_position_ = current_position;
                speed_first_time_ms_ = current_time_ms;
                speed_step_count_ = 0;
                speed_window_min_ms_ = SPEED_WINDOW_MIN_MS;
            }
        } else {
            // Same direction: accumulate steps
            speed_step_count_ += delta;
        }
        
        // Update last position and time
        speed_last_position_ = current_position;
        speed_last_time_ms_ = current_time_ms;
    }
    
    // Calculate elapsed time since first measurement
    uint32_t elapsed_time_ms = current_time_ms - speed_first_time_ms_;
    
    // Check if we have enough data to calculate speed
    if (elapsed_time_ms >= speed_window_min_ms_) {
        // Check if we have enough steps
        int32_t abs_steps = (speed_step_count_ >= 0) ? speed_step_count_ : -speed_step_count_;
        
        if (abs_steps >= SPEED_MIN_STEPS) {
            // Calculate speed in RPM
            // speed_step_count_ steps in elapsed_time_ms milliseconds
            // Each full revolution = SPEED_STEPS_PER_REVOLUTION steps
            // RPM = (steps / elapsed_time_ms) * (1000 ms/s) * (60 s/min) / SPEED_STEPS_PER_REVOLUTION
            
            float speed_rpm = (static_cast<float>(speed_step_count_) * 60000.0f) / 
                             (static_cast<float>(elapsed_time_ms) * SPEED_STEPS_PER_REVOLUTION);
            
            // Reset window to minimum for next measurement
            speed_window_min_ms_ = SPEED_WINDOW_MIN_MS;
            
            // Reset measurement window
            speed_first_position_ = speed_last_position_;
            speed_first_time_ms_ = speed_last_time_ms_;
            speed_step_count_ = 0;
            
            return speed_rpm;
        } else {
            // Not enough steps - double the window
            if (speed_window_min_ms_ < SPEED_WINDOW_MAX_MS) {
                speed_window_min_ms_ = speed_window_min_ms_ * 2;
                if (speed_window_min_ms_ > SPEED_WINDOW_MAX_MS) {
                    speed_window_min_ms_ = SPEED_WINDOW_MAX_MS;
                }
            } else {
                // Maximum window reached and still not enough steps - motor is stopped
                speed_first_position_ = speed_last_position_;
                speed_first_time_ms_ = speed_last_time_ms_;
                speed_step_count_ = 0;
                speed_window_min_ms_ = SPEED_WINDOW_MIN_MS;
                return 0.0f;
            }
        }
    }
    
    // Not enough time elapsed yet, return last known speed
    return status_.current_speed_rpm;
}

float BldcController::applyAccelerationLimit(float target_speed, float current_speed, float dt)
{
    float speed_diff = target_speed - current_speed;
    float max_change = params_.acceleration_rate * dt;
    
    if (std::abs(speed_diff) <= max_change) {
        return target_speed;
    }
    
    if (speed_diff > 0.0f) {
        return current_speed + max_change;
    } else {
        return current_speed - max_change;
    }
}

void BldcController::handleSafetyFault(SafetyFault fault)
{
    // Emergency stop for critical faults
    switch (fault) {
        case SafetyFault::OVERCURRENT:
        case SafetyFault::OVERTEMPERATURE:
        case SafetyFault::EMERGENCY_STOP:
            emergencyStop();
            break;
            
        case SafetyFault::UNDERVOLTAGE:
        case SafetyFault::OVERVOLTAGE:
        case SafetyFault::HALL_SENSOR_FAULT:
            // Stop motor but allow restart after fault clears
            stop();
            break;
            
        default:
            break;
    }
}

} // namespace libecu
