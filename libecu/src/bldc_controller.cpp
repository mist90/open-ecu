/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include <algorithm>
#include <stdio.h>

// Speed measurement configuration
#define SPEED_WINDOW_MIN_US       10000    ///< Minimum measurement window (ms)
#define SPEED_WINDOW_MAX_US       5000000  ///< Maximum measurement window (ms)
#define SPEED_MIN_STEPS           10     ///< Minimum steps required for valid speed measurement
#define SPEED_STEPS_PER_REVOLUTION 24   ///< Total steps per mechanical revolution (6 * num_poles / 2)

uint32_t time_us(void);

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
    , hall_data_head_(0)
    , hall_data_tail_(0)
    , hall_data_overflow_(false)
    , speed_first_position_(0)
    , speed_last_position_(0)
    , speed_first_time_us_(0)
    , speed_last_time_us_(0)
    , speed_step_count_(0)
    , speed_window_min_us_(SPEED_WINDOW_MIN_US)
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

    //printf("p:%d,v:%f\n", (int)speed_last_position_, status_.current_speed_rpm);
    
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
                if (target_duty_cycle < 0.0f)
                    target_duty_cycle = 0.0f;
                printf("s:%f,d:%f\n", status_.current_speed_rpm, target_duty_cycle);
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
        speed_window_min_us_ = SPEED_WINDOW_MIN_US;
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
     * Interrupt-driven speed measurement algorithm for BLDC motors
     * 
     * Algorithm overview:
     * 1. Use Hall sensor data captured by interrupt handlers
     * 2. Process multiple data points from circular buffer for accurate measurement
     * 3. Calculate speed based on time intervals between Hall state changes
     * 4. Handle direction changes and invalid states
     * 5. Use adaptive algorithm to handle various speed ranges
     */
    
    uint32_t current_time_us = time_us();
    
    // Check if we have any new Hall sensor data
    if (hall_data_head_ == hall_data_tail_) {
        // No new data available
        // Check if too much time has passed since last data - motor might be stopped
        if (speed_measurement_active_ && 
            (current_time_us - speed_last_time_us_ > SPEED_WINDOW_MAX_US)) {
            // Too long since last Hall transition - motor is stopped
            speed_measurement_active_ = false;
            return 0.0f;
        }
        return status_.current_speed_rpm; // Return last known speed
    }
    
    // Reset overflow flag (we're processing the data now)
    hall_data_overflow_ = false;
    
    // Count available data points
    size_t data_count = 0;
    if (hall_data_head_ >= hall_data_tail_) {
        data_count = hall_data_head_ - hall_data_tail_;
    } else {
        data_count = MAX_HALL_DATA_POINTS - hall_data_tail_ + hall_data_head_;
    }
    
    // Need at least 2 data points to calculate speed
    if (data_count < 2) {
        return status_.current_speed_rpm;
    }
    
    // Process Hall sensor data points to calculate speed
    int32_t total_steps = 0;
    uint32_t first_timestamp = 0;
    uint32_t last_timestamp = 0;
    uint8_t first_state = 0xFF;
    uint8_t last_state = 0xFF;
    size_t valid_transitions = 0;
    
    // Process all available data points
    size_t current_idx = hall_data_tail_;
    bool first_point = true;
    uint8_t prev_state = 0xFF;
    
    while (current_idx != hall_data_head_) {
        const HallInterruptData& data = hall_data_[current_idx];
        
        // Skip invalid states
        if (data.hall_state == 0xFF || data.hall_state > 5) {
            current_idx = (current_idx + 1) % MAX_HALL_DATA_POINTS;
            continue;
        }
        
        if (first_point) {
            first_timestamp = data.timestamp_us;
            first_state = data.hall_state;
            prev_state = data.hall_state;
            first_point = false;
        } else {
            // Calculate step delta between consecutive valid Hall states
            int8_t delta = 0;
            int8_t diff_forward = (data.hall_state - prev_state + 6) % 6;
            int8_t diff_backward = (prev_state - data.hall_state + 6) % 6;
            
            // Determine direction of movement (choose shorter path)
            if (diff_forward <= diff_backward) {
                delta = diff_forward;  // Forward movement
            } else {
                delta = -diff_backward; // Backward movement
            }
            
            total_steps += delta;
            valid_transitions++;
            prev_state = data.hall_state;
        }
        
        last_timestamp = data.timestamp_us;
        last_state = data.hall_state;
        current_idx = (current_idx + 1) % MAX_HALL_DATA_POINTS;
    }
    
    // Update tail pointer to mark data as processed
    hall_data_tail_ = hall_data_head_;
    
    // Update speed measurement state
    if (valid_transitions > 0) {
        speed_last_position_ = last_state;
        speed_last_time_us_ = last_timestamp;
        speed_measurement_active_ = true;
    }
    
    // Calculate speed if we have enough data
    if (valid_transitions == 0) {
        return status_.current_speed_rpm; // No valid transitions
    }
    
    uint32_t elapsed_time_us = last_timestamp - first_timestamp;
    
    // Ensure minimum time window for stability
    if (elapsed_time_us < SPEED_WINDOW_MIN_US) {
        return status_.current_speed_rpm; // Not enough time elapsed
    }
    
    // Check if we have enough steps for reliable measurement
    int32_t abs_steps = (total_steps >= 0) ? total_steps : -total_steps;
    
    if (abs_steps < SPEED_MIN_STEPS && elapsed_time_us < SPEED_WINDOW_MAX_US) {
        return status_.current_speed_rpm; // Not enough steps yet, wait for more data
    }
    
    if (abs_steps == 0) {
        // No net movement - could be oscillating or stopped
        if (elapsed_time_us > SPEED_WINDOW_MAX_US) {
            return 0.0f; // Likely stopped
        }
        return status_.current_speed_rpm; // Keep last speed
    }
    
    // Calculate speed in RPM
    // total_steps steps in elapsed_time_us microseconds
    // Each full revolution = SPEED_STEPS_PER_REVOLUTION steps
    // RPM = (steps / elapsed_time_us) * (1000000 us/s) * (60 s/min) / SPEED_STEPS_PER_REVOLUTION
    float speed_rpm = (static_cast<float>(total_steps) * 60000000.0f) / 
                     (static_cast<float>(elapsed_time_us) * SPEED_STEPS_PER_REVOLUTION);
    
    return speed_rpm;
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

void BldcController::hallSensorInterruptHandler(uint8_t hall_state)
{
    /**
     * Hall sensor interrupt handler - called from GPIO interrupt context
     * This function must be interrupt-safe and as fast as possible
     */
    
    // Get current timestamp immediately to minimize latency
    uint32_t timestamp_us = time_us();
    
    // Validate Hall state (0-5 are valid, 0xFF indicates invalid/error)
    if (hall_state > 5 && hall_state != 0xFF) {
        return; // Invalid Hall state, ignore
    }
    
    // Calculate next head position
    size_t next_head = (hall_data_head_ + 1) % MAX_HALL_DATA_POINTS;
    
    // Check for buffer overflow
    if (next_head == hall_data_tail_) {
        // Buffer is full, mark overflow and advance tail to overwrite oldest data
        hall_data_overflow_ = true;
        hall_data_tail_ = (hall_data_tail_ + 1) % MAX_HALL_DATA_POINTS;
    }
    
    // Store the new Hall data
    hall_data_[hall_data_head_].timestamp_us = timestamp_us;
    hall_data_[hall_data_head_].hall_state = hall_state;
    
    // Advance head pointer
    hall_data_head_ = next_head;
}

} // namespace libecu
