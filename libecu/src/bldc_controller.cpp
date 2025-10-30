/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include <algorithm>
#include <stdio.h>

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
    , last_hall_time_(0)
    , hall_period_(0)
    , initialized_(false)
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
    status_.position = commutation_controller_.getCurrentPosition();
    
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
    }
}

void BldcController::stop()
{
    status_.is_running = false;
    status_.duty_cycle = 0.0f;
    commutation_controller_.update(0.0f, direction_);
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
    // This is a simplified speed calculation
    // In a real implementation, you would measure the time between Hall transitions
    // and calculate RPM based on motor pole pairs
    
    if (hall_period_ == 0) {
        return 0.0f;
    }
    
    // Assuming 6 Hall transitions per electrical revolution
    // and motor has 1 pole pair (2 poles)
    float electrical_freq = 1000000.0f / (hall_period_ * 6.0f); // Hz
    float mechanical_rpm = electrical_freq * 60.0f; // RPM
    
    return mechanical_rpm;
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
