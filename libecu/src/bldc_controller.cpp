/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include "../include/platform/critical_section.hpp"
#include <algorithm>
#include <stdio.h>
#include <cmath>

// Platform-specific interrupt control
void disable_interrupts();
void enable_interrupts();

// Speed measurement configuration
#define SPEED_TIMEOUT_US          1000000  ///< Timeout for speed measurement (1 second)
#define BLDC_NUM_PHASES           3        ///< Number of phases in BLDC motor

// Platform specific time function
uint32_t time_us(void);

namespace libecu {

BldcController::BldcController(
    PwmInterface& pwm_interface,
    HallInterface& hall_interface,
    CommutationController& commutation_controller,
    PidController& pid_controller,
    SafetyMonitor& safety_monitor,
    const MotorControlParams& params,
    AdcInterface* adc_interface,
    CurrentController* current_controller)
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , commutation_controller_(commutation_controller)
    , pid_controller_(pid_controller)
    , safety_monitor_(safety_monitor)
    , adc_interface_(adc_interface)
    , current_controller_(current_controller)
    , params_(params)
    , direction_(RotationDirection::CLOCKWISE)
    , initialized_(false)
    , speed_start_time_us_(0)
    , speed_end_time_us_(0)
    , speed_pulse_count_(0)
    , last_hall_state_(0xFF)
    , last_pid_update_time_us_(0)
{
    // Initialize status
    status_.current_speed_rpm = 0.0f;
    status_.target_speed_rpm = 0.0f;
    status_.duty_cycle = 0.0f;
    status_.target_current = 0.0f;
    status_.measured_current = 0.0f;
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
    float measured_speed = calculateSpeed();
    
    // Update current speed only if measurement is valid (not NAN)
    if (!std::isnan(measured_speed)) {
        status_.current_speed_rpm = measured_speed;
    }
    // If NAN, keep previous status_.current_speed_rpm value
    
    // Update position status from current Hall sensor reading
    uint8_t current_position = commutation_controller_.getCurrentPosition();
    if (current_position <= 5) {
        status_.position = static_cast<MotorPosition>(current_position + 1);
    } else {
        status_.position = MotorPosition::INVALID;
    }
    
    // Control loop based on mode
    float target_duty_cycle = 0.0f;
    
    if (status_.is_running) {
        switch (status_.mode) {
            case ControlMode::OPEN_LOOP:
                target_duty_cycle = status_.duty_cycle;
                break;
                
            case ControlMode::CLOSED_LOOP:
                // Only run PID control if we have valid speed measurement
                if (!std::isnan(measured_speed)) {
                    // Calculate real time difference since last PID update
                    uint32_t current_time_us = time_us();
                    float dt;

                    if (last_pid_update_time_us_ == 0) {
                        // First PID update - use nominal period
                        dt = 1.0f / params_.control_frequency;
                    } else {
                        // Calculate actual time difference in seconds
                        uint32_t dt_us = current_time_us - last_pid_update_time_us_;
                        dt = static_cast<float>(dt_us) / 1000000.0f;
                    }

                    // Update timestamp for next iteration
                    last_pid_update_time_us_ = current_time_us;

                    // Apply acceleration limiting
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

                    //printf("s:%f,d:%f\n", status_.current_speed_rpm, target_duty_cycle);
                } else {
                    // No valid measurement yet, keep previous duty cycle
                    target_duty_cycle = status_.duty_cycle;
                }
                break;

            case ControlMode::CURRENT_CONTROL:
                // Cascaded control: outer loop (speed → current) runs here at 100Hz
                // Inner loop (current → duty) runs in pwmInterruptHandler() at 20kHz
                if (!std::isnan(measured_speed) && current_controller_) {
                    // Calculate time difference
                    uint32_t current_time_us = time_us();
                    float dt;

                    if (last_pid_update_time_us_ == 0) {
                        dt = 1.0f / params_.control_frequency;
                    } else {
                        uint32_t dt_us = current_time_us - last_pid_update_time_us_;
                        dt = static_cast<float>(dt_us) / 1000000.0f;
                    }

                    last_pid_update_time_us_ = current_time_us;

                    // Apply acceleration limiting
                    float limited_target = applyAccelerationLimit(
                        status_.target_speed_rpm,
                        status_.current_speed_rpm,
                        dt
                    );

                    // Speed PID outputs target current (not duty cycle)
                    float target_current = pid_controller_.update(
                        limited_target,
                        status_.current_speed_rpm,
                        dt
                    );

                    // Store target current for inner loop (atomic write)
                    {
                        CriticalSection cs;
                        status_.target_current = target_current;
                    }

                    // Enable current controller for inner loop execution
                    current_controller_->setEnabled(true);

                    //printf("s:%f,i:%f\n", status_.current_speed_rpm, target_current);
                } else {
                    // No valid measurement or no current controller
                    {
                        CriticalSection cs;
                        status_.target_current = 0.0f;
                    }
                    if (current_controller_) {
                        current_controller_->setEnabled(false);
                    }
                }
                // duty_cycle is updated by pwmInterruptHandler(), not here (atomic read)
                {
                    CriticalSection cs;
                    target_duty_cycle = status_.duty_cycle;
                }
                break;
        }
    }

    // Update commutation based on control mode
    // Protect duty_cycle write (can race with pwmInterruptHandler in CURRENT_CONTROL mode)
    {
        CriticalSection cs;
        status_.duty_cycle = target_duty_cycle;
    }

    if (status_.mode == ControlMode::OPEN_LOOP) {
        // Use target speed for open-loop control
        commutation_controller_.updateOpenLoop(target_duty_cycle, status_.target_speed_rpm, direction_);
    } else if (status_.mode == ControlMode::CLOSED_LOOP) {
        // Use Hall sensor feedback for closed-loop speed control
        commutation_controller_.update(target_duty_cycle, direction_);
    }
    // Note: CURRENT_CONTROL mode updates commutation in pwmInterruptHandler() at 20kHz
}

void BldcController::setTargetSpeed(float speed_rpm)
{
    CriticalSection cs;
    // Clamp to maximum speed
    status_.target_speed_rpm = std::min(std::abs(speed_rpm), params_.max_speed_rpm);

    // Set direction based on sign
    direction_ = (speed_rpm >= 0.0f) ? RotationDirection::CLOCKWISE : RotationDirection::COUNTER_CLOCKWISE;
}

void BldcController::setDutyCycle(float duty_cycle)
{
    CriticalSection cs;
    status_.duty_cycle = std::max(0.0f, std::min(duty_cycle, params_.max_duty_cycle));
}

void BldcController::setControlMode(ControlMode mode)
{
    CriticalSection cs;
    if (status_.mode != mode) {
        status_.mode = mode;

        // Reset PID when switching to closed loop
        if (mode == ControlMode::CLOSED_LOOP) {
            pid_controller_.reset();
            last_pid_update_time_us_ = 0;  // Reset timing for fresh start
        }
    }
}

void BldcController::setDirection(RotationDirection direction)
{
    CriticalSection cs;
    direction_ = direction;
}

void BldcController::start()
{
    if (status_.active_fault == SafetyFault::NONE) {
        status_.is_running = true;
        pwm_interface_.enable(true);
        
        // Reset speed measurement on motor start
        disable_interrupts();
        speed_start_time_us_ = 0;
        speed_end_time_us_ = 0;
        speed_pulse_count_ = 0;
        last_hall_state_ = 0xFF;
        enable_interrupts();
        
        // Reset PID timing
        last_pid_update_time_us_ = 0;
    }
}

void BldcController::stop()
{
    status_.is_running = false;
    status_.duty_cycle = 0.0f;
    commutation_controller_.update(0.0f, direction_);
    
    // Reset speed measurement on motor stop
    disable_interrupts();
    speed_start_time_us_ = 0;
    speed_end_time_us_ = 0;
    speed_pulse_count_ = 0;
    enable_interrupts();
    status_.current_speed_rpm = 0.0f;
    
    // Reset PID timing
    last_pid_update_time_us_ = 0;
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
     * Simplified speed measurement algorithm:
     * - Uses start and end timestamps from ISR
     * - Calculates speed based on pulse count and time difference
     * - Accounts for rotation direction
     */
    
    uint32_t current_time_us = time_us();
    
    // Atomically copy volatile variables (disable interrupts during copy)
    disable_interrupts();
    uint32_t start_time = speed_start_time_us_;
    uint32_t end_time = speed_end_time_us_;
    int32_t pulse_count = speed_pulse_count_;
    enable_interrupts();
    
    // Check if measurement has been initialized
    if (start_time == 0) {
        return NAN; // Not initialized yet
    }
    
    // Check for timeout - motor might be stopped
    if ((current_time_us - end_time) > SPEED_TIMEOUT_US) {
        return 0.0f; // Motor stopped (no pulses for 1 second)
    }
    
    // Check if we have any pulses to measure
    if (pulse_count == 0) {
        return NAN; // No new pulses, measurement not ready
    }
    
    // Calculate time difference
    uint32_t time_diff_us = end_time - start_time;
    
    // Avoid division by zero
    if (time_diff_us == 0) {
        return NAN; // Time difference too small, measurement not valid
    }
    
    // Calculate speed in RPM
    // pulse_count pulses in time_diff_us microseconds
    // Each full revolution = num_poles * BLDC_NUM_PHASES pulses (Hall transitions per revolution)
    // For num_poles=8: steps_per_rev = 8 * 3 = 24
    // RPM = (pulses / time_diff_us) * (1000000 us/s) / steps_per_revolution
    uint8_t num_poles = commutation_controller_.getNumPoles();
    uint32_t steps_per_revolution = num_poles * BLDC_NUM_PHASES;
    
    float speed_rpm = (static_cast<float>(pulse_count) * 1000000.0f) / 
                     (static_cast<float>(time_diff_us) * steps_per_revolution);
    
    // Account for direction setting
    // Positive pulse_count with CLOCKWISE = positive speed
    // Negative pulse_count with CLOCKWISE = negative speed (moving backwards)
    // Invert sign for COUNTER_CLOCKWISE
    if (direction_ == RotationDirection::COUNTER_CLOCKWISE) {
        speed_rpm = -speed_rpm;
    }
    
    // Update measurement window: move start time to current end time
    // Reset pulse counter for next measurement period
    disable_interrupts();
    speed_start_time_us_ = speed_end_time_us_;
    speed_pulse_count_ = 0;
    enable_interrupts();
    
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

void BldcController::hallSensorInterruptHandler()
{
    /**
     * Hall sensor interrupt handler - called from GPIO interrupt context
     * This function must be interrupt-safe and as fast as possible
     * 
     * Algorithm:
     * - Read current Hall state
     * - Calculate step delta (can be negative for reverse movement)
     * - Update end timestamp and increment pulse counter
     * - Initialize start timestamp on first call
     */
    
    // Get current timestamp immediately to minimize latency
    uint32_t timestamp_us = time_us();
    
    // Read current Hall sensor state via CommutationController
    uint8_t hall_state = commutation_controller_.getCurrentPosition();
    
    // Validate Hall state (0-5 are valid, 0xFF indicates invalid/error)
    if (hall_state > 5 && hall_state != 0xFF) {
        return; // Invalid Hall state, ignore
    }
    
    // Check if state has changed from previous reading
    if (hall_state == last_hall_state_) {
        return; // No change in Hall state, ignore
    }
    
    // Calculate step delta between current and previous Hall state
    int8_t delta = 0;
    if (last_hall_state_ != 0xFF) {
        // Calculate forward and backward differences
        int8_t diff_forward = (hall_state - last_hall_state_ + 6) % 6;
        int8_t diff_backward = (last_hall_state_ - hall_state + 6) % 6;
        
        // Choose shorter path (determines direction)
        if (diff_forward <= diff_backward) {
            delta = diff_forward;  // Forward movement (positive)
        } else {
            delta = -diff_backward; // Backward movement (negative)
        }
    }
    
    // Update last Hall state
    last_hall_state_ = hall_state;
    
    // Initialize start time on first valid transition
    if (speed_start_time_us_ == 0) {
        speed_start_time_us_ = timestamp_us;
        speed_end_time_us_ = timestamp_us;
        speed_pulse_count_ = 0;
        return;
    }
    
    // Update end timestamp and pulse counter
    speed_end_time_us_ = timestamp_us;
    speed_pulse_count_ += delta;
}

void BldcController::setTargetCurrent(float current_a) {
    CriticalSection cs;
    status_.target_current = current_a;
}

void BldcController::pwmInterruptHandler() {
    // Read shared data atomically (avoid torn reads from SysTick interrupt)
    ControlMode mode;
    float target_current;
    RotationDirection dir;
    {
        CriticalSection cs;
        mode = status_.mode;
        target_current = status_.target_current;
        dir = direction_;
    }

    // Only run current control loop in CURRENT_CONTROL mode
    if (mode != ControlMode::CURRENT_CONTROL) {
        return;
    }

    // Check if current controller and ADC are available
    if (!current_controller_ || !adc_interface_) {
        return;
    }

    // Check if current controller is enabled
    if (!current_controller_->isEnabled()) {
        return;
    }

    // Read current from active conducting phase
    float measured_current = getCurrentFromActivePhase();

    // Run current controller: target_current → delta
    // Current controller outputs delta around 0, where 0 = neutral (no current)
    float delta = current_controller_->update(target_current, measured_current);

    // Convert to duty cycle: 0.5 is neutral, add delta for positive/negative current
    float duty_cycle = 0.5f + delta;

    // Write results atomically (avoid torn writes from SysTick interrupt)
    {
        CriticalSection cs;
        status_.measured_current = measured_current;
        status_.duty_cycle = duty_cycle;
    }

    // Apply duty cycle via commutation controller
    // Get current commutation step and apply the calculated duty cycle
    uint8_t position = commutation_controller_.getCurrentPosition();
    if (position != 0xFF) {
        commutation_controller_.update(duty_cycle, dir);
    }
}

float BldcController::getCurrentFromActivePhase() {
    if (!adc_interface_) {
        return 0.0f;
    }

    // Determine which phase is actively conducting based on commutation step
    // In 6-step commutation, one phase is always actively switching (UP or DOWN)
    // We want to read current from the phase that is conducting (typically the DOWN phase)

    uint8_t step = commutation_controller_.getCurrentStep();

    // Based on commutation tables in commutation_controller.cpp:
    // Step 0: U=UP,   V=OFF,  W=DOWN → Read W
    // Step 1: U=OFF,  V=UP,   W=DOWN → Read W
    // Step 2: U=DOWN, V=UP,   W=OFF  → Read U
    // Step 3: U=DOWN, V=OFF,  W=UP   → Read U
    // Step 4: U=OFF,  V=DOWN, W=UP   → Read V
    // Step 5: U=UP,   V=DOWN, W=OFF  → Read V

    switch (step) {
        case 0:
        case 1:
            // W is conducting (DOWN)
            return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_W);
        case 2:
        case 3:
            // U is conducting (DOWN)
            return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_U);
        case 4:
        case 5:
            // V is conducting (DOWN)
            return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_V);
        default:
            // Invalid step, return average of all phases
            float i_u, i_v, i_w;
            adc_interface_->readAllCurrents(i_u, i_v, i_w);
            return (i_u + i_v + i_w) / 3.0f;
    }
}

} // namespace libecu
