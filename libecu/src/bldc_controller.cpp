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
#define SPEED_TIMEOUT_US          500000  ///< Timeout for speed measurement (1 second)
#define BLDC_NUM_PHASES           3        ///< Number of phases in BLDC motor

// Platform specific time function
uint32_t time_us(void);

namespace libecu {

BldcController::BldcController(
    PwmInterface& pwm_interface,
    HallInterface& hall_interface,
    CommutationController& commutation_controller,
    SafetyMonitor& safety_monitor,
    const MotorControlParams& params,
    AdcInterface* adc_interface)
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , commutation_controller_(commutation_controller)
    , safety_monitor_(safety_monitor)
    , adc_interface_(adc_interface)
    , pid_controller_(params.pid_voltage_mode)
    , current_controller_(params.pid_current_regulator)
    , params_(params)
    , direction_(RotationDirection::CLOCKWISE)
    , initialized_(false)
    , speed_measurement_active_(false)
    , speed_start_time_us_(0)
    , speed_end_time_us_(0)
    , speed_pulse_count_(0)
    , last_hall_state_(0xFF)
    , last_period_us_(0)
    , prev_position_(0xFF)
    , open_loop_step_(0)
    , open_loop_last_step_time_us_(0)
    , open_loop_running_(false)
    , last_pid_update_time_us_(0)
    , filtered_target_speed_(0.0f)
    , limited_target_speed_(0.0f)
#ifdef DEBUG_PWM_ISR
    , debug_buffer_{}
    , debug_write_index_(0)
    , debug_buffer_ready_(false)
#endif
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
    status_.control_mode = ControlMode::OPEN_LOOP;
    status_.electric_mode = ElectricMode::VOLTAGE_MODE;
    // PID controller already initialized with voltage mode parameters in initializer list
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

void BldcController::monitor(const SafetyData& safety_data)
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
}

void BldcController::update()
{
    status_.current_speed_rpm = calculateSpeed();
    
    uint8_t commutation_position = commutation_controller_.getCurrentPosition();
    if (commutation_position <= 5) {
        status_.position = static_cast<MotorPosition>(commutation_position + 1);
    } else {
        status_.position = MotorPosition::INVALID;
    }
    
    float target_duty_cycle = 0.0f;

    if (status_.is_running) {
        switch (status_.control_mode) {
            case ControlMode::OPEN_LOOP: {
                target_duty_cycle = status_.duty_cycle;
                
                uint32_t current_time_us = time_us();
                if (!open_loop_running_) {
                    open_loop_last_step_time_us_ = current_time_us;
                    open_loop_running_ = true;
                } else {
                    uint32_t step_interval_us = calculateOpenLoopStepInterval(status_.target_speed_rpm);
                    if (step_interval_us > 0 && (current_time_us - open_loop_last_step_time_us_) >= step_interval_us) {
                        open_loop_step_ = (open_loop_step_ + 1) % 6;
                        open_loop_last_step_time_us_ = current_time_us;
                    }
                }
                commutation_position = open_loop_step_;
                break;
            }

            case ControlMode::CLOSED_LOOP_VELOCITY: {
                // Velocity control with speed PID
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

                // Apply acceleration limiting (slew rate limiter on target_speed)
                 float limited_target = applyAccelerationLimit(
                    status_.target_speed_rpm,
                    dt
                );

                // Run speed PID (already configured for correct mode)
                float pid_output = pid_controller_.update(
                    limited_target,
                    status_.current_speed_rpm,
                    dt
                );

                // Electric mode determines how to use PID output
                if (status_.electric_mode == ElectricMode::VOLTAGE_MODE) {
                    // VOLTAGE_MODE: PID outputs duty cycle directly
                    target_duty_cycle = pid_output;
                } else {
                    // CURRENT_MODE: PID outputs target current
                    // Store for inner current loop (runs in pwmInterruptHandler at 20kHz)
                    {
                        CriticalSection cs;
                        status_.target_current = pid_output;
                    }

                    // duty_cycle is calculated by pwmInterruptHandler() (atomic read)
                    {
                        CriticalSection cs;
                        target_duty_cycle = status_.duty_cycle;
                    }
                }
                break;
            }

            case ControlMode::CLOSED_LOOP_TORQUE:
                // Torque control: fixed duty cycle or current + Hall sensor commutation
                // No speed PID - user sets fixed torque/current command
                if (status_.electric_mode == ElectricMode::VOLTAGE_MODE) {
                    // Fixed duty cycle with Hall sensor commutation
                    target_duty_cycle = status_.duty_cycle;
                } else {
                    // Fixed current target with Hall sensor commutation
                    // Current control runs in pwmInterruptHandler() at 20kHz
                    // duty_cycle is updated by pwmInterruptHandler() (atomic read)
                    {
                        CriticalSection cs;
                        target_duty_cycle = status_.duty_cycle;
                    }
                }
                break;
        }
    } else {
        open_loop_running_ = false;
    }

    if (commutation_position != 0xFF){
        CriticalSection cs;
        status_.duty_cycle = target_duty_cycle;
        
        if (status_.electric_mode == ElectricMode::VOLTAGE_MODE) {
            // VOLTAGE_MODE: Hall sensor commutation runs here and in hallSensorInterruptHandler
            // In hallSensorInterruptHandler is for precise switching, here is for starting at beginning rotation
            commutation_controller_.update(commutation_position, target_duty_cycle, direction_);
        } else {
            // CURRENT_MODE: Only update commutation on rotor position change
            if (commutation_position != prev_position_) {
                // Position changed - reset commutation and current controller
                commutation_controller_.update(commutation_position, 0.0f, direction_);
                current_controller_.reset();
                // Update previous position
                prev_position_ = commutation_position;
            }
        }
    }
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

void BldcController::setCurrent(float current_a)
{
    CriticalSection cs;
    status_.target_current = std::max(0.0f, std::min(current_a, params_.max_current));
}

void BldcController::setControlMode(ControlMode mode)
{
    CriticalSection cs;
    if (status_.control_mode != mode) {
        status_.control_mode = mode;

        // Reset PID when switching to velocity control
        if (mode == ControlMode::CLOSED_LOOP_VELOCITY) {
            pid_controller_.reset();
            last_pid_update_time_us_ = 0;  // Reset timing for fresh start
        }
    }
}

void BldcController::setElectricMode(ElectricMode mode)
{
    CriticalSection cs;
    status_.electric_mode = mode;

    // Reconfigure PID controller with appropriate parameters for the mode
    if (mode == ElectricMode::VOLTAGE_MODE) {
        // VOLTAGE_MODE: PID outputs duty cycle (0.0-1.0)
        pid_controller_.setParameters(params_.pid_voltage_mode);
    } else {
        // CURRENT_MODE: PID outputs current (Amperes)
        pid_controller_.setParameters(params_.pid_current_mode);
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
        speed_measurement_active_ = false;
        speed_start_time_us_ = 0;
        speed_end_time_us_ = 0;
        speed_pulse_count_ = 0;
        last_hall_state_ = 0xFF;
        last_period_us_ = 0;
        enable_interrupts();
        
        // Reset PID timing and target speed filters
        last_pid_update_time_us_ = 0;
        filtered_target_speed_ = 0.0f;
        limited_target_speed_ = 0.0f;
    }
}

void BldcController::stop()
{
    status_.is_running = false;
    status_.duty_cycle = 0.0f;
    open_loop_running_ = false;
    open_loop_step_ = 0;
    {
        CriticalSection cs;
        commutation_controller_.update(0, 0.0f, direction_);
    }
    
    // Reset speed measurement on motor stop
    disable_interrupts();
    speed_measurement_active_ = false;
    speed_start_time_us_ = 0;
    speed_end_time_us_ = 0;
    speed_pulse_count_ = 0;
    last_period_us_ = 0;
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
     * Improved speed measurement algorithm with extrapolation:
     * - State machine: STOPPED (returns 0) and ROTATING (returns measured/extrapolated speed)
     * - Uses start and end timestamps from ISR
     * - When new pulses arrive: calculates speed from measured period
     * - When no new pulses: extrapolates speed based on elapsed time vs last period
     * - Always returns a numeric value (never NAN) for consistent PID operation
     * - Transitions back to STOPPED after 5 second timeout
     */

    // Atomically copy volatile variables AND current time together
    // (prevents race where ISR updates end_time after we read current_time)
    disable_interrupts();
    uint32_t current_time_us = time_us();
    bool is_active = speed_measurement_active_;
    uint32_t start_time = speed_start_time_us_;
    uint32_t end_time = speed_end_time_us_;
    int32_t pulse_count = speed_pulse_count_;
    uint32_t last_period = last_period_us_;
    enable_interrupts();

    // STOPPED state: measurement not active
    if (!is_active) {
        return 0.0f;
    }

    // Calculate elapsed time since last pulse
    uint32_t elapsed_since_last_pulse_us = current_time_us - end_time;

    // Maximum extrapolation timeout (5 seconds) - transition back to STOPPED
    const uint32_t MAX_EXTRAPOLATION_TIMEOUT_US = 5000000;
    if (elapsed_since_last_pulse_us > MAX_EXTRAPOLATION_TIMEOUT_US) {
        // Motor has stopped - reset to STOPPED state
        disable_interrupts();
        speed_measurement_active_ = false;
        speed_start_time_us_ = 0;
        speed_end_time_us_ = 0;
        speed_pulse_count_ = 0;
        last_period_us_ = 0;
        enable_interrupts();
        return 0.0f;
    }

    // ROTATING state: we have received at least one pulse

    // If we have new pulses, calculate speed from measured period
    if (pulse_count > 0) {
        uint32_t period_us = end_time - start_time;

        // Avoid division by zero
        if (period_us == 0) {
            return 0.0f;
        }

        // Calculate speed in RPM
        // pulse_count pulses in period_us microseconds
        // Each full revolution = num_poles * BLDC_NUM_PHASES pulses (Hall transitions per revolution)
        // For num_poles=8: steps_per_rev = 8 * 3 = 24
        // RPM = (pulses / period_us) * (1000000 us/s) / steps_per_revolution
        uint8_t num_poles = commutation_controller_.getNumPoles();
        uint32_t steps_per_revolution = num_poles * BLDC_NUM_PHASES;

        float speed_rpm = (static_cast<float>(pulse_count) * 1000000.0f) /
                         (static_cast<float>(period_us) * steps_per_revolution);

        // Account for direction setting
        // Positive pulse_count with CLOCKWISE = positive speed
        // Negative pulse_count with CLOCKWISE = negative speed (moving backwards)
        // Invert sign for COUNTER_CLOCKWISE
        if (direction_ == RotationDirection::COUNTER_CLOCKWISE) {
            speed_rpm = -speed_rpm;
        }

        // Save measured period for future extrapolation
        // Update measurement window: move start time to current end time
        // Reset pulse counter for next measurement period
        disable_interrupts();
        last_period_us_ = period_us;
        speed_start_time_us_ = end_time;
        speed_pulse_count_ = 0;
        enable_interrupts();

        return speed_rpm;
    }

    // No new pulses - extrapolate based on last measured period
    if (last_period > 0) {
        // Extrapolation threshold: 1.5x last period
        // This provides noise immunity while still responding quickly
        const float EXTRAPOLATION_THRESHOLD = 1.5f;

        uint32_t extrapolation_time_us;

        if (elapsed_since_last_pulse_us > static_cast<uint32_t>(last_period * EXTRAPOLATION_THRESHOLD)) {
            // Motor is slowing down - use elapsed time for conservative speed estimate
            // This gives "speed is no more than..." estimate
            // Assumes at most 1 pulse would have occurred in the elapsed time
            extrapolation_time_us = elapsed_since_last_pulse_us;
        } else {
            // Within normal measurement window - use last measured period
            extrapolation_time_us = last_period;
        }

        // Calculate extrapolated speed
        // Assume 1 pulse per extrapolation_time_us
        uint8_t num_poles = commutation_controller_.getNumPoles();
        uint32_t steps_per_revolution = num_poles * BLDC_NUM_PHASES;

        float speed_rpm = 1000000.0f /
                         (static_cast<float>(extrapolation_time_us) * steps_per_revolution);

        // Account for direction
        if (direction_ == RotationDirection::COUNTER_CLOCKWISE) {
            speed_rpm = -speed_rpm;
        }

        return speed_rpm;
    }

    // No measurements yet (first pulse received but no period calculated) - return 0
    return 0.0f;
}

float BldcController::applyAccelerationLimit(float target_speed, float dt)
{
    // Stage 1: LPF to smooth noisy analog input
    float alpha = params_.target_speed_lpf_alpha;
    if (alpha > 0.0f && alpha < 1.0f) {
        filtered_target_speed_ = alpha * target_speed + (1.0f - alpha) * filtered_target_speed_;
    } else {
        filtered_target_speed_ = target_speed;
    }
    
    // Stage 2: Slew rate limiter
    if (params_.acceleration_rate == 0.0f) {
        limited_target_speed_ = filtered_target_speed_;
        return limited_target_speed_;
    }
    
    float speed_diff = filtered_target_speed_ - limited_target_speed_;
    float max_change = params_.acceleration_rate * dt;
    
    if (std::abs(speed_diff) <= max_change) {
        limited_target_speed_ = filtered_target_speed_;
    } else if (speed_diff > 0.0f) {
        limited_target_speed_ += max_change;
    } else {
        limited_target_speed_ -= max_change;
    }
    
    return limited_target_speed_;
}

uint32_t BldcController::calculateOpenLoopStepInterval(float speed_rpm)
{
    // step_interval_us = 1,000,000 / (speed_rpm / 60 * num_poles * 6)
    // = 1,000,000 * 60 / (speed_rpm * num_poles * 6)
    // = 10,000,000 / (speed_rpm * num_poles)
    if (speed_rpm <= 0.0f) {
        return 0;
    }
    uint8_t num_poles = commutation_controller_.getNumPoles();
    return static_cast<uint32_t>(10000000.0f / (speed_rpm * num_poles * BLDC_NUM_PHASES));
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

    // Initialize measurement on first valid transition (transition to ROTATING state)
    if (!speed_measurement_active_) {
        speed_measurement_active_ = true;
        speed_start_time_us_ = timestamp_us;
        speed_end_time_us_ = timestamp_us;
        speed_pulse_count_ = 0;
        return;
    }

    // Update end timestamp and pulse counter
    speed_end_time_us_ = timestamp_us;
    speed_pulse_count_ += delta;

    if (status_.electric_mode == ElectricMode::VOLTAGE_MODE) {
        CriticalSection cs;
        commutation_controller_.update(hall_state, status_.duty_cycle, direction_);
    } else if (status_.electric_mode == ElectricMode::CURRENT_MODE) {
        if (hall_state != prev_position_) {
            CriticalSection cs;
            commutation_controller_.update(hall_state, 0.0f, direction_);
            current_controller_.reset();
            prev_position_ = hall_state;
        }
    }
}

void BldcController::setTargetCurrent(float current_a) {
    CriticalSection cs;
    status_.target_current = current_a;
}

void BldcController::pwmInterruptHandler() {
    // Read shared data atomically (avoid torn reads from SysTick interrupt)
    ElectricMode electric_mode;
    float target_current;
    {
        CriticalSection cs;
        electric_mode = status_.electric_mode;
        target_current = status_.target_current;
    }

    // Only run current control loop in CURRENT_MODE
    if (electric_mode != ElectricMode::CURRENT_MODE) {
        return;
    }

    // Check if current controller and ADC are available
    if (!adc_interface_) {
        return;
    }

    // Read current from active conducting phase
    float measured_current = getCurrentFromActivePhase();

    // Run current controller: target_current → delta
    // Current controller outputs delta around 0, where 0 = neutral (no current)
    float duty_cycle = current_controller_.update(target_current, measured_current);

    // Write results atomically (avoid torn writes from SysTick interrupt)
    {
        CriticalSection cs;
        status_.measured_current = measured_current;
        status_.duty_cycle = duty_cycle;
        // Apply duty cycle without changing commutation step
        // Phase switching is handled in hallSensorInterruptHandler
        commutation_controller_.updateDutyCycle(duty_cycle);
    }

#ifdef DEBUG_PWM_ISR
    // Capture debug data for analysis (single buffer)
    if (!debug_buffer_ready_ && debug_write_index_ < DEBUG_BUFFER_SIZE) {
        auto& sample = debug_buffer_[debug_write_index_];
        sample.duty_cycle = duty_cycle;
        sample.target_current = target_current;
        sample.measured_current = measured_current;
        sample.current_position = commutation_controller_.getCurrentPosition();
        debug_write_index_++;

        // Buffer full - mark as ready for output
        if (debug_write_index_ >= DEBUG_BUFFER_SIZE) {
            debug_buffer_ready_ = true;
        }
    }
#endif
}

float BldcController::getCurrentFromActivePhase() {
    if (!adc_interface_) {
        return 0.0f;
    }

    // Determine which phase is actively conducting based on cached phase states
    // In 6-step commutation, we want to read current from the DOWN phase (low-side conducting)

    // Get cached phase states from CommutationController
    PwmState state_u = commutation_controller_.getCachedPhaseState(PwmChannel::PHASE_U);
    PwmState state_v = commutation_controller_.getCachedPhaseState(PwmChannel::PHASE_V);
    PwmState state_w = commutation_controller_.getCachedPhaseState(PwmChannel::PHASE_W);

    // Find the DOWN phase (low-side conducting)
    if (state_u == PwmState::DOWN) {
        return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_U);
    } else if (state_v == PwmState::DOWN) {
        return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_V);
    } else if (state_w == PwmState::DOWN) {
        return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_W);
    } else {
        // No DOWN phase found (e.g., all phases OFF), return 0
        return 0.0f;
    }
}

#ifdef DEBUG_PWM_ISR
void BldcController::processDebugOutput() {
    // Check if buffer is ready for reading
    if (!debug_buffer_ready_) {
        return;
    }

    // Print one sample
    for (size_t debug_read_index_ = 0; debug_read_index_ < DEBUG_BUFFER_SIZE; debug_read_index_++) {
        const auto& sample = debug_buffer_[debug_read_index_];
        printf("%.2f,%.2f,%.2f,%u\n",
               sample.duty_cycle,
               sample.target_current,
               sample.measured_current,
               (unsigned int)sample.current_position);
    }

    // Buffer finished - print extra newline and clear buffer
    printf("\n");

    // Clear buffer under disabled interrupts
    disable_interrupts();
    debug_write_index_ = 0;
    debug_buffer_ready_ = false;
    enable_interrupts();
}
#endif

} // namespace libecu
