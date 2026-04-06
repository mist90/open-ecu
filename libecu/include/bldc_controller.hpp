/**
 * @file bldc_controller.hpp
 * @brief High-level BLDC motor controller integrating all components
 */

#ifndef LIBECU_BLDC_CONTROLLER_HPP
#define LIBECU_BLDC_CONTROLLER_HPP

#include <cstddef>
#include <cstdint>
#include <array>

#include "interfaces/pwm_interface.hpp"
#include "interfaces/hall_interface.hpp"
#include "interfaces/adc_interface.hpp"
#include "algorithms/commutation_controller.hpp"
#include "algorithms/pid_controller.hpp"
#include "safety/safety_monitor.hpp"

namespace libecu {

/**
 * @brief Motor control mode (mechanical/commutation strategy)
 */
enum class ControlMode : uint8_t {
    OPEN_LOOP = 0,              ///< Open loop control (timing-based, no sensors)
    CLOSED_LOOP_VELOCITY = 1,   ///< Closed loop velocity control with PID + Hall sensors
    CLOSED_LOOP_TORQUE = 2      ///< Closed loop torque control (fixed duty/current + Hall sensors)
};

/**
 * @brief Electric drive mode (electrical control strategy)
 */
enum class ElectricMode : uint8_t {
    VOLTAGE_MODE = 0,  ///< Direct voltage/duty cycle control
    CURRENT_MODE = 1   ///< Current control with PI inner loop (20kHz)
};

/**
 * @brief Motor rotation mode
 */
enum class DriveMode : uint8_t {
    FORWARD = 0,
    REVERSE = 1
    // NEUTRAL = 2
    // PARKING = 3
    // BRAKE = 4
};

/**
 * @brief Motor control parameters
 */
struct MotorControlParams {
    float max_duty_cycle;     ///< Maximum duty cycle (0.0 to 1.0)
    float max_current;        ///< Maximum motor current (A)
    float max_speed_rpm;      ///< Maximum speed in RPM
    float acceleration_rate;  ///< Acceleration rate (RPM/s), 0 = disabled
    float target_speed_lpf_alpha; ///< LPF alpha for target speed (0.0-1.0), 0 = disabled, 1.0 = no filtering
    uint32_t control_frequency; ///< Control loop frequency (Hz)
    PidParameters pid_voltage_mode; ///< Velocity PID parameters for voltage mode (outputs duty cycle)
    PidParameters pid_current_mode; ///< Velocity PID parameters for current mode (outputs current)
    PidParameters pid_current_regulator; ///< Current PID parameters for current mode (outputs duty cycle)
};

/**
 * @brief Motor status information
 */
struct MotorStatus {
    float current_speed_rpm;  ///< Current motor speed (RPM)
    float target_speed_rpm;   ///< Target motor speed (RPM)
    float duty_cycle;         ///< Current duty cycle
    float target_current;     ///< Target motor current (A)
    float measured_current;   ///< Measured motor current (A)
    uint8_t target_position;   ///< Driven motor position
    uint8_t measured_position;   ///< Measured motor position
    SafetyFault active_fault; ///< Active safety fault
    bool is_running;          ///< Motor running status
    ControlMode control_mode;   ///< Current control mode (mechanical)
    ElectricMode electric_mode; ///< Current electric mode (electrical)
};

/**
 * @brief High-level BLDC motor controller
 */
class BldcController {
public:
    /**
     * @brief Constructor
     * @param pwm_interface PWM interface
     * @param hall_interface Hall sensor interface
     * @param commutation_controller Commutation controller
     * @param safety_monitor Safety monitor
     * @param params Motor control parameters (includes PID parameters for both modes)
     * @param adc_interface ADC interface for current sensing (optional, nullptr for voltage mode only)
     */
    BldcController(
        PwmInterface& pwm_interface,
        HallInterface& hall_interface,
        CommutationController& commutation_controller,
        SafetyMonitor& safety_monitor,
        const MotorControlParams& params,
        AdcInterface* adc_interface = nullptr
    );

    /**
     * @brief Initialize motor controller
     * @return true if initialization successful
     */
    bool initialize();


    /**
     * @brief Update monitor 
     * @param safety_data Current safety monitoring data
     */
    void monitor(const SafetyData& safety_data);

    /**
     * @brief Update motor control (call at control frequency)
     */
    void update();

    /**
     * @brief Set target speed for closed-loop control
     * @param speed_rpm Target speed in RPM
     */
    void setTargetSpeed(float speed_rpm);

    /**
     * @brief Set duty cycle for open-loop control
     * @param duty_cycle Duty cycle (0.0 to 1.0)
     *                   0.0 = 0V output (no torque)
     *                   1.0 = maximum voltage output (full torque)
     */
    void setDutyCycle(float duty_cycle);

    /**
     * @brief Set target current for torque control
     * @param current_a Target current in Amperes (0.0 to max_current)
     *                  Clamped to [0, max_current] range
     */
    void setCurrent(float current_a);

    /**
     * @brief Set control mode (mechanical/commutation strategy)
     * @param mode Control mode
     */
    void setControlMode(ControlMode mode);

    /**
     * @brief Set electric mode (electrical control strategy)
     * @param mode Electric mode
     */
    void setElectricMode(ElectricMode mode);

    /**
     * @brief Set motor drive mode
     * @param mode Rotation mode
     */
    void setDriveMode(DriveMode mode);

    /**
     * @brief Start motor
     */
    void start();

    /**
     * @brief Stop motor
     */
    void stop();

    /**
     * @brief Emergency stop
     */
    void emergencyStop();

    /**
     * @brief Get motor status
     * @return Current motor status
     */
    MotorStatus getStatus() const;

    /**
     * @brief Clear safety fault
     * @param fault Fault to clear
     */
    void clearFault(SafetyFault fault);

    /**
     * @brief Hall sensor interrupt handler (called from GPIO interrupt context)
     * This method must be called from the Hall sensor GPIO interrupt handlers.
     * It captures the current Hall sensor state and timestamp for speed calculation.
     * The hall state is read internally via CommutationController::getCurrentPosition().
     */
    void hallSensorInterruptHandler();

    /**
     * @brief PWM interrupt handler for high-frequency current control loop (20kHz)
     * Call this from TIM1 update interrupt when current control mode is active.
     * This runs the inner current control loop at PWM frequency.
     */
    void pwmInterruptHandler();

#ifdef DEBUG_PWM_ISR
    /**
     * @brief Process debug buffer output (call from main loop)
     * Outputs one sample per call to avoid blocking
     */
    void processDebugOutput();
#endif

private:
    void moveNextPosition(uint8_t position);
    // Component references
    PwmInterface& pwm_interface_;
    HallInterface& hall_interface_;
    CommutationController& commutation_controller_;
    SafetyMonitor& safety_monitor_;
    AdcInterface* adc_interface_;

    // Owned components
    PidController pid_controller_;           // Speed controller (outer loop)
    PidController current_controller_;       // Current controller (inner loop)

    // Configuration
    MotorControlParams params_;

    // State variables
    MotorStatus status_;
    DriveMode dmode_;
    bool initialized_;
    
    // Speed measurement state (interrupt-driven)
    volatile bool speed_measurement_active_; ///< True if speed measurement is active (ROTATING state)
    volatile uint32_t speed_start_time_us_;  ///< Start timestamp for speed measurement
    volatile uint32_t speed_end_time_us_;    ///< End timestamp for speed measurement
    volatile int32_t speed_pulse_count_;     ///< Pulse count (can be negative for reverse)
    volatile uint8_t last_hall_state_;       ///< Last Hall state to detect changes
    volatile uint32_t last_period_us_;       ///< Last measured period between pulses (for extrapolation)

    // Position tracking for CURRENT_MODE (to detect commutation events)
    volatile uint8_t prev_position_;         ///< Previous rotor position for change detection
    
    // Open-loop timing control
    uint8_t open_loop_step_;                 ///< Current step in open-loop mode (0-5)
    uint32_t open_loop_last_step_time_us_;   ///< Timestamp of last step change
    bool open_loop_running_;                 ///< Open-loop timing initialized
    
    // Control loop timing
    uint32_t last_pid_update_time_us_;       ///< Timestamp of last successful PID update
    
    // Target speed filtering state (LPF → slew rate limiter cascade)
    float filtered_target_speed_;            ///< LPF-filtered target speed
    float limited_target_speed_;             ///< Rate-limited target speed (after LPF)

#ifdef DEBUG_PWM_ISR
    // Debug data capture (single buffer)
    struct PwmDebugSample {
        float duty_cycle;
        float target_current;
        float measured_current;
        uint8_t current_position;
    };

    static constexpr size_t DEBUG_BUFFER_SIZE = 1000;
    std::array<PwmDebugSample, DEBUG_BUFFER_SIZE> debug_buffer_; ///< Single buffer for debug data
    volatile size_t debug_write_index_;       ///< Current write index
    volatile bool debug_buffer_ready_;        ///< True when buffer is full and ready for reading
#endif

    /**
     * @brief Calculate motor speed from Hall sensor transitions
     * @return Speed in RPM
     */
    float calculateSpeed();

    /**
     * @brief Apply acceleration/deceleration limits to target speed
     * 
     * Implements a slew rate limiter that restricts how fast target_speed can change.
     * Stores previous limited value internally - does NOT use measured speed.
     * 
     * @param target_speed Desired target speed
     * @param dt Time step in seconds
     * @return Rate-limited target speed
     */
    float applyAccelerationLimit(float target_speed, float dt);

    /**
     * @brief Handle safety faults
     * @param fault Active safety fault
     */
    void handleSafetyFault(SafetyFault fault);

    /**
     * @brief Get current from active conducting phase based on commutation state
     * @return Measured phase current in Amperes
     */
    float getCurrentFromActivePhase();
    
    /**
     * @brief Calculate step interval from target speed for open-loop control
     * @param speed_rpm Target speed in RPM
     * @return Step interval in microseconds
     */
    uint32_t calculateOpenLoopStepInterval(float speed_rpm);
};

} // namespace libecu

#endif // LIBECU_BLDC_CONTROLLER_HPP