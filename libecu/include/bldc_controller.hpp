/**
 * @file bldc_controller.hpp
 * @brief High-level BLDC motor controller integrating all components
 */

#ifndef LIBECU_BLDC_CONTROLLER_HPP
#define LIBECU_BLDC_CONTROLLER_HPP

#include <cstddef>
#include <cstdint>

#include "interfaces/pwm_interface.hpp"
#include "interfaces/hall_interface.hpp"
#include "interfaces/adc_interface.hpp"
#include "algorithms/commutation_controller.hpp"
#include "algorithms/pid_controller.hpp"
#include "algorithms/current_controller.hpp"
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
 * @brief Motor control parameters
 */
struct MotorControlParams {
    float max_duty_cycle;     ///< Maximum duty cycle (0.0 to 1.0)
    float max_speed_rpm;      ///< Maximum speed in RPM
    float acceleration_rate;  ///< Acceleration rate (RPM/s)
    uint32_t control_frequency; ///< Control loop frequency (Hz)
    PidParameters pid_voltage_mode; ///< PID parameters for voltage mode (outputs duty cycle)
    PidParameters pid_current_mode; ///< PID parameters for current mode (outputs current)
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
    MotorPosition position;   ///< Current motor position
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
     * @param current_controller Current PI controller (optional, nullptr for voltage mode only)
     */
    BldcController(
        PwmInterface& pwm_interface,
        HallInterface& hall_interface,
        CommutationController& commutation_controller,
        SafetyMonitor& safety_monitor,
        const MotorControlParams& params,
        AdcInterface* adc_interface = nullptr,
        CurrentController* current_controller = nullptr
    );

    /**
     * @brief Initialize motor controller
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Update motor control (call at control frequency)
     * @param safety_data Current safety monitoring data
     */
    void update(const SafetyData& safety_data);

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
     * @brief Set motor direction
     * @param direction Rotation direction
     */
    void setDirection(RotationDirection direction);

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
     * @brief Set target current for current control mode
     * @param current_a Target current in Amperes
     */
    void setTargetCurrent(float current_a);

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

private:
    // Component references
    PwmInterface& pwm_interface_;
    HallInterface& hall_interface_;
    CommutationController& commutation_controller_;
    SafetyMonitor& safety_monitor_;
    AdcInterface* adc_interface_;            // Optional: for current sensing
    CurrentController* current_controller_;   // Optional: for current control

    // Owned components
    PidController pid_controller_;           // Speed controller (outer loop)

    // Configuration
    MotorControlParams params_;

    // State variables
    MotorStatus status_;
    RotationDirection direction_;
    bool initialized_;
    
    // Speed measurement state (interrupt-driven)
    volatile bool speed_measurement_active_; ///< True if speed measurement is active (ROTATING state)
    volatile uint32_t speed_start_time_us_;  ///< Start timestamp for speed measurement
    volatile uint32_t speed_end_time_us_;    ///< End timestamp for speed measurement
    volatile int32_t speed_pulse_count_;     ///< Pulse count (can be negative for reverse)
    volatile uint8_t last_hall_state_;       ///< Last Hall state to detect changes
    volatile uint32_t last_period_us_;       ///< Last measured period between pulses (for extrapolation)
    
    // Control loop timing
    uint32_t last_pid_update_time_us_;       ///< Timestamp of last successful PID update

    /**
     * @brief Calculate motor speed from Hall sensor transitions
     * @return Speed in RPM
     */
    float calculateSpeed();

    /**
     * @brief Apply acceleration/deceleration limits
     * @param target_speed Target speed
     * @param current_speed Current speed
     * @param dt Time step
     * @return Limited speed
     */
    float applyAccelerationLimit(float target_speed, float current_speed, float dt);

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
};

} // namespace libecu

#endif // LIBECU_BLDC_CONTROLLER_HPP