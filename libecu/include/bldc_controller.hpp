/**
 * @file bldc_controller.hpp
 * @brief High-level BLDC motor controller integrating all components
 */

#ifndef LIBECU_BLDC_CONTROLLER_HPP
#define LIBECU_BLDC_CONTROLLER_HPP

#include "interfaces/pwm_interface.hpp"
#include "interfaces/hall_interface.hpp"
#include "algorithms/commutation_controller.hpp"
#include "algorithms/pid_controller.hpp"
#include "safety/safety_monitor.hpp"

namespace libecu {

/**
 * @brief Motor control mode
 */
enum class ControlMode : uint8_t {
    OPEN_LOOP = 0,    ///< Open loop control (direct duty cycle)
    CLOSED_LOOP = 1   ///< Closed loop speed control with PID
};

/**
 * @brief Motor control parameters
 */
struct MotorControlParams {
    float max_duty_cycle;     ///< Maximum duty cycle (0.0 to 1.0)
    float max_speed_rpm;      ///< Maximum speed in RPM
    float acceleration_rate;  ///< Acceleration rate (RPM/s)
    uint32_t control_frequency; ///< Control loop frequency (Hz)
};

/**
 * @brief Motor status information
 */
struct MotorStatus {
    float current_speed_rpm;  ///< Current motor speed (RPM)
    float target_speed_rpm;   ///< Target motor speed (RPM)
    float duty_cycle;         ///< Current duty cycle
    MotorPosition position;   ///< Current motor position
    SafetyFault active_fault; ///< Active safety fault
    bool is_running;          ///< Motor running status
    ControlMode mode;         ///< Current control mode
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
     * @param pid_controller PID controller
     * @param safety_monitor Safety monitor
     * @param params Motor control parameters
     */
    BldcController(
        PwmInterface& pwm_interface,
        HallInterface& hall_interface,
        CommutationController& commutation_controller,
        PidController& pid_controller,
        SafetyMonitor& safety_monitor,
        const MotorControlParams& params
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
     */
    void setDutyCycle(float duty_cycle);

    /**
     * @brief Set control mode
     * @param mode Control mode
     */
    void setControlMode(ControlMode mode);

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
     * @brief Clear safety fault
     * @param fault Fault to clear
     */
    void clearFault(SafetyFault fault);

private:
    // Component references
    PwmInterface& pwm_interface_;
    HallInterface& hall_interface_;
    CommutationController& commutation_controller_;
    PidController& pid_controller_;
    SafetyMonitor& safety_monitor_;

    // Configuration
    MotorControlParams params_;

    // State variables
    MotorStatus status_;
    RotationDirection direction_;
    uint32_t last_hall_time_;
    uint32_t hall_period_;
    bool initialized_;

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
};

} // namespace libecu

#endif // LIBECU_BLDC_CONTROLLER_HPP