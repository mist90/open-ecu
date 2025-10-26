/**
 * @file commutation_controller.hpp
 * @brief 6-step commutation algorithm for BLDC motors
 */

#ifndef LIBECU_COMMUTATION_CONTROLLER_HPP
#define LIBECU_COMMUTATION_CONTROLLER_HPP

#include "../interfaces/pwm_interface.hpp"
#include "../interfaces/hall_interface.hpp"
#include <cstdint>

namespace libecu {

/**
 * @brief Motor rotation direction
 */
enum class RotationDirection : uint8_t {
    CLOCKWISE = 0,
    COUNTER_CLOCKWISE = 1
};

/**
 * @brief Commutation step for 6-step algorithm
 */
struct CommutationStep {
    PwmState phase_u;    ///< Phase U PWM state
    PwmState phase_v;    ///< Phase V PWM state  
    PwmState phase_w;    ///< Phase W PWM state
};

/**
 * @brief 6-step commutation controller for BLDC motors
 */
class CommutationController {
public:
    /**
     * @brief Constructor
     * @param pwm_interface PWM interface
     * @param hall_interface Hall sensor interface
     * @param num_poles Number of motor pole pairs (default: 2 for 4-pole motor)
     */
    CommutationController(PwmInterface& pwm_interface, HallInterface& hall_interface, uint8_t num_poles = 2);

    /**
     * @brief Initialize commutation controller
     * @param pwm_frequency PWM frequency in Hz
     * @return true if initialization successful
     */
    bool initialize(uint32_t pwm_frequency = 20000);

    /**
     * @brief Update commutation based on Hall sensor state (closed-loop)
     * @param duty_cycle Motor duty cycle (0.0 to 1.0)
     *                   0.0 = 0V output (neutral, no torque)
     *                   1.0 = maximum voltage output (full torque)
     * @param direction Rotation direction
     * @return true if commutation updated successfully
     */
    bool update(float duty_cycle, RotationDirection direction = RotationDirection::CLOCKWISE);

    /**
     * @brief Update commutation based on target speed (open-loop)
     * @param duty_cycle Motor duty cycle (0.0 to 1.0)
     * @param target_speed_rpm Target motor speed in RPM
     * @param direction Rotation direction
     * @return true if commutation updated successfully
     */
    bool updateOpenLoop(float duty_cycle, float target_speed_rpm, RotationDirection direction = RotationDirection::CLOCKWISE);

    /**
     * @brief Emergency stop motor
     */
    void emergencyStop();

    /**
     * @brief Get current motor position
     * @return Current motor position
     */
    MotorPosition getCurrentPosition() const { return current_position_; }

    /**
     * @brief Get current commutation step
     * @return Current commutation step
     */
    uint8_t getCurrentStep() const { return current_step_; }

    /**
     * @brief Check if motor is running
     * @return true if motor is running
     */
    bool isRunning() const { return is_running_; }

private:
    PwmInterface& pwm_interface_;
    HallInterface& hall_interface_;
    
    MotorPosition current_position_;
    uint8_t current_step_;
    bool is_running_;
    uint8_t num_poles_;  ///< Number of motor pole pairs
    
    // Open-loop timing control
    uint32_t last_step_time_us_;
    uint32_t step_interval_us_;
    
    static const CommutationStep COMMUTATION_TABLE_CW[6];
    static const CommutationStep COMMUTATION_TABLE_CCW[6];
    
    /**
     * @brief Get commutation step from Hall position
     * @param position Motor position
     * @param direction Rotation direction
     * @return Commutation step index (0-5)
     */
    uint8_t getStepFromPosition(MotorPosition position, RotationDirection direction);
    
    /**
     * @brief Apply commutation step
     * @param step Commutation step
     * @param duty_cycle Duty cycle (0.0 to 1.0)
     *                   0.0 = 0V output, 1.0 = maximum voltage
     */
    void applyCommutationStep(const CommutationStep& step, float duty_cycle);
    
    /**
     * @brief Calculate step interval from target speed
     * @param speed_rpm Target speed in RPM
     * @return Step interval in microseconds
     */
    uint32_t calculateStepInterval(float speed_rpm);
    
    /**
     * @brief Get current time in microseconds
     * @return Current time in microseconds
     */
    uint32_t getCurrentTimeUs();
};

} // namespace libecu

#endif // LIBECU_COMMUTATION_CONTROLLER_HPP