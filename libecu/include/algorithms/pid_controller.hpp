/**
 * @file pid_controller.hpp
 * @brief PID controller for motor speed control with anti-windup
 */

#ifndef LIBECU_PID_CONTROLLER_HPP
#define LIBECU_PID_CONTROLLER_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief PID controller parameters
 */
struct PidParameters {
    float kp;           ///< Proportional gain
    float ki;           ///< Integral gain  
    float kd;           ///< Derivative gain
    float max_output;   ///< Maximum output value
    float min_output;   ///< Minimum output value
    float max_integral; ///< Maximum integral term (anti-windup)
};

/**
 * @brief PID controller with anti-windup for motor speed control
 */
class PidController {
public:
    /**
     * @brief Constructor
     * @param params PID parameters
     */
    explicit PidController(const PidParameters& params);

    /**
     * @brief Reset PID controller state
     */
    void reset();

    /**
     * @brief Update PID controller
     * @param setpoint Desired value
     * @param feedback Current value
     * @param dt Time step in seconds
     * @return Control output
     */
    float update(float setpoint, float feedback, float dt);

    /**
     * @brief Set PID parameters
     * @param params New PID parameters
     */
    void setParameters(const PidParameters& params);

    /**
     * @brief Get current PID parameters
     * @return Current PID parameters
     */
    const PidParameters& getParameters() const { return params_; }

    /**
     * @brief Get last error value
     * @return Last error value
     */
    float getLastError() const { return error_; }

    /**
     * @brief Get integral term
     * @return Current integral term
     */
    float getIntegral() const { return integral_; }

    /**
     * @brief Get derivative term
     * @return Current derivative term
     */
    float getDerivative() const { return derivative_; }

    /**
     * @brief Get last output
     * @return Last control output
     */
    float getOutput() const { return output_; }

private:
    PidParameters params_;
    
    float error_;           ///< Current error
    float previous_error_;  ///< Previous error for derivative calculation
    float integral_;        ///< Integral term
    float derivative_;      ///< Derivative term
    float output_;          ///< Controller output
    
    bool first_run_;        ///< Flag for first execution
    
    /**
     * @brief Clamp value between min and max
     * @param value Value to clamp
     * @param min_val Minimum value
     * @param max_val Maximum value
     * @return Clamped value
     */
    float clamp(float value, float min_val, float max_val);
};

} // namespace libecu

#endif // LIBECU_PID_CONTROLLER_HPP