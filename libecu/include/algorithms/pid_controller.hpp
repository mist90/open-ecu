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
    float sample_time_s; ///< Fixed sample time in seconds (for update(setpoint, feedback) overload)
};

/**
 * @brief PID controller with anti-windup for motor speed control
 */
class PidController {
public:
    PidController() noexcept = default;
    /**
     * @brief Constructor
     * @param params PID parameters
     */
    explicit PidController(const PidParameters& params) noexcept;

    /**
     * @brief Reset PID controller state
     */
    void reset() noexcept;

    /**
     * @brief Update PID controller
     * @param setpoint Desired value
     * @param feedback Current value
     * @param dt Time step in seconds
     * @return Control output
     */
    float update(float setpoint, float feedback, float dt) noexcept;

    /**
     * @brief Update PID controller with fixed sample time
     * @param setpoint Desired value
     * @param feedback Current value
     * @return Control output (returns 0 if disabled)
     *
     * Uses sample_time_s from parameters. Suitable for fixed-frequency loops.
     */
    float update(float setpoint, float feedback) noexcept;

    /**
     * @brief Set PID parameters
     * @param params New PID parameters
     */
    void setParameters(const PidParameters& params) noexcept;

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

    float error_;
    float previous_error_;
    float integral_;
    float derivative_;
    float output_;

    /**
     * @brief Clamp value between min and max
     * @param value Value to clamp
     * @param min_val Minimum value
     * @param max_val Maximum value
     * @return Clamped value
     */
    float clamp(float value, float min_val, float max_val) noexcept;
};

} // namespace libecu

#endif // LIBECU_PID_CONTROLLER_HPP