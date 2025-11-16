/**
 * @file current_controller.hpp
 * @brief PI controller for motor current regulation
 */

#ifndef LIBECU_CURRENT_CONTROLLER_HPP
#define LIBECU_CURRENT_CONTROLLER_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Current controller parameters
 */
struct CurrentControllerParameters {
    float kp;                  ///< Proportional gain
    float ki;                  ///< Integral gain
    float max_output;          ///< Maximum output (duty cycle, 1.0 = 100%)
    float min_output;          ///< Minimum output (duty cycle, 0.0 = 0%)
    float max_integral;        ///< Maximum integral windup limit
    float sample_time_s;       ///< Controller sample time in seconds (e.g., 0.00005 for 20kHz)
    float max_current;         ///< Maximum allowed current in Amperes (for limiting)
};

/**
 * @brief PI controller for fast current regulation
 *
 * Implements a digital PI controller for the inner current control loop.
 * Runs at PWM frequency (typically 20kHz) for fast response to current changes.
 *
 * Features:
 * - Anti-windup with integral clamping
 * - Output saturation (0-100% duty cycle)
 * - Sample-time aware for consistent behavior
 * - Current limiting for safety
 */
class CurrentController {
public:
    /**
     * @brief Constructor
     * @param params Controller parameters
     */
    explicit CurrentController(const CurrentControllerParameters& params);

    /**
     * @brief Update controller (call at fixed sample rate)
     * @param setpoint Desired current in Amperes
     * @param measured Measured current in Amperes
     * @return Control output (duty cycle, 0.0-1.0)
     */
    float update(float setpoint, float measured);

    /**
     * @brief Reset controller state (clear integral, etc.)
     */
    void reset();

    /**
     * @brief Set new parameters
     * @param params New controller parameters
     */
    void setParameters(const CurrentControllerParameters& params);

    /**
     * @brief Get current parameters
     * @return Current controller parameters
     */
    const CurrentControllerParameters& getParameters() const { return params_; }

    /**
     * @brief Get current error
     * @return Most recent error value
     */
    float getError() const { return error_; }

    /**
     * @brief Get integral term
     * @return Current integral accumulator value
     */
    float getIntegral() const { return integral_; }

    /**
     * @brief Enable/disable controller
     * @param enabled true to enable, false to disable (outputs zero)
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }

    /**
     * @brief Check if controller is enabled
     * @return true if enabled
     */
    bool isEnabled() const { return enabled_; }

private:
    CurrentControllerParameters params_;
    float integral_;          ///< Integral accumulator
    float error_;             ///< Most recent error
    float previous_output_;   ///< Previous output for tracking
    bool enabled_;            ///< Controller enable state

    /**
     * @brief Clamp value to range
     * @param value Value to clamp
     * @param min_val Minimum value
     * @param max_val Maximum value
     * @return Clamped value
     */
    float clamp(float value, float min_val, float max_val);
};

} // namespace libecu

#endif // LIBECU_CURRENT_CONTROLLER_HPP
