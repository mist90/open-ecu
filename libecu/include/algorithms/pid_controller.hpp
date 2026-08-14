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
    float integral_max; ///< Integral clamp upper limit (must be set explicitly)
    float integral_min; ///< Integral clamp lower limit (must be set explicitly)
    float kb;           ///< Back-calculation anti-windup gain (0 = disabled, typical 1-5)
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
     * @param external_saturation Which limit the downstream loop is stuck on:
     *        +1 clamped high, -1 clamped low, 0 free. Integration is frozen
     *        only while the error would drive further into that limit
     *        (conditional integration for cascaded loop anti-windup).
     * @return Control output
     */
    float update(float setpoint, float feedback, float dt, int external_saturation = 0) noexcept;

    /**
     * @brief Update PID controller with fixed sample time
     * @param setpoint Desired value
     * @param feedback Current value
     * @param external_saturation Which limit the downstream loop is stuck on:
     *        +1 clamped high, -1 clamped low, 0 free. See the dt overload.
     * @return Control output (returns 0 if disabled)
     *
     * Uses sample_time_s from parameters. Suitable for fixed-frequency loops.
     */
    float update(float setpoint, float feedback, int external_saturation = 0) noexcept;

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

    /**
     * @brief Check if output was saturated (clamped) on last update
     * @return true if output hit min_output or max_output limit
     */
    bool isSaturated() const { return saturation_ != 0; }

    /**
     * @brief Which limit the output was clamped to on the last update
     * @return +1 clamped at max_output, -1 clamped at min_output, 0 not clamped
     *
     * Feed this to the outer loop's update() so it can keep integrating in the
     * direction that backs out of the saturation.
     */
    int saturationSign() const { return saturation_; }

private:
    PidParameters params_;

    float previous_error_;
    float integral_;
    float derivative_;
    float output_;
    int saturation_;        ///< +1/-1 when output was clamped high/low on last update, else 0

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
