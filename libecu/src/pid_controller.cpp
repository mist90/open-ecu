/**
 * @file pid_controller.cpp
 * @brief Implementation of PID controller with anti-windup
 */

#include "../include/pid_controller.hpp"
#include <algorithm>

namespace libecu {

PidController::PidController(const PidParameters& params) noexcept
    : params_(params)
    , previous_error_(0.0f)
    , integral_(0.0f)
    , derivative_(0.0f)
    , output_(0.0f)
    , saturation_(0)
{
}

void PidController::reset() noexcept
{
    previous_error_ = 0.0f;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    output_ = 0.0f;
    saturation_ = 0;
}

float PidController::update(float setpoint, float feedback, float dt, int external_saturation) noexcept
{
    float error = setpoint - feedback;

    float i_max = params_.integral_max;
    float i_min = params_.integral_min;

    float proportional = params_.kp * error;

    // Conditional integration, one-sided. A saturated downstream loop only
    // means "I cannot give you more" (or less) in the direction it is stuck;
    // asking it for the opposite is always achievable. Freezing both ways
    // latches the integral: under load it winds up, and after the load is
    // removed the inner loop stays saturated - at no load the current is set
    // by back-EMF, not by duty - so the demand it can never meet is the very
    // thing that keeps the outer integral from unwinding. The motor then runs
    // away at no-load speed until the drive mode is cycled.
    const bool pushing_into_limit =
        (external_saturation > 0 && error > 0.0f) ||
        (external_saturation < 0 && error < 0.0f);

    float potential_integral;
    if (pushing_into_limit) {
        potential_integral = integral_;
    } else {
        // Trapezoidal integration with clamping
        potential_integral = integral_ + params_.ki * (error + previous_error_) * 0.5f * dt;
        potential_integral = clamp(potential_integral, i_min, i_max);
    }

    derivative_ = (dt > 0.0f) ? params_.kd * (error - previous_error_) / dt : 0.0f;

    float unclamped_output = proportional + potential_integral + derivative_;
    output_ = clamp(unclamped_output, params_.min_output, params_.max_output);
    saturation_ = (unclamped_output > params_.max_output) ? 1
                : (unclamped_output < params_.min_output) ? -1 : 0;

    // Back-calculation anti-windup: reduce integral when output is saturated
    if (params_.kb > 0.0f && dt > 0.0f) {
        float saturation_error = output_ - unclamped_output;
        if (saturation_error != 0.0f) {
            potential_integral += params_.kb * saturation_error * dt;
            potential_integral = clamp(potential_integral, i_min, i_max);
        }
    }

    integral_ = potential_integral;
    previous_error_ = error;

    return output_;
}

float PidController::update(float setpoint, float feedback, int external_saturation) noexcept
{
    return update(setpoint, feedback, params_.sample_time_s, external_saturation);
}

void PidController::setParameters(const PidParameters& params) noexcept
{
    params_ = params;
    integral_ = clamp(integral_, params_.integral_min, params_.integral_max);
}

float PidController::clamp(float value, float min_val, float max_val) noexcept
{
    return std::max(min_val, std::min(value, max_val));
}

} // namespace libecu
