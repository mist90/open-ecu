/**
 * @file pid_controller.cpp
 * @brief Implementation of PID controller with anti-windup
 */

#include "../include/algorithms/pid_controller.hpp"
#include <algorithm>

namespace libecu {

PidController::PidController(const PidParameters& params) noexcept
    : params_(params)
    , previous_error_(0.0f)
    , integral_(0.0f)
    , derivative_(0.0f)
    , output_(0.0f)
{
}

void PidController::reset() noexcept
{
    previous_error_ = 0.0f;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    output_ = 0.0f;
}

float PidController::update(float setpoint, float feedback, float dt) noexcept
{
    float error = setpoint - feedback;

    float i_max = params_.integral_max;
    float i_min = params_.integral_min;

    float proportional = params_.kp * error;

    // Trapezoidal integration with clamping
    float potential_integral = integral_ + params_.ki * (error + previous_error_) * 0.5f * dt;
    potential_integral = clamp(potential_integral, i_min, i_max);

    derivative_ = (dt > 0.0f) ? params_.kd * (error - previous_error_) / dt : 0.0f;

    float unclamped_output = proportional + potential_integral + derivative_;
    output_ = clamp(unclamped_output, params_.min_output, params_.max_output);

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

float PidController::update(float setpoint, float feedback) noexcept
{
    return update(setpoint, feedback, params_.sample_time_s);
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
