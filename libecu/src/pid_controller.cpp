/**
 * @file pid_controller.cpp
 * @brief Implementation of PID controller with anti-windup
 */

#include "../include/algorithms/pid_controller.hpp"
#include <algorithm>

namespace libecu {

PidController::PidController(const PidParameters& params)
    : params_(params)
    , error_(0.0f)
    , previous_error_(0.0f)
    , integral_(0.0f)
    , derivative_(0.0f)
    , output_(0.0f)
{
}

void PidController::reset()
{
    error_ = 0.0f;
    previous_error_ = 0.0f;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    output_ = 0.0f;
}

float PidController::update(float setpoint, float feedback, float dt)
{
    error_ = setpoint - feedback;

    float proportional = params_.kp * error_;
    float potential_integral_ = integral_ + params_.ki * (error_ + previous_error_) * 0.5f * dt;

    derivative_ = (dt > 0.0f) ? params_.kd * (error_ - previous_error_) / dt : 0.0f;

    output_ = proportional + potential_integral_ + derivative_;

    // Anti-windup + update integral
    if (output_ > params_.max_output) {
        output_ = params_.max_output;
        if (error_ < 0.0f)
            integral_ = potential_integral_;
    } else if (output_ < params_.min_output) {
        output_ = params_.min_output;
        if (error_ > 0)
            integral_ = potential_integral_;
    } else {
        integral_ = potential_integral_;
    }

    previous_error_ = error_;

    return output_;
}

float PidController::update(float setpoint, float feedback)
{
    return update(setpoint, feedback, params_.sample_time_s);
}

void PidController::setParameters(const PidParameters& params)
{
    params_ = params;

    integral_ = clamp(integral_, params_.min_output, params_.max_output);
}

float PidController::clamp(float value, float min_val, float max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

} // namespace libecu