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
    , first_run_(true)
{
}

void PidController::reset()
{
    error_ = 0.0f;
    previous_error_ = 0.0f;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    output_ = 0.0f;
    first_run_ = true;
}

float PidController::update(float setpoint, float feedback, float dt)
{
    // Calculate error
    error_ = setpoint - feedback;
    
    // Proportional term
    float proportional = params_.kp * error_;
    
    // Integral term with anti-windup
    integral_ += error_ * dt;
    integral_ = clamp(integral_, -params_.max_integral, params_.max_integral);
    float integral_term = params_.ki * integral_;
    
    // Derivative term (avoid derivative kick on setpoint changes)
    if (first_run_) {
        derivative_ = 0.0f;
        first_run_ = false;
    } else {
        derivative_ = (error_ - previous_error_) / dt;
    }
    float derivative_term = params_.kd * derivative_;
    
    // Calculate output
    output_ = proportional + integral_term + derivative_term;
    
    // Clamp output to limits
    output_ = clamp(output_, params_.min_output, params_.max_output);
    
    // Anti-windup: if output is saturated, prevent integral windup
    if ((output_ >= params_.max_output && error_ > 0.0f) ||
        (output_ <= params_.min_output && error_ < 0.0f)) {
        // Don't accumulate integral when saturated
        integral_ -= error_ * dt;
    }
    
    // Store error for next iteration
    previous_error_ = error_;
    
    return output_;
}

void PidController::setParameters(const PidParameters& params)
{
    params_ = params;
    
    // Clamp existing integral to new limits
    integral_ = clamp(integral_, -params_.max_integral, params_.max_integral);
}

float PidController::clamp(float value, float min_val, float max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

} // namespace libecu