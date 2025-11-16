/**
 * @file current_controller.cpp
 * @brief Implementation of PI current controller
 */

#include "../include/algorithms/current_controller.hpp"
#include <cmath>

namespace libecu {

CurrentController::CurrentController(const CurrentControllerParameters& params)
    : params_(params)
    , integral_(0.0f)
    , error_(0.0f)
    , previous_output_(0.0f)
    , enabled_(false)
{
}

float CurrentController::update(float setpoint, float measured) {
    if (!enabled_) {
        return 0.0f;
    }

    // Limit setpoint to maximum current
    setpoint = clamp(setpoint, -params_.max_current, params_.max_current);

    // Calculate error
    error_ = setpoint - measured;

    // Proportional term
    float p_term = params_.kp * error_;

    // Integral term (with anti-windup)
    integral_ += error_ * params_.sample_time_s;
    integral_ = clamp(integral_, -params_.max_integral, params_.max_integral);
    float i_term = params_.ki * integral_;

    // Calculate output
    float output = p_term + i_term;

    // Clamp output to valid duty cycle range
    output = clamp(output, params_.min_output, params_.max_output);

    // Anti-windup: back-calculate integral if output is saturated
    if (output >= params_.max_output || output <= params_.min_output) {
        // Prevent integral from growing when output is saturated
        float output_unclamped = p_term + i_term;
        if ((output_unclamped - output) != 0.0f) {
            // Recalculate integral to prevent windup
            integral_ -= (error_ * params_.sample_time_s);
        }
    }

    previous_output_ = output;
    return output;
}

void CurrentController::reset() {
    integral_ = 0.0f;
    error_ = 0.0f;
    previous_output_ = 0.0f;
}

void CurrentController::setParameters(const CurrentControllerParameters& params) {
    params_ = params;
    // Reset state when parameters change to avoid transients
    reset();
}

float CurrentController::clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

} // namespace libecu
