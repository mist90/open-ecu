/**
 * @file motor_pll.cpp
 * @brief Phase-Locked Loop (PLL) for motor rotor angle estimation - implementation
 * 
 * Uses "steps" as angle units where one electrical period = 6.0 steps.
 * Working range is 0.0...60.0 (10 electrical periods) to prevent phase wrap-around.
 */

#include "../include/algorithms/motor_pll.hpp"
#include "../include/bldc_controller.hpp"
#include <cmath>

namespace libecu {

MotorPLL::MotorPLL(float freq_pwm, float max_electrical_speed, bool is_inverse_commutation_table) noexcept
    : is_inverse_commutation_table_(is_inverse_commutation_table)
{
    DT_ = 1.0f / freq_pwm;
    max_electrical_speed_ = max_electrical_speed;
}

void MotorPLL::updateHall(uint8_t hall_state) noexcept {
    if (!use_pll_)
        hall_state_ = hall_state;
    else {
        int8_t diff = (hall_state - (hall_state_ % 6) + 9) % 6 - 3;
        int8_t next_state = int8_t(hall_state_) + diff;

        if (next_state < 0) {
            hall_state_ = next_state + 60;
        } else if (next_state >= 60) {
            hall_state_ = next_state % 60;
        } else {
            hall_state_ = (uint8_t)next_state;
        }
    }
    time_since_last_hall_ = 0;
}

void MotorPLL::updateTick() noexcept {
    if (!use_pll_)
        return;

    /* PID */
    float angle_error = static_cast<float>(hall_state_) - angle_;

    if (angle_error < 0.0f)
        angle_error += 60.0f;

    updateAdaptiveGains();

    pll_integral_ += angle_error * pll_ki_ * DT_;
    
    if (pll_integral_ > max_electrical_speed_)  pll_integral_ = max_electrical_speed_;
    if (pll_integral_ < -max_electrical_speed_) pll_integral_ = -max_electrical_speed_;

    angle_per_second_ = (angle_error * pll_kp_) + pll_integral_;

    /* Integration */
    time_since_last_hall_ += DT_;

    if (time_since_last_hall_ >= HALL_TIMEOUT_SEC) {
        angle_per_second_ = 0.0f;
        pll_integral_ = 0.0f;
        return; 
    }

    angle_ += angle_per_second_ * DT_;
    
    angle_ = fmodf(angle_, 60.0f);
    if (angle_ < 0.0f) {
        angle_ += 60.0f;
    }
}

uint8_t MotorPLL::getNextHall(const volatile DriveMode &mode) noexcept {
    if (!use_pll_) {
        if (mode == DriveMode::FORWARD)
            return !is_inverse_commutation_table_ ? (hall_state_ + 1) % 6 : (hall_state_ + 5) % 6;
        else if (mode == DriveMode::REVERSE)
            return !is_inverse_commutation_table_ ? (hall_state_ + 5) % 6 : (hall_state_ + 1) % 6;
        else
            return hall_state_;
    }

    float direction = 0.0f;
    if (mode == DriveMode::FORWARD)
        direction = !is_inverse_commutation_table_ ? 1.0f : -1.0f;
    if (mode == DriveMode::REVERSE)
        direction = !is_inverse_commutation_table_ ? -1.0f : 1.0f;

    float next_angle = angle_ + (1.5f * direction); 

    return static_cast<uint8_t>(std::round(next_angle)) % 6;
}

void MotorPLL::setUsePLL(bool use) noexcept {
    use_pll_ = use;
    if (!use) reset();
}

bool MotorPLL::isUsingPLL() const noexcept {
    return use_pll_;
}

void MotorPLL::reset() noexcept {
    angle_ = hall_state_;
    angle_per_second_ = 0.0f;
    pll_integral_ = 0.0f;
    time_since_last_hall_ = 0.0f;
}

float MotorPLL::getAngle() const noexcept {
    return fmodf(angle_, 6.0f);
}

float MotorPLL::getSpeedStepsSec() const noexcept {
    return angle_per_second_;
}

void MotorPLL::updateAdaptiveGains() noexcept {
    float abs_speed = std::abs(angle_per_second_);
    
    float base_kp = 15.0f;
    float base_ki = 0.0f;

    float speed_factor = abs_speed / max_electrical_speed_; 
    if (speed_factor > 1.0f) speed_factor = 1.0f;

    pll_kp_ = base_kp + (45.0f * speed_factor);  
    pll_ki_ = base_ki + (0.0f * speed_factor); 
}

} // namespace libecu
