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

MotorPLL::MotorPLL(float freq_pwm, bool is_inverse_commutation_table) noexcept
    : is_inverse_commutation_table_(is_inverse_commutation_table)
{
    DT_ = 1.0f / freq_pwm;
}

void MotorPLL::updateHall(uint8_t hall_state, uint32_t timestamp_us) noexcept {
    if (hall_state == 0xFF || hall_state > 5) {
        hall_state_ = 0xFF;
        return;
    }

    uint32_t delta_us = timestamp_us - last_timestamp_us_;
    last_timestamp_us_ = timestamp_us;
    float est_dt_hall = static_cast<float>(delta_us) * 0.000001f;

    time_since_last_hall_ = 0.0f;

    float target_base_angle = static_cast<float>(hall_state);

    if (est_dt_hall > HALL_TIMEOUT_SEC || est_dt_hall <= 0.0f) {
        hall_state_ = hall_state;
        angle_ = target_base_angle;
        angle_per_second_ = 0.0f;
        pll_integral_ = 0.0f;
        return;
    }

    if (!use_pll_) {
        hall_state_ = hall_state;
        return;
    }

    float current_period_base = std::floor(angle_ / 6.0f) * 6.0f;
    float real_rotor_angle = current_period_base + target_base_angle;

    float angle_error = real_rotor_angle - angle_;

    if (angle_error > 3.0f)  angle_error -= 6.0f;
    if (angle_error < -3.0f) angle_error += 6.0f;

    updateAdaptiveGains();

    pll_integral_ += angle_error * pll_ki_ * est_dt_hall;
    
    if (pll_integral_ > MAX_ELECTRICAL_SPEED)  pll_integral_ = MAX_ELECTRICAL_SPEED;
    if (pll_integral_ < -MAX_ELECTRICAL_SPEED) pll_integral_ = -MAX_ELECTRICAL_SPEED;

    angle_per_second_ = (angle_error * pll_kp_) + pll_integral_;
    
    hall_state_ = hall_state;
}

void MotorPLL::updateTick() noexcept {
    if (!use_pll_) return;

    time_since_last_hall_ += DT_;

    if (time_since_last_hall_ >= HALL_TIMEOUT_SEC) {
        angle_per_second_ = 0.0f;
        pll_integral_ = 0.0f;
        return; 
    }

    angle_ += angle_per_second_ * DT_;
    
    if (angle_ >= ANGLE_MAX) {
        angle_ -= ANGLE_MAX;
    } else if (angle_ < 0.0f) {
        angle_ += ANGLE_MAX;
    }
}

uint8_t MotorPLL::getNextHall(const volatile DriveMode &mode) noexcept {
    if (hall_state_ == 0xFF) return 0xFF;

    if (!use_pll_) {
        if (mode == DriveMode::FORWARD)
            return !is_inverse_commutation_table_ ? (hall_state_ + 1) % 6 : (hall_state_ + 5) % 6;
        else if (mode == DriveMode::REVERSE)
            return !is_inverse_commutation_table_ ? (hall_state_ + 5) % 6 : (hall_state_ + 1) % 6;
        else
            return hall_state_;
    }

    float direction = 0.0f;
    if (mode == DriveMode::FORWARD)  direction = !is_inverse_commutation_table_ ? 1.0f : -1.0f;
    if (mode == DriveMode::REVERSE)  direction = !is_inverse_commutation_table_ ? -1.0f : 1.0f;

    if (direction == 0.0f) {
        float clamped_angle = angle_ - std::floor(angle_ / 6.0f) * 6.0f;
        uint8_t sector = static_cast<uint8_t>(clamped_angle);
        return sector > 5 ? 5 : sector;
    }

    float field_angle = angle_ + (1.5f * direction);

    field_angle = field_angle - std::floor(field_angle / 6.0f) * 6.0f;
    if (field_angle < 0.0f) field_angle += 6.0f;

    uint8_t sector = static_cast<uint8_t>(field_angle);
    if (sector > 5) sector = 5; 

    return sector;
}

void MotorPLL::setUsePLL(bool use) noexcept {
    use_pll_ = use;
    if (!use) reset();
}

bool MotorPLL::isUsingPLL() const noexcept {
    return use_pll_;
}

void MotorPLL::reset() noexcept {
    angle_ = static_cast<float>(hall_state_ == 0xFF ? 0 : hall_state_);
    angle_per_second_ = 0.0f;
    pll_integral_ = 0.0f;
    time_since_last_hall_ = 0.0f;
}

float MotorPLL::getAngle() const noexcept {
    return angle_;
}

float MotorPLL::getSpeedStepsSec() const noexcept {
    return angle_per_second_;
}

float MotorPLL::getMechanicalRPS() const noexcept {
    return (angle_per_second_ / 6.0f) / 20.0f;
}

void MotorPLL::updateAdaptiveGains() noexcept {
    float abs_speed = std::abs(angle_per_second_);
    
    float base_kp = 15.0f;
    float base_ki = 180.0f;

    float speed_factor = abs_speed / MAX_ELECTRICAL_SPEED; 
    if (speed_factor > 1.0f) speed_factor = 1.0f;

    pll_kp_ = base_kp + (45.0f * speed_factor);  
    pll_ki_ = base_ki + (2500.0f * speed_factor); 
}

} // namespace libecu
