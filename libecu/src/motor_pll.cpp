/**
 * @file motor_pll.cpp
 * @brief Phase-Locked Loop (PLL) for motor rotor angle estimation - implementation
 */

#include "../include/algorithms/motor_pll.hpp"
#include "../include/bldc_controller.hpp" // DriveMode enum definition
#include <cmath>

namespace libecu {

MotorPLL::MotorPLL(float freq_pwm, bool is_inverse_commutation_table) noexcept
    : is_inverse_commutation_table_(is_inverse_commutation_table)
{
    DT = 1.0f / freq_pwm;
}

void MotorPLL::updateHall(uint8_t hall_state, uint32_t timestamp_us) noexcept {
    hall_state_ = hall_state;

    // Time delta between Hall interrupts.
    // Unsigned arithmetic correctly handles timer overflow (0xFFFFFFFF → 0).
    uint32_t delta_us = timestamp_us - last_timestamp_us_;
    last_timestamp_us_ = timestamp_us;

    // Convert microseconds to seconds (float)
    float est_dt_hall = static_cast<float>(delta_us) * 0.000001f;

    // Guard against first call or abnormally long motor stall
    if (est_dt_hall > 0.5f || est_dt_hall <= 0.0f) {
        est_dt_hall = 0.001f; // Safe default
    }

    // Exit if PLL disabled or Hall sensor error
    if (!use_pll_) {
        return;
    }

    // 1. Convert Hall step (0..5) to electrical rotor angle
    float real_rotor_angle = static_cast<float>(hall_state_) * 60.0f;

    // 2. Phase error between reality and virtual model
    float angle_error = real_rotor_angle - angle_;

    // Wrap error across 0/360 boundary
    angle_error = fmodf(angle_error, 360.0f);
    if (angle_error > 180.0f)  angle_error -= 360.0f;
    if (angle_error < -180.0f) angle_error += 360.0f;

    // 3. Adapt PLL gains to current speed
    updateAdaptiveGains();

    // 4. PLL PI controller
    pll_integral_ += angle_error * pll_ki_ * est_dt_hall;

    // Anti-windup: clamp integrator to speed limits
    if (pll_integral_ > MAX_ELECTRICAL_SPEED)  pll_integral_ = MAX_ELECTRICAL_SPEED;
    if (pll_integral_ < -MAX_ELECTRICAL_SPEED) pll_integral_ = -MAX_ELECTRICAL_SPEED;

    angle_per_second_ = (angle_error * pll_kp_) + pll_integral_;
}

void MotorPLL::updateTick() noexcept {
    if (!use_pll_) return;

    angle_ += angle_per_second_ * DT;

    // Normalize virtual angle to [0.0, 360.0)
    angle_ = fmodf(angle_, 360.0f);
    if (angle_ < 0.0f) {
        angle_ += 360.0f;
    }
}

uint8_t MotorPLL::getNextHall(const volatile DriveMode &mode) noexcept {
    // Propagate Hall sensor error for emergency handling
    if (hall_state_ == 0xFF) return 0xFF;

    // Legacy mode: discrete commutation (direct/inverse table)
    if (!use_pll_) {
        if (mode == DriveMode::FORWARD)
            return !is_inverse_commutation_table_ ? (hall_state_ + 1) % 6 : (hall_state_ + 5) % 6;
        else if (mode == DriveMode::REVERSE)
            return !is_inverse_commutation_table_ ? (hall_state_ + 5) % 6 : (hall_state_ + 1) % 6;
        else
            return hall_state_;
    }

    // PLL mode: commutation based on virtual angle with 90-degree field offset
    float direction = 0.0f;
    if (mode == DriveMode::FORWARD)  direction = !is_inverse_commutation_table_ ? 1.0f : -1.0f;
    if (mode == DriveMode::REVERSE)  direction = !is_inverse_commutation_table_ ? -1.0f : 1.0f;

    // IDLE/stop: hold current sector (zero torque vector)
    if (direction == 0.0f) {
        uint8_t current_rotor_sector = static_cast<uint8_t>(angle_ / 60.0f);
        return current_rotor_sector > 5 ? 5 : current_rotor_sector;
    }

    // Offset stator field by 90 degrees relative to rotor for maximum torque
    float field_angle = angle_ + (90.0f * direction);

    // Normalize field angle
    if (field_angle >= 360.0f) field_angle -= 360.0f;
    if (field_angle < 0.0f)    field_angle += 360.0f;

    // Quantize field angle into 6 commutation sectors (0..5)
    uint8_t sector = static_cast<uint8_t>(field_angle / 60.0f);
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
    angle_ = static_cast<float>(hall_state_ == 0xFF ? 0 : hall_state_) * 60.0f;
    angle_per_second_ = 0.0f;
    pll_integral_ = 0.0f;
}

float MotorPLL::getAngle() const noexcept {
    return angle_;
}

float MotorPLL::getSpeedDegSec() const noexcept {
    return angle_per_second_;
}

void MotorPLL::updateAdaptiveGains() noexcept {
    float abs_speed = std::abs(angle_per_second_);
    float base_kp = 1.0f;
    float base_ki = 8.0f;

    float speed_factor = abs_speed / MAX_ELECTRICAL_SPEED;
    if (speed_factor > 1.0f) speed_factor = 1.0f;

    pll_kp_ = base_kp + (60.0f * speed_factor);
    pll_ki_ = base_ki + (300.0f * speed_factor);
}

} // namespace libecu
