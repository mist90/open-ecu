/**
 * @file motor_pll.cpp
 * @brief Phase-Locked Loop (PLL) for motor rotor angle estimation - implementation
 */

#include "../include/algorithms/motor_pll.hpp"
#include "../include/bldc_controller.hpp"
#include <cmath>

namespace libecu {

MotorPLL::MotorPLL(float freq_pwm, bool is_inverse_commutation_table) noexcept
    : is_inverse_commutation_table_(is_inverse_commutation_table)
{
    DT = 1.0f / freq_pwm;
}

void MotorPLL::updateHall(uint8_t hall_state, uint32_t timestamp_us) noexcept {
    uint8_t prev_hall_state = hall_state_;
    hall_state_ = hall_state;

    uint32_t delta_us = timestamp_us - last_timestamp_us_;
    last_timestamp_us_ = timestamp_us;

    float est_dt_hall = static_cast<float>(delta_us) * 0.000001f;

    if (est_dt_hall > 0.5f || est_dt_hall <= 0.0f) {
        est_dt_hall = 0.001f;
    }

    if (!use_pll_) {
        return;
    }

    int8_t delta = (hall_state_ - prev_hall_state + 9) % 6 - 3;
    rotation_count_ += delta;

    float measured_speed = (static_cast<float>(delta) * 60.0f) / est_dt_hall;
    angle_per_second_ = measured_speed;
    
    float absolute_real_angle = static_cast<float>(rotation_count_) * 60.0f;
    angle_ = absolute_real_angle;
    last_absolute_real_angle_ = absolute_real_angle;
}

void MotorPLL::updateTick() noexcept {
    if (!use_pll_) return;

    angle_ += angle_per_second_ * DT;
    
    // Check if PLL lost sync (when motor stops but hall doesn't change)
    // Use modular arithmetic to compare angles correctly
    float normalized_angle = fmodf(angle_, 360.0f);
    if (normalized_angle < 0.0f) normalized_angle += 360.0f;
    
    float normalized_real = fmodf(last_absolute_real_angle_, 360.0f);
    if (normalized_real < 0.0f) normalized_real += 360.0f;
    
    float angle_diff = std::abs(normalized_angle - normalized_real);
    if (angle_diff > 180.0f) angle_diff = 360.0f - angle_diff;
    
    if (angle_diff > 90.0f) {
        angle_ = last_absolute_real_angle_;
        angle_per_second_ = 0.0f;
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

    // Normalize angle to [0, 360) for sector calculation
    float normalized_angle = fmodf(angle_, 360.0f);
    if (normalized_angle < 0.0f) normalized_angle += 360.0f;

    // IDLE/stop: hold current sector (zero torque vector)
    if (direction == 0.0f) {
        uint8_t current_rotor_sector = static_cast<uint8_t>(normalized_angle / 60.0f);
        return current_rotor_sector > 5 ? 5 : current_rotor_sector;
    }

    // Offset stator field by 90 degrees relative to rotor for maximum torque
    float field_angle = normalized_angle + (90.0f * direction);

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
    rotation_count_ = 0;
}

float MotorPLL::getAngle() const noexcept {
    // Return normalized angle in [0, 360) range
    float normalized = fmodf(angle_, 360.0f);
    if (normalized < 0.0f) normalized += 360.0f;
    return normalized;
}

float MotorPLL::getSpeedDegSec() const noexcept {
    return angle_per_second_;
}

} // namespace libecu
