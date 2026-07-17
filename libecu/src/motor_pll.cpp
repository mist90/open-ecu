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
    int step_diff = (static_cast<int>(hall_state) - static_cast<int>(hall_state_raw_) + 6) % 6;
    float direction = 0.0f;
    if (step_diff == 1) direction = 1.0f;
    else if (step_diff == 5) direction = -1.0f;

    if (direction != 0.0f && has_previous_edge_ &&
        time_since_last_hall_ > 0.0f && time_since_last_hall_ < HALL_TIMEOUT_SEC) {
        float measured_speed = direction / time_since_last_hall_;
        if (std::abs(measured_speed) <= 5.0f * max_electrical_speed_) {
            measured_speed_filtered_ = 0.3f * measured_speed + 0.7f * measured_speed_filtered_;
        }
    }
    has_previous_edge_ = true;
    time_since_last_hall_ = 0;
    hall_state_raw_ = hall_state;
}

void MotorPLL::updateTick() noexcept {
    // Compute angle error with proper circular wrapping to [-ANGLE_MAX/2, +ANGLE_MAX/2).
    // fmodf alone preserves the sign of the dividend, so fmodf(-5.5, 6.0) = -5.5
    // instead of the correct +0.5.  This caused the PLL to reverse direction at
    // every Hall-sensor wraparound (5→0), preventing lock at low speeds.
    float angle_error = static_cast<float>(hall_state_raw_) - angle_;
    angle_error = fmodf(angle_error, ANGLE_MAX);
    if (angle_error > ANGLE_MAX * 0.5f)
        angle_error -= ANGLE_MAX;
    else if (angle_error < -ANGLE_MAX * 0.5f)
        angle_error += ANGLE_MAX;
    bool reset_angle = false;

    if (angle_error > LIMIT_ANGLE_ERROR)
        angle_error = LIMIT_ANGLE_ERROR;
    else if (angle_error < -LIMIT_ANGLE_ERROR)
        angle_error = -LIMIT_ANGLE_ERROR;

    pll_step_error_filtered = (0.05f * std::abs(angle_error)) + (0.95f * pll_step_error_filtered);

    if (is_sync) {
        if (pll_step_error_filtered >= LIMIT_ANGLE_ERROR * 0.8f) {
            is_sync = false;
            reset_angle = true;
        }
    } else {
        if (std::abs(angle_error) == LIMIT_ANGLE_ERROR)
            reset_angle = true;
        else if (std::abs(angle_per_second_) > SYNC_SPEED)
            is_sync = true;
    }

    pll_integral_ += angle_error * pll_ki_ * DT_;

    if (pll_integral_ > max_electrical_speed_)
        pll_integral_ = max_electrical_speed_;
    if (pll_integral_ < -max_electrical_speed_)
        pll_integral_ = -max_electrical_speed_;

    angle_per_second_ = (angle_error * pll_kp_) + pll_integral_;

    /* Integration */
    time_since_last_hall_ += DT_;

    if (time_since_last_hall_ >= HALL_TIMEOUT_SEC) {
        angle_per_second_ = 0.0f;
        pll_integral_ = 0.0f;
        measured_speed_filtered_ = 0.0f;
        has_previous_edge_ = false;
        return;
    }

    if (!reset_angle) {
        angle_ += angle_per_second_ * DT_;

        angle_ = fmodf(angle_, ANGLE_MAX);
        if (angle_ < 0.0f) {
            angle_ += ANGLE_MAX;
        }
    } else {
        pll_integral_ = angle_per_second_;
        angle_ = static_cast<float>(hall_state_raw_);
    }
}

uint8_t MotorPLL::getNextHall(const volatile DriveMode &mode) noexcept {
    if (!use_pll_) {
        if (mode == DriveMode::FORWARD)
            return !is_inverse_commutation_table_ ? (hall_state_raw_ + 1) % 6 : (hall_state_raw_ + 5) % 6;
        else if (mode == DriveMode::REVERSE)
            return !is_inverse_commutation_table_ ? (hall_state_raw_ + 5) % 6 : (hall_state_raw_ + 1) % 6;
        else
            return hall_state_raw_;
    }

    float direction = 0.0f;
    if (mode == DriveMode::FORWARD)
        direction = !is_inverse_commutation_table_ ? 1.0f : -1.0f;
    if (mode == DriveMode::REVERSE)
        direction = !is_inverse_commutation_table_ ? -1.0f : 1.0f;

    float next_angle = angle_ + (1.0f * direction);

    int next_step = static_cast<int>(std::round(next_angle));
    next_step = ((next_step % 6) + 6) % 6;
    return static_cast<uint8_t>(next_step);
}

void MotorPLL::setUsePLL(bool use) noexcept {
    use_pll_ = use;
    if (!use) reset();
}

bool MotorPLL::isUsingPLL() const noexcept {
    return use_pll_;
}

void MotorPLL::reset() noexcept {
    angle_ = static_cast<float>(hall_state_raw_);
    angle_per_second_ = 0.0f;
    pll_integral_ = 0.0f;
    time_since_last_hall_ = 0.0f;
    measured_speed_filtered_ = 0.0f;
    has_previous_edge_ = false;
}

float MotorPLL::getAngle() const noexcept {
    return angle_;
}

float MotorPLL::getSpeedStepsSec() const noexcept {
    return angle_per_second_;
}

MotorPLL::PllInfo MotorPLL::getInfo() const noexcept {
    PllInfo info;
    info.use_pll = use_pll_;
    info.hall_state_raw = hall_state_raw_;
    info.angle = angle_;
    info.angle_per_second = angle_per_second_;
    info.pll_integral = pll_integral_;
    info.measured_speed = measured_speed_filtered_;
    info.time_since_last_hall = time_since_last_hall_;
    info.kp = pll_kp_;
    info.ki = pll_ki_;
    info.is_sync = is_sync;
    return info;
}

void MotorPLL::setGains(float kp_base, float ki_base) noexcept {
    pll_kp_ = kp_base;
    pll_ki_ = ki_base;
}

void MotorPLL::getBaseGains(float& kp_base, float& ki_base) const noexcept {
    kp_base = pll_kp_;
    ki_base = pll_ki_;
}

} // namespace libecu
