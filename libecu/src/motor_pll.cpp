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
    hall_state_raw_ = hall_state;

    int8_t diff = (hall_state - (hall_state_accumulated_ % 6) + 9) % 6 - 3;
    int8_t next_state = int8_t(hall_state_accumulated_) + diff;

    if (next_state < 0) {
        hall_state_accumulated_ = next_state + ANGLE_MAX;
    } else if (next_state >= ANGLE_MAX) {
        hall_state_accumulated_ = next_state % ANGLE_MAX;
    } else {
        hall_state_accumulated_ = (uint8_t)next_state;
    }

    time_since_last_hall_ = 0;
}

void MotorPLL::updateTick() noexcept {
    /* PID */
    float angle_error = fmodf(static_cast<float>(hall_state_accumulated_) - angle_, static_cast<float>(ANGLE_MAX));

    if (angle_error > static_cast<float>(ANGLE_MAX)/2.0f)
        angle_error -= static_cast<float>(ANGLE_MAX);
    if (angle_error < -static_cast<float>(ANGLE_MAX)/2.0f)
        angle_error += static_cast<float>(ANGLE_MAX);

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
        return;
    }

    angle_ += angle_per_second_ * DT_;

    angle_ = fmodf(angle_, static_cast<float>(ANGLE_MAX));
    if (angle_ < 0.0f) {
        angle_ += static_cast<float>(ANGLE_MAX);
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
    angle_ = static_cast<float>(hall_state_accumulated_);
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

MotorPLL::PllInfo MotorPLL::getInfo() const noexcept {
    PllInfo info;
    info.use_pll = use_pll_;
    info.hall_state_raw = hall_state_raw_;
    info.hall_state_accumulated = hall_state_accumulated_;
    info.angle = angle_;
    info.angle_per_second = angle_per_second_;
    info.pll_integral = pll_integral_;
    info.time_since_last_hall = time_since_last_hall_;
    info.kp = pll_kp_;
    info.ki = pll_ki_;
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
