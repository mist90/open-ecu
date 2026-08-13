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

    /* Hall wire break detection.
     * While locked and actually turning, the rotor crosses one Hall step every
     * 1/|angle_per_second_| seconds (angle_per_second_ is in steps/sec), so an
     * edge must arrive within that period.  A silent line past the deadline
     * means a broken Hall wire or a dead sensor.
     */
    if (is_sync) {
        const float speed_abs = std::abs(angle_per_second_);
        if (speed_abs > SYNC_SPEED && time_since_last_hall_ > 1.5f * (1.0f / speed_abs))
            is_hall_fault_ = true;
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
    // Step offset applied to the rotor angle to place the stator field.
    //
    // FORWARD uses +1 step: the rotor sweeps 60 degrees inside a Hall sector, so
    // the field leads it by 120..60 degrees — centred on the 90-degree torque
    // optimum.  REVERSE must produce the *opposite* torque, which means flipping
    // the field by 180 degrees, i.e. 3 steps of the 6-step table: +1 - 3 = -2.
    // Using -1 (the naive mirror of +1) leaves the field only 120 degrees from
    // the forward vector; the torque angle then sweeps 0..-60 degrees, so torque
    // collapses to zero at every sector entry.  The motor cannot self-start and
    // jerks once turning.
    float offset = 0.0f;
    if (mode == DriveMode::FORWARD)
        offset = 1.0f;
    else if (mode == DriveMode::REVERSE)
        offset = -2.0f;

    // An inverse commutation table runs the step sequence backwards, so the
    // sign of the offset flips with it.
    if (is_inverse_commutation_table_)
        offset = -offset;

    if (!use_pll_) {
        int next_step = static_cast<int>(hall_state_raw_) + static_cast<int>(offset);
        next_step = ((next_step % 6) + 6) % 6;
        return static_cast<uint8_t>(next_step);
    }

    float next_angle = angle_ + offset;

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

bool MotorPLL::isHallFault() const noexcept {
    return is_hall_fault_;
}

void MotorPLL::reset() noexcept {
    angle_ = static_cast<float>(hall_state_raw_);
    angle_per_second_ = 0.0f;
    pll_integral_ = 0.0f;
    time_since_last_hall_ = 0.0f;
}

void MotorPLL::resetHallFault() noexcept {
    is_hall_fault_ = false;
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
    info.time_since_last_hall = time_since_last_hall_;
    info.kp = pll_kp_;
    info.ki = pll_ki_;
    info.is_sync = is_sync;
    info.is_hall_fault = is_hall_fault_;
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
