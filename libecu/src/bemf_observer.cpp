/**
 * @file bemf_observer.cpp
 * @brief BEMF zero-crossing observer implementation for sensorless 6-step BLDC
 *
 * Zero-crossing detection principle (UM3042 §4.1.2):
 *  - The floating phase BEMF crosses Vbus/2 mid-step (30° after step start)
 *  - After detecting ZC, wait another 30° before firing commutation
 *  - 30° delay = 0.5 * step_period = 0.5 * pwm_frequency / speed_steps_per_sec ticks
 */

#include "../include/algorithms/bemf_observer.hpp"

namespace libecu {

BemfObserver::BemfObserver(float pwm_frequency) noexcept
    : pwm_frequency_(pwm_frequency)
    , params_{10.0f, 0.03f, 0.005f, 600.0f, 1200.0f, false}
    , blanking_counter_(0.0f)
    , zc_detected_(false)
    , delay_counter_(0.0f)
    , event_pending_(false)
    , synthetic_step_(0)
    , signal_high_(false)
    , need_reinit_(true)
    , last_floating_voltage_(0.0f)
    , bemf_was_active_(false)
{
}

void BemfObserver::setParameters(const BemfObserverParams& params) noexcept {
    params_ = params;
}

bool BemfObserver::update(float floating_voltage, float bus_voltage,
                          uint8_t current_step, float speed_steps_per_sec) noexcept {
    last_floating_voltage_ = floating_voltage;
    event_pending_ = false;

    // Phase 1: Demagnetization blanking
    if (blanking_counter_ > 0.0f) {
        blanking_counter_ -= 1.0f;
        return false;
    }

    // Phase 2: Waiting for 30° delay after ZC detection
    if (zc_detected_) {
        delay_counter_ -= 1.0f;
        if (delay_counter_ <= 0.0f) {
            event_pending_ = true;
            zc_detected_ = false;
            return true;
        }
        return false;
    }

    // Phase 3: Zero-crossing detection with hysteresis
    float threshold_high = bus_voltage * params_.zc_threshold_high;
    float threshold_low = bus_voltage * params_.zc_threshold_low;

    // First sample after blanking: initialize state without triggering edge
    if (need_reinit_) {
        need_reinit_ = false;
        signal_high_ = (floating_voltage > threshold_high);
        return false;
    }

    bool zc_event = false;
    if (!signal_high_ && floating_voltage > threshold_high) {
        // Rising edge through hysteresis — ZC detected
        signal_high_ = true;
        zc_event = true;
    } else if (signal_high_ && floating_voltage < threshold_low) {
        // Falling edge through hysteresis — ZC detected
        signal_high_ = false;
        zc_event = true;
    }
    // Else: in hysteresis band or no transition — no state change

    if (zc_event) {
        zc_detected_ = true;

        // Compute the next Hall position that the real Hall sensor would report
        // after the rotor advances one step. The caller feeds this to
        // MotorPLL::updateHall(), so it must be a Hall position, not a
        // commutation step. The mapping between commutation step C and Hall
        // position H depends on the commutation table direction:
        //   Non-inverse: C = (H + 1) % 6  ->  H = (C + 5) % 6  ->  next_H = C
        //   Inverse:     C = (H + 5) % 6  ->  H = (C + 1) % 6  ->  next_H = (C + 2) % 6
        if (params_.is_inverse_commutation) {
            synthetic_step_ = (current_step + 2) % 6;
        } else {
            synthetic_step_ = current_step;
        }

        // Compute 30° delay in PWM ticks
        // 30° = half of one step period (one step = 60°)
        // step_period = 1.0 / speed_steps_per_sec seconds (time for ONE step)
        // half_step = 0.5 / speed_steps_per_sec seconds
        // delay_ticks = half_step * pwm_frequency = 0.5 * pwm_frequency / speed_steps_per_sec
        if (speed_steps_per_sec >= 1.0f) {
            delay_counter_ = 0.5f * pwm_frequency_ / speed_steps_per_sec;
        } else {
            // Speed too low — cannot compute meaningful delay
            zc_detected_ = false;
            return false;
        }
    }

    return false;
}

uint8_t BemfObserver::getSyntheticHallStep() const noexcept {
    return synthetic_step_;
}

void BemfObserver::onCommutation(uint8_t new_step) noexcept {
    (void)new_step;
    blanking_counter_ = params_.blanking_cycles;
    zc_detected_ = false;
    delay_counter_ = 0.0f;
    event_pending_ = false;
    need_reinit_ = true;
}

bool BemfObserver::isBemfModeActive(float speed_steps_per_sec) const noexcept {
    // Hysteresis: use high threshold when transitioning Hall→BEMF,
    //             use low threshold when transitioning BEMF→Hall
    if (bemf_was_active_) {
        // Currently in BEMF mode — drop out at low threshold
        if (speed_steps_per_sec < params_.transition_speed_low) {
            bemf_was_active_ = false;
        }
    } else {
        // Currently in Hall mode — enter BEMF at high threshold
        if (speed_steps_per_sec > params_.transition_speed_high) {
            bemf_was_active_ = true;
        }
    }
    return bemf_was_active_;
}

bool BemfObserver::shouldIgnoreHall(float speed_steps_per_sec) const noexcept {
    return speed_steps_per_sec > params_.transition_speed_high;
}

BemfObserver::BemfInfo BemfObserver::getInfo() const noexcept {
    BemfInfo info;
    info.bemf_active = bemf_was_active_;
    info.zc_detected = zc_detected_;
    info.blanking_counter = blanking_counter_;
    info.synthetic_step = synthetic_step_;
    info.floating_voltage = last_floating_voltage_;
    return info;
}

} // namespace libecu
