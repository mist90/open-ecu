/**
 * @file current_feedforward.cpp
 * @brief Model-based feed-forward duty command - implementation
 */

#include "../include/algorithms/current_feedforward.hpp"
#include <cmath>

namespace libecu {

namespace {

inline float clampf(float v, float lo, float hi) noexcept {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/// Electrical radians per commutation step (60 degrees).
constexpr float RAD_PER_STEP = 1.04719755f;   // pi/3

/// Below this the bus reading is not trustworthy enough to divide by.
constexpr float MIN_BUS_VOLTAGE = 1.0f;

} // namespace

CurrentFeedforward::CurrentFeedforward(float pwm_frequency) noexcept
    : pwm_frequency_(pwm_frequency > 1.0f ? pwm_frequency : 1.0f)
    , params_{}
    , info_{}
    , bemf_filtered_(0.0f)
    , bemf_primed_(false)
{
    params_.enabled           = false;   // opt in explicitly
    params_.ke_v_s_per_rad    = 0.0f;
    params_.r_phase_ohm       = 0.0f;
    params_.l_phase_h         = 0.0f;
    params_.v_offset          = 0.0f;
    params_.use_observer_bemf = true;
    params_.bemf_lpf_alpha    = 0.002f;   // ~25 ms at 20 kHz; see the header
    params_.di_dt_gain        = 0.0f;    // see the note in the header
    params_.max_di_dt_duty    = 0.10f;
    params_.min_duty          = 0.0f;
    params_.max_duty          = 0.95f;
}

void CurrentFeedforward::setParameters(const CurrentFeedforwardParams& params) noexcept {
    params_ = params;
}

void CurrentFeedforward::setMotorConstants(float ke_v_s_per_rad, float r_phase_ohm,
                                           float l_phase_h, float v_offset) noexcept {
    params_.ke_v_s_per_rad = ke_v_s_per_rad;
    params_.r_phase_ohm    = r_phase_ohm;
    params_.l_phase_h      = l_phase_h;
    params_.v_offset       = v_offset;
}

void CurrentFeedforward::reset() noexcept {
    info_          = Info{};
    bemf_filtered_ = 0.0f;
    bemf_primed_   = false;
}

float CurrentFeedforward::predictBemfVolts(float speed_steps_per_sec) const noexcept {
    return params_.ke_v_s_per_rad * RAD_PER_STEP * std::fabs(speed_steps_per_sec);
}

float CurrentFeedforward::update(const CurrentFeedforwardInput& in) noexcept {
    if (!params_.enabled || in.bus_voltage < MIN_BUS_VOLTAGE) {
        info_          = Info{};
        bemf_primed_   = false;
        return 0.0f;
    }

    // ---- back-EMF ----------------------------------------------------------
    // The observer measures E directly, but only while it is receiving samples
    // - above min_duty, with a phase floating, and fast enough for the ramp to
    // be fittable.  ke * omega covers everything else, including standstill.
    float e_raw = 0.0f;
    bool  measured = false;
    if (params_.use_observer_bemf && in.bemf_valid && in.bemf_peak_volts > 0.0f) {
        e_raw    = in.bemf_peak_volts;
        measured = true;
    } else {
        e_raw = predictBemfVolts(in.speed_steps_per_sec);
    }

    // Filter it. The speed estimate carries PWM-rate PLL ripple that would
    // otherwise be amplified into the duty; see bemf_lpf_alpha in the header.
    const float alpha = clampf(params_.bemf_lpf_alpha, 0.0f, 1.0f);
    if (!bemf_primed_ || alpha <= 0.0f || alpha >= 1.0f) {
        bemf_filtered_ = e_raw;
        bemf_primed_   = true;
    } else {
        bemf_filtered_ += alpha * (e_raw - bemf_filtered_);
    }
    const float e = bemf_filtered_;

    // ---- the three plant terms, in volts across the driven pair ------------
    // Two phases in series, so every term carries a factor of 2.
    const float v_bemf      = 2.0f * e;
    const float v_resistive = 2.0f * params_.r_phase_ohm * in.target_current;

    float v_inductive = 0.0f;
    if (params_.di_dt_gain > 0.0f && params_.l_phase_h > 0.0f) {
        // Deadbeat: the volt-seconds needed to move the current to target in
        // one PWM period, scaled down by di_dt_gain and clamped below.
        v_inductive = params_.di_dt_gain * 2.0f * params_.l_phase_h *
                      (in.target_current - in.measured_current) * pwm_frequency_;
    }

    const float inv_bus = 1.0f / in.bus_voltage;
    const float d_bemf  = v_bemf * inv_bus;
    const float d_res   = v_resistive * inv_bus;
    const float d_off   = params_.v_offset * inv_bus;
    const float d_ind   = clampf(v_inductive * inv_bus,
                                 -params_.max_di_dt_duty, params_.max_di_dt_duty);

    const float raw  = d_bemf + d_res + d_ind + d_off;
    const float duty = clampf(raw, params_.min_duty, params_.max_duty);

    info_.duty           = duty;
    info_.duty_bemf      = d_bemf;
    info_.duty_resistive = d_res;
    info_.duty_inductive = d_ind;
    info_.duty_offset    = d_off;
    info_.bemf_volts     = e;
    info_.bemf_raw_volts = e_raw;
    info_.bemf_measured  = measured;
    info_.saturated      = (raw != duty);
    return duty;
}

} // namespace libecu
