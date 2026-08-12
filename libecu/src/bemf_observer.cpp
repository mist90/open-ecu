/**
 * @file bemf_observer.cpp
 * @brief BEMF zero-crossing observer implementation for sensorless 6-step BLDC
 *
 * Signal model (ON-time sensing, complementary PWM)
 * -------------------------------------------------
 * While the high-side of the driven phase conducts, one phase sits at Vbus,
 * one at 0 V and the third floats.  The motor neutral is then Vbus/2 and the
 * floating phase reads
 *
 *      V_float = Vbus/2 + e_c(theta)
 *
 * where e_c is the BEMF of the floating winding.  e_c ramps linearly from one
 * trapezoid plateau to the other across the 60-degree step and crosses zero at
 * 30 degrees - the zero-crossing (ZC).  A real Hall edge would occur another
 * 30 degrees later, which is where the synthetic Hall event has to be placed.
 *
 * Polarity normalisation
 * ----------------------
 * Which way e_c ramps alternates from step to step.  Instead of accepting
 * either edge direction (which forces two thresholds and makes the detected
 * instant depend on the BEMF amplitude), the raw error is multiplied by a
 * per-step sign so that
 *
 *      v_diff = sign(step) * (V_float - V_ref)
 *
 * is negative before the ZC and positive after it on every step and in both
 * rotation directions.  Detection is then a single "crosses zero upwards"
 * test at the true neutral, with no amplitude-dependent bias
 *
 * Sub-sample interpolation
 * ------------------------
 * At 20 kHz PWM and 2400 steps/s there are only ~8 samples per step, so
 * quantising the ZC to a whole PWM tick costs up to 7.5 electrical degrees.
 * The crossing instant is therefore linearly interpolated between the last
 * negative and the first positive sample.
 *
 * Commutation timing
 * ------------------
 * FLUX_INTEGRATE integrates v_diff from the interpolated crossing and fires
 * when the integral reaches a limit.  Because |e_c| grows with speed while the
 * 30-degree arc shrinks with speed, that integral is speed-independent, so one
 * constant covers the whole range and no speed estimate enters the loop.
 * DELAY_30DEG keeps the original speed-derived countdown for comparison.
 */

#include "../include/algorithms/bemf_observer.hpp"
#include <cmath>

namespace libecu {

namespace {

inline float clampf(float v, float lo, float hi) noexcept {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/// Steps whose flux-integral backstop fires even if the limit is mis-tuned.
/// 0.75 of a step period past the ZC is 45 electrical degrees - well past the
/// nominal 30, but still inside the step, so the PLL is never left stranded.
constexpr float INTEGRATOR_BACKSTOP_FRACTION = 0.75f;

/// Consecutive steps that must contradict the current polarity before it flips.
constexpr int32_t POLARITY_FLIP_STEPS = 4;

/// Minimum samples in a step before its slope fit is trusted.
constexpr int32_t MIN_FIT_SAMPLES = 4;

/// Electrical radians per commutation step (60 degrees).
constexpr float RAD_PER_STEP = 1.04719755f;   // pi/3

} // namespace

BemfObserver::BemfObserver(float pwm_frequency) noexcept
    : pwm_frequency_(pwm_frequency > 1.0f ? pwm_frequency : 1.0f)
    , dt_(1.0f / (pwm_frequency > 1.0f ? pwm_frequency : 1.0f))
    , params_{}
    , time_since_comm_(0.0f)
    , samples_valid_(0)
    , v_diff_prev_(0.0f)
    , confirm_count_(0)
    , pending_(false)
    , pending_integral_(0.0f)
    , zc_fraction_(0.0f)
    , zc_active_(false)
    , integrator_(0.0f)
    , delay_ticks_(0.0f)
    , fired_(false)
    , synthetic_step_(0)
    , last_step_period_(0.0f)
    , learned_limit_(0.0f)
    , polarity_(1)
    , polarity_disagree_(0)
    , sync_lost_(false)
    , sync_loss_count_(0)
    , bemf_was_active_(false)
    , last_v_float_(0.0f)
    , last_v_ref_(0.0f)
    , last_v_diff_(0.0f)
    , v_neg_last_(0.0f)
    , neg_age_(0)
    , have_neg_(false)
    , t_since_zc_(0.0f)
    , ticks_since_comm_(0)
    , fit_n_(0)
    , fit_sx_(0.0f)
    , fit_sxx_(0.0f)
    , fit_sy_(0.0f)
    , fit_sxy_(0.0f)
    , amp_peak_(0.0f)
    , amp_slope_(0.0f)
    , amp_ke_(0.0f)
    , amp_step_period_(0.0f)
    , amp_fit_samples_(0)
    , amp_valid_(false)
{
    // Defaults tuned for ON-time sensing at 20 kHz on a 24 V bus.
    params_.min_duty              = 0.15f;
    params_.transition_speed_low  = 600.0f;
    params_.transition_speed_high = 1200.0f;
    params_.is_inverse_commutation = false;

    params_.blanking_cycles   = 2.0f;
    params_.blanking_fraction = 0.20f;

    params_.zc_deadband_volts  = 0.0f;   // 0 => auto, 1% of Vbus
    params_.zc_confirm_samples = 2;
    params_.use_virtual_neutral = false;
    params_.auto_polarity       = true;
    params_.rail_margin         = 0.05f;

    params_.timing_mode        = BemfTimingMode::FLUX_INTEGRATE;
    params_.integrator_limit_vs = 0.0f;  // 0 => learn it, fall back to the 30-degree countdown
    params_.phase_advance      = 0.0f;
    params_.auto_learn_limit   = true;
    params_.learn_alpha        = 0.2f;

    params_.max_step_periods   = 2.0f;
    params_.amplitude_lpf_alpha = 0.2f;
}

void BemfObserver::setParameters(const BemfObserverParams& params) noexcept {
    params_ = params;
}

float BemfObserver::effectiveLimit() const noexcept {
    float limit = params_.integrator_limit_vs;
    if (params_.auto_learn_limit && learned_limit_ > 0.0f) {
        limit = learned_limit_;
    }
    if (limit <= 0.0f) {
        return 0.0f;  // not known yet - caller falls back to the timed countdown
    }
    return limit * (1.0f - clampf(params_.phase_advance, 0.0f, 0.9f));
}

float BemfObserver::expectedStepPeriod(float speed_steps_per_sec) const noexcept {
    const float speed = std::fabs(speed_steps_per_sec);
    if (speed >= 1.0f) {
        return 1.0f / speed;
    }
    return last_step_period_;  // 0.0f while nothing has been measured yet
}

int8_t BemfObserver::baseSlopeSign(uint8_t step, float speed_steps_per_sec) const noexcept {
    // With the CommutationController table, phase W floats on steps 0 and 3,
    // V on 1 and 4, U on 2 and 5.  Tracing each floating winding between the
    // step where it was driven UP and the step where it is driven DOWN shows
    // the BEMF falls on the even steps and rises on the odd ones.
    int8_t sign = (step & 1u) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);

    // The inverse table runs the sequence backwards, and so does REVERSE:
    // either one flips which end of the ramp the step starts from.
    if (params_.is_inverse_commutation) sign = static_cast<int8_t>(-sign);
    if (speed_steps_per_sec < 0.0f)     sign = static_cast<int8_t>(-sign);
    return sign;
}

void BemfObserver::resetStepState() noexcept {
    time_since_comm_  = 0.0f;
    samples_valid_    = 0;
    v_diff_prev_      = 0.0f;
    confirm_count_    = 0;
    pending_          = false;
    pending_integral_ = 0.0f;
    zc_active_        = false;
    integrator_       = 0.0f;
    delay_ticks_      = 0.0f;
    fired_            = false;
    v_neg_last_       = 0.0f;
    neg_age_          = 0;
    have_neg_         = false;
    t_since_zc_       = 0.0f;

    ticks_since_comm_ = 0;
    fit_n_            = 0;
    fit_sx_           = 0.0f;
    fit_sxx_          = 0.0f;
    fit_sy_           = 0.0f;
    fit_sxy_          = 0.0f;
}

void BemfObserver::accumulateFit(float v_diff_raw) noexcept {
    const float k = static_cast<float>(ticks_since_comm_);
    ++fit_n_;
    fit_sx_  += k;
    fit_sxx_ += k * k;
    fit_sy_  += v_diff_raw;
    fit_sxy_ += k * v_diff_raw;
}

void BemfObserver::finishAmplitudeFit(float step_period) noexcept {
    amp_fit_samples_ = fit_n_;
    amp_valid_       = false;

    if (fit_n_ < MIN_FIT_SAMPLES || step_period <= 0.0f) {
        return;
    }

    const float n     = static_cast<float>(fit_n_);
    const float denom = n * fit_sxx_ - fit_sx_ * fit_sx_;
    if (denom <= 0.0f) {
        return;   // all samples landed on the same tick - degenerate
    }

    // Slope in Volts per PWM tick, then per second.
    const float slope_per_tick = (n * fit_sxy_ - fit_sx_ * fit_sy_) / denom;
    const float slope          = slope_per_tick / dt_;

    // The ramp must rise: v_diff is normalised negative-before-ZC by
    // construction, so a falling fit means the polarity is wrong or the step
    // was garbage.  Drop it rather than poison the estimate.
    if (slope <= 0.0f) {
        return;
    }

    // v_diff sweeps -E .. +E across the 60-degree step, so slope = 2E/T.
    const float peak = 0.5f * slope * step_period;

    // omega_e = (pi/3) / T, so ke = E / omega_e = E * T / (pi/3).
    const float ke = peak * step_period / RAD_PER_STEP;

    const float alpha = clampf(params_.amplitude_lpf_alpha, 0.0f, 1.0f);
    if (amp_peak_ > 0.0f && alpha > 0.0f && alpha < 1.0f) {
        amp_peak_  = (1.0f - alpha) * amp_peak_  + alpha * peak;
        amp_slope_ = (1.0f - alpha) * amp_slope_ + alpha * slope;
        amp_ke_    = (1.0f - alpha) * amp_ke_    + alpha * ke;
    } else {
        amp_peak_  = peak;
        amp_slope_ = slope;
        amp_ke_    = ke;
    }
    amp_step_period_ = step_period;
    amp_valid_       = true;
}

bool BemfObserver::update(const BemfObserverInput& in) noexcept {
    time_since_comm_ += dt_;
    ++ticks_since_comm_;

    const float step_period = expectedStepPeriod(in.speed_steps_per_sec);

    // ---- neutral reference ------------------------------------------------
    // Vbus/2 comes from a different divider and a different ADC channel than
    // the phase taps, so its tolerance mismatch shows up as a fixed offset on
    // the ZC.  The virtual neutral (Vu+Vv+Vw)/3 travels the same divider path
    // and cancels that mismatch - but only where the phase dividers do not
    // clip on the phase that sits at Vbus, so it stays opt-in.
    const float v_ref = params_.use_virtual_neutral
                      ? (in.v_u + in.v_v + in.v_w) * (1.0f / 3.0f)
                      : 0.5f * in.bus_voltage;

    // The virtual neutral scales the error by 2/3 (the floating phase
    // contributes a third of its own BEMF to the mean); undo that so the
    // integrator limit means the same thing in both reference modes.
    const float gain = params_.use_virtual_neutral ? 1.5f : 1.0f;

    const int8_t sign = static_cast<int8_t>(baseSlopeSign(in.step, in.speed_steps_per_sec) * polarity_);

    // The amplitude fit uses the raw value: the deadband would flatten the
    // samples nearest the crossing and bias the fitted slope towards zero.
    float v_diff_raw = static_cast<float>(sign) * (in.v_float - v_ref) * gain;
    float v_diff     = v_diff_raw;

    // Deadband: everything inside the ADC/EMI noise floor reads as exactly
    // zero, so noise alone can neither create nor cancel a crossing.
    const float deadband = (params_.zc_deadband_volts > 0.0f)
                         ? params_.zc_deadband_volts
                         : 0.01f * in.bus_voltage;
    if (v_diff > -deadband && v_diff < deadband) {
        v_diff = 0.0f;
    }

    last_v_float_ = in.v_float;
    last_v_ref_   = v_ref;
    last_v_diff_  = v_diff;

    // ---- demagnetisation blanking -----------------------------------------
    // A fixed tick count is wrong at both ends of the speed range: too short
    // for a big motor at low speed, and longer than the whole 30-degree arc at
    // high speed.  Blank for whichever is longer - the hard demagnetisation
    // floor, or a fixed fraction of the step
    const float period_for_blanking = (last_step_period_ > 0.0f) ? last_step_period_ : step_period;
    float blank_time = params_.blanking_cycles * dt_;
    if (period_for_blanking > 0.0f) {
        const float fractional = clampf(params_.blanking_fraction, 0.0f, 0.45f) * period_for_blanking;
        if (fractional > blank_time) blank_time = fractional;
    }
    if (time_since_comm_ < blank_time) {
        return false;
    }

    // ---- loss-of-lock watchdog --------------------------------------------
    if (!fired_ && step_period > 0.0f && params_.max_step_periods > 0.0f &&
        time_since_comm_ > params_.max_step_periods * step_period) {
        if (!sync_lost_) {
            sync_lost_ = true;
            ++sync_loss_count_;   // rising edge only
        }
    }

    if (fired_) {
        // One synthetic event per commutation step. Anything after it is noise
        // as far as the PLL is concerned.
        return false;
    }

    // ---- rail guard --------------------------------------------------------
    // A floating phase pinned to 0 V or to Vbus is not measuring BEMF - it is
    // freewheeling through a body diode.  That is what demagnetisation looks
    // like, and how long it lasts depends on current and speed, so no fixed
    // blanking window covers it reliably.  Discarding railed samples outright
    // is both simpler and safer than trying to time the window
    const float rail = clampf(params_.rail_margin, 0.0f, 0.3f) * in.bus_voltage;
    if (in.v_float < rail || in.v_float > in.bus_voltage - rail) {
        return false;
    }

    ++samples_valid_;
    if (have_neg_)             ++neg_age_;
    if (pending_ || zc_active_) t_since_zc_ += dt_;

    // ---- polarity self-check ----------------------------------------------
    // The first sample after blanking is, by construction, still on the
    // pre-crossing side of the ramp.  If it reads positive step after step,
    // the slope-sign table is inverted for this motor wiring.
    if (params_.auto_polarity && samples_valid_ == 1 &&
        step_period > 0.0f && time_since_comm_ < 0.45f * step_period) {
        if (std::fabs(v_diff) > 3.0f * deadband) {
            if (v_diff > 0.0f) {
                ++polarity_disagree_;
            } else {
                polarity_disagree_ = 0;
            }
            if (polarity_disagree_ >= POLARITY_FLIP_STEPS) {
                polarity_          = static_cast<int8_t>(-polarity_);
                polarity_disagree_ = 0;
                v_diff             = -v_diff;
                v_diff_raw         = -v_diff_raw;
                last_v_diff_       = v_diff;
                // Anything already accumulated this step has the old sign.
                fit_n_ = 0; fit_sx_ = 0.0f; fit_sxx_ = 0.0f; fit_sy_ = 0.0f; fit_sxy_ = 0.0f;
            }
        }
    }

    // Feed the amplitude fit with every sample that survived the rail guard,
    // whether or not the crossing has been found yet.
    accumulateFit(v_diff_raw);

    // ---- zero-crossing detection ------------------------------------------
    if (!zc_active_) {
        if (v_diff < 0.0f) {
            // Still below the neutral. Remember it as the crossing's left
            // bracket and throw away any half-confirmed candidate: a positive
            // blip that falls back is noise, not a crossing.
            v_neg_last_       = v_diff;
            neg_age_          = 0;
            have_neg_         = true;
            pending_          = false;
            confirm_count_    = 0;
            pending_integral_ = 0.0f;
            t_since_zc_       = 0.0f;
        } else if (v_diff > 0.0f && have_neg_) {
            if (!pending_) {
                // Interpolate the crossing between the bracketing samples.
                const float span = static_cast<float>(neg_age_) * dt_;
                const float frac = v_neg_last_ / (v_neg_last_ - v_diff);  // in (0,1)
                zc_fraction_      = frac;
                t_since_zc_       = span * (1.0f - frac);
                pending_integral_ = 0.5f * v_diff * t_since_zc_;
                pending_          = true;
                confirm_count_    = 1;
            } else {
                pending_integral_ += 0.5f * (v_diff_prev_ + v_diff) * dt_;
                ++confirm_count_;
            }

            const int32_t needed = (params_.zc_confirm_samples > 0)
                                 ? static_cast<int32_t>(params_.zc_confirm_samples) : 1;
            if (confirm_count_ >= needed) {
                zc_active_  = true;
                pending_    = false;
                integrator_ = pending_integral_;
                sync_lost_  = false;

                // Ticks left to the 30-degree point, discounting the time
                // already spent confirming the crossing.
                const float target = 0.5f * (1.0f - clampf(params_.phase_advance, 0.0f, 0.9f)) * step_period;
                delay_ticks_ = (target - t_since_zc_) / dt_;
            }
        } else if (v_diff > 0.0f && !have_neg_ &&
                   samples_valid_ <= static_cast<int32_t>(params_.zc_confirm_samples > 0
                                                          ? params_.zc_confirm_samples : 1)) {
            // The very first sample after blanking is already above the
            // neutral, so the crossing happened while we were blanked: the
            // bridge is running behind the rotor.  There is no bracket left to
            // interpolate against, and no negative sample can arrive any more
            // this step, so anchor the crossing here and let the normal timing
            // pull the phase back over the next couple of steps.  Without this
            // the step times out every time and the loop latches permanently
            // behind the rotor.
            // Confirm it the same way as a normal crossing before acting: the
            // samples right after blanking are the least trustworthy ones.
            ++confirm_count_;
            pending_ = true;
            const int32_t needed_late = (params_.zc_confirm_samples > 0)
                                      ? static_cast<int32_t>(params_.zc_confirm_samples) : 1;
            if (confirm_count_ >= needed_late) {
                // Anchor the crossing at the step start, not at "now": it is
                // already in the past, so the time still owed before
                // commutation is less than a full 30 degrees.  Anchoring at
                // "now" would add another 30 degrees of lag every step and the
                // loop would run away instead of catching up.
                zc_active_   = true;
                pending_     = false;
                integrator_  = 0.0f;
                t_since_zc_  = time_since_comm_;
                zc_fraction_ = 0.0f;
                sync_lost_   = false;
                const float target = 0.5f * (1.0f - clampf(params_.phase_advance, 0.0f, 0.9f)) * step_period;
                delay_ticks_ = (target - t_since_zc_) / dt_;
            }
        }
        // v_diff == 0 (inside the deadband): hold whatever state we are in.
    } else {
        // Only positive contributions, so a noise dip cannot unwind the flux
        // already accumulated.
        const float prev = (v_diff_prev_ > 0.0f) ? v_diff_prev_ : 0.0f;
        const float now  = (v_diff > 0.0f) ? v_diff : 0.0f;
        integrator_ += 0.5f * (prev + now) * dt_;
        delay_ticks_ -= 1.0f;
    }

    v_diff_prev_ = v_diff;

    // ---- commutation instant ----------------------------------------------
    if (!zc_active_) {
        return false;
    }

    bool fire = false;
    const float limit = effectiveLimit();
    if (params_.timing_mode == BemfTimingMode::FLUX_INTEGRATE && limit > 0.0f) {
        fire = (integrator_ >= limit);
        // Backstop for a mis-tuned or not-yet-learned limit: never let a step
        // run past 45 degrees after the ZC without producing an event.
        if (!fire && step_period > 0.0f &&
            t_since_zc_ >= INTEGRATOR_BACKSTOP_FRACTION * step_period) {
            fire = true;
        }
    } else {
        // DELAY_30DEG, and the FLUX_INTEGRATE fallback while the limit is
        // still unknown.
        fire = (delay_ticks_ <= 0.0f);
    }

    if (!fire) {
        return false;
    }

    // The observer must report the next *Hall position*, not the next
    // commutation step, because the caller feeds it to MotorPLL::updateHall().
    // Commutation step C maps to Hall position H as:
    //   Non-inverse: C = (H + 1) % 6  ->  next_H = C
    //   Inverse:     C = (H + 5) % 6  ->  next_H = (C + 2) % 6
    synthetic_step_ = params_.is_inverse_commutation
                    ? static_cast<uint8_t>((in.step + 2u) % 6u)
                    : in.step;

    fired_     = true;
    zc_active_ = false;
    sync_lost_ = false;
    return true;
}

uint8_t BemfObserver::getSyntheticHallStep() const noexcept {
    return synthetic_step_;
}

void BemfObserver::onCommutation(uint8_t new_step) noexcept {
    (void)new_step;

    // Measured step duration, lightly filtered so one jittery step does not
    // move the blanking window or the watchdog.
    if (time_since_comm_ > 0.0f) {
        last_step_period_ = (last_step_period_ > 0.0f)
                          ? 0.5f * (last_step_period_ + time_since_comm_)
                          : time_since_comm_;
    }

    // Fit the BEMF ramp over the step that just ended.  Doing it here rather
    // than at the commutation event itself means every sample of the step is
    // in, at the cost of one step of latency on a slowly-varying quantity.
    finishAmplitudeFit(last_step_period_);

    // Learn the flux limit from steps the observer did *not* commutate.
    // During the hybrid handover the Hall sensors still place the commutation,
    // so the flux accumulated between our ZC and their edge is exactly the
    // 30-degree integral the observer should be aiming for.
    if (params_.auto_learn_limit && zc_active_ && !fired_ && integrator_ > 0.0f) {
        const float alpha = clampf(params_.learn_alpha, 0.0f, 1.0f);
        learned_limit_ = (learned_limit_ > 0.0f)
                       ? (1.0f - alpha) * learned_limit_ + alpha * integrator_
                       : integrator_;
    }

    resetStepState();
}

bool BemfObserver::isBemfModeActive(float speed_steps_per_sec, float duty_cycle) noexcept {
    // ON-time sensing needs a minimum conduction window for the ADC to sample in.
    if (duty_cycle < params_.min_duty) {
        bemf_was_active_ = false;
        return false;
    }
    // The PLL speed is signed (negative in REVERSE); the thresholds are magnitudes.
    const float speed = std::fabs(speed_steps_per_sec);
    if (bemf_was_active_) {
        if (speed < params_.transition_speed_low) {
            bemf_was_active_ = false;
        }
    } else {
        if (speed > params_.transition_speed_high) {
            bemf_was_active_ = true;
        }
    }
    return bemf_was_active_;
}

bool BemfObserver::shouldIgnoreHall(float speed_steps_per_sec, float duty_cycle) const noexcept {
    if (duty_cycle < params_.min_duty) {
        return false;
    }
    // Hand the loop back to the Hall sensors while the observer is not
    // producing events - it keeps running and re-takes over once it re-locks.
    if (sync_lost_) {
        return false;
    }
    if (std::fabs(speed_steps_per_sec) < params_.transition_speed_low) {
        return false;
    }
    return bemf_was_active_;
}

BemfAmplitude BemfObserver::getAmplitude() const noexcept {
    BemfAmplitude a;
    a.peak_volts         = amp_peak_;
    a.line_to_line_volts = 2.0f * amp_peak_;
    a.slope_v_per_s      = amp_slope_;
    a.ke_v_s_per_rad     = amp_ke_;
    a.step_period_s      = amp_step_period_;
    a.fit_samples        = static_cast<uint32_t>(amp_fit_samples_ > 0 ? amp_fit_samples_ : 0);
    a.valid              = amp_valid_;
    return a;
}

float BemfObserver::predictBemfVolts(float speed_steps_per_sec) const noexcept {
    if (amp_ke_ <= 0.0f) {
        return 0.0f;
    }
    return amp_ke_ * RAD_PER_STEP * std::fabs(speed_steps_per_sec);
}

BemfObserver::BemfInfo BemfObserver::getInfo() const noexcept {
    BemfInfo info;
    info.bemf_active        = bemf_was_active_;
    info.zc_detected        = zc_active_;
    info.sync_lost          = sync_lost_;
    info.sync_loss_count    = sync_loss_count_;
    info.blanked            = (samples_valid_ == 0);
    info.synthetic_step     = synthetic_step_;
    info.polarity           = polarity_;
    info.floating_voltage   = last_v_float_;
    info.v_ref              = last_v_ref_;
    info.v_diff             = last_v_diff_;
    info.integrator_vs      = integrator_;
    info.integrator_limit   = effectiveLimit();
    info.zc_fraction        = zc_fraction_;
    info.last_step_period_s = last_step_period_;
    return info;
}

} // namespace libecu
