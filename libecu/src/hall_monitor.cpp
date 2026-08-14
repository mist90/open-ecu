/**
 * @file hall_monitor.cpp
 * @brief Hall sensor health monitor - implementation
 */

#include "../include/algorithms/hall_monitor.hpp"
#include <cmath>

namespace libecu {

namespace {

/// Signed cyclic distance between two Hall positions, in steps, range [-2, +3].
inline int stepDelta(uint8_t from, uint8_t to) noexcept {
    int d = static_cast<int>(to) - static_cast<int>(from);
    d = ((d % 6) + 6) % 6;        // 0..5
    if (d > 3) d -= 6;            // -2..3
    return d;
}

} // namespace

HallMonitor::HallMonitor(float pwm_frequency) noexcept
    : dt_(1.0f / (pwm_frequency > 1.0f ? pwm_frequency : 1.0f))
    , params_{}
    , invalid_score_(0.0f)
    , erratic_score_(0.0f)
    , edge_accum_(0.0f)
    , advance_accum_(0.0f)
    , last_position_(0)
    , have_last_(false)
    , invalid_run_(0)
    , invalid_time_(0.0f)
    , last_invalid_(false)
    , invalid_counted_(false)
    , invalid_events_(0)
    , edges_(0)
    , fault_(HallFault::NONE)
{
    // Sized for a 40-pole motor at 20 kHz.  With decay_time_s = 0.5:
    //  - a stuck line at 600 steps/s puts one illegal code per electrical
    //    revolution, i.e. ~100/s, settling the accumulator near 50
    //  - an isolated glitch or two peaks at 2 and fades
    //  - sustained chatter above 40 non-advancing edges/s trips the erratic
    //    detector, while a burst of fewer than 20 does not
    params_.decay_time_s           = 0.5f;
    params_.invalid_threshold      = 0.0f;    // disabled - see the header
    params_.invalid_debounce       = 1;       // readings are already deferred
    params_.invalid_persist_time_s = 0.0003f; // 300 us standing illegal code
    params_.erratic_fraction       = 0.35f;   // healthy under load measures ~0.06
    params_.erratic_min_edges      = 10.0f;
    params_.require_drive_active   = true;
}

void HallMonitor::setParameters(const HallMonitorParams& params) noexcept {
    params_ = params;
}

void HallMonitor::clearFault() noexcept {
    fault_           = HallFault::NONE;
    invalid_score_   = 0.0f;
    erratic_score_   = 0.0f;
    edge_accum_      = 0.0f;
    advance_accum_   = 0.0f;
    invalid_run_     = 0;
    invalid_time_    = 0.0f;
    invalid_counted_ = false;
}

void HallMonitor::reset() noexcept {
    clearFault();
    last_position_  = 0;
    have_last_      = false;
    last_invalid_   = false;
    invalid_events_ = 0;
    edges_          = 0;
}

void HallMonitor::tick(bool drive_active) noexcept {
    // The leak is what separates a burst from a breakdown, so it has to run on
    // the PWM time base rather than on Hall events - those arrive at the
    // commutation rate, which is exactly what is unreliable when something is
    // broken.
    const float tau  = (params_.decay_time_s > 0.0f) ? params_.decay_time_s : 1e-3f;
    const float keep = 1.0f - (dt_ / tau);
    const float decay = (keep > 0.0f) ? keep : 0.0f;
    invalid_score_ *= decay;
    edge_accum_    *= decay;
    advance_accum_ *= decay;

    const bool accumulate = drive_active || !params_.require_drive_active;

    // Age a standing illegal code.  A disconnected loom yields one edge as the
    // lines settle and then silence, so without this it would never be caught.
    if (last_invalid_ && accumulate) {
        invalid_time_ += dt_;
        if (params_.invalid_persist_time_s > 0.0f &&
            invalid_time_ >= params_.invalid_persist_time_s &&
            fault_ == HallFault::NONE) {
            fault_ = HallFault::INVALID_CODE;
        }
    }

    evaluate();
}

void HallMonitor::onPosition(uint8_t position, bool drive_active) noexcept {
    const bool accumulate = drive_active || !params_.require_drive_active;

    if (position == HALL_POSITION_INVALID) {
        last_invalid_ = true;
        if (invalid_run_ < 255) ++invalid_run_;
        const uint8_t need = (params_.invalid_debounce > 0) ? params_.invalid_debounce : 1;
        if (!invalid_counted_ && invalid_run_ >= need) {
            invalid_counted_ = true;
            ++invalid_events_;
            if (accumulate) invalid_score_ += 1.0f;
        }
        // An illegal code carries no position, so the edge bookkeeping is left
        // alone - the next valid reading continues from the last valid one.
    } else {
        last_invalid_    = false;
        invalid_run_     = 0;
        invalid_time_    = 0.0f;
        invalid_counted_ = false;

        if (!have_last_) {
            last_position_ = position;
            have_last_     = true;
        } else if (position != last_position_) {
            const int d = stepDelta(last_position_, position);
            last_position_ = position;
            ++edges_;
            if (accumulate) {
                edge_accum_    += 1.0f;
                advance_accum_ += static_cast<float>(d);
            }
        }
    }

    evaluate();
}

void HallMonitor::evaluate() noexcept {
    // Healthy motion advances one step per edge in a consistent direction, so
    // edge_accum_ == |advance_accum_|.  Chatter piles up edges that cancel, and
    // the surplus is the fault signal.  No direction input is needed.
    // Express it as a *fraction* of all edges, not a count: the count scales
    // with the edge rate, so any absolute threshold is really a speed limit.
    const float surplus = edge_accum_ - std::fabs(advance_accum_);
    erratic_score_ = (edge_accum_ >= params_.erratic_min_edges && surplus > 0.0f)
                   ? (surplus / edge_accum_)
                   : 0.0f;

    // A threshold of zero disables that detector.
    if (fault_ == HallFault::NONE) {
        if (params_.invalid_threshold > 0.0f &&
            invalid_score_ >= params_.invalid_threshold) {
            fault_ = HallFault::INVALID_CODE;
        } else if (params_.erratic_fraction > 0.0f &&
                   erratic_score_ >= params_.erratic_fraction) {
            fault_ = HallFault::ERRATIC_SEQUENCE;
        }
    }
}

HallMonitor::Info HallMonitor::getInfo() const noexcept {
    Info info;
    info.invalid_score  = invalid_score_;
    info.erratic_score  = erratic_score_;
    info.edge_accum     = edge_accum_;
    info.advance_accum  = advance_accum_;
    info.invalid_events = invalid_events_;
    info.invalid_run    = invalid_run_;
    info.invalid_time_s = invalid_time_;
    info.edges          = edges_;
    info.last_position  = last_position_;
    info.fault          = fault_;
    return info;
}

} // namespace libecu
