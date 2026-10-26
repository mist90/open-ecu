/** @file hall_monitor_core.hpp
 * @brief Profile-parametric Hall health monitor with integer persistence timing
 */
#ifndef LIBECU_HALL_MONITOR_CORE_HPP
#define LIBECU_HALL_MONITOR_CORE_HPP

#include "hall_fault.hpp"
#include "control_timing.hpp"
#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"

namespace libecu {

template<class Profile>
struct HallMonitorParamsT {
    uint32_t decay_time_us = 500000;
    typename Profile::Count invalid_threshold{};
    uint8_t invalid_debounce = 1;
    uint32_t invalid_persist_time_us = 300;
    typename Profile::Fraction erratic_fraction = number<typename Profile::Fraction>(35, 100);
    typename Profile::Count erratic_min_edges = number<typename Profile::Count>(10);
    bool require_drive_active = true;
};

/** @brief Health monitor, consuming at most one coalesced Hall reading per PWM tick. */
template<class Profile, uint32_t FrequencyHz>
class HallMonitorCore {
public:
    using Count = typename Profile::Count;
    using State = typename Profile::CountAccumulator;
    using Fraction = typename Profile::Fraction;
    using Params = HallMonitorParamsT<Profile>;
    using Clock = SampleClock<FrequencyHz>;
    struct Info {
        Count invalid_score;
        Fraction erratic_score;
        Count edge_accum;
        Count reversal_accum;
        uint32_t invalid_events;
        uint8_t invalid_run;
        uint32_t invalid_time_us;
        uint32_t edges;
        uint8_t last_position;
        HallFault fault;
    };

    HallMonitorCore() noexcept {
        static_assert(defaultWindowFits(), "Widen the Hall count format for this sampling rate");
        setParameters(Params{});
    }

    /** @return false if the leaky-count range cannot cover the configured event window. */
    bool setParameters(const Params& params) noexcept {
        const uint32_t tau = params.decay_time_us ? params.decay_time_us : 1000u;
        const uint64_t rate_window = uint64_t(FrequencyHz) * tau;
        if constexpr (!std::is_same<Count, float>::value) {
            constexpr uint64_t max_events = uint64_t(std::numeric_limits<typename Count::StorageType>::max())
                                           / Count::SCALE;
            if (rate_window > max_events * 1000000u) return false;
            alpha_ = numericClamp(number<Fraction>(1000000, static_cast<int64_t>(rate_window)),
                                  Fraction{}, number<Fraction>(1));
        } else {
            alpha_ = 1000000.0f / (float(FrequencyHz) * tau);
        }
        keep_ = numericClamp(number<Fraction>(1) - alpha_, Fraction{}, number<Fraction>(1));
        params_ = params;
        persistence_ticks_ = Clock::ticksFromUs(params.invalid_persist_time_us);
        return true;
    }

    void tick(bool active) noexcept {
        decay(invalid_score_);
        decay(edges_accum_);
        decay(reversals_accum_);
        if (last_invalid_ && (active || !params_.require_drive_active)) {
            if (invalid_ticks_ < UINT32_MAX) ++invalid_ticks_;
            if (persistence_ticks_ && invalid_ticks_ >= persistence_ticks_ && fault_ == HallFault::NONE)
                fault_ = HallFault::INVALID_CODE;
        }
        evaluate();
    }

    void onPosition(uint8_t position, bool active) noexcept {
        const bool accumulate = active || !params_.require_drive_active;
        if (position > 5) {
            last_invalid_ = true;
            if (invalid_run_ < 255) ++invalid_run_;
            const uint8_t need = params_.invalid_debounce ? params_.invalid_debounce : 1;
            if (!invalid_counted_ && invalid_run_ >= need) {
                invalid_counted_ = true;
                ++invalid_events_;
                if (accumulate) invalid_score_ += number_v<State, 1>;
            }
        } else {
            last_invalid_ = false;
            invalid_run_ = 0;
            invalid_ticks_ = 0;
            invalid_counted_ = false;
            if (!have_last_) {
                have_last_ = true;
                last_position_ = position;
            } else if (position != last_position_) {
                int delta = (int(position) - last_position_ + 6) % 6;
                if (delta > 3) delta -= 6;
                last_position_ = position;
                ++edges_;
                const int8_t sign = delta > 0 ? 1 : -1;
                if (accumulate) {
                    edges_accum_ += number_v<State, 1>;
                    if (last_sign_ && last_sign_ != sign) reversals_accum_ += number_v<State, 1>;
                }
                last_sign_ = sign;
            }
        }
        evaluate();
    }

    void clearFault() noexcept {
        fault_ = HallFault::NONE;
        invalid_score_ = edges_accum_ = reversals_accum_ = State{};
        last_sign_ = 0;
        invalid_run_ = 0;
        invalid_ticks_ = 0;
        invalid_counted_ = false;
    }
    void reset() noexcept {
        clearFault();
        have_last_ = false;
        last_position_ = 0;
        last_invalid_ = false;
        invalid_events_ = edges_ = 0;
    }
    bool isFaulted() const noexcept { return fault_ != HallFault::NONE; }
    HallFault faultCause() const noexcept { return fault_; }
    const Params& getParameters() const noexcept { return params_; }
    Info getInfo() const noexcept {
        const Count edges = numberCast<Count>(edges_accum_);
        const Count reversals = numberCast<Count>(reversals_accum_);
        const Fraction erratic = edges > Count{} && edges >= params_.erratic_min_edges
                               ? divideTo<Fraction>(reversals, edges) : Fraction{};
        return {numberCast<Count>(invalid_score_), erratic, edges,
                numberCast<Count>(reversals_accum_), invalid_events_, invalid_run_,
                Clock::ticksToUs(invalid_ticks_), edges_, last_position_, fault_};
    }

private:
    static constexpr bool defaultWindowFits() noexcept {
        if constexpr (std::is_same<Count, float>::value) return true;
        else return uint64_t(FrequencyHz) <=
                    2u * (uint64_t(std::numeric_limits<typename Count::StorageType>::max()) / Count::SCALE);
    }
    void decay(State& state) noexcept {
        if constexpr (std::is_same<State, float>::value) {
            state *= keep_;
        } else {
            if (state == State{}) return;
            // Retain fractional leakage below the public count format's LSB.
            const State leak = multiplyTo<State>(alpha_, numberCast<Count>(state));
            // Both operands are nonnegative, so their signed difference cannot
            // overflow. Avoid generic saturating 64-bit subtraction here.
            state = State::fromRaw(state.raw() - leak.raw());
            if (state < State{}) state = State{};
        }
    }
    void evaluate() noexcept {
        if (fault_ != HallFault::NONE) return;
        if (params_.invalid_threshold > Count{} &&
            invalid_score_ >= numberCast<State>(params_.invalid_threshold)) fault_ = HallFault::INVALID_CODE;
        else if (params_.erratic_fraction > Fraction{}) {
            const Count edges = numberCast<Count>(edges_accum_);
            const Count reversals = numberCast<Count>(reversals_accum_);
            if (edges < params_.erratic_min_edges || edges <= Count{} || reversals <= Count{}) return;
            // Cross products keep fault detection exact without a division in every ISR.
            bool erratic = false;
            if constexpr (std::is_same<Count, float>::value) {
                erratic = reversals >= params_.erratic_fraction * edges;
            } else {
                erratic = int64_t(reversals.raw()) * int64_t(Fraction::SCALE) >=
                           int64_t(params_.erratic_fraction.raw()) * edges.raw();
            }
            if (erratic) fault_ = HallFault::ERRATIC_SEQUENCE;
        }
    }

    Params params_{};
    Fraction alpha_{};
    Fraction keep_{};
    uint32_t persistence_ticks_ = 0;
    State invalid_score_{};
    State edges_accum_{};
    State reversals_accum_{};
    uint32_t invalid_ticks_ = 0;
    uint32_t invalid_events_ = 0;
    uint32_t edges_ = 0;
    uint8_t invalid_run_ = 0;
    uint8_t last_position_ = 0;
    int8_t last_sign_ = 0;
    bool have_last_ = false;
    bool last_invalid_ = false;
    bool invalid_counted_ = false;
    HallFault fault_ = HallFault::NONE;
};

} // namespace libecu
#endif // LIBECU_HALL_MONITOR_CORE_HPP
