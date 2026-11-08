/** @file hall_monitor_core.hpp
 * @brief Profile-parametric Hall health monitor with integer persistence timing
 */
#ifndef LIBECU_HALL_MONITOR_CORE_HPP
#define LIBECU_HALL_MONITOR_CORE_HPP

#include "hall_fault.hpp"
#include "control_timing.hpp"
#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"
#include "control_optimization.hpp"

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
    struct PreparedParameters {
        Params params;
        Fraction alpha;
        Fraction keep;
        uint32_t persistence_ticks;
    };
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

    /** @brief Validate and calculate coefficients without accessing live monitor state.
     *  @return false if the leaky-count range cannot cover the configured event window.
     */
    LIBECU_SLOW_PATH static bool prepareParameters(const Params& params, PreparedParameters& prepared) noexcept {
        const uint32_t tau = params.decay_time_us ? params.decay_time_us : 1000u;
        const uint64_t rate_window = uint64_t(FrequencyHz) * tau;
        Fraction alpha{};
        if constexpr (!std::is_same<Count, float>::value) {
            constexpr uint64_t max_events = uint64_t(std::numeric_limits<typename Count::StorageType>::max())
                                           / Count::SCALE;
            if (rate_window > max_events * 1000000u) return false;
            alpha = numericClamp(number<Fraction>(1000000, static_cast<int64_t>(rate_window)),
                                 Fraction{}, number<Fraction>(1));
        } else {
            alpha = 1000000.0f / (float(FrequencyHz) * tau);
        }
        prepared = {params, alpha, numericClamp(number<Fraction>(1) - alpha, Fraction{}, number<Fraction>(1)),
                    Clock::ticksFromUs(params.invalid_persist_time_us)};
        return true;
    }

    /** @brief Publish configuration only; retain every event, score, timer and fault.
     *  @note Caller serializes this bounded assignment with the sampling ISR.
     */
    void applyPrepared(const PreparedParameters& prepared) noexcept {
        params_ = prepared.params;
        alpha_ = prepared.alpha;
        keep_ = prepared.keep;
        persistence_ticks_ = prepared.persistence_ticks;
    }

    /** @brief Convenience for construction or callers already owning the monitor. */
    bool setParameters(const Params& params) noexcept {
        PreparedParameters prepared;
        if (!prepareParameters(params, prepared)) return false;
        applyPrepared(prepared);
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
                if (accumulate) addEvent(invalid_score_);
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
                // Both positions are in [0, 5], so one correction replaces
                // modulo-six division while retaining the positive half-turn.
                int delta = int(position) - last_position_;
                if (delta < 0) delta += 6;
                last_position_ = position;
                ++edges_;
                const int8_t sign = delta <= 3 ? 1 : -1;
                if (accumulate) {
                    addEvent(edges_accum_);
                    if (last_sign_ && last_sign_ != sign) addEvent(reversals_accum_);
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
        const Count edges = toCount(edges_accum_);
        const Count reversals = toCount(reversals_accum_);
        const Fraction erratic = edges > Count{} && edges >= params_.erratic_min_edges
                               ? divideTo<Fraction>(reversals, edges) : Fraction{};
        return {toCount(invalid_score_), erratic, edges,
                reversals, invalid_events_, invalid_run_,
                Clock::ticksToUs(invalid_ticks_), edges_, last_position_, fault_};
    }

private:
    static constexpr bool defaultWindowFits() noexcept {
        if constexpr (std::is_same<Count, float>::value) return true;
        else return uint64_t(FrequencyHz) <=
                    2u * (uint64_t(std::numeric_limits<typename Count::StorageType>::max()) / Count::SCALE);
    }
    static Count toCount(State state) noexcept {
        if constexpr (std::is_same<State, float>::value || std::is_same<Count, float>::value) {
            return numberCast<Count>(state);
        } else if constexpr (State::FRACTION_BITS > Count::FRACTION_BITS) {
            // Hall scores are nonnegative. An unsigned bias cannot overflow,
            // even at the signed accumulator limit; retain public saturation
            // rather than assuming the configured decay bounds the state.
            using Raw = typename Count::StorageType;
            using Wide = std::make_unsigned_t<typename State::StorageType>;
            constexpr unsigned shift = State::FRACTION_BITS - Count::FRACTION_BITS;
            constexpr Wide half = Wide(1) << (shift - 1u);
            const Wide biased = Wide(state.raw()) + half;
            constexpr unsigned limit_bit = shift + std::numeric_limits<Raw>::digits;
            if constexpr (limit_bit < std::numeric_limits<Wide>::digits) {
                if (biased >> limit_bit) return Count::fromRaw(std::numeric_limits<Raw>::max());
            }
            return Count::fromRaw(Raw(biased >> shift));
        } else {
            return numberCast<Count>(state);
        }
    }
    static void addEvent(State& state) noexcept {
        if constexpr (std::is_same<State, float>::value) {
            state += number_v<State, 1>;
        } else {
            using Raw = typename State::StorageType;
            using Unsigned = std::make_unsigned_t<Raw>;
            // The sum of two nonnegative signed values fits unsigned storage.
            // This also preserves saturation at the full accumulator limit.
            const Unsigned sum = Unsigned(state.raw()) + Unsigned(number_v<State, 1>.raw());
            constexpr Raw maximum = std::numeric_limits<Raw>::max();
            state = State::fromRaw(sum > Unsigned(maximum) ? maximum : Raw(sum));
        }
    }
    void decay(State& state) noexcept {
        if constexpr (std::is_same<State, float>::value) {
            state *= keep_;
        } else {
            if (state == State{}) return;
            // Retain fractional leakage below the public count format's LSB.
            const State leak = multiplyTo<State>(alpha_, toCount(state));
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
            const Count reversals = toCount(reversals_accum_);
            // The usual unidirectional trace has no reversals. Avoid narrowing
            // its wide edge accumulator on every PWM tick in that case.
            if (reversals <= Count{}) return;
            const Count edges = toCount(edges_accum_);
            if (edges < params_.erratic_min_edges || edges <= Count{}) return;
            // Cross products keep fault detection exact without a division in every ISR.
            bool erratic = false;
            if constexpr (std::is_same<Count, float>::value) {
                erratic = reversals >= params_.erratic_fraction * edges;
            } else if constexpr (sizeof(typename Count::StorageType) <= 4 &&
                                 sizeof(typename Fraction::StorageType) <= 4) {
                // The guards above make every operand positive. Use the native
                // word product to avoid sign-word products on 32-bit CPUs.
                erratic = uint64_t(uint32_t(reversals.raw())) * Fraction::SCALE >=
                           uint64_t(fixed_detail::wideProduct(params_.erratic_fraction.raw(), edges.raw()));
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
    // A single-byte latch is also read by foreground drive-inhibit checks.
    // Compound score snapshots still require the controller's critical section.
    volatile HallFault fault_ = HallFault::NONE;
};

} // namespace libecu
#endif // LIBECU_HALL_MONITOR_CORE_HPP
