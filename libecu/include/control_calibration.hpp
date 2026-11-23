/** @file control_calibration.hpp @brief Speed-dependent current-loop weights and speed-PI limits. */
#ifndef LIBECU_CONTROL_CALIBRATION_HPP
#define LIBECU_CONTROL_CALIBRATION_HPP

#include "control_optimization.hpp"
#include "numeric_ops.hpp"
#include "numeric_profiles.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace libecu {

/** @brief A linear transition between an outer weight and unity, in absolute Hall sectors/s. */
struct ErrorWeightRamp {
    uint16_t begin_steps_s{}, end_steps_s{};
    float outer_weight = 1.0f; // Unity disables this ramp.
    constexpr bool valid() const noexcept {
        return outer_weight == 1.0f || (outer_weight >= 0.0f && outer_weight < 1.0f && begin_steps_s < end_steps_s);
    }
};

struct CurrentErrorCurve {
    ErrorWeightRamp low{}, high{}; // Rise to unity, then optionally fall away from it.
    constexpr bool valid() const noexcept {
        return low.valid() && high.valid() &&
               (low.outer_weight == 1.0f || high.outer_weight == 1.0f || low.end_steps_s <= high.begin_steps_s);
    }
};

struct CurrentLoopCalibration {
    CurrentErrorCurve d{}, q{};
    bool pi_only = false; // Omit model-led Iq and decoupling on a classical PI drive.
};
inline constexpr CurrentLoopCalibration unweighted_current_loop{};

namespace calibration_detail {
template<class P>
using SpeedMagnitude = std::conditional_t<std::is_same_v<typename P::PllSpeed, float>, float, uint32_t>;

template<class P>
constexpr SpeedMagnitude<P> speedCoordinate(uint16_t steps_s) noexcept {
    if constexpr (std::is_same_v<typename P::PllSpeed, float>) return float(steps_s);
    else return uint32_t(steps_s) * P::PllSpeed::SCALE;
}

template<class P>
LIBECU_DEADLINE_INLINE inline typename P::Current scaleError(typename P::Current error,
                                                            typename P::Fraction weight) noexcept {
    if constexpr (std::is_same_v<typename P::Current, float>) return error * weight;
    // The calibrated weight stays in [0,1]; the rounded result fits the original current format.
    else return P::Current::fromRaw(fixed_detail::boundedRoundedProduct<P::Fraction::FRACTION_BITS>(
        weight.raw(), error.raw()));
}

template<class P, const CurrentLoopCalibration& Config, bool QAxis, bool Rising>
struct WeightSegment {
    using Speed = typename P::PllSpeed;
    using Unit = typename P::Fraction;
    using Current = typename P::Current;
    static constexpr auto curve = QAxis ? Config.q : Config.d;
    static constexpr auto ramp = Rising ? curve.low : curve.high;
    static constexpr bool active = ramp.outer_weight != 1.0f;
    static constexpr auto begin = speedCoordinate<P>(ramp.begin_steps_s), end = speedCoordinate<P>(ramp.end_steps_s);
    static constexpr Unit outer = constant<Unit>(ramp.outer_weight);

    static LIBECU_DEADLINE_INLINE Current tail(Current error) noexcept {
        if constexpr (!active) return error;
        if constexpr (!std::is_same_v<Current, float>) {
            // A constant dyadic tail needs only a rounded shift, including negative ties.
            constexpr uint32_t raw = outer.raw();
            if constexpr (raw == 0) return {};
            else if constexpr ((raw & (raw - 1)) == 0 && raw < Unit::SCALE)
                return Current::fromRaw(fixed_detail::roundedShift<Unit::FRACTION_BITS + 1 -
                                        numeric_detail::bitWidth(raw)>(error.raw()));
        }
        return scaleError<P>(error, outer);
    }

    static LIBECU_DEADLINE_INLINE Unit interpolate(SpeedMagnitude<P> speed) noexcept {
        const auto distance = Rising ? speed - begin : end - speed;
        if constexpr (!active) return number_v<Unit, 1>;
        else if constexpr (std::is_same_v<Speed, float>)
            return outer + distance * ((1.0f - outer) / (end - begin));
        else {
            constexpr uint32_t span = end - begin, rise = Unit::SCALE - outer.raw();
            if constexpr (rise % span == 0)
                return Unit::fromRaw(outer.raw() + distance * (rise / span)); // Exact integer slope.
            else
                return Unit::fromRaw(outer.raw() + (uint64_t(distance) * rise + span / 2) / span);
        }
    }
};
} // namespace calibration_detail

/** @brief Weight one axis's PI error, symmetrically in both directions of rotation. */
template<class P, const CurrentLoopCalibration& Config, bool QAxis>
LIBECU_DEADLINE_INLINE inline typename P::Current weightedCurrentError(typename P::Current error,
                                                                      typename P::PllSpeed velocity) noexcept {
    constexpr auto curve = QAxis ? Config.q : Config.d;
    static_assert(curve.valid(), "Invalid current-error weight or overlapping speed ramps");
    using Speed = typename P::PllSpeed;
    // Unsigned magnitude represents the most negative PLL word without saturation.
    const auto speed = [](Speed v) {
        if constexpr (std::is_same_v<Speed, float>) return std::fabs(v);
        else return v.raw() < 0 ? 0u - uint32_t(v.raw()) : uint32_t(v.raw());
    }(velocity);
    using Low = calibration_detail::WeightSegment<P, Config, QAxis, true>;
    using High = calibration_detail::WeightSegment<P, Config, QAxis, false>;
    if (Low::active && speed <= Low::begin) return Low::tail(error);
    if (High::active && speed >= High::end) return High::tail(error);
    typename P::Fraction weight;
    if (Low::active && speed < Low::end) weight = Low::interpolate(speed);
    else if (High::active && speed > High::begin) weight = High::interpolate(speed);
    else return error;
    return calibration_detail::scaleError<P>(error, weight);
}

struct RollingIntegralLimit {
    float standstill_a{}, start_rps{}, amps_per_rps{}; // Zero slope disables the rolling limit.
};

/** @brief Bound the positive integral by motion in the commanded direction and the actual current limit. */
template<class P, const RollingIntegralLimit& Limit>
inline typename P::Current rollingIntegralCeiling(typename P::Speed measured_rps,
                                                  typename P::Current maximum) noexcept {
    static_assert(Limit.standstill_a >= 0 && Limit.start_rps >= 0 && Limit.amps_per_rps >= 0);
    using Speed = typename P::Speed;
    using Current = typename P::Current;
    if constexpr (Limit.amps_per_rps == 0) return maximum;
    constexpr auto start = constant<Speed>(Limit.start_rps), slope = constant<Speed>(Limit.amps_per_rps);
    constexpr auto base = constant<Current>(Limit.standstill_a);
    return std::min(maximum, base + multiplyTo<Current>(std::max(measured_rps, start) - start, slope));
}

} // namespace libecu
#endif // LIBECU_CONTROL_CALIBRATION_HPP
