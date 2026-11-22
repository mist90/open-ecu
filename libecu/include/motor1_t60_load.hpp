/** @file motor1_t60_load.hpp @brief MOTOR_1 loaded-roller experiment: T60 current weights and rolling authority. */
#ifndef LIBECU_MOTOR1_T60_LOAD_HPP
#define LIBECU_MOTOR1_T60_LOAD_HPP
#include "profile_math.hpp"

namespace libecu {

/** @brief Additional integral authority is earned by forward motion, never by a stationary speed error. */
template<class Profile, int MaxCurrentAmps>
inline typename Profile::Current motor1RollingIntegralCeiling(typename Profile::Speed measured_rps) noexcept {
    using Speed = typename Profile::Speed;
    using Current = typename Profile::Current;
    static_assert(MaxCurrentAmps >= 12);
    // Keep 12 A below 7.5 RPM and the same slope (16 A per RPS) on both load profiles.
    // Full integral at 1/8 + (max - 12)/16 RPS: 37.5 RPM for 20 A, 131.25 RPM for 45 A.
    const Speed rolling = numericClamp(measured_rps, number_v<Speed, 1, 8>,
                                       number_v<Speed, MaxCurrentAmps - 10, 16>);
    return number_v<Current, 12> +
        multiplyTo<Current>(rolling - number_v<Speed, 1, 8>, number_v<Speed, 16>);
}

/** @brief T60's original d/q error weights. Speed is electrical Hall sectors/s, not RPM. */
template<bool QAxis>
inline FixedProfile::Current motor1T60Error(FixedProfile::Current error, FixedProfile::PllSpeed velocity) noexcept {
    using Speed = FixedProfile::PllSpeed;
    using Unit = FixedProfile::Fraction;
    const uint32_t raw = velocity.raw();
    const uint32_t speed = velocity.raw() < 0 ? 0u - raw : raw;
#if defined(LIBECU_PWM_DEADLINE_ONLY) && LIBECU_PWM_DEADLINE_ONLY
    // Both flat tails are exactly 1/16; keep the same nearest/ties-away
    // rounding without synthesizing a Q30 weight and multiplying each axis.
    if (speed <= 60u * Speed::SCALE || (QAxis && speed >= 480u * Speed::SCALE))
        return FixedProfile::Current::fromRaw(fixed_detail::roundedShift<4>(error.raw()));
#endif
    constexpr unsigned shift = Unit::FRACTION_BITS - Speed::FRACTION_BITS;
    static_assert(shift >= 7);
    int32_t weight;
    if (speed < 75u * Speed::SCALE) {
        const auto v = std::max(speed, uint32_t(60u * Speed::SCALE));
        weight = int32_t((v - 59u * Speed::SCALE) << (shift - 4));
    } else if (QAxis && speed > 360u * Speed::SCALE) {
        const auto v = std::min(speed, uint32_t(480u * Speed::SCALE));
        weight = int32_t((488u * Speed::SCALE - v) << (shift - 7));
    } else {
        return error;
    }
    // Both speed envelopes leave weight in [0,1]; the rounded error still fits Current.
    return FixedProfile::Current::fromRaw(fixed_detail::boundedRoundedProduct<Unit::FRACTION_BITS>(
        weight, error.raw()));
}
} // namespace libecu
#endif
