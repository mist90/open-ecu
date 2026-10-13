/**
 * @file control_timing.hpp
 * @brief Integer, compile-time control clocks and duration conversions
 */
#ifndef LIBECU_CONTROL_TIMING_HPP
#define LIBECU_CONTROL_TIMING_HPP

#include <cstdint>
#include <limits>

namespace libecu {

/** @brief Sampling clock; durations are rounded up to complete samples. */
template<uint32_t FrequencyHz>
struct SampleClock {
    static_assert(FrequencyHz > 0, "Sampling frequency must be positive");
    static constexpr uint32_t FREQUENCY_HZ = FrequencyHz;

    /** @return Number of samples covering the requested duration. */
    static constexpr uint32_t ticksFromUs(uint32_t duration_us) noexcept {
        const uint64_t ticks = (uint64_t(duration_us) * FrequencyHz + 999999u) / 1000000u;
        return ticks > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(ticks);
    }

    /** @return Elapsed microseconds, saturated to the telemetry counter range. */
    static constexpr uint32_t ticksToUs(uint32_t ticks) noexcept {
        const uint64_t us = uint64_t(ticks) * 1000000u / FrequencyHz;
        return us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(us);
    }
};

/** @brief Board-supplied sampling rates, independent of the number representation. */
template<uint32_t CurrentHz, uint32_t SpeedHz>
struct ControlTiming {
    using CurrentClock = SampleClock<CurrentHz>;
    using SpeedClock = SampleClock<SpeedHz>;
    static_assert(CurrentHz > 0 && SpeedHz > 0, "Control frequencies must be positive");
    static constexpr uint32_t CURRENT_HZ = CurrentHz;
    static constexpr uint32_t SPEED_HZ = SpeedHz;
};

} // namespace libecu
#endif // LIBECU_CONTROL_TIMING_HPP
