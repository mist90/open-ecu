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

/** @brief Work due on one SysTick. Service includes the coincident speed PI step. */
enum class ControlTick : uint8_t { None, SpeedPiOnly, Service };

/** @brief Board-supplied current, slow service (SpeedHz), and speed PI clocks. */
template<uint32_t CurrentHz, uint32_t SpeedHz, uint32_t SpeedPiHz = SpeedHz, uint32_t SysTickHz = 1000>
struct ControlTiming {
    using CurrentClock = SampleClock<CurrentHz>;
    using SpeedClock = SampleClock<SpeedHz>;
    using SpeedPiClock = SampleClock<SpeedPiHz>;
    static_assert(CurrentHz > 0 && SpeedHz > 0 && SpeedPiHz > 0, "Control frequencies must be positive");
    static_assert(SpeedHz && SpeedPiHz % SpeedHz == 0, "Every service tick must be a speed PI tick");
    static constexpr uint32_t CURRENT_HZ = CurrentHz;
    static constexpr uint32_t SPEED_HZ = SpeedHz;
    static constexpr uint32_t SPEED_PI_HZ = SpeedPiHz;
    static constexpr uint32_t SYSTICK_HZ = SysTickHz;

    /** @brief Advance a zero-initialized SysTick counter; wrap on each slow service tick. */
    static constexpr ControlTick nextSysTick(uint32_t& phase) noexcept {
        static_assert(SysTickHz && SpeedPiHz && SysTickHz % SpeedPiHz == 0,
                      "Speed PI frequency must divide the board's SysTick frequency");
        if (++phase >= SysTickHz / SpeedHz) {
            phase = 0;
            return ControlTick::Service;
        }
        if constexpr (SpeedPiHz != SpeedHz) {
            if (phase % (SysTickHz / SpeedPiHz) == 0) return ControlTick::SpeedPiOnly;
        }
        return ControlTick::None;
    }
};

} // namespace libecu
#endif // LIBECU_CONTROL_TIMING_HPP
