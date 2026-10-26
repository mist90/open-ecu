/** @file open_loop_phase.hpp
 * @brief PWM-clocked open-loop electrical phase with mechanical-RPS input
 */
#ifndef LIBECU_OPEN_LOOP_PHASE_HPP
#define LIBECU_OPEN_LOOP_PHASE_HPP

#include "motor_direction.hpp"
#include "numeric_ops.hpp"
#include <cstdint>
#include <limits>

namespace libecu {

/**
 * @brief Direct digital phase generator, independent of physical Hall feedback
 *
 * One unsigned 32-bit turn wraps naturally. Unlike repeated float additions,
 * this retains low-speed increments near the end of a revolution. Conversion
 * of the speed request happens only when the command changes, not in the ISR.
 */
template<class Profile, uint32_t FrequencyHz>
class OpenLoopPhase {
    static_assert(FrequencyHz > 0, "A phase generator needs a PWM clock");
public:
    using Speed = typename Profile::Speed;
    using Rate = typename Profile::PllSpeed;
    using Angle = typename Profile::Angle;

    /**
     * @brief Set mechanical RPS without changing the present phase
     * @param speed Nonnegative mechanical revolutions per second
     * @param poles Even, nonzero number of motor poles
     * @return false on invalid/unrepresentable rate; the previous rate is retained
     */
    bool setSpeed(Speed speed, uint8_t poles) noexcept {
        if (!poles || (poles & 1) || speed < Speed{}) return false;
        uint32_t increment = 0;
        if constexpr (std::is_same<Speed, float>::value) {
            if (!std::isfinite(speed)) return false;
            const float units = speed * float(poles) * (2147483648.0f / FrequencyHz);
            if (!(units < 4294967296.0f)) return false;
            increment = static_cast<uint32_t>(units + 0.5f);
        } else {
            static_assert(Speed::FRACTION_BITS <= 31, "Speed resolution exceeds the DDS conversion scale");
            constexpr uint64_t scale = uint64_t(1) << (31 - Speed::FRACTION_BITS);
            const uint64_t product = uint64_t(speed.raw()) * poles;
            if (product > (UINT64_MAX - FrequencyHz / 2) / scale) return false;
            const uint64_t units = (product * scale + FrequencyHz / 2) / FrequencyHz;
            if (units > UINT32_MAX) return false;
            increment = static_cast<uint32_t>(units);
        }
        const Rate rate = scaleInteger<Rate>(speed, int32_t(poles) * 3);
        if constexpr (!std::is_same<Rate, float>::value) {
            if (rate.raw() == std::numeric_limits<typename Rate::StorageType>::max()) return false;
        }
        increment_ = increment;
        speed_steps_s_ = rate;
        return true;
    }

    /**
     * @brief Seed from a normalized Hall/PLL angle; invalid input seeds zero
     * @param angle Electrical step units in [0,6)
     */
    void seed(Angle angle) noexcept {
        phase_ = 0;
        if (angle < Angle{} || angle >= number_v<Angle, 6>) return;
        if constexpr (std::is_same<Angle, float>::value) {
            if (!std::isfinite(angle)) return;
            const float units = angle * (4294967296.0f / 6.0f);
            if (units < 4294967296.0f) phase_ = static_cast<uint32_t>(units);
        } else {
            static_assert(sizeof(typename Angle::StorageType) <= 4, "Use a 32-bit public angle format");
            phase_ = static_cast<uint32_t>((uint64_t(angle.raw()) << 32) / (6 * uint64_t(Angle::SCALE)));
        }
    }

    /** @brief Advance one PWM sample. @param mode Direction; NEUTRAL holds phase. */
    void advance(DriveMode mode) noexcept {
        if (mode == DriveMode::FORWARD) phase_ += increment_;
        else if (mode == DriveMode::REVERSE) phase_ -= increment_;
    }

    /** @return Continuous virtual angle in electrical step units [0,6). */
    Angle angle() const noexcept {
        if constexpr (std::is_same<Angle, float>::value) {
            const float angle = float(phase_) * (6.0f / 4294967296.0f);
            return angle < 6.0f ? angle : 0.0f;
        } else {
            static_assert(Angle::FRACTION_BITS <= 32, "Unsupported angle scale");
            const uint64_t raw = (uint64_t(phase_) * 6) >> (32 - Angle::FRACTION_BITS);
            return Angle::fromRaw(static_cast<typename Angle::StorageType>(raw));
        }
    }

    /**
     * @brief Quantize the field with the same torque alignment as the Hall PLL
     * @param mode Commanded drive direction
     * @param inverse Motor-specific commutation alignment selection
     * @return Nearest six-step sector in [0,5]
     */
    uint8_t step(DriveMode mode, bool inverse) const noexcept {
        int sector = static_cast<int>((uint64_t(phase_) * 6 + (uint64_t(1) << 31)) >> 32);
        if (sector == 6) sector = 0;
        sector += commutationOffset(mode, inverse);
        if (sector < 0) sector += 6;
        else if (sector >= 6) sector -= 6;
        return static_cast<uint8_t>(sector);
    }

    /** @param mode Drive direction. @return Signed field speed in electrical steps/s. */
    Rate speed(DriveMode mode) const noexcept {
        if (mode == DriveMode::FORWARD) return speed_steps_s_;
        if (mode == DriveMode::REVERSE) return -speed_steps_s_;
        return Rate{};
    }

private:
    uint32_t phase_ = 0;
    uint32_t increment_ = 0;
    Rate speed_steps_s_{};
};

} // namespace libecu
#endif // LIBECU_OPEN_LOOP_PHASE_HPP
