/** @file motor_pll_core.hpp
 * @brief Numerically parameterized Hall PLL with a compile-time sample clock
 */
#ifndef LIBECU_MOTOR_PLL_CORE_HPP
#define LIBECU_MOTOR_PLL_CORE_HPP

#include "motor_direction.hpp"
#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"
#include "control_optimization.hpp"

namespace libecu {

#if defined(LIBECU_PLL_DIAGNOSTICS) && LIBECU_PLL_DIAGNOSTICS
/** @brief Optional board-owned event counter; called only on an internal phase reseed. */
void recordPllPhaseReset(bool lost_sync) noexcept;
#endif

/** @brief Hall-sector PLL; one electrical revolution is six step units. */
template<class Profile, uint32_t FrequencyHz>
class MotorPLLCore {
    static_assert(FrequencyHz > 0 && FrequencyHz <= UINT32_MAX / 6u, "Invalid PLL sample frequency");
public:
    using Angle = typename Profile::Angle;
    using Speed = typename Profile::PllSpeed;
    using Gain = typename Profile::PllGain;
    using StepGain = typename Profile::PllStepGain;
    using Fraction = typename Profile::Fraction;
    using Integral = typename Profile::PllAccumulator;
    using Phase = typename Profile::PhaseAccumulator;
    static constexpr Angle ANGLE_MAX = number<Angle>(6);
    static constexpr Angle LIMIT_ANGLE_ERROR = number<Angle>(1, 2);
    struct PreparedGains {
        Gain kp;
        Gain ki;
        StepGain ki_step;
    };

    struct Info {
        bool use_pll;
        uint8_t hall_state_raw;
        Angle angle;
        Speed angle_per_second;
        Speed pll_integral;
        Gain kp;
        Gain ki;
        bool is_sync;
    };

    /** @param max_speed Maximum electrical speed magnitude in steps/s. */
    explicit MotorPLLCore(Speed max_speed, bool inverse = false) noexcept
        : inverse_(inverse), integral_limit_(numberCast<Integral>(max_speed)) {
        setGains(number<Gain>(100), number<Gain>(5000));
    }

    /** @brief Accept a decoded, valid Hall sector. */
    void updateHall(uint8_t hall) noexcept { if (hall <= 5) hall_ = hall; }

    /** @brief Advance exactly one sample of the PLL. */
    LIBECU_HOT_LOOP LIBECU_RAM_INLINE void updateTick() noexcept {
        const Angle error = phaseError();

        const Fraction alpha = lockFilterAlpha();
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // error and the low-pass state are in [0, 0.5] after abs(). With
            // alpha in [0.0001, 0.05], their rounded convex update stays there.
            // Neither this difference nor the final sum can saturate.
            const int32_t absolute_error = error.raw() < 0 ? -error.raw() : error.raw();
            const int32_t delta = absolute_error - lock_error_.raw();
            const int32_t increment =
                fixed_detail::boundedRoundedProduct<Fraction::FRACTION_BITS>(alpha.raw(), delta);
            lock_error_ = Angle::fromRaw(lock_error_.raw() + increment);
        } else {
            lock_error_ += multiplyTo<Angle>(alpha, numericAbs(error) - lock_error_);
        }
        bool reset_angle = false;
        if (synchronized_) {
            if (lock_error_ >= number_v<Angle, 2, 5>) {
                synchronized_ = false;
                reset_angle = true;
#if defined(LIBECU_PLL_DIAGNOSTICS) && LIBECU_PLL_DIAGNOSTICS
                recordPllPhaseReset(true);
#endif
            }
        } else {
            if (numericAbs(error) == LIMIT_ANGLE_ERROR) {
                reset_angle = true;
#if defined(LIBECU_PLL_DIAGNOSTICS) && LIBECU_PLL_DIAGNOSTICS
                recordPllPhaseReset(false);
#endif
            }
            else if (numericAbs(speed_) > number_v<Speed, 5>) synchronized_ = true;
        }

        const Integral increment = multiplyTo<Integral>(error, ki_step_);
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            constexpr int bound_bit = 31 + Integral::FRACTION_BITS - Speed::FRACTION_BITS;
            constexpr int increment_bit = 62 + Integral::FRACTION_BITS - Angle::FRACTION_BITS - StepGain::FRACTION_BITS;
            static_assert(sizeof(typename Integral::StorageType) == 8 && bound_bit < 62 && increment_bit < 62,
                          "PLL bound plus one increment must fit int64");
            integral_ = Integral::fromRaw(integral_.raw() + increment.raw());
            integral_ = Integral::fromRaw(fixed_detail::clampWideState(integral_.raw(), integral_limit_.raw()));
        } else {
            integral_ += increment;
            integral_ = numericClamp(integral_, -integral_limit_, integral_limit_);
        }
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // |error| <= 0.5; even INT32_MIN gain produces at most 2^30
            // speed raw units. The rounded product cannot require saturation.
            static_assert(Speed::FRACTION_BITS == Gain::FRACTION_BITS &&
                          Angle::FRACTION_BITS > 0, "Bounded PLL proportional product");
            const auto proportional = Speed::fromRaw(
                fixed_detail::boundedRoundedProduct<Angle::FRACTION_BITS>(error.raw(), kp_.raw()));
            // The preceding clamp is to a limit converted FROM the Speed
            // format. Rounding back to that same format cannot saturate.
            constexpr unsigned shift = Integral::FRACTION_BITS - Speed::FRACTION_BITS;
            static_assert(shift > 0 && shift < 31);
            reported_speed_ = Speed::fromRaw(int32_t(fixed_detail::roundedLowWord<shift>(integral_.raw())));
            speed_ = proportional + reported_speed_;
        } else {
            speed_ = multiplyTo<Speed>(error, kp_) + numberCast<Speed>(integral_);
        }
        if (reset_angle) {
            integral_ = numberCast<Integral>(speed_);
            if constexpr (std::is_same<Profile, FixedProfile>::value) reported_speed_ = speed_;
            phase_ = number<Phase>(hall_);
        } else {
            const Phase advance = multiplyTo<Phase>(speed_, number_v<Fraction, 1, FrequencyHz>);
            if constexpr (std::is_same<Profile, FixedProfile>::value) {
                constexpr int advance_bit = 62 + Phase::FRACTION_BITS - Speed::FRACTION_BITS - Fraction::FRACTION_BITS;
                static_assert(sizeof(typename Phase::StorageType) == 8 && advance_bit < 62 &&
                              Phase::FRACTION_BITS + 3 < 62, "Wrapped phase plus advance must fit int64");
                constexpr int64_t period = int64_t(Phase::SCALE) * 6;
                int64_t raw = phase_.raw() + advance.raw();
                constexpr int64_t max_advance = -multiplyTo<Phase>(Speed::fromRaw(INT32_MIN),
                    number_v<Fraction, 1, FrequencyHz>).raw();
                if constexpr (Phase::FRACTION_BITS == 32 && max_advance <= 2 * period) {
                    // The fractional word is unchanged by adding/subtracting a turn.
                    // An old phase in [0,6) gives -12 <= sector < 18 after the advance.
                    int32_t sector = int32_t(raw >> 32);
                    if (uint32_t(sector) >= 6) {
                        if (sector < 0) sector += sector < -6 ? 12 : 6;
                        else sector -= sector >= 12 ? 12 : 6;
                        raw = (int64_t(sector) << 32) | uint32_t(raw);
                    }
                } else if (raw >= period || raw < 0) {
                    raw %= period;
                    if (raw < 0) raw += period;
                }
                phase_ = Phase::fromRaw(raw);
            } else {
                phase_ += advance;
                phase_ = wrapValue(phase_, number_v<Phase, 6>);
            }
        }
    }

    /** @brief Commutation step with the motor-specific torque alignment. */
    uint8_t getNextHall(DriveMode mode) const noexcept {
        return getNextHall(mode, getAngle());
    }
    /** @brief Reuse the same just-sampled angle for telemetry and commutation. */
    uint8_t getNextHall(DriveMode mode, Angle angle) const noexcept {
        const int offset = commutationOffset(mode, inverse_);
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // getAngle() is in [0,6), offset is -2,0,1: the exact raw sum
            // fits int32, and its rounded sector is in [-2,7]. Keep rounding
            // AFTER adding the offset (negative half-ties round away from zero).
            const int next = use_pll_ ? roundInteger(Angle::fromRaw(
                angle.raw() + offset * int32_t(Angle::SCALE))) : int(hall_) + offset;
            return static_cast<uint8_t>(next < 0 ? next + 6 : next >= 6 ? next - 6 : next);
        }
        const int next = use_pll_ ? roundInteger(angle + number<Angle>(offset)) : int(hall_) + offset;
        return static_cast<uint8_t>((next % 6 + 6) % 6);
    }

    /** @brief Calculate the integration coefficient without accessing tracking state. */
    LIBECU_SLOW_PATH static PreparedGains prepareGains(Gain kp, Gain ki) noexcept {
        return {kp, ki, divideByInteger<StepGain>(ki, FrequencyHz)};
    }
    /** @brief Publish only the three gains, retaining phase, integrator and lock state.
     *  @note Caller serializes this bounded assignment with the sampling ISR.
     */
    void applyPrepared(const PreparedGains& prepared) noexcept {
        kp_ = prepared.kp;
        ki_ = prepared.ki;
        ki_step_ = prepared.ki_step;
    }
    /** @brief Convenience for construction or callers already owning the PLL. */
    void setGains(Gain kp, Gain ki) noexcept {
        applyPrepared(prepareGains(kp, ki));
    }
    /** @brief Re-seed tracking from the most recent real/virtual Hall sector. */
    void reset() noexcept {
        phase_ = number<Phase>(hall_);
        speed_ = Speed{};
        reported_speed_ = Speed{};
        integral_ = Integral{};
        lock_error_ = Angle{};
        synchronized_ = false;
    }
    void setUsePLL(bool enabled) noexcept { use_pll_ = enabled; if (!enabled) reset(); }
    bool isUsingPLL() const noexcept { return use_pll_; }
    LIBECU_DEADLINE_INLINE Angle getAngle() const noexcept {
        // Narrowing may round a phase just below six up to the endpoint.
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // phase_ is always wrapped into [0,6), so conversion neither
            // needs saturation nor a negative-number rounding path.
            static_assert(Phase::FRACTION_BITS > Angle::FRACTION_BITS && Angle::FRACTION_BITS + 3 <= 31,
                          "Wrapped phase must fit the public angle format");
            constexpr unsigned shift = Phase::FRACTION_BITS - Angle::FRACTION_BITS;
            const uint32_t rounded = uint32_t(phase_.raw() >> shift) + ((uint32_t(phase_.raw()) >> (shift - 1)) & 1u);
            return Angle::fromRaw(rounded == uint32_t(ANGLE_MAX.raw()) ? 0 : int32_t(rounded));
        } else {
            const Angle angle = numberCast<Angle>(phase_);
            return angle >= ANGLE_MAX ? angle - ANGLE_MAX : angle;
        }
    }
    Speed getSpeedStepsSec() const noexcept {
        if constexpr (std::is_same<Profile, FixedProfile>::value) return reported_speed_;
        else return numberCast<Speed>(integral_);
    }
    uint8_t hallState() const noexcept { return hall_; }
    void getGains(Gain& kp, Gain& ki) const noexcept { kp = kp_; ki = ki_; }
    Info getInfo() const noexcept {
        return {use_pll_, hall_, getAngle(), speed_, getSpeedStepsSec(), kp_, ki_, synchronized_};
    }

private:
    Fraction lockFilterAlpha() const noexcept {
        constexpr auto low = number_v<Fraction, 1, 10000>, high = number_v<Fraction, 1, 20>;
        constexpr auto scale = number_v<Fraction, 1, 6u * FrequencyHz>;
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            constexpr unsigned shift = Speed::FRACTION_BITS;
            constexpr uint32_t half = 1u << (shift - 1);
            constexpr bool clips_at_max = ((uint64_t(INT32_MAX) * scale.raw() + half) >> shift) >=
                                          uint32_t(high.raw());
            uint32_t magnitude = speed_.raw() < 0 ? 0u - uint32_t(speed_.raw()) : uint32_t(speed_.raw());
            // Generic numericAbs(INT_MIN) saturates to INT_MAX. Only omit this
            // correction when both possible magnitudes select the upper clamp.
            if constexpr (!clips_at_max) magnitude -= magnitude >> 31;
            const uint64_t q = (uint64_t(magnitude) * uint32_t(scale.raw()) + half) >> shift;
            if (q > uint32_t(high.raw())) return high;
            return uint32_t(q) < uint32_t(low.raw()) ? low : Fraction::fromRaw(int32_t(q));
        } else {
            return numericClamp(multiplyTo<Fraction>(numericAbs(speed_), scale), low, high);
        }
    }

    Angle phaseError() const noexcept {
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            static_assert(Angle::FRACTION_BITS + 3 <= 31, "One electrical turn must fit int32");
            // Only valid sectors enter hall_, and getAngle() is in [0,6).
            // Native subtraction/wrapping is therefore exact over [-6,6].
            int32_t raw = int32_t(hall_) * int32_t(Angle::SCALE) - getAngle().raw();
            if (raw > number_v<Angle, 3>.raw()) raw -= ANGLE_MAX.raw();
            else if (raw < -number_v<Angle, 3>.raw()) raw += ANGLE_MAX.raw();
            return Angle::fromRaw(numericClamp(raw, -LIMIT_ANGLE_ERROR.raw(), LIMIT_ANGLE_ERROR.raw()));
        } else {
            Angle error = number<Angle>(hall_) - getAngle();
            if (error > number_v<Angle, 3>) error -= ANGLE_MAX;
            else if (error < -number_v<Angle, 3>) error += ANGLE_MAX;
            return numericClamp(error, -LIMIT_ANGLE_ERROR, LIMIT_ANGLE_ERROR);
        }
    }

    uint8_t hall_ = 0;
    bool inverse_ = false;
    bool synchronized_ = false;
    bool use_pll_ = false;
    Integral integral_limit_{};
    Phase phase_{};
    Speed speed_{};
    Speed reported_speed_{};
    Integral integral_{};
    Angle lock_error_{};
    Gain kp_{};
    Gain ki_{};
    StepGain ki_step_{};
};

} // namespace libecu
#endif // LIBECU_MOTOR_PLL_CORE_HPP
