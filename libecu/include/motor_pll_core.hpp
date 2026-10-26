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
    LIBECU_HOT_LOOP void updateTick() noexcept {
        Angle error = number<Angle>(hall_) - getAngle();
        if (error > number_v<Angle, 3>) error -= ANGLE_MAX;
        else if (error < -number_v<Angle, 3>) error += ANGLE_MAX;
        error = numericClamp(error, -LIMIT_ANGLE_ERROR, LIMIT_ANGLE_ERROR);

        const Fraction alpha = numericClamp(
            multiplyTo<Fraction>(numericAbs(speed_), number_v<Fraction, 1, 6u * FrequencyHz>),
            number_v<Fraction, 1, 10000>, number_v<Fraction, 1, 20>);
        lock_error_ += multiplyTo<Angle>(alpha, numericAbs(error) - lock_error_);
        bool reset_angle = false;
        if (synchronized_) {
            if (lock_error_ >= number_v<Angle, 2, 5>) {
                synchronized_ = false;
                reset_angle = true;
            }
        } else {
            if (numericAbs(error) == LIMIT_ANGLE_ERROR) reset_angle = true;
            else if (numericAbs(speed_) > number_v<Speed, 5>) synchronized_ = true;
        }

        const Integral increment = multiplyTo<Integral>(error, ki_step_);
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            constexpr int bound_bit = 31 + Integral::FRACTION_BITS - Speed::FRACTION_BITS;
            constexpr int increment_bit = 62 + Integral::FRACTION_BITS - Angle::FRACTION_BITS - StepGain::FRACTION_BITS;
            static_assert(sizeof(typename Integral::StorageType) == 8 && bound_bit < 62 && increment_bit < 62,
                          "PLL bound plus one increment must fit int64");
            integral_ = Integral::fromRaw(integral_.raw() + increment.raw());
            integral_ = numericClamp(integral_, Integral::fromRaw(-integral_limit_.raw()), integral_limit_);
        } else {
            integral_ += increment;
            integral_ = numericClamp(integral_, -integral_limit_, integral_limit_);
        }
        speed_ = multiplyTo<Speed>(error, kp_) + numberCast<Speed>(integral_);
        if (reset_angle) {
            integral_ = numberCast<Integral>(speed_);
            phase_ = number<Phase>(hall_);
        } else {
            const Phase advance = multiplyTo<Phase>(speed_, number_v<Fraction, 1, FrequencyHz>);
            if constexpr (std::is_same<Profile, FixedProfile>::value) {
                constexpr int advance_bit = 62 + Phase::FRACTION_BITS - Speed::FRACTION_BITS - Fraction::FRACTION_BITS;
                static_assert(sizeof(typename Phase::StorageType) == 8 && advance_bit < 62 &&
                              Phase::FRACTION_BITS + 3 < 62, "Wrapped phase plus advance must fit int64");
                constexpr int64_t period = int64_t(Phase::SCALE) * 6;
                int64_t raw = phase_.raw() + advance.raw();
                if (raw >= period) raw -= period;
                else if (raw < 0) raw += period;
                if (raw >= period || raw < 0) {
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
        const int offset = commutationOffset(mode, inverse_);
        const int next = use_pll_ ? roundInteger(getAngle() + number<Angle>(offset)) : int(hall_) + offset;
        return static_cast<uint8_t>((next % 6 + 6) % 6);
    }

    /** @brief Change the continuous-time gains and cache the integration coefficient. */
    void setGains(Gain kp, Gain ki) noexcept {
        kp_ = kp;
        ki_ = ki;
        ki_step_ = divideByInteger<StepGain>(ki, FrequencyHz);
    }
    /** @brief Re-seed tracking from the most recent real/virtual Hall sector. */
    void reset() noexcept {
        phase_ = number<Phase>(hall_);
        speed_ = Speed{};
        integral_ = Integral{};
        lock_error_ = Angle{};
        synchronized_ = false;
    }
    void setUsePLL(bool enabled) noexcept { use_pll_ = enabled; if (!enabled) reset(); }
    bool isUsingPLL() const noexcept { return use_pll_; }
    Angle getAngle() const noexcept {
        // Narrowing may round a phase just below six up to the endpoint.
        Angle angle{};
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // phase_ is always wrapped into [0,6), so conversion neither
            // needs saturation nor a negative-number rounding path.
            static_assert(Phase::FRACTION_BITS > Angle::FRACTION_BITS && Angle::FRACTION_BITS + 3 <= 31,
                          "Wrapped phase must fit the public angle format");
            constexpr unsigned shift = Phase::FRACTION_BITS - Angle::FRACTION_BITS;
            angle = Angle::fromRaw(static_cast<int32_t>((phase_.raw() + (int64_t(1) << (shift - 1))) >> shift));
        } else {
            angle = numberCast<Angle>(phase_);
        }
        return angle >= ANGLE_MAX ? angle - ANGLE_MAX : angle;
    }
    Speed getSpeedStepsSec() const noexcept { return numberCast<Speed>(integral_); }
    Info getInfo() const noexcept {
        return {use_pll_, hall_, getAngle(), speed_, getSpeedStepsSec(), kp_, ki_, synchronized_};
    }

private:
    uint8_t hall_ = 0;
    bool inverse_ = false;
    bool synchronized_ = false;
    bool use_pll_ = false;
    Integral integral_limit_{};
    Phase phase_{};
    Speed speed_{};
    Integral integral_{};
    Angle lock_error_{};
    Gain kp_{};
    Gain ki_{};
    StepGain ki_step_{};
};

} // namespace libecu
#endif // LIBECU_MOTOR_PLL_CORE_HPP
