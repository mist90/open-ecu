/**
 * @file pi_controller.hpp
 * @brief Fixed-frequency PI with trapezoidal integration and anti-windup
 */
#ifndef LIBECU_PI_CONTROLLER_HPP
#define LIBECU_PI_CONTROLLER_HPP

#include "numeric_profiles.hpp"
#include "control_timing.hpp"
#include <algorithm>
#include <limits>
#include <type_traits>

namespace libecu {

/** @brief Continuous-time tuning and physical output limits for one PI loop. */
template<class Types = FloatProfile::PiTypes>
struct PiParametersT {
    typename Types::Gain kp{};
    typename Types::Gain ki{};
    typename Types::Output max_output{};
    typename Types::Output min_output{};
    typename Types::Output integral_max{};
    typename Types::Output integral_min{};
    typename Types::Gain kb{};
};

using PiParameters = PiParametersT<>;

/**
 * @brief PI sampled at a compile-time integer frequency
 * @note Publish setParameters() atomically with respect to update().
 */
template<class Types, uint32_t SampleRateHz>
class PiController {
public:
    static_assert(SampleRateHz > 0 && SampleRateHz <= UINT32_MAX / 2u, "Invalid PI frequency");
    static constexpr uint32_t FREQUENCY_HZ = SampleRateHz;
    using Input = typename Types::Input;
    using Output = typename Types::Output;
    using Accumulator = typename Types::Accumulator;
    using StepGain = typename Types::StepGain;
    using WideOutput = typename Types::WideOutput;
    using Gain = typename Types::Gain;
    using Parameters = PiParametersT<Types>;

    struct Gains { Gain kp{}, ki{}; StepGain ki_step{}; };
    struct OperatingPoint { Input error{}; Output output{}; };
    /** @brief Tuning in the exact formats used by step(), with no sampled state. */
    struct Tuning {
        Accumulator output_low, output_high, integral_low, integral_high;
        Gain kp, ki, kb;
        StepGain ki_step, kb_step;

        Tuning() noexcept = default;
        explicit Tuning(const Parameters& p) noexcept :
            output_low(p.min_output), output_high(p.max_output),
            integral_low(p.integral_min), integral_high(p.integral_max), kp(p.kp), ki(p.ki), kb(p.kb),
            ki_step(divideByInteger<StepGain>(p.ki, 2u * SampleRateHz)),
            kb_step(divideByInteger<StepGain>(p.kb, SampleRateHz)) {}
    };

    PiController() noexcept = default;
    explicit PiController(const Parameters& params) noexcept { setParameters(params); }

    /** @brief Clear dynamic state while retaining the tuning. */
    void reset() noexcept {
        point_ = {};
        integral_ = Accumulator{};
        saturation_ = 0;
    }

    /**
     * @brief Advance one sample
     * @param external_saturation +1/-1 when the inner loop is clamped high/low
     * @return Applied, bounded control output
     */
    Output update(Input setpoint, Input feedback, int external_saturation = 0) noexcept {
        return step(setpoint, feedback, external_saturation, tuning_.integral_high,
                    tuning_.output_low, tuning_.output_high);
    }

    /** @brief Tighten the integral ceiling for this tick, even when the inner loop blocks integration.
     * Per-sample caps are local; the configured limits are never overwritten.
     */
    Output updateCapped(Input setpoint, Input feedback, int external_saturation, Output ceiling) noexcept {
        return updateBounded(setpoint - feedback, external_saturation, ceiling, outputFloor(), outputCeiling());
    }

    /** @brief Tighten the positive output rail while preserving the configured negative rail. */
    Output updateCapped(Input setpoint, Input feedback, int external_saturation,
                        Output integral_ceiling, Output output_ceiling) noexcept {
        const auto floor = outputFloor();
        return updateBounded(setpoint - feedback, external_saturation, integral_ceiling,
                             floor, std::max(output_ceiling, floor));
    }

    /** Tighten both actuator rails. Anti-windup must see the actually applied Iq*, including braking. */
    template<bool SampledKi = false>
    Output updateBounded(Input error, int external_saturation, Output integral_ceiling,
                         Output floor, Output ceiling, StepGain previous_ki_step = {}) noexcept {
        floor = std::max(floor, asLimit(tuning_.output_low));
        ceiling = std::min(ceiling, asLimit(tuning_.output_high));
        const auto high = Accumulator(std::clamp(integral_ceiling, integralFloor(), integralCeiling()));
        integral_ = std::clamp(integral_, tuning_.integral_low, high);
        if (tuning_.kb_step <= StepGain{}) {
            if (error > Input{} && ceiling < asLimit(tuning_.output_high) &&
                (saturation_ > 0 || point_.output >= ceiling)) external_saturation = 1;
            if (error < Input{} && floor > asLimit(tuning_.output_low) &&
                (saturation_ < 0 || point_.output <= floor)) external_saturation = -1;
        }
        return step<SampledKi>(error, {}, external_saturation, high, Accumulator(floor), Accumulator(ceiling),
                               previous_ki_step);
    }

private:
    // These values originated in Output units. Unlike a dynamic integral or
    // request, a stored rail is exactly recoverable: no rounding or saturation.
    static Output asLimit(Accumulator limit) noexcept {
        if constexpr (std::is_floating_point_v<Accumulator>) return Output(limit);
        else {
            constexpr int shift = int(Accumulator::FRACTION_BITS) - int(Output::FRACTION_BITS);
            static_assert(shift >= 0 && std::numeric_limits<typename Output::StorageType>::digits + shift
                                     <= std::numeric_limits<typename Accumulator::StorageType>::digits,
                          "Configured bounds must widen exactly into the PI accumulator");
            return Output::fromRaw(static_cast<typename Output::StorageType>(limit.raw() >> shift));
        }
    }
    template<bool SampledKi = false>
    Output step(Input setpoint, Input feedback, int external_saturation,
                Accumulator integral_high, Accumulator output_low, Accumulator output_high,
                StepGain previous_ki_step = {}) noexcept {
        const Input error = setpoint - feedback;
        const Accumulator proportional = multiplyTo<Accumulator>(tuning_.kp, error);
        const bool pushing_into_limit = (external_saturation > 0 && error > Input{}) ||
                                        (external_saturation < 0 && error < Input{});
        Accumulator next_integral = integral_;
        if (!pushing_into_limit) {
            // A schedule change belongs to the new endpoint only. Constant gains
            // retain the original single multiply and its exact fixed-point rounding.
            if (SampledKi && previous_ki_step != tuning_.ki_step)
                next_integral += multiplyTo<Accumulator>(tuning_.ki_step, error)
                               + multiplyTo<Accumulator>(previous_ki_step, point_.error);
            else
                next_integral += multiplyTo<Accumulator>(tuning_.ki_step, error + point_.error);
            next_integral = std::max(tuning_.integral_low, std::min(next_integral, integral_high));
        }
        const Accumulator requested = proportional + next_integral;
        Accumulator applied = requested;
        if constexpr (std::is_same_v<Accumulator, FixedProfile::Accumulator> &&
                      (std::is_same_v<Output, FixedProfile::Current> || std::is_same_v<Output, FixedProfile::Duty>)) {
            if (requested > output_high) { applied = output_high; saturation_ = 1; }
            else if (requested < output_low) { applied = output_low; saturation_ = -1; }
            else saturation_ = 0;
            // Even an endpoint is representable: 'applied' is now bounded by
            // two exactly widened Output values. Only the normal rounding remains.
            constexpr unsigned shift = Accumulator::FRACTION_BITS - Output::FRACTION_BITS;
            point_.output = Output::fromRaw(int32_t(fixed_detail::roundedLowWord<shift>(applied.raw())));
        } else {
            applied = std::max(output_low, std::min(requested, output_high));
            saturation_ = requested > output_high ? 1 : requested < output_low ? -1 : 0;
            point_.output = Output(applied);
        }

        if (tuning_.kb_step > StepGain{} && saturation_ != 0) {
            const WideOutput excess(applied - requested);
            next_integral += multiplyTo<Accumulator>(tuning_.kb_step, excess);
            next_integral = std::max(tuning_.integral_low, std::min(next_integral, integral_high));
        }
        integral_ = next_integral;
        point_.error = error;
        return point_.output;
    }

public:
    /** @brief Install decoded tuning, retaining history unless the caller requests a fresh loop. */
    template<bool Reset>
    void applyTuning(const Tuning& tuning) noexcept {
        tuning_ = tuning;
        if constexpr (Reset) reset();
        else integral_ = std::max(tuning_.integral_low, std::min(integral_, tuning_.integral_high));
    }
    void setParameters(const Parameters& params) noexcept {
        applyTuning<false>(Tuning(params));
    }
    /** @brief Borrow the last sample in place. Readers in another context must synchronize. */
    const OperatingPoint& operatingPoint() const noexcept { return point_; }

    /** @brief Rebase I in output units to hold the last command at the same error.
     * Refuse an infeasible transfer instead of clipping I and introducing a step. */
    static bool rebaseIntegral(const OperatingPoint& point, Gain kp, Output low, Output high,
                               Accumulator& result) noexcept {
        const auto proportional = multiplyTo<Accumulator>(kp, point.error);
        const auto integral = Accumulator(point.output) - proportional;
        if (!(integral >= Accumulator(low) && integral <= Accumulator(high)) ||
            Output(proportional + integral) != point.output) return false;
        result = integral;
        return true;
    }

    /** @brief Commit prepared gains between two samples. Preserve error history, output and anti-windup. */
    void applyGains(const Gains& gains, const Accumulator* rebased) noexcept {
        tuning_.kp = gains.kp;
        tuning_.ki = gains.ki;
        tuning_.ki_step = gains.ki_step;
        if (rebased) integral_ = *rebased;
    }
    /** @brief Cheap individual reads for callers snapshotting gains/rails under an interrupt mask. */
    Gain proportionalGain() const noexcept { return tuning_.kp; }
    Gain integralGain() const noexcept { return tuning_.ki; }
    StepGain integralStepGain() const noexcept { return tuning_.ki_step; }
    Output outputFloor() const noexcept { return asLimit(tuning_.output_low); }
    Output outputCeiling() const noexcept { return asLimit(tuning_.output_high); }
    Output integralFloor() const noexcept { return asLimit(tuning_.integral_low); }
    Output integralCeiling() const noexcept { return asLimit(tuning_.integral_high); }

    /** @return Configured tuning in output units; never the transient limits of a capped tick. */
    Parameters getParameters() const noexcept {
        return {tuning_.kp, tuning_.ki, asLimit(tuning_.output_high), asLimit(tuning_.output_low),
                integralCeiling(), integralFloor(), tuning_.kb};
    }
    /** @return Integral term in output units. */
    Output getIntegral() const noexcept { return Output(integral_); }
    /** @return Last bounded output. */
    Output getOutput() const noexcept { return point_.output; }
    /** @return Whether the last requested output exceeded the actuator limits. */
    bool isSaturated() const noexcept { return saturation_ != 0; }
    /** @return +1 above the high limit, -1 below the low limit, otherwise zero. */
    int saturationSign() const noexcept { return saturation_; }

private:
    // A configured limit originates in Output and widens exactly in both production fixed profiles.
    // Store it only in comparison units: Q32 for both Q20 current and Q30 duty outputs.
    Tuning tuning_{};
    Accumulator integral_{};
    OperatingPoint point_{};
    int saturation_ = 0;
};

} // namespace libecu
#endif // LIBECU_PI_CONTROLLER_HPP
