/** @file hybrid_iq_control.hpp
 * @brief Fast resistive feed-forward with a slow, bounded current-error trim
 */
#ifndef LIBECU_HYBRID_IQ_CONTROL_HPP
#define LIBECU_HYBRID_IQ_CONTROL_HPP

#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"

namespace libecu {

/** @brief Experimental q-axis tuning for a noisy current sensor; SI units. */
struct HybridIqParams {
    bool enabled = false;
    float r_phase_ohm = 0.0f;          ///< Feed-forward phase resistance, V/A.
    float averaging_time_s = 0.02f;   ///< Error LPF time constant (~8 Hz).
    float integral_gain = 2.0f;       ///< Slow trim gain, V/(A*s); no measured-current P term.
    float correction_limit_v = 4.0f;  ///< Maximum magnitude of the model correction.
    float correction_slew_v_s = 10.0f;///< Maximum correction rate, independent of the fast model.
};

template<class P>
struct HybridIqParamsT {
    bool enabled = false;
    typename P::Gain resistance{};
    typename P::Fraction alpha{};
    typename P::FocStepGain ki_step{};
    typename P::Voltage correction_limit{};
    typename P::FocStepGain correction_step{};
};

/** @brief Validate/design outside the ISR. Failure leaves output unchanged. */
template<class P, uint32_t FrequencyHz>
bool makeHybridIqParams(const HybridIqParams& input, HybridIqParamsT<P>& output) noexcept {
    static_assert(FrequencyHz > 0, "Hybrid Iq requires a positive sample rate");
    HybridIqParamsT<P> next{};
    next.enabled = input.enabled;
    if (!std::isfinite(input.averaging_time_s) || input.averaging_time_s <= 0.0f ||
        input.r_phase_ohm < 0.0f || (input.enabled && input.r_phase_ohm <= 0.0f) ||
        input.integral_gain < 0.0f || input.correction_limit_v < 0.0f ||
        input.correction_slew_v_s <= 0.0f ||
        !tryNumber(input.r_phase_ohm, next.resistance) ||
        !tryNumber(1.0f / (1.0f + FrequencyHz * input.averaging_time_s), next.alpha) ||
        !tryNumber(input.integral_gain / FrequencyHz, next.ki_step) ||
        !tryNumber(input.correction_limit_v, next.correction_limit) ||
        !tryNumber(input.correction_slew_v_s / FrequencyHz, next.correction_step)) return false;
    if (next.alpha <= typename P::Fraction{} || next.alpha >= number<typename P::Fraction>(1) ||
        next.correction_step <= typename P::FocStepGain{} ||
        (input.integral_gain > 0.0f && next.ki_step <= typename P::FocStepGain{})) return false;
    output = next;
    return true;
}

/** @brief No heap, divisions or floating-point conversions in the sample path.
 *
 * Filter the error, equivalently filtering reference and measurement with the
 * SAME pole. Filtering only Iq would interpret a fast feed-forward step as a
 * sustained current deficit. Start from zero error so a first-sample spike
 * cannot seed a long-lived correction.
 */
template<class P>
class HybridIqControl {
public:
    using Current = typename P::Current;
    using Request = typename P::VoltageRequest;
    using Voltage = typename P::Voltage;
    using Integral = typename P::Accumulator;
    using Params = HybridIqParamsT<P>;

    /** @brief Clear adaptation on parking, mode/configuration or frame changes. */
    void reset() noexcept { error_ = Current{}; correction_ = Integral{}; }

    /** @brief Resistive voltage plus trim; the caller adds back-EMF/decoupling. */
    Request request(Current target, Current measured, const Params& params) noexcept {
        const Current difference = (target - measured) - error_;
        if constexpr (std::is_same<P, FixedProfile>::value) {
            // With 0 < alpha < 1, this is a convex step even when the
            // innovation saturates. Product and final sum fit Current; avoid
            // generic wide saturation in this extra PWM-rate filter.
            const auto delta = static_cast<int32_t>(
                fixed_detail::roundedProduct<P::Fraction::FRACTION_BITS>(
                    params.alpha.raw(), difference.raw()));
            error_ = Current::fromRaw(error_.raw() + delta);
        } else {
            error_ += multiplyTo<Current>(params.alpha, difference);
        }
        return multiplyTo<Request>(params.resistance, target) + numberCast<Request>(correction_);
    }

    /** @brief Integrate after the FINAL vector limiter, with conditional anti-windup.
     *
     * Do not back-calculate the instantaneous clipping into this slow trim:
     * that would reintroduce d-axis ADC noise through d-priority saturation.
     * Allow integration out of saturation, but never further into it.
     */
    void integrate(Request requested, Voltage applied, const Params& params) noexcept {
        const Integral step = numberCast<Integral>(params.correction_step);
        const Integral delta = numericClamp(multiplyTo<Integral>(params.ki_step, error_), -step, step);
        const Request actual = numberCast<Request>(applied);
        if ((requested > actual && delta > Integral{}) ||
            (requested < actual && delta < Integral{})) {
            // The averaging state can wind up too: do not remember an
            // unreachable demand and integrate it after the limiter releases.
            error_ = Current{};
            return;
        }
        if constexpr (std::is_same<P, FixedProfile>::value) {
            // Both bounds have 32-bit storage. A voltage-bounded Q32 state
            // plus a step-bounded Q32 delta has ample signed-64 headroom.
            constexpr int bound_bit = 31 + Integral::FRACTION_BITS - Voltage::FRACTION_BITS;
            constexpr int step_bit = 31 + Integral::FRACTION_BITS - P::FocStepGain::FRACTION_BITS;
            static_assert(bound_bit < 62 && step_bit < 62, "Bounded trim addition must fit int64");
            correction_ = Integral::fromRaw(correction_.raw() + delta.raw());
        } else {
            correction_ += delta;
        }
        const Integral limit = numberCast<Integral>(params.correction_limit);
        correction_ = numericClamp(correction_, -limit, limit);
    }

    /** @brief Slow model correction, volts (diagnostics/tests, not raw-current protection). */
    Integral correction() const noexcept { return correction_; }

private:
    Current error_{};
    Integral correction_{};
};

} // namespace libecu
#endif // LIBECU_HYBRID_IQ_CONTROL_HPP
