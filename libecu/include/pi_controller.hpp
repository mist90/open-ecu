/**
 * @file pi_controller.hpp
 * @brief Fixed-frequency PI with trapezoidal integration and anti-windup
 */
#ifndef LIBECU_PI_CONTROLLER_HPP
#define LIBECU_PI_CONTROLLER_HPP

#include "numeric_profiles.hpp"
#include "control_timing.hpp"
#include <algorithm>

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
    using Parameters = PiParametersT<Types>;

    PiController() noexcept = default;
    explicit PiController(const Parameters& params) noexcept { setParameters(params); }

    /** @brief Clear dynamic state while retaining the tuning. */
    void reset() noexcept {
        previous_error_ = Input{};
        integral_ = Accumulator{};
        output_ = Output{};
        saturation_ = 0;
    }

    /**
     * @brief Advance one sample
     * @param external_saturation +1/-1 when the inner loop is clamped high/low
     * @return Applied, bounded control output
     */
    Output update(Input setpoint, Input feedback, int external_saturation = 0) noexcept {
        const Input error = setpoint - feedback;
        const Accumulator proportional = multiplyTo<Accumulator>(params_.kp, error);
        const bool pushing_into_limit = (external_saturation > 0 && error > Input{}) ||
                                        (external_saturation < 0 && error < Input{});
        Accumulator next_integral = integral_;
        if (!pushing_into_limit) {
            next_integral += multiplyTo<Accumulator>(ki_step_, error + previous_error_);
            next_integral = clampIntegral(next_integral);
        }
        const Accumulator requested = proportional + next_integral;
        const Accumulator high(params_.max_output);
        const Accumulator low(params_.min_output);
        const Accumulator applied = std::max(low, std::min(requested, high));
        output_ = Output(applied);
        saturation_ = requested > high ? 1 : requested < low ? -1 : 0;

        if (kb_step_ > StepGain{} && saturation_ != 0) {
            const WideOutput excess(applied - requested);
            next_integral += multiplyTo<Accumulator>(kb_step_, excess);
            next_integral = clampIntegral(next_integral);
        }
        integral_ = next_integral;
        previous_error_ = error;
        return output_;
    }

    /** @brief Update continuous gains and cache their per-sample equivalents. */
    void setParameters(const Parameters& params) noexcept {
        params_ = params;
        ki_step_ = divideByInteger<StepGain>(params.ki, 2u * SampleRateHz);
        kb_step_ = divideByInteger<StepGain>(params.kb, SampleRateHz);
        integral_ = clampIntegral(integral_);
    }
    /** @return Current continuous-time tuning. */
    const Parameters& getParameters() const noexcept { return params_; }
    /** @return Integral term in output units. */
    Output getIntegral() const noexcept { return Output(integral_); }
    /** @return Last bounded output. */
    Output getOutput() const noexcept { return output_; }
    /** @return Whether the last requested output exceeded the actuator limits. */
    bool isSaturated() const noexcept { return saturation_ != 0; }
    /** @return +1 above the high limit, -1 below the low limit, otherwise zero. */
    int saturationSign() const noexcept { return saturation_; }

private:
    Accumulator clampIntegral(Accumulator value) const noexcept {
        return std::max(Accumulator(params_.integral_min),
                        std::min(value, Accumulator(params_.integral_max)));
    }

    Parameters params_{};
    StepGain ki_step_{};
    StepGain kb_step_{};
    Input previous_error_{};
    Accumulator integral_{};
    Output output_{};
    int saturation_ = 0;
};

} // namespace libecu
#endif // LIBECU_PI_CONTROLLER_HPP
