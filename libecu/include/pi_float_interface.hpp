/**
 * @file pi_float_interface.hpp
 * @brief Float-facing integration boundary for selectable numerical PI kernels
 */
#ifndef LIBECU_PI_FLOAT_INTERFACE_HPP
#define LIBECU_PI_FLOAT_INTERFACE_HPP

#include "pi_controller.hpp"
#include "control_optimization.hpp"
#include <cmath>
#include <type_traits>

namespace libecu {

namespace pi_interface_detail {
template<class T>
bool convert(float value, T& result) noexcept {
    if constexpr (std::is_same<T, float>::value) {
        if (!std::isfinite(value)) return false;
        result = value;
        return true;
    } else {
        bool clipped = false;
        result = T::fromFloat(value, &clipped);
        return !clipped;
    }
}

template<class T>
float toFloat(T value) noexcept {
    if constexpr (std::is_same<T, float>::value) return value;
    else return value.toFloat();
}
} // namespace pi_interface_detail

/**
 * @brief Adapt physical float I/O to a typed PI kernel during integration
 *
 * The kernel owns all PI arithmetic and accumulated state. Configuration and
 * compatibility update() use checked float conversions; updateNative() is the
 * typed path used by the firmware's periodic control loops.
 */
template<class Types, uint32_t SampleRateHz>
class PiFloatInterface {
public:
    using Kernel = PiController<Types, SampleRateHz>;
    using Input = typename Kernel::Input;
    using Output = typename Kernel::Output;
    using StepGain = typename Kernel::StepGain;
    using Gains = typename Kernel::Gains;

    /** @brief Configure atomically with respect to update(); reject unrepresentable tuning. */
    LIBECU_SLOW_PATH bool setParameters(const PiParameters& params) noexcept {
        using pi_interface_detail::convert;
        typename Kernel::Parameters converted{};
        if (!(convert(params.kp, converted.kp) && convert(params.ki, converted.ki) &&
              convert(params.kb, converted.kb) && convert(params.min_output, converted.min_output) &&
              convert(params.max_output, converted.max_output) &&
              convert(params.integral_min, converted.integral_min) &&
              convert(params.integral_max, converted.integral_max))) return false;
        if (converted.min_output > converted.max_output ||
            converted.integral_min > converted.integral_max) return false;

        // Check the per-sample formats as well as the continuous gain formats.
        // Configuration is infrequent; division in the kernel itself is integer
        // and takes place only when these parameters are installed.
        StepGain step{};
        if (!convert(pi_interface_detail::toFloat(converted.ki) / (2u * SampleRateHz), step) ||
            !convert(pi_interface_detail::toFloat(converted.kb) / SampleRateHz, step)) return false;

        kernel_.setParameters(converted);
        params_ = params;
        configured_ = true;
        return true;
    }

    /** @brief Execute one sample using the selected kernel representation. */
    float update(float setpoint, float feedback, int external_saturation = 0) noexcept {
        Input target{}, measured{};
        if (!pi_interface_detail::convert(setpoint, target) ||
            !pi_interface_detail::convert(feedback, measured)) {
            kernel_.reset();
            return 0.0f;
        }
        return pi_interface_detail::toFloat(kernel_.update(target, measured, external_saturation));
    }

    /** @brief Typed data path; configuration remains at the checked float boundary. */
    Output updateNative(Input setpoint, Input feedback, int external_saturation = 0) noexcept {
        return kernel_.update(setpoint, feedback, external_saturation);
    }

    /** @brief As updateNative, with a per-sample upper bound on the integral contribution. */
    Output updateNativeCapped(Input setpoint, Input feedback, int external_saturation, Output ceiling) noexcept {
        return kernel_.updateCapped(setpoint, feedback, external_saturation, ceiling);
    }

    /** @brief Independently bound the accumulated load current and the applied traction current. */
    Output updateNativeCapped(Input setpoint, Input feedback, int external_saturation,
                              Output ceiling, Output output_ceiling) noexcept {
        return kernel_.updateCapped(setpoint, feedback, external_saturation, ceiling, output_ceiling);
    }

    /** @brief Clear accumulated state. */
    void reset() noexcept { kernel_.reset(); }
    /** @return Last successfully accepted physical tuning. */
    const PiParameters& getParameters() const noexcept { return params_; }
    /** @return Whether a complete valid configuration has been installed. */
    bool isConfigured() const noexcept { return configured_; }
    /** @return Integral term in physical output units. */
    float getIntegral() const noexcept { return pi_interface_detail::toFloat(kernel_.getIntegral()); }
    /** @return Last bounded output in physical units. */
    float getOutput() const noexcept { return pi_interface_detail::toFloat(kernel_.getOutput()); }
    /** @return Direction of the last output saturation. */
    int saturationSign() const noexcept { return kernel_.saturationSign(); }
    /** @return Whether the last output saturated. */
    bool isSaturated() const noexcept { return kernel_.isSaturated(); }

    /** @brief Check continuous and per-sample gain ranges outside the sampling interrupt mask. */
    static bool prepareGains(float kp, float ki, Gains& gains) noexcept {
        using pi_interface_detail::convert;
        if (!convert(kp, gains.kp) || !convert(ki, gains.ki)) return false;
        StepGain checked{};
        if (!convert(pi_interface_detail::toFloat(gains.ki) / (2u * SampleRateHz), checked)) return false;
        gains.ki_step = divideByInteger<StepGain>(gains.ki, 2u * SampleRateHz);
        return true;
    }
    /** @brief Native read-only state for a coherent retuning snapshot. */
    const Kernel& native() const noexcept { return kernel_; }
    /** @brief Publish a validated gain pair and optional compensated integral under the caller's mask. */
    void applyGains(float kp, float ki, const Gains& gains, const typename Kernel::Accumulator* integral) noexcept {
        kernel_.applyGains(gains, integral);
        params_.kp = kp;
        params_.ki = ki;
    }

private:
    Kernel kernel_{};
    PiParameters params_{};
    bool configured_ = false;
};

} // namespace libecu
#endif // LIBECU_PI_FLOAT_INTERFACE_HPP
