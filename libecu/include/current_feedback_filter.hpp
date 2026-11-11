/** @file current_feedback_filter.hpp
 * @brief First-order current-feedback filter for float/fixed FOC
 */
#ifndef LIBECU_CURRENT_FEEDBACK_FILTER_HPP
#define LIBECU_CURRENT_FEEDBACK_FILTER_HPP

#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"

namespace libecu {

enum class CurrentFilterOrder : uint8_t { OFF, FIRST_ORDER };

template<class Profile>
struct CurrentFilterCoefficients {
    using Coefficient = typename Profile::Fraction;
    CurrentFilterOrder order = CurrentFilterOrder::OFF;
    Coefficient alpha = number<Coefficient>(1);
};

/** @brief Design a digital -3 dB cutoff. Call outside the PWM ISR. */
template<class Profile>
bool makeCurrentFilterCoefficients(CurrentFilterOrder order, float cutoff_hz, uint32_t sample_hz,
                                   CurrentFilterCoefficients<Profile>& output) noexcept {
    if (!sample_hz || !std::isfinite(cutoff_hz) || cutoff_hz < 0.0f) return false;
    CurrentFilterCoefficients<Profile> next{};
    next.order = order;
    if (order == CurrentFilterOrder::OFF) { output = next; return true; }
    if (order != CurrentFilterOrder::FIRST_ORDER) return false;
    if (!(cutoff_hz > 0.0f && cutoff_hz < 0.5f * sample_hz)) return false;
    const float omega = 6.28318530718f * cutoff_hz / sample_hz;
    const float half_sine = std::sin(0.5f * omega);
    const float k = 2.0f * half_sine * half_sine; // 1-cos(omega), without cancellation near zero
    const float alpha = std::sqrt(k * k + 2.0f * k) - k;
    if (!tryNumber(alpha, next.alpha)) return false;
    // The quantized pole 1-alpha must remain strictly inside the unit circle.
    const float quantized_alpha = numberCast<float>(next.alpha);
    if (!(quantized_alpha > 0.0f && quantized_alpha < 1.0f)) return false;
    output = next;
    return true;
}

/** @brief One current channel: y += alpha * (input - y). */
template<class Profile>
class CurrentFeedbackFilter {
public:
    using Current = typename Profile::Current;
    using Coefficients = CurrentFilterCoefficients<Profile>;

    /** @brief Forget history; the next measurement initializes the DC state. */
    void reset() noexcept { initialized_ = false; value_ = Current{}; }
    /** @brief Most recently produced current value. */
    Current value() const noexcept { return value_; }

    /** @brief Filter one valid dq sample using precomputed coefficients. */
    [[gnu::always_inline]] Current update(Current input, const Coefficients& c) noexcept {
        if (c.order == CurrentFilterOrder::OFF) {
            initialized_ = false;
            value_ = input;
            return value_;
        }
        if (!initialized_) {
            initialized_ = true;
            value_ = input;
        } else {
            value_ += multiplyTo<Current>(c.alpha, input - value_);
        }
        return value_;
    }

private:
    Current value_{};
    bool initialized_ = false;
};

} // namespace libecu
#endif // LIBECU_CURRENT_FEEDBACK_FILTER_HPP
