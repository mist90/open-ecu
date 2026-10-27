/** @file current_feedback_filter.hpp
 * @brief First-order and Butterworth current-feedback filters for float/fixed FOC
 */
#ifndef LIBECU_CURRENT_FEEDBACK_FILTER_HPP
#define LIBECU_CURRENT_FEEDBACK_FILTER_HPP

#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"

namespace libecu {

enum class CurrentFilterOrder : uint8_t { OFF, FIRST_ORDER, BUTTERWORTH_SECOND_ORDER };

template<class Profile>
struct CurrentFilterCoefficients {
    using Coefficient = typename Profile::Fraction;
    CurrentFilterOrder order = CurrentFilterOrder::OFF;
    Coefficient b0 = number<Coefficient>(1);
    Coefficient b1{}, b2{}, a1{}, a2{};
};

/** @brief Design a digital -3 dB cutoff. Call outside the PWM ISR. */
template<class Profile>
bool makeCurrentFilterCoefficients(CurrentFilterOrder order, float cutoff_hz, uint32_t sample_hz,
                                   CurrentFilterCoefficients<Profile>& output) noexcept {
    if (!sample_hz || !std::isfinite(cutoff_hz) || cutoff_hz < 0.0f) return false;
    CurrentFilterCoefficients<Profile> next{};
    next.order = order;
    if (order == CurrentFilterOrder::OFF) { output = next; return true; }
    if (!(cutoff_hz > 0.0f && cutoff_hz < 0.5f * sample_hz)) return false;
    const float omega = 6.28318530718f * cutoff_hz / sample_hz;
    const float half_sine = std::sin(0.5f * omega);
    const float k = 2.0f * half_sine * half_sine; // 1-cos(omega), without cancellation near zero
    float b0, b1 = 0.0f, b2 = 0.0f, a1, a2 = 0.0f;
    if (order == CurrentFilterOrder::FIRST_ORDER) {
        b0 = std::sqrt(k * k + 2.0f * k) - k;
        a1 = b0 - 1.0f;
    } else if (order == CurrentFilterOrder::BUTTERWORTH_SECOND_ORDER) {
        const float damping = std::sin(omega) * 0.70710678119f;
        const float inverse = 1.0f / (1.0f + damping);
        b0 = 0.5f * k * inverse;
        b1 = k * inverse;
        b2 = b0;
        a1 = -2.0f * std::cos(omega) * inverse;
        a2 = (1.0f - damping) * inverse;
    } else {
        return false;
    }
    if (!tryNumber(b0, next.b0) || !tryNumber(b1, next.b1) || !tryNumber(b2, next.b2) ||
        !tryNumber(a1, next.a1) || !tryNumber(a2, next.a2)) return false;
    // Reject coefficients whose quantized denominator lost strict stability.
    const float qa1 = numberCast<float>(next.a1), qa2 = numberCast<float>(next.a2);
    if (!(numberCast<float>(next.b0) > 0.0f && std::fabs(qa2) < 1.0f &&
          1.0f + qa1 + qa2 > 0.0f && 1.0f - qa1 + qa2 > 0.0f)) return false;
    output = next;
    return true;
}

/** @brief One current channel; transposed direct form II for the second order. */
template<class Profile>
class CurrentFeedbackFilter {
public:
    using Current = typename Profile::Current;
    using Coefficients = CurrentFilterCoefficients<Profile>;

    /** @brief Forget history; the next measurement initializes the DC state. */
    void reset() noexcept { initialized_ = false; value_ = z1_ = z2_ = Current{}; }
    /** @brief Most recently produced current value. */
    Current value() const noexcept { return value_; }

    /** @brief Filter one valid dq sample using precomputed coefficients. */
    Current update(Current input, const Coefficients& c) noexcept {
        if (c.order == CurrentFilterOrder::OFF) {
            initialized_ = false;
            value_ = input;
            return value_;
        }
        if (!initialized_) {
            initialized_ = true;
            value_ = input;
            z1_ = input - multiplyTo<Current>(c.b0, input);
            z2_ = multiplyTo<Current>(c.b2 - c.a2, input);
            return value_;
        }
        if (c.order == CurrentFilterOrder::FIRST_ORDER) {
            value_ += multiplyTo<Current>(c.b0, input - value_);
        } else {
            value_ = multiplyTo<Current>(c.b0, input) + z1_;
            z1_ = multiplyTo<Current>(c.b1, input) - multiplyTo<Current>(c.a1, value_) + z2_;
            z2_ = multiplyTo<Current>(c.b2, input) - multiplyTo<Current>(c.a2, value_);
        }
        return value_;
    }

private:
    Current value_{}, z1_{}, z2_{};
    bool initialized_ = false;
};

} // namespace libecu
#endif // LIBECU_CURRENT_FEEDBACK_FILTER_HPP
