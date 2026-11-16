/** Verify a compiled coefficient against its transfer function, and that
 * unsupported runtime settings cannot silently change the requested filter. */
#include "current_feedback_filter.hpp"
#include <cassert>
#include <cmath>
#include <limits>

int main() {
    using namespace libecu;
    using P = FixedProfile;
    CurrentFilterCoefficients<P> coefficient;
    constexpr float cutoff = LIBECU_CURRENT_FILTER_FIXED_HZ;
    constexpr uint32_t rate = LIBECU_CURRENT_FILTER_FIXED_RATE;
    assert(makeCurrentFilterCoefficients(CurrentFilterOrder::OFF, 0, rate, coefficient));
    assert(coefficient.order == CurrentFilterOrder::OFF && numberCast<float>(coefficient.alpha) == 1);
    if (cutoff > 0) {
        assert(makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER, cutoff, rate, coefficient));
        const double alpha = double(coefficient.alpha.raw()) / P::Fraction::SCALE;
        const double omega = 6.2831853071795864769 * cutoff / rate;
        const double pole = 1 - alpha;
        // -3 dB means the power response is exactly 1/2.
        const double response = alpha * alpha / (1 + pole * pole - 2 * pole * std::cos(omega));
        assert(std::abs(response - 0.5) < 2e-6);
    }
    const auto before = coefficient;
    for (float invalid : {cutoff + 1, -1.0f, std::numeric_limits<float>::infinity()}) {
        assert(!makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER, invalid, rate, coefficient));
        assert(coefficient.order == before.order && coefficient.alpha == before.alpha);
    }
    assert(!makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER, cutoff, rate + 1, coefficient));
    assert(coefficient.order == before.order && coefficient.alpha == before.alpha);
    assert(makeCurrentFilterCoefficients(CurrentFilterOrder::OFF, 0, rate, coefficient));
    assert(coefficient.order == CurrentFilterOrder::OFF && numberCast<float>(coefficient.alpha) == 1);
}
