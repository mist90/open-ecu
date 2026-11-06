/** @file numeric_ops.hpp
 * @brief Format-explicit scalar operations shared by numerical profiles
 */
#ifndef LIBECU_NUMERIC_OPS_HPP
#define LIBECU_NUMERIC_OPS_HPP

#include "fixed_point.hpp"
#include <array>
#include <cmath>
#include <type_traits>

namespace libecu {

/** @brief Rational value in the requested number representation. */
template<class T>
constexpr T number(int64_t numerator, int64_t denominator) noexcept {
    if constexpr (std::is_same<T, float>::value) return float(numerator) / float(denominator);
    else return T::fromRatio(numerator, denominator);
}

template<class T>
constexpr T number(int64_t integer) noexcept {
    if constexpr (std::is_same<T, float>::value) return float(integer);
    else return T::fromInteger(integer);
}

/** @brief Force evaluation of rational constants even in size-optimized builds. */
template<class T, int64_t Numerator, int64_t Denominator = 1>
inline constexpr T number_v = number<T>(Numerator, Denominator);

/** @brief Configuration literal; use in constexpr declarations. */
template<class T>
constexpr T constant(float value) noexcept {
    if constexpr (std::is_same<T, float>::value) return value;
    else return T::fromConstant(value);
}

/** @brief Explicit conversion between physical-value formats. */
template<class Out, class In>
[[gnu::always_inline]] inline Out numberCast(In value) noexcept {
    if constexpr (std::is_same<Out, In>::value) return value;
    else if constexpr (std::is_same<Out, float>::value) return value.toFloat();
    else if constexpr (std::is_same<In, float>::value) return Out::fromFloat(value);
    else return Out(value);
}

/** @brief Checked conversion at a float-facing configuration boundary. */
template<class T>
bool tryNumber(float value, T& result) noexcept {
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
constexpr T numericAbs(T value) noexcept { return value < T{} ? -value : value; }

template<class T>
constexpr T numericClamp(T value, T low, T high) noexcept {
    return value < low ? low : value > high ? high : value;
}

/** @brief Fold a value into [0, period). */
template<class T>
T wrapValue(T value, T period) noexcept {
    if (period <= T{}) return T{};
    if (value >= T{} && value < period) return value;
    if (value >= period && value - period < period) return value - period;
    if (value < T{} && value >= -period) return value + period;
    T result{};
    if constexpr (std::is_same<T, float>::value) result = std::fmod(value, period);
    else result = value % period;
    return result < T{} ? result + period : result;
}

template<class T>
int32_t roundInteger(T value) noexcept {
    if constexpr (std::is_same<T, float>::value) return static_cast<int32_t>(std::round(value));
    else return FixedPointType<0>(value).raw();
}

/** @brief Multiply by an integer with an explicit output format. */
template<class Out, unsigned Bits, class Storage>
constexpr Out scaleInteger(FixedPointType<Bits, Storage> value, int32_t factor) noexcept {
    return multiplyTo<Out>(value, FixedPointType<0>::fromRaw(factor));
}
template<class Out>
constexpr Out scaleInteger(float value, int32_t factor) noexcept { return Out(value * factor); }

/** @brief Divide two 32-bit formats, rounding once in the output format. */
template<class Out, unsigned A, class SA, unsigned B, class SB>
constexpr Out divideTo(FixedPointType<A, SA> a, FixedPointType<B, SB> b) noexcept {
    static_assert(sizeof(SA) <= 4 && sizeof(SB) <= 4, "Division operands must fit in 32 bits");
    constexpr int shift = int(Out::FRACTION_BITS) + int(B) - int(A);
    static_assert(shift >= -31 && shift <= 31, "Rescale the operands before division");
    if constexpr (shift >= 0) {
        return Out::fromWideRaw(fixed_detail::roundedDivide(int64_t(a.raw()) * (int64_t(1) << shift), b.raw()));
    } else {
        return Out::fromWideRaw(fixed_detail::roundedDivide(a.raw(), int64_t(b.raw()) * (int64_t(1) << -shift)));
    }
}
template<class Out>
constexpr Out divideTo(float a, float b) noexcept { return Out(a / b); }

namespace numeric_detail {
constexpr uint32_t restoringSqrt(uint64_t value) noexcept {
    uint64_t result = 0;
    uint64_t bit = uint64_t(1) << 62;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<uint32_t>(result);
}

constexpr unsigned bitWidth(uint32_t value) noexcept {
#if defined(__GNUC__)
    return value ? 32u - __builtin_clz(value) : 0;
#else
    unsigned bits = 0;
    while (value) { ++bits; value >>= 1; }
    return bits;
#endif
}

// Upper square-root estimates for a normalized 32-bit radicand's top byte.
// Store ceil(sqrt((index+65)*2^24))-1, allowing the 65536 endpoint in uint16_t.
inline constexpr auto SQRT_UPPER = [] {
    std::array<uint16_t, 192> table{};
    for (unsigned i = 0; i < table.size(); ++i) {
        const uint64_t value = uint64_t(i + 65u) << 24;
        uint32_t root = restoringSqrt(value);
        if (uint64_t(root) * root < value) ++root;
        table[i] = static_cast<uint16_t>(root - 1u);
    }
    return table;
}();

// For every table bin, the seed's maximum error e satisfies e^2/(2*seed)<1.
// Verify the bound at compile time, using floor(sqrt(bin start)) conservatively.
static_assert([] {
    for (unsigned i = 0; i < SQRT_UPPER.size(); ++i) {
        const uint32_t seed = uint32_t(SQRT_UPPER[i]) + 1u;
        const uint32_t lower = restoringSqrt(uint64_t(i + 64u) << 24);
        const uint32_t error = seed - lower;
        if (error * error >= 2u * seed) return false;
    }
    return true;
}(), "One normalized Newton step must be within one count of the exact root");

// Exact floor sqrt for a normalized radicand in [2^30, 2^32). One 32-bit
// division leaves either the exact floor or its successor; check its square.
constexpr uint32_t normalizedSqrt(uint32_t value) noexcept {
    uint32_t root = uint32_t(SQRT_UPPER[(value >> 24) - 64u]) + 1u;
    root = (root + value / root) / 2u;
    if (root * root > value) --root; // After Newton, root <=65535: the square fits.
    return root;
}
} // namespace numeric_detail

/** @brief Exact floor of the unsigned square root, without floating arithmetic. */
constexpr uint32_t integerSqrt(uint64_t value) noexcept {
    if (!value) return 0;
    if (value <= UINT32_MAX) {
        const uint32_t input = static_cast<uint32_t>(value);
        uint32_t root = 1u << ((numeric_detail::bitWidth(input) + 1u) / 2u);
        while (true) {
            const uint32_t next = (root + input / root) / 2u;
            if (next >= root) return root;
            root = next;
        }
    }
    if (value < (uint64_t(1) << 62)) {
        const unsigned bits = 32u + numeric_detail::bitWidth(static_cast<uint32_t>(value >> 32));
        const unsigned scale = (bits + 1u) / 2u - 16u;
        const uint32_t normalized = static_cast<uint32_t>(value >> (2u * scale));
        const uint32_t high_root = numeric_detail::normalizedSqrt(normalized);
        const uint32_t coarse = high_root << scale;
        const uint64_t remainder = value - uint64_t(coarse) * coarse;
        // coarse <=sqrt(value)<coarse+2^scale, high_root is in [32768,65535],
        // and scale<=15. Thus remainder>>scale fits uint32, and this division
        // is EXACTLY floor(remainder/(2*coarse)), with no wide quotient work.
        const uint32_t correction = static_cast<uint32_t>(remainder >> scale) / (2u * high_root);
        uint32_t root = coarse + correction;
        // Newton from below overshoots sqrt(value) by e^2/(2*coarse)<1/2.
        // Its floor cannot undershoot, and at most one decrement is required.
        if (uint64_t(root) * root > value) --root;
        return root;
    }
    return numeric_detail::restoringSqrt(value);
}

/** @brief Map normalized duty to a timer compare, using integer arithmetic for fixed point. */
template<class T>
uint32_t dutyToCompare(T duty, uint32_t period) noexcept {
    if constexpr (std::is_same<T, float>::value) {
        return static_cast<uint32_t>(numericClamp(duty, T{}, number<T>(1)) * period);
    } else {
        // Keep the upper clamp unsigned too: signed clamping followed by a
        // cast can still generate sign-extension multiplies on 32-bit targets.
        using Unsigned = std::make_unsigned_t<typename T::StorageType>;
        Unsigned raw = duty < T{} ? Unsigned{} : static_cast<Unsigned>(duty.raw());
        constexpr Unsigned maximum = static_cast<Unsigned>(number<T>(1).raw());
        if (raw > maximum) raw = maximum;
        return static_cast<uint32_t>(uint64_t(raw) * period / T::SCALE);
    }
}

} // namespace libecu
#endif // LIBECU_NUMERIC_OPS_HPP
