/**
 * @file fixed_point.hpp
 * @brief Saturating binary fixed point with explicit format conversions
 */
#ifndef LIBECU_FIXED_POINT_HPP
#define LIBECU_FIXED_POINT_HPP

#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace libecu {

namespace fixed_detail {
constexpr uint64_t magnitude(int64_t value) noexcept {
    return value < 0 ? uint64_t(-(value + 1)) + 1u : uint64_t(value);
}

constexpr int64_t roundedDivide(int64_t numerator, int64_t denominator) noexcept {
    if (denominator == 0) {
        return numerator == 0 ? 0 : (numerator < 0 ? INT64_MIN : INT64_MAX);
    }
    const bool negative = (numerator < 0) != (denominator < 0);
    const uint64_t n = magnitude(numerator);
    const uint64_t d = magnitude(denominator);
    if (n <= UINT32_MAX && d <= UINT32_MAX) {
        const uint32_t small_n = static_cast<uint32_t>(n);
        const uint32_t small_d = static_cast<uint32_t>(d);
        const uint32_t remainder = small_n % small_d;
        const int64_t result = uint64_t(small_n / small_d) + (remainder >= small_d - remainder ? 1u : 0u);
        return negative ? -result : result;
    }
    const uint64_t remainder = n % d;
    const uint64_t result = n / d + (remainder >= d - remainder ? 1u : 0u);
    if (result > uint64_t(INT64_MAX)) return negative ? INT64_MIN : INT64_MAX;
    return negative ? -int64_t(result) : int64_t(result);
}
} // namespace fixed_detail

/**
 * @brief Signed fixed point: value = raw * 2^-FractionBits
 *
 * Arithmetic and narrowing saturate. Rounding is nearest, ties away from zero.
 * A 64-bit storage type is supported for accumulators and widened products;
 * multiplication/division operands are at most 32 bits, making their full
 * intermediate results representable without a target-specific 128-bit type.
 */
template<unsigned FractionBits, typename Storage = int32_t>
class FixedPointType {
    static_assert(std::is_integral<Storage>::value && std::is_signed<Storage>::value,
                  "Fixed point requires signed integer storage");
    static_assert(sizeof(Storage) <= sizeof(int64_t), "Storage exceeds 64 bits");
    static_assert(FractionBits <= std::numeric_limits<Storage>::digits && FractionBits < 63,
                  "Invalid number of fractional bits");
public:
    using StorageType = Storage;
    static constexpr unsigned FRACTION_BITS = FractionBits;
    static constexpr uint64_t SCALE = uint64_t(1) << FractionBits;

    constexpr FixedPointType() noexcept = default;

    /** @brief Explicit conversion, with one rounding operation at the destination. */
    template<unsigned OtherBits, typename OtherStorage>
    [[gnu::always_inline]] explicit constexpr FixedPointType(FixedPointType<OtherBits, OtherStorage> other) noexcept {
        const int64_t value = other.raw();
        if constexpr (FractionBits > OtherBits) {
            constexpr unsigned shift = FractionBits - OtherBits;
            constexpr int64_t factor = int64_t(1) << shift;
            // Widening into an accumulator usually fits by construction. Let
            // the type ranges remove all run-time saturation/sign handling.
            if constexpr (std::numeric_limits<OtherStorage>::digits + shift >
                          std::numeric_limits<Storage>::digits) {
                if (value > std::numeric_limits<Storage>::max() / factor) {
                    raw_ = std::numeric_limits<Storage>::max();
                    return;
                }
                if (value < std::numeric_limits<Storage>::min() / factor) {
                    raw_ = std::numeric_limits<Storage>::min();
                    return;
                }
            }
            raw_ = static_cast<Storage>(value * factor);
        } else if constexpr (FractionBits < OtherBits) {
            constexpr unsigned shift = OtherBits - FractionBits;
            constexpr int64_t half = int64_t(1) << (shift - 1u);
            static_assert((int64_t(-1) >> 1) == -1, "Fixed point requires arithmetic right shift");
            // The signed bias rounds ties away from zero. Only a wide positive
            // input near INT64_MAX can overflow the bias addition.
            if constexpr (sizeof(OtherStorage) == sizeof(int64_t)) {
                if (value > INT64_MAX - half) {
                    raw_ = fromWideRaw((value >> shift) + 1).raw();
                    return;
                }
            }
            raw_ = fromWideRaw((value + half - int64_t(value < 0)) >> shift).raw();
        } else {
            raw_ = fromWideRaw(value).raw();
        }
    }

    /** @brief Construct from the underlying integer representation. */
    static constexpr FixedPointType fromRaw(Storage raw) noexcept {
        FixedPointType result;
        result.raw_ = raw;
        return result;
    }

    /** @brief Saturate a widened raw result to this storage type. */
    [[gnu::always_inline]] static constexpr FixedPointType fromWideRaw(int64_t raw) noexcept {
        const Storage narrowed = static_cast<Storage>(raw);
        // For a 32-bit result this is one sign-extension comparison on ARM,
        // instead of two signed 64-bit range comparisons.
        if (int64_t(narrowed) == raw) return fromRaw(narrowed);
        return fromRaw(raw < 0 ? std::numeric_limits<Storage>::min() : std::numeric_limits<Storage>::max());
    }

    /** @brief Integer input without routing a sample through rational long division. */
    static constexpr FixedPointType fromInteger(int64_t value) noexcept {
        if (value > std::numeric_limits<Storage>::max() / int64_t(SCALE))
            return fromRaw(std::numeric_limits<Storage>::max());
        if (value < std::numeric_limits<Storage>::min() / int64_t(SCALE))
            return fromRaw(std::numeric_limits<Storage>::min());
        return fromRaw(static_cast<Storage>(value * int64_t(SCALE)));
    }

    /** @brief Convert a configuration literal in a constexpr initializer. */
    static constexpr FixedPointType fromConstant(float value) noexcept {
        const float scaled = value * static_cast<float>(SCALE);
        if (scaled != scaled) return FixedPointType{};
        if (scaled >= static_cast<float>(std::numeric_limits<Storage>::max()))
            return fromRaw(std::numeric_limits<Storage>::max());
        if (scaled <= static_cast<float>(std::numeric_limits<Storage>::min()))
            return fromRaw(std::numeric_limits<Storage>::min());
        int64_t integral = static_cast<int64_t>(scaled);
        const float remainder = scaled - static_cast<float>(integral);
        if (remainder >= 0.5f) ++integral;
        else if (remainder <= -0.5f) --integral;
        return fromWideRaw(integral);
    }

    /**
     * @brief Explicit IEEE-754 conversion at a float-facing interface
     * @param clipped Set on range overflow, infinity or NaN; optional
     * @return Rounded/saturated value; NaN converts to zero
     *
     * Decode the significand with integer operations. This avoids undefined
     * out-of-range float-to-integer casts, including at the int64 boundaries.
     */
    static FixedPointType fromFloat(float value, bool* clipped = nullptr) noexcept {
        static_assert(sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559,
                      "The float interface requires IEEE-754 binary32");
        if (clipped) *clipped = false;
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        const bool negative = (bits >> 31) != 0;
        const unsigned exponent = (bits >> 23) & 0xffu;
        const uint32_t fraction = bits & 0x7fffffu;
        if (exponent == 0xffu) {
            if (clipped) *clipped = true;
            return fraction ? FixedPointType{} : fromRaw(signedMagnitude(UINT64_MAX, negative));
        }
        const uint64_t significand = exponent ? (fraction | 0x800000u) : fraction;
        const int shift = (exponent ? int(exponent) - 127 : -126) - 23 + int(FractionBits);
        uint64_t raw = 0;
        if (shift >= 0) {
            if (shift >= 64 || significand > (limit(negative) >> shift)) {
                if (clipped) *clipped = true;
                return fromRaw(signedMagnitude(UINT64_MAX, negative));
            }
            raw = significand << shift;
        } else {
            const unsigned discard = static_cast<unsigned>(-shift);
            if (discard > 24) return FixedPointType{};
            raw = (significand + (uint64_t(1) << (discard - 1))) >> discard;
        }
        if (raw > limit(negative) && clipped) *clipped = true;
        return fromRaw(signedMagnitude(raw, negative));
    }

    /**
     * @brief Construct a rational constant without floating-point intermediates
     * @note Nonzero / zero saturates by sign; zero / zero produces zero.
     */
    static constexpr FixedPointType fromRatio(int64_t numerator, int64_t denominator = 1) noexcept {
        if (denominator == 0) {
            return numerator == 0 ? FixedPointType{} : fromRaw(numerator < 0
                ? std::numeric_limits<Storage>::min() : std::numeric_limits<Storage>::max());
        }
        const bool negative = (numerator < 0) != (denominator < 0);
        const uint64_t n = fixed_detail::magnitude(numerator);
        const uint64_t d = fixed_detail::magnitude(denominator);
        if (d == 1) {
            if (n > (limit(negative) >> FractionBits)) return fromRaw(signedMagnitude(UINT64_MAX, negative));
            return fromRaw(signedMagnitude(n << FractionBits, negative));
        }
        const uint64_t whole = n / d;
        if (whole > limit(negative) / SCALE) {
            return fromRaw(signedMagnitude(UINT64_MAX, negative));
        }
        uint64_t remainder = n % d;
        uint64_t fraction = 0;
        for (unsigned bit = 0; bit < FractionBits; ++bit) {
            fraction <<= 1;
            // Doubling a remainder this way also works for denominator == 2^63.
            if (remainder >= d - remainder) {
                remainder -= d - remainder;
                fraction |= 1u;
            } else {
                remainder *= 2u;
            }
        }
        const uint64_t raw = whole * SCALE + fraction;
        const bool round_up = remainder >= d - remainder;
        if (raw >= limit(negative)) return fromRaw(signedMagnitude(limit(negative), negative));
        return fromRaw(signedMagnitude(raw + (round_up ? 1u : 0u), negative));
    }

    /** @return Underlying signed fixed-point representation. */
    constexpr Storage raw() const noexcept { return raw_; }
    /** @return Integer part, truncated towards zero. */
    constexpr int64_t toInteger() const noexcept { return int64_t(raw_) / int64_t(SCALE); }
    /** @return Explicit conversion for host models and float-facing diagnostics. */
    constexpr float toFloat() const noexcept { return float(raw_) / float(SCALE); }

    [[gnu::always_inline]] constexpr FixedPointType operator-() const noexcept {
        return fromRaw(raw_ == std::numeric_limits<Storage>::min()
                       ? std::numeric_limits<Storage>::max() : -raw_);
    }
    [[gnu::always_inline]] constexpr FixedPointType operator+(FixedPointType rhs) const noexcept {
#if defined(__ARM_FEATURE_DSP) && defined(__GNUC__) && __GNUC__ >= 9 && !defined(__clang__)
        if constexpr (sizeof(Storage) == sizeof(int32_t)) {
            if (!__builtin_is_constant_evaluated()) return fromRaw(__builtin_arm_qadd(raw_, rhs.raw_));
        }
#endif
        if (rhs.raw_ > 0 && raw_ > std::numeric_limits<Storage>::max() - rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::max());
        if (rhs.raw_ < 0 && raw_ < std::numeric_limits<Storage>::min() - rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::min());
        return fromRaw(static_cast<Storage>(raw_ + rhs.raw_));
    }
    [[gnu::always_inline]] constexpr FixedPointType operator-(FixedPointType rhs) const noexcept {
#if defined(__ARM_FEATURE_DSP) && defined(__GNUC__) && __GNUC__ >= 9 && !defined(__clang__)
        if constexpr (sizeof(Storage) == sizeof(int32_t)) {
            if (!__builtin_is_constant_evaluated()) return fromRaw(__builtin_arm_qsub(raw_, rhs.raw_));
        }
#endif
        if (rhs.raw_ < 0 && raw_ > std::numeric_limits<Storage>::max() + rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::max());
        if (rhs.raw_ > 0 && raw_ < std::numeric_limits<Storage>::min() + rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::min());
        return fromRaw(static_cast<Storage>(raw_ - rhs.raw_));
    }
    constexpr FixedPointType& operator+=(FixedPointType rhs) noexcept { return *this = *this + rhs; }
    constexpr FixedPointType& operator-=(FixedPointType rhs) noexcept { return *this = *this - rhs; }
    constexpr FixedPointType operator*(FixedPointType rhs) const noexcept {
        static_assert(sizeof(Storage) <= 4, "Use an explicit widened product for accumulator operands");
        return FixedPointType(FixedPointType<2 * FractionBits, int64_t>::fromRaw(int64_t(raw_) * rhs.raw_));
    }
    constexpr FixedPointType operator/(FixedPointType rhs) const noexcept {
        static_assert(sizeof(Storage) <= 4, "Division operands must fit in 32 bits");
        return fromWideRaw(fixed_detail::roundedDivide(int64_t(raw_) * int64_t(SCALE), rhs.raw_));
    }
    constexpr FixedPointType operator%(FixedPointType rhs) const noexcept {
        if (rhs.raw_ == 0 || rhs.raw_ == -1) return FixedPointType{};
        return fromRaw(static_cast<Storage>(raw_ % rhs.raw_));
    }
    constexpr bool operator==(FixedPointType rhs) const noexcept { return raw_ == rhs.raw_; }
    constexpr bool operator!=(FixedPointType rhs) const noexcept { return raw_ != rhs.raw_; }
    constexpr bool operator<(FixedPointType rhs) const noexcept { return raw_ < rhs.raw_; }
    constexpr bool operator>(FixedPointType rhs) const noexcept { return raw_ > rhs.raw_; }
    constexpr bool operator<=(FixedPointType rhs) const noexcept { return raw_ <= rhs.raw_; }
    constexpr bool operator>=(FixedPointType rhs) const noexcept { return raw_ >= rhs.raw_; }

private:
    static constexpr uint64_t limit(bool negative) noexcept {
        return uint64_t(std::numeric_limits<Storage>::max()) + (negative ? 1u : 0u);
    }
    [[gnu::always_inline]] static constexpr Storage signedMagnitude(uint64_t value, bool negative) noexcept {
        if (value >= limit(negative)) {
            return negative ? std::numeric_limits<Storage>::min() : std::numeric_limits<Storage>::max();
        }
        return static_cast<Storage>(negative ? -int64_t(value) : int64_t(value));
    }
    Storage raw_ = 0;
};

/** @brief Multiply 32-bit operands into an explicitly chosen result format. */
template<class Out, unsigned A, typename SA, unsigned B, typename SB>
[[gnu::always_inline]] constexpr Out multiplyTo(FixedPointType<A, SA> a, FixedPointType<B, SB> b) noexcept {
    static_assert(sizeof(SA) <= 4 && sizeof(SB) <= 4, "Multiply operands must fit in 32 bits");
    const int64_t product = int64_t(a.raw()) * b.raw();
    if constexpr (A + B > Out::FRACTION_BITS) {
        constexpr unsigned shift = A + B - Out::FRACTION_BITS;
        constexpr int64_t half = int64_t(1) << (shift - 1u);
        // A signed 32x32 product is at most 2^62 in magnitude. Adding the
        // rounding bias (at most 2^61) is therefore always representable.
        return Out::fromWideRaw((product + half - int64_t(product < 0)) >> shift);
    } else {
        return Out(FixedPointType<A + B, int64_t>::fromRaw(product));
    }
}

/** @brief Divide by an integer, preserving the destination's fractional precision. */
template<class Out, unsigned Bits, typename Storage>
constexpr Out divideByInteger(FixedPointType<Bits, Storage> value, uint32_t divisor) noexcept {
    static_assert(sizeof(Storage) <= 4, "Division input must fit in 32 bits");
    constexpr int shift = int(Out::FRACTION_BITS) - int(Bits);
    static_assert(shift <= 31 && shift >= -31, "Unsupported division rescaling");
    if constexpr (shift >= 0) {
        const int64_t numerator = int64_t(value.raw()) * (int64_t(1) << shift);
        return Out::fromWideRaw(fixed_detail::roundedDivide(numerator, divisor));
    } else {
        const int64_t denominator = int64_t(divisor) * (int64_t(1) << -shift);
        return Out::fromWideRaw(fixed_detail::roundedDivide(value.raw(), denominator));
    }
}

/** @brief Compile-time divisor, allowing shifts instead of a generic division call. */
template<class Out, uint32_t Divisor, unsigned Bits, typename Storage>
[[gnu::always_inline]] constexpr Out divideByConstant(FixedPointType<Bits, Storage> value) noexcept {
    static_assert(Divisor > 0, "Constant divisor must be positive");
    if constexpr ((Divisor & (Divisor - 1u)) == 0) {
        constexpr unsigned log2 = [] {
            unsigned count = 0;
            for (uint32_t n = Divisor; n > 1; n >>= 1) ++count;
            return count;
        }();
        // Extra fractional bits encode division without losing small remainders.
        return Out(FixedPointType<Bits + log2, int64_t>::fromRaw(value.raw()));
    } else {
        return divideByInteger<Out>(value, Divisor);
    }
}

/** @brief Float counterparts of the explicit-format arithmetic operations. */
template<class Out>
constexpr Out multiplyTo(float a, float b) noexcept { return static_cast<Out>(a * b); }
template<class Out>
constexpr Out divideByInteger(float value, uint32_t divisor) noexcept {
    return static_cast<Out>(value / static_cast<float>(divisor));
}
template<class Out, uint32_t Divisor>
constexpr Out divideByConstant(float value) noexcept { return Out(value / Divisor); }

} // namespace libecu
#endif // LIBECU_FIXED_POINT_HPP
