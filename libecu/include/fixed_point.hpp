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
    explicit constexpr FixedPointType(FixedPointType<OtherBits, OtherStorage> other) noexcept {
        const bool negative = other.raw() < 0;
        uint64_t value = fixed_detail::magnitude(other.raw());
        if constexpr (FractionBits > OtherBits) {
            constexpr unsigned shift = FractionBits - OtherBits;
            if (value > (limit(negative) >> shift)) {
                raw_ = negative ? std::numeric_limits<Storage>::min() : std::numeric_limits<Storage>::max();
                return;
            }
            value <<= shift;
        } else if constexpr (FractionBits < OtherBits) {
            constexpr unsigned shift = OtherBits - FractionBits;
            constexpr uint64_t divisor = uint64_t(1) << shift;
            value = value / divisor + ((value % divisor) >= divisor / 2u ? 1u : 0u);
        }
        raw_ = signedMagnitude(value, negative);
    }

    /** @brief Construct from the underlying integer representation. */
    static constexpr FixedPointType fromRaw(Storage raw) noexcept {
        FixedPointType result;
        result.raw_ = raw;
        return result;
    }

    /** @brief Saturate a widened raw result to this storage type. */
    static constexpr FixedPointType fromWideRaw(int64_t raw) noexcept {
        return fromRaw(signedMagnitude(fixed_detail::magnitude(raw), raw < 0));
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

    constexpr FixedPointType operator-() const noexcept {
        return fromRaw(signedMagnitude(fixed_detail::magnitude(raw_), raw_ >= 0));
    }
    constexpr FixedPointType operator+(FixedPointType rhs) const noexcept {
        if (rhs.raw_ > 0 && raw_ > std::numeric_limits<Storage>::max() - rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::max());
        if (rhs.raw_ < 0 && raw_ < std::numeric_limits<Storage>::min() - rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::min());
        return fromRaw(static_cast<Storage>(raw_ + rhs.raw_));
    }
    constexpr FixedPointType operator-(FixedPointType rhs) const noexcept {
        if (rhs.raw_ < 0 && raw_ > std::numeric_limits<Storage>::max() + rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::max());
        if (rhs.raw_ > 0 && raw_ < std::numeric_limits<Storage>::min() + rhs.raw_)
            return fromRaw(std::numeric_limits<Storage>::min());
        return fromRaw(static_cast<Storage>(raw_ - rhs.raw_));
    }
    constexpr FixedPointType& operator+=(FixedPointType rhs) noexcept { return *this = *this + rhs; }
    constexpr FixedPointType& operator-=(FixedPointType rhs) noexcept { return *this = *this - rhs; }
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
    static constexpr Storage signedMagnitude(uint64_t value, bool negative) noexcept {
        if (value >= limit(negative)) {
            return negative ? std::numeric_limits<Storage>::min() : std::numeric_limits<Storage>::max();
        }
        return static_cast<Storage>(negative ? -int64_t(value) : int64_t(value));
    }
    Storage raw_ = 0;
};

/** @brief Multiply 32-bit operands into an explicitly chosen result format. */
template<class Out, unsigned A, typename SA, unsigned B, typename SB>
constexpr Out multiplyTo(FixedPointType<A, SA> a, FixedPointType<B, SB> b) noexcept {
    static_assert(sizeof(SA) <= 4 && sizeof(SB) <= 4, "Multiply operands must fit in 32 bits");
    return Out(FixedPointType<A + B, int64_t>::fromRaw(int64_t(a.raw()) * b.raw()));
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

/** @brief Float counterparts of the explicit-format arithmetic operations. */
template<class Out>
constexpr Out multiplyTo(float a, float b) noexcept { return static_cast<Out>(a * b); }
template<class Out>
constexpr Out divideByInteger(float value, uint32_t divisor) noexcept {
    return static_cast<Out>(value / static_cast<float>(divisor));
}

} // namespace libecu
#endif // LIBECU_FIXED_POINT_HPP
