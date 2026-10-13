/** @file profile_math.hpp
 * @brief Integer trig/voltage limits and representation-independent FOC transforms
 */
#ifndef LIBECU_PROFILE_MATH_HPP
#define LIBECU_PROFILE_MATH_HPP

#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"
#include "foc_math.hpp"

namespace libecu {

/** @brief Fixed-point math backend; LUT values are signed Q30. */
class FixedMath {
public:
    using Angle = FixedProfile::Angle;
    using Fraction = FixedProfile::Fraction;
    using Voltage = FixedProfile::Voltage;
    using Request = FixedProfile::VoltageRequest;
    inline static constexpr Angle TWO_PI = number<Angle>(628318530718LL, 100000000000LL);
    static_assert(Angle::FRACTION_BITS <= 28, "A signed 32-bit angle must hold a full radian revolution");

    static void sinCos(Angle angle, Fraction& sine, Fraction& cosine) noexcept {
        int32_t wrapped = angle.raw() % TWO_PI.raw();
        if (wrapped < 0) wrapped += TWO_PI.raw();
        // Reciprocal rounded down: error < 7 phase counts (one count is 2^-32 turn).
        constexpr uint32_t phase_scale = (uint64_t(1) << (Angle::FRACTION_BITS + 32)) / uint32_t(TWO_PI.raw());
        const uint32_t phase = static_cast<uint32_t>((uint64_t(wrapped) * phase_scale) >> Angle::FRACTION_BITS);
        const unsigned index = phase >> 24;
        const uint32_t fraction = phase & 0xffffffu;
        sine = Fraction(FixedPointType<30>::fromRaw(interpolate(index, fraction)));
        cosine = Fraction(FixedPointType<30>::fromRaw(interpolate(index + 64u, fraction)));
    }

    /** @brief d-priority circular voltage limit, rounded inward. */
    static Voltage qVoltageLimit(Voltage d, Voltage limit) noexcept {
        if (limit <= Voltage{}) return Voltage{};
        d = numericClamp(d, -limit, limit);
        const uint64_t room = uint64_t(int64_t(limit.raw()) * limit.raw()) -
                              uint64_t(int64_t(d.raw()) * d.raw());
        return Voltage::fromWideRaw(integerSqrt(room));
    }
    static Voltage voltageMagnitude(Voltage d, Voltage q) noexcept {
        return Voltage::fromWideRaw(integerSqrt(squared(d.raw(), q.raw())));
    }
    static bool voltageLimitExceeded(Request d, Request q, Voltage limit) noexcept {
        static_assert(Voltage::FRACTION_BITS >= Request::FRACTION_BITS,
                      "Unclamped requests require at least the applied voltage's range");
        if (limit <= Voltage{}) return d != Request{} || q != Request{};
        constexpr unsigned shift = 2u * (Voltage::FRACTION_BITS - Request::FRACTION_BITS);
        const uint64_t limit_sq = uint64_t(int64_t(limit.raw()) * limit.raw());
        return squared(d.raw(), q.raw()) > (limit_sq >> shift);
    }

    /** @brief Natural logarithm of a positive bounded integer ratio (NTC divider). */
    static FixedProfile::Logarithm logRatio(uint64_t numerator, uint64_t denominator) noexcept {
        using Log = FixedProfile::Logarithm;
        if (!numerator || !denominator) return Log{};
        int exponent = 0;
        // ADC/resistor products are at most 48 bits, leaving normalization headroom.
        while (numerator < denominator) { numerator <<= 1; --exponent; }
        while (numerator >= denominator * 2u) { denominator <<= 1; ++exponent; }
        const Fraction y = Fraction::fromRatio(numerator - denominator, numerator + denominator);
        const Fraction y2 = multiplyTo<Fraction>(y, y);
        // ln(x) = 2*atanh((x-1)/(x+1)); |y| <= 1/3 after normalization.
        Fraction polynomial = number<Fraction>(1, 11);
        polynomial = number<Fraction>(1, 9) + multiplyTo<Fraction>(y2, polynomial);
        polynomial = number<Fraction>(1, 7) + multiplyTo<Fraction>(y2, polynomial);
        polynomial = number<Fraction>(1, 5) + multiplyTo<Fraction>(y2, polynomial);
        polynomial = number<Fraction>(1, 3) + multiplyTo<Fraction>(y2, polynomial);
        polynomial = number<Fraction>(1) + multiplyTo<Fraction>(y2, polynomial);
        constexpr Log ln2 = number<Log>(6931471805599LL, 10000000000000LL);
        return scaleInteger<Log>(numberCast<Log>(multiplyTo<Fraction>(y, polynomial)), 2)
             + scaleInteger<Log>(ln2, exponent);
    }

private:
    static uint64_t squared(int32_t a, int32_t b) noexcept {
        return uint64_t(int64_t(a) * a) + uint64_t(int64_t(b) * b);
    }
    static int32_t sample(unsigned index) noexcept {
        index &= 255u;
        const unsigned position = index & 63u;
        switch (index >> 6) {
            case 0: return SIN_QUARTER[position];
            case 1: return SIN_QUARTER[64u - position];
            case 2: return -SIN_QUARTER[position];
            default: return -SIN_QUARTER[64u - position];
        }
    }
    static int32_t interpolate(unsigned index, uint32_t fraction) noexcept {
        const int32_t first = sample(index);
        const int64_t delta = int64_t(sample(index + 1u)) - first;
        return first + static_cast<int32_t>(fixed_detail::roundedDivide(delta * fraction, int64_t(1) << 24));
    }
    // round(sin(pi*i/128) * 2^30), i=0..64. Quadrant symmetry gives 256 samples.
    inline static constexpr int32_t SIN_QUARTER[65] = {
        0, 26350943, 52686014, 78989349, 105245103, 131437462, 157550647, 183568930,
        209476638, 235258165, 260897982, 286380643, 311690799, 336813204, 361732726, 386434353,
        410903207, 435124548, 459083786, 482766489, 506158392, 529245404, 552013618, 574449320,
        596538995, 618269338, 639627258, 660599890, 681174602, 701339000, 721080937, 740388522,
        759250125, 777654384, 795590213, 813046808, 830013654, 846480531, 862437520, 877875009,
        892783698, 907154608, 920979082, 934248793, 946955747, 959092290, 970651112, 981625251,
        992008094, 1001793390, 1010975242, 1019548121, 1027506862, 1034846671, 1041563127, 1047652185,
        1053110176, 1057933813, 1062120190, 1065666786, 1068571464, 1070832474, 1072448455, 1073418433,
        1073741824,
    };
};

template<class Scalar> struct AlphaBetaT { Scalar alpha{}; Scalar beta{}; };
template<class Scalar> struct DqVectorT { Scalar d{}; Scalar q{}; };
template<class Scalar> struct PhaseDutiesT { Scalar u{}; Scalar v{}; Scalar w{}; };

/** @brief Clarke/Park and SVPWM with explicit quantity-dependent result formats. */
template<class Profile>
struct FocTransforms {
    using Unit = typename Profile::Fraction;
    using Voltage = typename Profile::Voltage;
    using Duty = typename Profile::Duty;
    inline static constexpr Unit INV_SQRT3 = number<Unit>(577350269, 1000000000);
    inline static constexpr Unit SQRT3_OVER2 = number<Unit>(866025404, 1000000000);

    template<class Scalar>
    static AlphaBetaT<Scalar> clarke(Scalar u, Scalar v, Scalar w) noexcept {
        return {divideByInteger<Scalar>(scaleInteger<Scalar>(u, 2) - v - w, 3),
                multiplyTo<Scalar>(v - w, INV_SQRT3)};
    }
    /** @brief Clarke after reconstructing one phase: u+v+w is already zero. */
    template<class Scalar>
    static AlphaBetaT<Scalar> clarkeZeroSum(Scalar u, Scalar v, Scalar w) noexcept {
        return {u, multiplyTo<Scalar>(v - w, INV_SQRT3)};
    }
    template<class Scalar>
    static DqVectorT<Scalar> park(AlphaBetaT<Scalar> ab, Unit sine, Unit cosine) noexcept {
        return {multiplyTo<Scalar>(ab.alpha, cosine) + multiplyTo<Scalar>(ab.beta, sine),
                multiplyTo<Scalar>(-ab.alpha, sine) + multiplyTo<Scalar>(ab.beta, cosine)};
    }
    template<class Scalar>
    static AlphaBetaT<Scalar> invPark(DqVectorT<Scalar> dq, Unit sine, Unit cosine) noexcept {
        return {multiplyTo<Scalar>(dq.d, cosine) - multiplyTo<Scalar>(dq.q, sine),
                multiplyTo<Scalar>(dq.d, sine) + multiplyTo<Scalar>(dq.q, cosine)};
    }
    static Voltage voltageLimit(Voltage bus) noexcept { return multiplyTo<Voltage>(bus, INV_SQRT3); }
    static PhaseDutiesT<Duty> svpwm(Voltage alpha, Voltage beta, Voltage bus) noexcept {
        const Voltage a = alpha;
        const Voltage half_alpha = divideByConstant<Voltage, 2>(alpha);
        const Voltage beta_component = multiplyTo<Voltage>(beta, SQRT3_OVER2);
        const Voltage b = -half_alpha + beta_component;
        const Voltage c = -half_alpha - beta_component;
        const Voltage highest = a > b ? (a > c ? a : c) : (b > c ? b : c);
        const Voltage lowest = a < b ? (a < c ? a : c) : (b < c ? b : c);
        const Voltage common = divideByConstant<Voltage, 2>(highest + lowest);
        constexpr Duty half = number<Duty>(1, 2);
        if (bus <= number_v<Voltage, 1>) return {half, half, half};
        const Unit inverse_bus = divideTo<Unit>(number_v<Voltage, 1>, bus);
        return {half + multiplyTo<Duty>(a - common, inverse_bus),
                half + multiplyTo<Duty>(b - common, inverse_bus),
                half + multiplyTo<Duty>(c - common, inverse_bus)};
    }
};

} // namespace libecu
#endif // LIBECU_PROFILE_MATH_HPP
