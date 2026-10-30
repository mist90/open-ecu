/** @file main.cpp
 * @brief Numerical boundaries and PI behaviour across representations and sample rates
 */
#include "../../libecu/include/pi_controller.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <type_traits>

using namespace libecu;

static int failures = 0;
static int checks = 0;
static uint64_t oracle_cases = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

// Independent wide-integer oracle: rational arithmetic rather than the
// implementation's shifts. Include random full-range raw values, not just
// plausible motor inputs, so saturation and signed boundary cases remain exact.
template<class Out>
static constexpr typename Out::StorageType oracleRaw(__int128 numerator, __int128 denominator = 1) {
    const __int128 rounded = numerator < 0 ? -((-numerator + denominator / 2) / denominator)
                                          : (numerator + denominator / 2) / denominator;
    using S = typename Out::StorageType;
    return static_cast<S>(std::max<__int128>(std::numeric_limits<S>::min(),
                          std::min<__int128>(rounded, std::numeric_limits<S>::max())));
}

static uint64_t nextRandom(uint64_t& state) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
}

template<typename S, class Verify>
static void visitRawInputs(Verify verify) {
    if constexpr (sizeof(S) <= 2) {
        // Exhaustive inputs for both narrow storage types.
        for (int32_t raw = std::numeric_limits<S>::min(); raw <= std::numeric_limits<S>::max(); ++raw)
            verify(raw);
    } else {
        uint64_t random = 0x9162347abacdf109ULL;
        for (unsigned index = 0; index < 10000; ++index) verify(static_cast<S>(nextRandom(random)));
    }
    for (unsigned bit = 0; bit <= std::numeric_limits<S>::digits; ++bit) {
        for (int sign : {-1, 1}) {
            for (int offset : {-1, 0, 1}) {
                const __int128 raw = sign * (__int128(1) << bit) + offset;
                if (raw >= std::numeric_limits<S>::min() && raw <= std::numeric_limits<S>::max()) verify(raw);
            }
        }
    }
}

template<class Out, class In>
static void testConversion() {
    using S = typename In::StorageType;
    bool exact = true;
    const auto verify = [&](const __int128 candidate) {
        if (candidate < std::numeric_limits<S>::min() || candidate > std::numeric_limits<S>::max()) return;
        const S raw = static_cast<S>(candidate);
        const auto expected = oracleRaw<Out>(candidate * Out::SCALE, In::SCALE);
        ++oracle_cases;
        if (Out(In::fromRaw(raw)).raw() != expected) {
            if (exact) std::printf("Conversion Q%u/%zu -> Q%u/%zu, raw %lld\n", In::FRACTION_BITS,
                                  sizeof(S) * 8, Out::FRACTION_BITS, sizeof(typename Out::StorageType) * 8,
                                  static_cast<long long>(raw));
            exact = false;
        }
        if constexpr (In::FRACTION_BITS == 0) exact &= Out::fromInteger(raw).raw() == expected;
    };
    visitRawInputs<S>(verify);
    // Rounding ties and adjacent inputs at zero and both saturation thresholds.
    const __int128 anchors[] = {std::numeric_limits<typename Out::StorageType>::min(), -2, -1, 0, 1, 2,
                               std::numeric_limits<typename Out::StorageType>::max()};
    for (const auto anchor : anchors) {
        const __int128 tie = (anchor * In::SCALE + In::SCALE / 2) / Out::SCALE;
        for (int offset = -2; offset <= 2; ++offset) verify(tie + offset);
    }
    if constexpr (In::FRACTION_BITS > Out::FRACTION_BITS) {
        const __int128 half = (__int128(1) << (In::FRACTION_BITS - Out::FRACTION_BITS)) / 2;
        for (int offset = -2; offset <= 2; ++offset) {
            verify(__int128(std::numeric_limits<S>::max()) - half + offset);
            verify(__int128(std::numeric_limits<S>::min()) + half + offset);
        }
    }
    check(exact, "fixed conversion matches the wide rational oracle over boundaries and full-range inputs");
}

template<unsigned A, unsigned B, class Out, typename SA = int32_t, typename SB = int32_t>
static void testProducts() {
    using Left = FixedPointType<A, SA>;
    using Right = FixedPointType<B, SB>;
    uint64_t random = 0xb83902147ea3675fULL;
    bool exact = true;
    const auto verify = [&](const __int128 a, const __int128 b) {
        if (a < std::numeric_limits<SA>::min() || a > std::numeric_limits<SA>::max() ||
            b < std::numeric_limits<SB>::min() || b > std::numeric_limits<SB>::max()) return;
        const auto left = Left::fromRaw(static_cast<SA>(a));
        const auto right = Right::fromRaw(static_cast<SB>(b));
        const auto expected = oracleRaw<Out>(a * b * Out::SCALE, __int128(Left::SCALE) * Right::SCALE);
        ++oracle_cases;
        if (multiplyTo<Out>(left, right).raw() != expected) {
            if (exact) std::printf("Product Q%u/%zu * Q%u/%zu -> Q%u/%zu, raw %lld * %lld\n",
                                  A, sizeof(SA) * 8, B, sizeof(SB) * 8, Out::FRACTION_BITS,
                                  sizeof(typename Out::StorageType) * 8,
                                  static_cast<long long>(a), static_cast<long long>(b));
            exact = false;
        }
        if constexpr (std::is_same<Out, Left>::value && std::is_same<Out, Right>::value)
            exact &= (left * right).raw() == expected;
    };
    for (unsigned i = 0; i < 10000; ++i) {
        nextRandom(random);
        verify(static_cast<SA>(random), static_cast<SB>(random >> 32));
    }
    const SA left_edges[] = {std::numeric_limits<SA>::min(), SA(std::numeric_limits<SA>::min() + 1),
                            -2, -1, 0, 1, 2, SA(std::numeric_limits<SA>::max() - 1),
                            std::numeric_limits<SA>::max()};
    const SB right_edges[] = {std::numeric_limits<SB>::min(), SB(std::numeric_limits<SB>::min() + 1),
                             -2, -1, 0, 1, 2, SB(std::numeric_limits<SB>::max() - 1),
                             std::numeric_limits<SB>::max()};
    for (SA a : left_edges) for (SB b : right_edges) verify(a, b);
    // Factor exact ties and saturation boundaries in different ways, including
    // negative products with zero operands and the INT_MIN * INT_MIN endpoint.
    const __int128 anchors[] = {std::numeric_limits<typename Out::StorageType>::min(), -2, -1, 0, 1, 2,
                               std::numeric_limits<typename Out::StorageType>::max()};
    const __int128 denominator = __int128(Left::SCALE) * Right::SCALE;
    for (unsigned bit = 0; bit <= std::numeric_limits<SA>::digits; ++bit) {
        for (int sign : {-1, 1}) {
            for (int neighbor : {-1, 0, 1}) {
                const __int128 a = sign * (__int128(1) << bit) + neighbor;
                if (a == 0) continue;
                for (const auto anchor : anchors) {
                    const __int128 target = (anchor * denominator + denominator / 2) / Out::SCALE;
                    for (int offset : {-1, 0, 1}) verify(a, target / a + offset);
                }
            }
        }
    }
    if constexpr (sizeof(SA) == 1 && sizeof(SB) == 1) {
        for (int a = std::numeric_limits<SA>::min(); a <= std::numeric_limits<SA>::max(); ++a)
            for (int b = std::numeric_limits<SB>::min(); b <= std::numeric_limits<SB>::max(); ++b)
                verify(a, b);
    }
    check(exact, "mixed fixed multiplication matches exact rational rounding and saturation");
}

template<unsigned Shift, class Storage>
static void testRightShifts() {
    testConversion<FixedPointType<0, Storage>, FixedPointType<Shift, Storage>>();
    if constexpr (Shift > 1) testRightShifts<Shift - 1, Storage>();
}

template<unsigned Shift>
static void testProductShifts() {
    using Storage = std::conditional_t<(62 - Shift > 31), int64_t, int32_t>;
    testProducts<31, 31, FixedPointType<62 - Shift, Storage>>();
    if constexpr (Shift <= 31) testProducts<Shift, 0, FixedPointType<0>>();
    if constexpr (Shift > 1) testProductShifts<Shift - 1>();
}

template<class Out>
static void testWideRaw() {
    bool exact = true;
    const auto verify = [&](const __int128 raw) {
        if (raw < INT64_MIN || raw > INT64_MAX) return;
        ++oracle_cases;
        exact &= Out::fromWideRaw(static_cast<int64_t>(raw)).raw() == oracleRaw<Out>(raw);
    };
    visitRawInputs<int64_t>(verify);
    for (int offset : {-1, 0, 1}) {
        verify(__int128(std::numeric_limits<typename Out::StorageType>::min()) + offset);
        verify(__int128(std::numeric_limits<typename Out::StorageType>::max()) + offset);
    }
    check(exact, "fromWideRaw matches the wide oracle at both saturation endpoints and full-range inputs");
}

template<class Out, uint32_t Divisor, class In>
static void testConstantDivision() {
    bool exact = true;
    visitRawInputs<typename In::StorageType>([&](const __int128 raw) {
        ++oracle_cases;
        const auto expected = oracleRaw<Out>(raw * Out::SCALE, __int128(Divisor) * In::SCALE);
        exact &= divideByConstant<Out, Divisor>(In::fromRaw(static_cast<typename In::StorageType>(raw))).raw()
                 == expected;
    });
    check(exact, "constant division preserves exact rational rounding across native and widened encodings");
}

template<unsigned Bits, class Storage>
static void testConversionDestinations() {
    using In = FixedPointType<Bits, Storage>;
    testConversion<FixedPointType<0>, In>();
    testConversion<FixedPointType<4, int8_t>, In>();
    testConversion<FixedPointType<0, int16_t>, In>();
    testConversion<FixedPointType<15, int16_t>, In>();
    testConversion<FixedPointType<12>, In>();
    testConversion<FixedPointType<18>, In>();
    testConversion<FixedPointType<24>, In>();
    testConversion<FixedPointType<30>, In>();
    testConversion<FixedPointType<31>, In>();
    testConversion<FixedPointType<0, int64_t>, In>();
    testConversion<FixedPointType<32, int64_t>, In>();
    testConversion<FixedPointType<62, int64_t>, In>();
}

template<class T> static T number(int64_t numerator, int64_t denominator = 1) {
    if constexpr (std::is_same<T, float>::value) return float(numerator) / float(denominator);
    else return T::fromRatio(numerator, denominator);
}
template<class T> static float asFloat(T value) {
    if constexpr (std::is_same<T, float>::value) return value;
    else return value.toFloat();
}

static void testFixed() {
    using Q = FixedPointType<16>;
    using Tiny = FixedPointType<4, int8_t>;
    using Acc = FixedPointType<32, int64_t>;
    static_assert(Q::fromRatio(3, 2).raw() == 98304, "constexpr rational");
    static_assert(!std::is_convertible<Q, float>::value, "No implicit float arithmetic");
    static_assert(!std::is_convertible<Q, FixedPointType<8>>::value, "Explicit narrowing");
    static_assert((Q::fromRaw(INT32_MAX) + Q::fromRaw(1)).raw() == INT32_MAX, "constexpr add overflow");
    static_assert((Q::fromRaw(INT32_MIN) - Q::fromRaw(1)).raw() == INT32_MIN, "constexpr subtract overflow");
    static_assert(FixedPointType<0>(FixedPointType<1>::fromRaw(INT32_MAX)).raw() == 1073741824,
                  "constexpr native rounding at positive storage limit");
    static_assert(FixedPointType<0>(FixedPointType<1>::fromRaw(-1)).raw() == -1,
                  "constexpr native negative half rounds away");
    static_assert(FixedPointType<0, int64_t>(FixedPointType<1, int64_t>::fromRaw(INT64_MAX)).raw() ==
                  (int64_t(1) << 62), "constexpr wide rounding cannot overflow its bias");
    static_assert(FixedPointType<0>(FixedPointType<32, int64_t>::fromRaw(INT64_MAX)).raw() == INT32_MAX,
                  "constexpr 32-bit discard retains saturation");
    static_assert(FixedPointType<0, int8_t>(FixedPointType<8, int16_t>::fromRaw(INT16_MAX)).raw() == INT8_MAX,
                  "constexpr sole positive endpoint saturates for small storage");
    static_assert(FixedPointType<0, int64_t>(FixedPointType<62, int64_t>::fromRaw(INT64_MIN)).raw() == -2,
                  "constexpr maximum discard handles the wide minimum");
    static_assert(FixedPointType<0, int8_t>(FixedPointType<1, int8_t>::fromRaw(INT8_MAX)).raw() == 64,
                  "constexpr small-storage rounding");
    static_assert(multiplyTo<FixedPointType<0>>(FixedPointType<15, int16_t>::fromRaw(INT16_MIN),
                  FixedPointType<15, int16_t>::fromRaw(INT16_MIN)).raw() == 1,
                  "constexpr small product fits a native intermediate");
    static_assert((FixedPointType<31>::fromRaw(INT32_MIN) * FixedPointType<31>::fromRaw(INT32_MIN)).raw() ==
                  INT32_MAX, "constexpr product rounds and saturates its positive endpoint");
    static_assert(Tiny::fromWideRaw(INT64_MIN).raw() == INT8_MIN &&
                  Tiny::fromWideRaw(INT64_MAX).raw() == INT8_MAX, "constexpr sign-mask saturation");
    {
        const int32_t edges[] = {INT32_MIN, INT32_MIN + 1, -1, 0, 1, INT32_MAX - 1, INT32_MAX};
        uint64_t random = 0x7492adcf136897bdULL;
        bool exact = true;
        for (unsigned i = 0; i < 10049; ++i) {
            random ^= random << 13; random ^= random >> 7; random ^= random << 17;
            const int32_t a = i < 49 ? edges[i / 7] : int32_t(random);
            const int32_t b = i < 49 ? edges[i % 7] : int32_t(random >> 32);
            const auto clamp = [](int64_t value) {
                return std::max<int64_t>(INT32_MIN, std::min<int64_t>(INT32_MAX, value));
            };
            exact &= (Q::fromRaw(a) + Q::fromRaw(b)).raw() == clamp(int64_t(a) + b);
            exact &= (Q::fromRaw(a) - Q::fromRaw(b)).raw() == clamp(int64_t(a) - b);
        }
        check(exact, "32-bit saturating sums/differences match wide oracle, boundaries and random inputs");
    }
    check(Q::fromRatio(-3, 2).raw() == -98304, "negative rational");
    check(Tiny::fromRatio(1, 32).raw() == 1, "positive half rounds away from zero");
    check(Tiny::fromRatio(-1, 32).raw() == -1, "negative half rounds away from zero");
    check(Tiny(Q::fromRatio(-1, 32)).raw() == -1, "narrowing uses the same rounding rule");
    check(Q(Tiny::fromRatio(-3, 2)).raw() == -98304, "negative widening is exact");
    check(Q::fromRatio(INT64_MIN, INT64_MIN) == Q::fromRatio(1), "extreme rational denominators");
    check(Q::fromRatio(INT64_MAX, 1).raw() == INT32_MAX, "rational overflow saturates");
    check(Q::fromRatio(INT64_MIN, 1).raw() == INT32_MIN, "negative rational overflow saturates");
    check(Q::fromRatio(1, 0).raw() == INT32_MAX, "positive division by zero saturates");
    check(Q::fromRatio(-1, 0).raw() == INT32_MIN, "negative division by zero saturates");
    check(Q::fromRatio(0, 0).raw() == 0, "zero divided by zero has defined output");
    check((-Q::fromRaw(INT32_MIN)).raw() == INT32_MAX, "negation of minimum saturates");
    check((Q::fromRaw(INT32_MAX) + Q::fromRaw(1)).raw() == INT32_MAX, "addition saturates high");
    check((Q::fromRaw(INT32_MIN) - Q::fromRaw(1)).raw() == INT32_MIN, "subtraction saturates low");
    check((Acc::fromRaw(INT64_MIN) - Acc::fromRaw(1)).raw() == INT64_MIN, "wide subtraction boundary");
    check((Acc::fromRaw(INT64_MAX) + Acc::fromRaw(1)).raw() == INT64_MAX, "wide addition boundary");
    check(Q(Acc::fromRaw(INT64_MIN)).raw() == INT32_MIN, "wide negative narrowing boundary");
    check(Acc(Q::fromRatio(-3, 2)).raw() == -6442450944LL, "widening preserves fractional bits");
    check(divideByInteger<Acc>(Q::fromRatio(1), 20000).raw() == 214748,
          "per-tick division preserves extra destination precision");
    check(multiplyTo<Acc>(Q::fromRatio(-3, 2), Q::fromRatio(5, 4)).raw() == -8053063680LL,
          "mixed accumulator product retains exact physical value");

    bool clipped = true;
    check(Q::fromFloat(-1.5f, &clipped).raw() == -98304 && !clipped, "float boundary preserves sign and scale");
    check(Tiny::fromFloat(0.03125f).raw() == 1, "float positive tie rounds away from zero");
    check(Tiny::fromFloat(-0.03125f).raw() == -1, "float negative tie rounds away from zero");
    check(Q::fromFloat(-0.0f, &clipped).raw() == 0 && !clipped, "negative float zero is exact");
    check(Q::fromFloat(std::numeric_limits<float>::denorm_min(), &clipped).raw() == 0 && !clipped,
          "subnormal float rounds to fixed zero");
    check(Q::fromFloat(32768.0f, &clipped).raw() == INT32_MAX && clipped, "float upper boundary is detected");
    check(Q::fromFloat(-32768.0f, &clipped).raw() == INT32_MIN && !clipped, "float lower boundary is representable");
    check(Acc::fromFloat(-2147483648.0f, &clipped).raw() == INT64_MIN && !clipped,
          "float conversion handles the signed 64-bit minimum");
    check(Acc::fromFloat(2147483648.0f, &clipped).raw() == INT64_MAX && clipped,
          "float conversion saturates above the signed 64-bit maximum");
    check(Q::fromFloat(std::numeric_limits<float>::infinity(), &clipped).raw() == INT32_MAX && clipped,
          "float infinity is reported as clipped");
    check(Q::fromFloat(std::numeric_limits<float>::quiet_NaN(), &clipped).raw() == 0 && clipped,
          "float NaN has a defined rejected representation");

    for (int a = -100; a <= 100; ++a) {
        for (int b : {-97, -1, 0, 1, 97}) {
            const auto result = multiplyTo<FixedPointType<18>>(Q::fromRatio(a, 4),
                                                               FixedPointType<20>::fromRatio(b, 8));
            check(std::abs(result.toFloat() - float(a * b) / 32.0f) < 1e-5f,
                  "mixed-format product across signs");
        }
    }
}

template<class Types, uint32_t Frequency>
static void testPi() {
    using Controller = PiController<Types, Frequency>;
    using Input = typename Controller::Input;
    using Output = typename Controller::Output;
    using Gain = typename Types::Gain;
    typename Controller::Parameters params{};
    params.ki = number<Gain>(2);
    params.max_output = params.integral_max = number<Output>(1);
    params.min_output = params.integral_min = number<Output>(-1);
    Controller controller(params);
    const Input half = number<Input>(1, 2);
    // Prime trapezoidal history with integration blocked by the inner loop.
    controller.update(half, Input{}, +1);
    for (uint32_t tick = 0; tick < Frequency / 2; ++tick) controller.update(half, Input{});
    check(std::abs(asFloat(controller.getIntegral()) - 0.5f) < 0.0003f,
          "PI integrates physical error for half a second at any frequency");

    const float held = asFloat(controller.getIntegral());
    for (int tick = 0; tick < 20; ++tick) controller.update(half, Input{}, +1);
    check(asFloat(controller.getIntegral()) == held, "downstream high limit freezes positive integration");
    for (int tick = 0; tick < 20; ++tick) controller.update(-half, Input{}, +1);
    check(asFloat(controller.getIntegral()) < held, "opposing error can unwind the high limit");

    controller.reset();
    for (uint32_t tick = 0; tick < Frequency * 2; ++tick) controller.update(half, Input{});
    check(std::abs(asFloat(controller.getIntegral()) - 1.0f) < 1e-6f, "integral clamps at its physical bound");
    params.integral_max = number<Output>(1, 4);
    controller.setParameters(params);
    check(std::abs(asFloat(controller.getIntegral()) - 0.25f) < 1e-6f, "retuning clamps existing integral");

    params.kp = number<Gain>(4);
    params.ki = Gain{};
    params.kb = number<Gain>(100);
    params.integral_max = number<Output>(1);
    controller.setParameters(params);
    controller.reset();
    controller.update(half, Input{});
    check(controller.saturationSign() == 1, "upper saturation is reported");
    check(asFloat(controller.getIntegral()) < 0.0f, "back-calculation opposes excess positive output");
    controller.reset();
    controller.update(-half, Input{});
    check(controller.saturationSign() == -1, "lower saturation is reported");
    check(asFloat(controller.getIntegral()) > 0.0f, "back-calculation opposes excess negative output");
}

static void testSmallIncrements() {
    using Types = FixedProfile::SpeedPiTypes;
    PiParametersT<Types> params{};
    params.ki = Types::Gain::fromRatio(1, 5);
    params.max_output = params.integral_max = Types::Output::fromRatio(1);
    params.min_output = params.integral_min = Types::Output::fromRatio(-1);
    PiController<Types, 100> controller(params);
    const auto error = Types::Input::fromRatio(1, 10000);
    for (int tick = 0; tick < 10000; ++tick) controller.update(error, Types::Input{});
    check(controller.getIntegral().toFloat() > 0.0019f,
          "sub-output-LSB PI increments accumulate instead of disappearing");
    check(controller.getIntegral().toFloat() < 0.0023f, "small-error accumulation matches elapsed time");
}

int main() {
    testRightShifts<7, int8_t>();
    testRightShifts<15, int16_t>();
    testRightShifts<31, int32_t>();
    testRightShifts<62, int64_t>();
    testProductShifts<62>();
    testProducts<20, 16, FixedPointType<12>>();
    testProducts<20, 30, FixedPointType<20>>();
    testProducts<24, 14, FixedPointType<14>>();
    testProducts<14, 30, FixedPointType<32, int64_t>>();
    testProducts<18, 30, FixedPointType<18>>();
    testProducts<16, 20, FixedPointType<32, int64_t>>();
    testProducts<30, 30, FixedPointType<30>>();
    testProducts<0, 0, FixedPointType<32, int64_t>>();
    testProducts<0, 0, FixedPointType<0>>();
    testProducts<16, 16, FixedPointType<16>>();
    testProducts<31, 31, FixedPointType<31>>();
    testProducts<0, 0, FixedPointType<62, int64_t>>();
    testProducts<7, 7, FixedPointType<7, int8_t>, int8_t, int8_t>();
    testProducts<7, 7, FixedPointType<0>, int8_t, int8_t>();
    testProducts<0, 0, FixedPointType<7, int8_t>, int8_t, int8_t>();
    testProducts<1, 0, FixedPointType<0>, int16_t, int16_t>();
    testProducts<15, 15, FixedPointType<0>, int16_t, int16_t>();
    testProducts<15, 15, FixedPointType<15, int16_t>, int16_t, int16_t>();
    testProducts<7, 15, FixedPointType<7, int8_t>, int8_t, int16_t>();
    testProducts<15, 31, FixedPointType<14, int16_t>, int16_t, int32_t>();
    testProducts<31, 15, FixedPointType<15, int16_t>, int32_t, int16_t>();
    testProducts<31, 31, FixedPointType<0, int8_t>>();
    testConversionDestinations<0, int32_t>();
    testConversionDestinations<4, int8_t>();
    testConversionDestinations<7, int8_t>();
    testConversionDestinations<0, int16_t>();
    testConversionDestinations<15, int16_t>();
    testConversionDestinations<12, int32_t>();
    testConversionDestinations<18, int32_t>();
    testConversionDestinations<24, int32_t>();
    testConversionDestinations<30, int32_t>();
    testConversionDestinations<31, int32_t>();
    testConversionDestinations<0, int64_t>();
    testConversionDestinations<32, int64_t>();
    testConversionDestinations<62, int64_t>();
    testConversion<FixedPointType<0, int8_t>, FixedPointType<8, int16_t>>();
    testConversion<FixedPointType<0, int8_t>, FixedPointType<24>>();
    testConversion<FixedPointType<0, int16_t>, FixedPointType<16>>();
    testConversion<FixedPointType<0, int8_t>, FixedPointType<56, int64_t>>();
    testConversion<FixedPointType<0, int16_t>, FixedPointType<48, int64_t>>();
    testWideRaw<FixedPointType<4, int8_t>>();
    testWideRaw<FixedPointType<12, int16_t>>();
    testWideRaw<FixedPointType<20>>();
    testWideRaw<FixedPointType<32, int64_t>>();
    testConstantDivision<FixedPointType<20>, 8, FixedPointType<20>>();
    testConstantDivision<FixedPointType<0>, 2, FixedPointType<31>>();
    testConstantDivision<FixedPointType<31>, 0x80000000u, FixedPointType<31>>();
    testConstantDivision<FixedPointType<62, int64_t>, 0x80000000u, FixedPointType<0>>();
    testConstantDivision<FixedPointType<4, int8_t>, 4, FixedPointType<4, int8_t>>();
    testConstantDivision<FixedPointType<0>, 2, FixedPointType<7, int8_t>>();
    testConstantDivision<FixedPointType<15, int16_t>, 32768, FixedPointType<15, int16_t>>();
    testConstantDivision<FixedPointType<15, int16_t>, 1, FixedPointType<0, int16_t>>();
    testConstantDivision<FixedPointType<20>, 2, FixedPointType<32, int64_t>>();
    testFixed();
    testPi<FloatProfile::PiTypes, 100>();
    testPi<FloatProfile::PiTypes, 20000>();
    testPi<FixedProfile::SpeedPiTypes, 100>();
    testPi<FixedProfile::SpeedPiTypes, 1000>();
    testPi<FixedProfile::CurrentPiTypes, 20000>();
    testPi<FixedProfile::CurrentPiTypes, 40000>();
    testSmallIncrements();
    static_assert(SampleClock<20000>::ticksFromUs(300) == 6, "exact persistence threshold");
    static_assert(SampleClock<20000>::ticksFromUs(301) == 7, "threshold rounds up");
    static_assert(SampleClock<20000>::ticksToUs(6) == 300, "integer telemetry duration");
    std::printf("Numeric/PI: %d checks, %llu exact oracle cases, %d failures\n",
                checks, static_cast<unsigned long long>(oracle_cases), failures);
    return failures != 0;
}
