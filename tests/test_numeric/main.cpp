/** @file main.cpp
 * @brief Numerical boundaries and PI behaviour across representations and sample rates
 */
#include "../../libecu/include/pi_controller.hpp"
#include <cmath>
#include <cstdio>
#include <type_traits>

using namespace libecu;

static int failures = 0;
static int checks = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

// Independent wide-integer oracle: rational arithmetic rather than the
// implementation's shifts. Include random full-range raw values, not just
// plausible motor inputs, so saturation and signed boundary cases remain exact.
template<class Out, class In>
static void testConversion() {
    uint64_t random = 0x9162347abacdf109ULL;
    bool exact = true;
    for (unsigned index = 0; index < 10000; ++index) {
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        using S = typename In::StorageType;
        const S boundaries[] = {0, 1, -1, std::numeric_limits<S>::min(), std::numeric_limits<S>::max()};
        const S raw = index < 5 ? boundaries[index] : static_cast<S>(random);
        __int128 expected = raw;
        if constexpr (Out::FRACTION_BITS >= In::FRACTION_BITS) {
            expected *= __int128(1) << (Out::FRACTION_BITS - In::FRACTION_BITS);
        } else {
            const __int128 divisor = __int128(1) << (In::FRACTION_BITS - Out::FRACTION_BITS);
            expected = expected < 0 ? -((-expected + divisor / 2) / divisor)
                                    : (expected + divisor / 2) / divisor;
        }
        expected = std::max<__int128>(std::numeric_limits<typename Out::StorageType>::min(),
                   std::min<__int128>(expected, std::numeric_limits<typename Out::StorageType>::max()));
        exact &= Out(In::fromRaw(raw)).raw() == expected;
        if constexpr (In::FRACTION_BITS == 0) exact &= Out::fromInteger(raw).raw() == expected;
    }
    check(exact, "fixed conversion matches the wide rational oracle over boundaries and full-range inputs");
}

template<unsigned A, unsigned B, class Out>
static void testProducts() {
    using Left = FixedPointType<A>;
    using Right = FixedPointType<B>;
    uint64_t random = 0xb83902147ea3675fULL;
    bool exact = true;
    for (unsigned i = 0; i < 10000; ++i) {
        random ^= random << 13; random ^= random >> 7; random ^= random << 17;
        const int32_t a = i == 0 ? INT32_MIN : int32_t(random);
        const int32_t b = i == 0 ? INT32_MIN : int32_t(random >> 32);
        __int128 expected = __int128(a) * b;
        if constexpr (A + B > Out::FRACTION_BITS) {
            const __int128 divisor = __int128(1) << (A + B - Out::FRACTION_BITS);
            expected = expected < 0 ? -((-expected + divisor / 2) / divisor)
                                    : (expected + divisor / 2) / divisor;
        } else {
            expected *= __int128(1) << (Out::FRACTION_BITS - A - B);
        }
        expected = std::max<__int128>(std::numeric_limits<typename Out::StorageType>::min(),
                   std::min<__int128>(expected, std::numeric_limits<typename Out::StorageType>::max()));
        exact &= multiplyTo<Out>(Left::fromRaw(a), Right::fromRaw(b)).raw() == expected;
    }
    check(exact, "mixed fixed multiplication matches exact rational rounding and saturation");
}

template<unsigned Bits, class Storage>
static void testConversionDestinations() {
    using In = FixedPointType<Bits, Storage>;
    testConversion<FixedPointType<0>, In>();
    testConversion<FixedPointType<4, int8_t>, In>();
    testConversion<FixedPointType<12>, In>();
    testConversion<FixedPointType<18>, In>();
    testConversion<FixedPointType<24>, In>();
    testConversion<FixedPointType<30>, In>();
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
    testProducts<20, 16, FixedPointType<12>>();
    testProducts<20, 30, FixedPointType<20>>();
    testProducts<24, 14, FixedPointType<14>>();
    testProducts<14, 30, FixedPointType<32, int64_t>>();
    testProducts<18, 30, FixedPointType<18>>();
    testProducts<16, 20, FixedPointType<32, int64_t>>();
    testProducts<30, 30, FixedPointType<30>>();
    testProducts<0, 0, FixedPointType<32, int64_t>>();
    testConversionDestinations<0, int32_t>();
    testConversionDestinations<4, int8_t>();
    testConversionDestinations<12, int32_t>();
    testConversionDestinations<18, int32_t>();
    testConversionDestinations<24, int32_t>();
    testConversionDestinations<30, int32_t>();
    testConversionDestinations<0, int64_t>();
    testConversionDestinations<32, int64_t>();
    testConversionDestinations<62, int64_t>();
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
    std::printf("Numeric/PI: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
