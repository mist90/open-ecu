/**
 * @file main.cpp
 * @brief Float/fixed differential control traces and boundary validation
 */
#include "pi_float_interface.hpp"
#include "control_calibration.hpp"
#include "motor_config.hpp"
#include <cmath>
#include <cstdio>
#include <limits>

using namespace libecu;
static int failures = 0;
static int checks = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

static void currentTrace() {
    PiFloatInterface<FloatProfile::CurrentPiTypes, 20000> floating;
    PiFloatInterface<FixedProfile::CurrentPiTypes, 20000> fixed;
    PiParameters params{0.15f, 200.0f, 0.95f, 0.0f, 0.95f, 0.0f, 5.0f};
    check(floating.setParameters(params) && fixed.setParameters(params), "current tuning is representable");
    float current = 0.0f;
    float worst = 0.0f;
    for (int tick = 0; tick < 120000; ++tick) {
        const float target = tick < 20000 ? 0.15f : tick < 40000 ? 0.4f :
                             tick < 60000 ? 0.0f : tick < 80000 ? 100.0f : 0.15f;
        // Shared ADC trace, including quantisation, ripple and voltage saturation.
        const float ripple = 0.006f * std::sin(float(tick) * 0.17f);
        const float measured = std::round((current + ripple) / 0.008394f) * 0.008394f;
        const float f = floating.update(target, measured);
        const float q = fixed.update(target, measured);
        worst = std::max(worst, std::abs(f - q));
        // Two RL phases at a small nonzero speed, matching the bench's order of magnitude.
        current += (20.8f * f - 3.0f * current - 0.3f) / 0.00225f / 20000.0f;
        if (current < 0.0f) current = 0.0f;
        check(std::isfinite(q) && q >= 0.0f && q <= 0.950001f, "fixed current output remains bounded");
    }
    std::printf("current: maximum float/fixed duty difference %.9f\n", worst);
    check(worst < 0.00024f, "current arithmetic differs by less than one PWM timer count");
}

static void speedTrace(bool voltage) {
    PiFloatInterface<FloatProfile::SpeedPiTypes, 100> floating;
    PiFloatInterface<FixedProfile::SpeedPiTypes, 100> fixed;
    PiParameters params{};
    params.kp = voltage ? 0.005f : 0.01f;
    params.ki = voltage ? 0.02f : 0.05f;
    params.kb = 2.0f;
    params.max_output = params.integral_max = voltage ? 0.95f : 6.0f;
    params.min_output = params.integral_min = voltage ? 0.0f : -4.0f;
    check(floating.setParameters(params) && fixed.setParameters(params), "speed tuning is representable");
    float worst = 0.0f;
    for (int tick = 0; tick < 10000; ++tick) {
        const float target = (tick / 500) % 2 ? 20.0f : 2.0f;
        const float measured = target + 0.3f * std::sin(float(tick) * 0.11f);
        const int saturation = tick % 1200 < 50 ? 1 : tick % 1200 > 1150 ? -1 : 0;
        worst = std::max(worst, std::abs(floating.update(target, measured, saturation) -
                                        fixed.update(target, measured, saturation)));
    }
    std::printf("speed (%s): maximum float/fixed output difference %.9f\n",
                voltage ? "duty" : "current", worst);
    check(worst < 0.0001f, "speed arithmetic stays below PWM/ADC resolution");
}

static void validation() {
    PiFloatInterface<FixedProfile::CurrentPiTypes, 20000> current;
    PiParameters params{0.15f, 200.0f, 0.95f, 0.0f, 0.95f, 0.0f, 5.0f};
    check(current.setParameters(params), "valid current parameters accepted");
    current.update(0.4f, 0.1f);
    const float previous = current.getIntegral();
    params.ki = 40000.0f;
    check(!current.setParameters(params), "continuous gain overflow rejected");
    check(current.getParameters().ki == 200.0f && current.getIntegral() == previous,
          "rejected tuning preserves parameters and state");
    params.ki = 200.0f;
    params.max_output = 2.0f;
    check(!current.setParameters(params), "fixed duty format overflow rejected");
    params.max_output = 0.95f;
    PiFloatInterface<FixedProfile::CurrentPiTypes, 1> slow;
    check(!slow.setParameters(params), "per-sample coefficient overflow rejected");
    params.kp = std::numeric_limits<float>::infinity();
    check(!current.setParameters(params), "non-finite tuning rejected");
    check(current.update(std::numeric_limits<float>::quiet_NaN(), 0.0f) == 0.0f &&
          current.getIntegral() == 0.0f, "invalid input clears the regulator output and state");
    PiFloatInterface<FixedProfile::SpeedPiTypes, 100> motor;
    check(motor.setParameters({BLDC_SPEED_CURRENT_KP, BLDC_SPEED_CURRENT_KI,
                               BLDC_MAX_CURRENT, BLDC_MIN_CURRENT, BLDC_MAX_CURRENT, -4.0f, 2.0f}),
          "selected motor's speed tuning fits the fixed profile");
}

template<class P>
static void rollingCurve() {
    // Signed mechanical speed in rev/s, independent of the electrical Hall-step schedule.
    for (float cap : {6.f, 12.f, 17.375f, 20.f, 45.f}) {
        for (float speed : {-4.f, 0.f, 0.0625f, 0.125f, 0.25f, 0.625f, 1.5f, 2.1875f, 8.f}) {
            const auto actual = rollingIntegralCeiling<P, motor_config::rolling_integral>(
                numberCast<typename P::Speed>(speed), numberCast<typename P::Current>(cap));
#ifdef MOTOR_1
            const float expected = std::min(cap, 12.f + 16.f * std::max(0.f, speed - 0.125f));
#else
            const float expected = cap;
#endif
            check(numberCast<float>(actual) == expected, "measured rolling authority respects the caller's rail");
        }
    }
}

#ifdef MOTOR_1
static void rollingLoadTrace(float max_iq) {
    using P = FixedProfile;
    PiFloatInterface<P::SpeedPiTypes, 100> pi;
    check(pi.setParameters({BLDC_SPEED_CURRENT_KP, BLDC_SPEED_CURRENT_KI, max_iq, BLDC_MIN_CURRENT,
                            max_iq, -4.f, 2.f}), "scooter speed PI representable");
    const auto ceiling = [max_iq](P::Speed speed) {
        return rollingIntegralCeiling<P, motor_config::rolling_integral>(speed, numberCast<P::Current>(max_iq));
    };
    // Leave room below even the 20 A output rail for 14 A of integral plus P.
    constexpr auto error = number_v<P::Speed, 1, 8>;
    constexpr float proportional = BLDC_SPEED_CURRENT_KP / 8.f;
    auto step = [&](int eighths, int sat) {
        const auto v = number<P::Speed>(eighths, 8);
        return pi.updateNativeCapped(v + error, v, sat, ceiling(v));
    };
    for (int i = 0; i < 1000; ++i) step(0, 0);
    check(pi.getIntegral() == 12.f, "a stationary speed error cannot earn rolling headroom");
    for (int i = 0; i < 200; ++i) step(2, 0);
    check(pi.getIntegral() == 14.f, "partial rolling authority");
    for (int i = 0; i < 3000; ++i) step(40, 0);
    check(pi.getIntegral() >= max_iq - proportional && pi.getOutput() == max_iq,
          "sustained rolling demand reaches the actual board/motor command limit");
    step(-1, 1);
    check(pi.getIntegral() == 12.f && pi.getOutput() == 12.f + proportional,
          "loss of motion drops stored excess even under saturation");
    step(8, 1);
    check(pi.getIntegral() == 12.f, "no stale integral on recovery");
    for (int i = 0; i < 100; ++i)
        pi.updateNativeCapped({}, number_v<P::Speed, 10>, 0, ceiling(number_v<P::Speed, 10>));
    check(pi.getOutput() == -6.f, "negative current limit remains -6 A");
}
#endif

// Oracle: a separately configured regulator receives the effective rails before its ordinary
// update. Retuning its rails preserves the trapezoid history. Compare every applied output,
// integral and saturation sign (no float tolerance when the two paths use the same profile).
template<class Types, unsigned Hz>
static void dynamicRailTrace() {
    using K = PiController<Types, Hz>;
    using G = typename K::Gain;
    using I = typename K::Input;
    using O = typename K::Output;
    typename K::Parameters base{number<G>(3, 4), number<G>(10), number<O>(7, 8), number<O>(-5, 8),
                                number<O>(5, 8), number<O>(-3, 8), number<G>(2)};
    K actual(base), reference(base);
    bool same = true, saw_high = false, saw_low = false, saw_free = false;
    for (int k = 0; k < 2000; ++k) {
        const int mode = (k / 7) % 3; // capped -> ordinary and immediate cap removal
        const auto icap = number<O>((k / 11) % 19 - 8, 8); // spans above AND below the configured rails
        const auto low = number<O>(-((k / 19) % 9), 8);
        const auto high = number<O>((k / 13) % 9, 8);
        const auto target = number<I>((k / 17) % 17 - 8, 4);
        const auto fb = number<I>((k % 19) - 9, 16);
        const int ext = (k / 9) % 3 - 1;
        auto p = base;
        if (mode) p.integral_max = std::clamp(icap, p.integral_min, p.integral_max);
        if (mode == 2) {
            p.min_output = std::max(low, p.min_output);
            p.max_output = std::min(high, p.max_output);
        }
        reference.setParameters(p);
        const auto expected = reference.update(target, fb, ext);
        const auto got = !mode ? actual.update(target, fb, ext) :
                         mode == 1 ? actual.updateCapped(target, fb, ext, icap) :
                                      actual.updateBounded(target - fb, ext, icap, low, high);
        const auto config = actual.getParameters();
        same &= expected == got && reference.getIntegral() == actual.getIntegral() &&
                reference.saturationSign() == actual.saturationSign() && config.max_output == base.max_output &&
                config.min_output == base.min_output && config.integral_max == base.integral_max &&
                config.integral_min == base.integral_min;
        saw_high |= actual.saturationSign() > 0;
        saw_low |= actual.saturationSign() < 0;
        saw_free |= actual.saturationSign() == 0;
    }
    check(same && saw_high && saw_low && saw_free,
          "changing integral and both output rails matches configured-limit oracle on every tick");
}

// Without back-calculation, a tightened drive or braking rail must hold the integral while
// the error pushes farther into saturation, even as protection begins to release that rail.
template<class P>
static void protectionWithoutBackCalculation() {
    using Pi = PiFloatInterface<typename P::SpeedPiTypes, 100>;
    using I = typename Pi::Input;
    using O = typename Pi::Output;
    Pi pi;
    check(pi.setParameters({6, 24, 45, -6, 12, -4, 0}), "configure speed PI without back-calculation");
    for (int sign : {-1, 1}) {
        pi.reset();
        const auto error = number<I>(2 * sign);
        const auto step = [&](O rail) {
            return pi.updateNativeBounded(error, 0, number<O>(12), -rail, rail);
        };
        check(step(number<O>(1)) == number<O>(sign) && pi.saturationSign() == sign,
              "first sample reaches the selected protection rail");
        const float held = pi.getIntegral();
        for (int tick = 0; tick < 100; ++tick) step(number<O>(1));
        check(std::abs(held) < 0.25f && pi.getIntegral() == held, "sustained current limiting holds the integral");
        for (int tick = 1; tick <= 50; ++tick) step(number<O>(25 + tick, 25));
        check(pi.getIntegral() == held && pi.getOutput() == 3.f * sign,
              "the applied command follows the recovering rail without winding up");
    }
    const auto brake = pi.updateNativeBounded(number<I>(-2), 0, number<O>(12), number<O>(-6), O{});
    check(brake == number<O>(-6) && pi.saturationSign() == -1, "a zero positive rail still permits braking");
}

template<class P>
static void oneSidedCapCompatibility() {
    using Pi = PiFloatInterface<typename P::SpeedPiTypes, 100>;
    using I = typename Pi::Input;
    using O = typename Pi::Output;
    const auto floor = number<O>(-6), ceiling = number<O>(45), integral = number<O>(12);
    for (float kb : {0.f, 2.f}) {
        Pi one_sided, bounded;
        const PiParameters tuning{6, 24, 45, -6, 12, -4, kb};
        check(one_sided.setParameters(tuning) && bounded.setParameters(tuning), "configure equivalent PI paths");
        for (int cap : {1, 3, -8, 0, 50}) {
            const auto limit = number<O>(cap);
            const auto effective = std::clamp(limit, floor, ceiling);
            for (int tick = 0; tick < 20; ++tick) {
                const auto target = number<I>(tick < 10 ? 5 : 1), measured = number<I>(3);
                const auto actual = one_sided.updateNativeCapped(target, measured, 0, integral, limit);
                const auto expected = bounded.updateNativeBounded(target - measured, 0, integral, floor, effective);
                check(actual == expected && one_sided.getIntegral() == bounded.getIntegral() &&
                      one_sided.saturationSign() == bounded.saturationSign(),
                      "a per-sample upper rail preserves the configured floor and both anti-windup modes");
            }
        }
    }
}

template<class Types>
static void exactRailReadback() {
    using K = PiController<Types, 500>;
    using O = typename K::Output;
    typename K::Parameters p{};
    p.max_output = O::fromRaw(INT32_MAX - 1);
    p.min_output = O::fromRaw(INT32_MIN + 1);
    p.integral_max = O::fromRaw(1);
    p.integral_min = O::fromRaw(-1);
    K k(p);
    k.updateBounded({}, 0, O::fromRaw(INT32_MAX), O::fromRaw(-1), O::fromRaw(1));
    const auto saved = k.getParameters();
    check(saved.min_output == p.min_output && saved.max_output == p.max_output &&
          saved.integral_min == p.integral_min && saved.integral_max == p.integral_max,
          "negative and positive single-LSB rails round-trip without touching float");

    // Reinstall a native readback, then test round-to-nearest/ties-away around both signs,
    // including just inside/outside the configured raw output rails and wide-word boundaries.
    p.integral_min = O::fromRaw(INT32_MIN);
    p.integral_max = O::fromRaw(INT32_MAX);
    k.setParameters(p);
    k.setParameters(k.getParameters());
    using A = typename K::Accumulator;
    constexpr int64_t scale = int64_t(A::SCALE / O::SCALE);
    bool rounded = true;
    for (int32_t base : {INT32_MIN, INT32_MIN + 1, -105, -1, 0, 1, 105, INT32_MAX - 1}) {
        for (int delta : {-1, 0, 1}) {
            const A integral = A::fromRaw(int64_t(base) * scale + scale / 2 + delta);
            k.applyGains({}, &integral);
            const auto expected = std::clamp(integral, A(p.min_output), A(p.max_output));
            rounded &= k.update({}, {}) == O(expected);
        }
    }
    check(rounded, "bounded output conversion preserves half ties, their neighbours and both near-extreme rails");
}

int main() {
    currentTrace();
    speedTrace(false);
    speedTrace(true);
    validation();
    rollingCurve<FloatProfile>();
    rollingCurve<FixedProfile>();
#ifdef MOTOR_1
    rollingLoadTrace(20.f);
    rollingLoadTrace(45.f);
#endif
    dynamicRailTrace<FloatProfile::PiTypes, 500>();
    dynamicRailTrace<FixedProfile::SpeedPiTypes, 100>();
    dynamicRailTrace<FixedProfile::SpeedPiTypes, 500>();
    dynamicRailTrace<FixedProfile::CurrentPiTypes, 16000>();
    protectionWithoutBackCalculation<FloatProfile>();
    protectionWithoutBackCalculation<FixedProfile>();
    oneSidedCapCompatibility<FloatProfile>();
    oneSidedCapCompatibility<FixedProfile>();
    exactRailReadback<FixedProfile::SpeedPiTypes>();
    exactRailReadback<FixedProfile::CurrentPiTypes>();
    std::printf("PI interface: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
