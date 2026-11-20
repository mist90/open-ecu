/**
 * @file main.cpp
 * @brief Float/fixed differential control traces and boundary validation
 */
#include "pi_float_interface.hpp"
#include "motor_config.hpp"
#include "motor1_t60_load.hpp"
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
    // MOTOR_1's actual speed-loop tuning must fit the more precise speed gain format.
    PiFloatInterface<FixedProfile::SpeedPiTypes, 100> scooter;
    check(scooter.setParameters(PiParameters{BLDC_SPEED_CURRENT_KP, BLDC_SPEED_CURRENT_KI,
                                           BLDC_MAX_CURRENT, BLDC_MIN_CURRENT, 12.0f, -4.0f, 2.0f}),
          "scooter speed tuning fits the fixed profile");
}

static void rollingLoadTrace() {
    using P = FixedProfile;
    PiFloatInterface<P::SpeedPiTypes, 100> pi;
    constexpr int max_iq = int(BLDC_MAX_CURRENT);
    check(pi.setParameters({6.f, 24.f, BLDC_MAX_CURRENT, BLDC_MIN_CURRENT, BLDC_MAX_CURRENT, -4.f, 2.f}),
          "loaded speed PI representable");
    auto step = [&](int eighths, int sat) {
        const auto v = number<P::Speed>(eighths, 8);
        return pi.updateNativeCapped(v + number_v<P::Speed, 1>, v, sat, motor1RollingIntegralCeiling<P, max_iq>(v));
    };
    for (int i = 0; i < 300; ++i) step(0, 0);
    check(pi.getIntegral() == 12.f, "a stationary speed error cannot earn rolling headroom");
    for (int i = 0; i < 200; ++i) step(2, 0);
    check(pi.getIntegral() == 14.f, "partial rolling authority");
    const auto at_37rpm = motor1RollingIntegralCeiling<P, max_iq>(number_v<P::Speed, 5, 8>);
    check(at_37rpm == number_v<P::Current, 20>, "20 A integral at 37.5 RPM in both profiles");
    for (int i = 0; i < 300; ++i) step(40, 0);
    check(pi.getIntegral() > max_iq - 6.f && pi.getOutput() == float(max_iq),
          "sustained rolling demand reaches the profile's actual positive command limit");
    step(-1, 1);
    check(pi.getIntegral() == 12.f && pi.getOutput() == 18.f, "loss of motion drops stored excess even under saturation");
    step(8, 1);
    check(pi.getIntegral() == 12.f, "no stale integral on recovery");
    pi.updateNativeCapped(number_v<P::Speed, 0>, number_v<P::Speed, 10>, 0,
                          motor1RollingIntegralCeiling<P, max_iq>(number_v<P::Speed, 10>));
    check(pi.getOutput() == -6.f, "negative current limit remains -6 A");
    for (int sign : {-1, 1}) {
        const auto q = number<P::Current>(sign * 16);
        for (int dir : {-1, 1}) {
            check(motor1T60Error<false>(q, number<P::PllSpeed>(dir*60)) == number<P::Current>(sign) &&
                  motor1T60Error<true>(q, number<P::PllSpeed>(dir*360)) == q &&
                  motor1T60Error<true>(q, number<P::PllSpeed>(dir*420)) == number<P::Current>(sign*17,2) &&
                  motor1T60Error<true>(q, number<P::PllSpeed>(dir*480)) == number<P::Current>(sign) &&
                  motor1T60Error<false>(q, number<P::PllSpeed>(dir*480)) == q,
                  "historical T60 current-error schedule on both axes and directions");
        }
    }
}

int main() {
    currentTrace();
    speedTrace(false);
    speedTrace(true);
    validation();
    rollingLoadTrace();
    std::printf("PI interface: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
