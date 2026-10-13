/**
 * @file main.cpp
 * @brief Float/fixed differential control traces and boundary validation
 */
#include "pi_float_interface.hpp"
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
    check(scooter.setParameters(PiParameters{2.0f, 2.0f, 18.0f, -6.0f, 12.0f, -4.0f, 2.0f}),
          "scooter speed tuning fits the fixed profile");
}

int main() {
    currentTrace();
    speedTrace(false);
    speedTrace(true);
    validation();
    std::printf("PI interface: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
