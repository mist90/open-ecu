/** @file main.cpp
 * @brief Paired Hall traces across numerical profiles and compile-time sample rates
 */
#include "motor_pll_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace libecu;
static int failures = 0;
static void check(bool ok, const char* message) {
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

template<uint32_t Frequency>
void trace(float target) {
    MotorPLLCore<FloatProfile, Frequency> floating(6000.0f, true);
    MotorPLLCore<FixedProfile, Frequency> fixed(number<FixedProfile::PllSpeed>(6000), true);
    floating.setUsePLL(true);
    fixed.setUsePLL(true);
    float phase = 0.0f;
    float worst_angle = 0.0f, worst_speed = 0.0f;
    float mean_speed = 0.0f;
    uint32_t count = 0;
    // Stay within the nominal tracking envelope (alpha / Ki < the 0.5-step
    // error clamp). Overslew is tested separately because reacquisition is
    // discontinuous and need not happen on the same tick after quantisation.
    const float ramp_time = std::max(0.1f, std::abs(target) / 1000.0f);
    const uint32_t ticks = static_cast<uint32_t>((ramp_time + 3.0f) * Frequency);
    for (uint32_t tick = 0; tick < ticks; ++tick) {
        const float elapsed = float(tick) / Frequency;
        const float speed = target * std::min(elapsed / ramp_time, 1.0f);
        phase = std::fmod(phase + speed / Frequency, 6.0f);
        if (phase < 0.0f) phase += 6.0f;
        const auto hall = static_cast<uint8_t>(phase);
        floating.updateHall(hall);
        fixed.updateHall(hall);
        floating.updateTick();
        fixed.updateTick();
        const float delta = std::remainder(floating.getAngle() - fixed.getAngle().toFloat(), 6.0f);
        worst_angle = std::max(worst_angle, std::abs(delta));
        worst_speed = std::max(worst_speed, std::abs(floating.getSpeedStepsSec() - fixed.getSpeedStepsSec().toFloat()));
        check(fixed.getAngle() >= number<FixedProfile::Angle>(0) &&
              fixed.getAngle() < number<FixedProfile::Angle>(6), "fixed phase stays in one revolution");
        if (elapsed >= ramp_time + 1.0f) { mean_speed += fixed.getSpeedStepsSec().toFloat(); ++count; }
    }
    mean_speed /= count;
    std::printf("%u Hz, %+7.1f steps/s: max phase delta %.7f, speed delta %.5f, mean %.3f\n",
                Frequency, target, worst_angle, worst_speed, mean_speed);
    check(worst_angle < 0.01f, "fixed/float angle difference stays below 0.6 electrical degrees");
    check(worst_speed < 1.0f, "fixed/float speed estimates agree within one electrical step/s");
    check(std::abs(mean_speed - target) < std::max(0.2f, std::abs(target) * 0.02f),
          "mean speed follows the independent Hall trace");
}

void overslew() {
    MotorPLLCore<FixedProfile, 20000> fixed(number<FixedProfile::PllSpeed>(6000));
    fixed.setUsePLL(true);
    float phase = 0.0f, sum = 0.0f;
    unsigned samples = 0;
    for (unsigned tick = 0; tick < 80000; ++tick) {
        const float elapsed = float(tick) / 20000;
        const float speed = 4800.0f * std::min(elapsed, 1.0f);
        phase = std::fmod(phase + speed / 20000, 6.0f);
        fixed.updateHall(static_cast<uint8_t>(phase));
        fixed.updateTick();
        check(fixed.getAngle() >= number<FixedProfile::Angle>(0) &&
              fixed.getAngle() < number<FixedProfile::Angle>(6), "overslew keeps the phase wrapped");
        check(numericAbs(fixed.getSpeedStepsSec()) <= number<FixedProfile::PllSpeed>(6050),
              "overslew keeps the speed state bounded");
        if (elapsed > 2.0f) { sum += fixed.getSpeedStepsSec().toFloat(); ++samples; }
    }
    check(std::abs(sum / samples - 4800.0f) < 5.0f, "PLL reacquires after an excessive acceleration");
}

int main() {
    static_assert(integerSqrt(0) == 0 && integerSqrt(1) == 1 && integerSqrt(15) == 3, "integer root");
    static_assert(integerSqrt(UINT64_MAX) == UINT32_MAX, "full-width integer root");
    static_assert(constant<FixedPointType<24>>(1.5f).raw() == 25165824, "compile-time configuration literal");
    static_assert(divideTo<FixedPointType<30>>(number<FixedPointType<18>>(20),
                                              number<FixedPointType<18>>(40)).raw() == 536870912,
                  "mixed-format division");
    for (float speed : {-4800.0f, -480.0f, -48.0f, 0.0f, 48.0f, 480.0f, 4800.0f}) trace<20000>(speed);
    trace<10000>(480.0f);
    trace<40000>(-480.0f);
    trace<8000>(480.0f);
    trace<16000>(-480.0f);
    overslew();
    std::printf("Numerical PLL: %d failures\n", failures);
    return failures != 0;
}
