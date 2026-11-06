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

// Independent rational-integer model: no FixedPointType arithmetic, profile
// constants, shifts for rounding, or implementation-private state is reused.
static int64_t roundRatio(__int128 numerator, int64_t denominator) {
    return numerator < 0 ? -int64_t((-numerator + denominator / 2) / denominator)
                         : int64_t((numerator + denominator / 2) / denominator);
}
static int32_t narrow(int64_t value) {
    return int32_t(std::clamp<int64_t>(value, INT32_MIN, INT32_MAX));
}

template<uint32_t Frequency>
static void exactTrace() {
    constexpr int64_t angle_scale = 16777216, speed_scale = 16384;
    constexpr int64_t unit_scale = 1073741824, phase_scale = 4294967296LL;
    constexpr int64_t period = 6 * phase_scale;
    const int64_t alpha_gain = roundRatio(unit_scale, 6 * Frequency);
    const int64_t phase_gain = roundRatio(unit_scale, Frequency);
    uint64_t random = 0x584db5ca918ef091ULL;
    bool exact = true;
    for (int32_t max_speed : {1, int32_t(6000 * speed_scale), INT32_MAX}) {
        MotorPLLCore<FixedProfile, Frequency> actual(FixedProfile::PllSpeed::fromRaw(max_speed));
        int64_t phase = 0, integral = 0, lock = 0;
        int32_t speed = 0, kp = 100 * speed_scale, ki = 5000 * speed_scale;
        int32_t ki_step = narrow(roundRatio(__int128(ki) * 1024, Frequency));
        uint8_t hall = 0;
        bool synchronized = false;
        for (unsigned tick = 0; tick < 30000; ++tick) {
            random ^= random << 13; random ^= random >> 7; random ^= random << 17;
            const uint8_t incoming = tick % 19 ? uint8_t((tick / 11) % 6) : uint8_t(random);
            actual.updateHall(incoming);
            if (incoming <= 5) hall = incoming;
            if (tick % 521 == 0) {
                const int32_t gains[] = {0, 1, -1, 100 * speed_scale, 5000 * speed_scale, INT32_MIN, INT32_MAX};
                kp = gains[(tick / 521) % 7];
                ki = gains[(tick / 521 + 3) % 7];
                ki_step = narrow(roundRatio(__int128(ki) * 1024, Frequency));
                actual.setGains(FixedProfile::PllGain::fromRaw(kp), FixedProfile::PllGain::fromRaw(ki));
            }
            if (tick % 997 == 0) {
                actual.reset();
                phase = hall * phase_scale; integral = lock = 0; speed = 0; synchronized = false;
            }
            const auto angle = [&] { return roundRatio(phase, 256) % (6 * angle_scale); };
            int64_t error = hall * angle_scale - angle();
            if (error > 3 * angle_scale) error -= 6 * angle_scale;
            else if (error < -3 * angle_scale) error += 6 * angle_scale;
            error = std::clamp(error, -angle_scale / 2, angle_scale / 2);
            const int32_t absolute_speed = narrow(std::abs(int64_t(speed)));
            const int64_t alpha = std::clamp<int64_t>(narrow(roundRatio(__int128(absolute_speed) * alpha_gain,
                speed_scale)), roundRatio(unit_scale, 10000), roundRatio(unit_scale, 20));
            lock += roundRatio(__int128(alpha) * (std::abs(error) - lock), unit_scale);
            bool reset = false;
            if (synchronized) {
                if (lock >= roundRatio(2 * angle_scale, 5)) { synchronized = false; reset = true; }
            } else if (std::abs(error) == angle_scale / 2) reset = true;
            else if (absolute_speed > 5 * speed_scale) synchronized = true;
            integral = std::clamp(integral + roundRatio(__int128(error) * ki_step, angle_scale),
                                  -int64_t(max_speed) * 1024, int64_t(max_speed) * 1024);
            speed = narrow(int64_t(narrow(roundRatio(__int128(error) * kp, angle_scale))) +
                           narrow(roundRatio(integral, 1024)));
            if (reset) { integral = int64_t(speed) * 1024; phase = hall * phase_scale; }
            else {
                phase = (phase + roundRatio(__int128(speed) * phase_gain, 4096)) % period;
                if (phase < 0) phase += period;
            }
            actual.updateTick();
            const auto info = actual.getInfo();
            exact &= info.angle.raw() == angle() && info.angle_per_second.raw() == speed &&
                     info.pll_integral.raw() == narrow(roundRatio(integral, 1024)) &&
                     info.is_sync == synchronized && info.hall_state_raw == hall;
        }
    }
    check(exact, "PLL exactly matches the independent rational model at full-range gains and state boundaries");
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
    exactTrace<1>(); // Forces multi-turn wrapping as well as single-turn updates.
    exactTrace<8000>();
    exactTrace<16000>();
    exactTrace<40000>();
    std::printf("Numerical PLL: %d failures\n", failures);
    return failures != 0;
}
