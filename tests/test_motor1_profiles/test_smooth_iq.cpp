/** @file test_smooth_iq.cpp @brief Check the fixed-point 16-kHz bench PI, including its handoff. */
#include "foc_control_core.hpp"
#include <cassert>
#include <cmath>
#include <limits>

using namespace libecu;
using Core = FocControlCore<FixedProfile, 16000>;

int main() {
    Core foc;
    Core::Params params{};
    foc.setParams(params);
    foc.setCurrentPiGains(number<Core::Gain>(1), number<Core::Gain>(16));
    Core::Input in{};
    in.drive_mode = DriveMode::FORWARD;
    in.electric_mode = ElectricMode::CURRENT_MODE;
    in.bus_voltage = number<Core::Voltage>(40);
    // Zero crossing of the independent 315-degree sixth-order feedforward.
    in.angle_steps = number<FixedProfile::Angle>(1, 8);
    auto publish = [&]() { foc.applyBenchPoint(Motor1BenchPoint::fromSpeed(in.speed_steps_s)); };
    struct Knot { int speed; float d, q; };
    constexpr float low = 1.f/16;
    constexpr Knot knots[]{
        {0, 1, 1}, {16, 1, 1}, {20, .75f, .75f}, {30, .125f, .125f}, {31, low, low},
        {60, low, low}, {64, low, low}, {71, .5f, .5f}, {79, 1, 1}, {87, 1, 1.25f},
        {93, 1, 1.4375f}, {99, 1, 1.25f}, {107, 1, 1}, {232, 1, 1}, {248, 1, .5f},
        {261, 1, 3.f/32}, {262, 1, low}, {900, 1, low}
    };
    // No motor motion model here: isolate the q regulator from d and back-EMF.
    for (int direction : {-1, 1}) {
        for (const auto& k : knots) {
            foc.reset();
            in.target_current = number<Core::Current>(1);
            in.speed_steps_s = number<FixedProfile::PllSpeed>(direction * k.speed);
            publish();
            const float mid = k.speed <= 68 || k.speed >= 100 ? 1.f :
                              .5f + std::abs(k.speed - 84) / 32.f;
            const float q_gain = k.q * mid;
            const auto first = foc.update(in);
            assert(std::abs(first.v_q.toFloat() - q_gain) < .0003f);
            Core::Output last{};
            for (int i = 0; i < 1000; ++i) last = foc.update(in);
            assert(std::abs(last.v_q.toFloat() - 2 * q_gain) < .0004f);
            // With zero error the stored voltage must survive crossing the
            // breakpoints. Scaling/resetting the integral would cause a step.
            in.target_current = {};
            in.speed_steps_s = number<FixedProfile::PllSpeed>(k.speed <= 232 ? 600 : 0);
            publish();
            assert(std::abs(foc.update(in).v_q.toFloat() - q_gain) < .002f);
        }
    }
    // A one-LSB speed crossing must not step the voltage request (apart from
    // one Q12 voltage-request LSB), even where the above-unity branch begins.
    in.target_current = number<Core::Current>(1);
    for (int boundary : {16, 31, 64, 68, 79, 84, 93, 100, 107, 232, 262}) {
        const auto raw = number<FixedProfile::PllSpeed>(boundary).raw();
        foc.reset();
        in.speed_steps_s = FixedProfile::PllSpeed::fromRaw(raw - 1);
        publish();
        const float left = foc.update(in).v_q.toFloat();
        foc.reset();
        in.speed_steps_s = FixedProfile::PllSpeed::fromRaw(raw + 1);
        publish();
        assert(std::abs(foc.update(in).v_q.toFloat() - left) <= 1.f / 4096);
    }
    // The local q boost uses saturating addition; it cannot wrap a positive
    // or negative full-scale error into an opposite-sign voltage command.
    in.speed_steps_s = number<FixedProfile::PllSpeed>(93);
    publish();
    const auto boosted = Motor1BenchPoint::fromSpeed(in.speed_steps_s).q;
    assert((boosted > number_v<FixedProfile::Fraction, 1>));
    for (const auto raw : {std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()}) {
        assert(Motor1BenchPoint::weight(Core::Current::fromRaw(raw), boosted).raw() == raw);
        const auto far = Motor1BenchPoint::fromSpeed(FixedProfile::PllSpeed::fromRaw(raw));
        assert((far.d == number_v<FixedProfile::Fraction, 1>));
        assert((far.q == number_v<FixedProfile::Fraction, 1, 16>));
        assert(far.ripple_gain == 0 && far.angle_gain == 0);
    }
    for (int direction : {-1, 1}) {
        foc.reset();
        in.target_current = number<Core::Current>(direction * 1800);
        assert(direction * foc.update(in).v_q.toFloat() > 1);
    }
    // The high-speed taper is q-only. A d-axis command must retain its
    // full response there; at crawl the same bounded taper applies to both.
    in.target_current = {};
    params.id_target = number<Core::Current>(1);
    foc.setParams(params);
    for (const auto& k : knots) {
        foc.reset();
        in.speed_steps_s = number<FixedProfile::PllSpeed>(k.speed);
        publish();
        assert(std::abs(foc.update(in).v_d.toFloat() - k.d) < .0003f);
    }

    // One zero-mean 6fe cycle per Hall step, not per whole electrical turn.
    const auto one_amp = number<Core::Current>(1);
    for (int rpm : {0, 16, 30, 34, 38, 45, 48, 50, 54, 58, 300}) {
        in.speed_steps_s = number<FixedProfile::PllSpeed>(2 * rpm);
        const auto fade = Motor1BenchPoint::fromSpeed(in.speed_steps_s).ripple_gain;
        int32_t previous[64]{};
        int64_t sum = 0;
        int32_t peak = 0;
        for (int i = 0; i < 6 * 64; ++i) {
            in.angle_steps = number<Core::Angle>(i, 64);
            const auto value = motor1BenchRipple(in, one_amp, fade).raw();
            if (i >= 64) assert(value == previous[i % 64]);
            previous[i % 64] = value;
            sum += value;
            peak = std::max(peak, value);
        }
        const int32_t full = number<Core::Current>(1, 16).raw();
        assert(sum == 0 && peak <= full);
        if (rpm <= 30 || rpm >= 58) assert(peak == 0);
        if (rpm == 34 || rpm == 54) assert(peak == full / 2);
        if (rpm >= 38 && rpm <= 50) {
            assert(peak == full);
            // Trim at 315 degrees is negative with Hall phase zero.
            assert(previous[0] < 0 && previous[8] == 0 && previous[24] == full);
        }
    }
    in.speed_steps_s = number<FixedProfile::PllSpeed>(96);
    in.angle_steps = number<Core::Angle>(3, 8); // positive crest
    for (int amps : {-2, 0, 3, 20}) assert(motor1BenchRipple(in, number<Core::Current>(amps), 256).raw() == 0);
    assert(motor1BenchRipple(in, number<Core::Current>(1, 16), 256) == number<Core::Current>(1, 32));
    assert(motor1BenchRipple(in, number<Core::Current>(11, 4), 256) == number<Core::Current>(1, 32));
    in.drive_mode = DriveMode::REVERSE;
    assert(motor1BenchRipple(in, one_amp, 256).raw() == 0);

    for (int rpm : {0, 8, 16, 30, 36, 40, 300}) {
        int64_t sum = 0;
        for (int i = 0; i < 256; ++i) {
            const auto a = number<Core::Angle>(i, 256);
            const auto v = number<FixedProfile::PllSpeed>(2 * rpm);
            const auto forward = motor1BenchParkAngle(a, Motor1BenchPoint::fromSpeed(v).angle_gain) - a;
            const auto reverse = motor1BenchParkAngle(a, Motor1BenchPoint::fromSpeed(-v).angle_gain) - a;
            assert(forward.raw() == -reverse.raw());
            assert(std::abs(forward.toFloat()) <= 1.f/16);
            if (rpm <= 8 || rpm >= 40) assert(forward.raw() == 0);
            sum += forward.raw();
        }
        assert(std::abs(float(sum) / (256 * float(Core::Angle::SCALE))) < .00002f);
    }
}
