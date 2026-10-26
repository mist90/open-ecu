/** @file main.cpp
 * @brief Fixed trig, transform accuracy and conservative voltage limiting
 */
#include "profile_math.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace libecu;
static int failures = 0;
static void check(bool ok, const char* message) {
    if (!ok) { if (failures < 15) std::printf("FAIL: %s\n", message); ++failures; }
}

template<class P>
static void testLowSideClamped() {
    using T = FocTransforms<P>;
    using V = typename P::Voltage;
    for (float bus : {10.0f, 31.3f, 42.0f}) {
        for (float modulation : {0.0f, 0.3f, 0.6f, 0.8f, 0.95f}) {
            for (int degrees = 0; degrees < 360; ++degrees) {
                const float theta = degrees * FOC_PI / 180.0f;
                const float alpha = bus * FOC_INV_SQRT3 * modulation * std::cos(theta);
                const float beta = bus * FOC_INV_SQRT3 * modulation * std::sin(theta);
                const auto duty = T::svpwm(numberCast<V>(alpha), numberCast<V>(beta), numberCast<V>(bus), true);
                const float u = numberCast<float>(duty.u);
                const float v = numberCast<float>(duty.v);
                const float w = numberCast<float>(duty.w);
                check(std::min({u, v, w}) == 0.0f, "DPWMMIN always holds one low side on");
                check(std::max({u, v, w}) <= modulation + 0.000002f,
                      "DPWMMIN duty never exceeds the modulation limit");
                // Reconstruct actual motor voltages, independently of the modulator.
                check(std::abs(bus * (2.0f * u - v - w) / 3.0f - alpha) < 0.0001f,
                      "clamped duties reproduce requested alpha voltage through the entire linear range");
                check(std::abs(bus * (v - w) / FOC_SQRT3 - beta) < 0.0001f,
                      "clamped duties reproduce requested beta voltage through the entire linear range");
            }
        }
    }
    const auto dead = T::svpwm(number<V>(10), number<V>(10), V{}, true);
    check(dead.u == typename P::Duty{} && dead.v == dead.u && dead.w == dead.u,
          "DPWMMIN dead bus produces a non-switching zero vector");
}

int main() {
    testLowSideClamped<FloatProfile>();
    testLowSideClamped<FixedProfile>();
    using P = FixedProfile;
    using M = P::Math;
    using T = FocTransforms<P>;
    float worst_sine = 0.0f, worst_transform = 0.0f, worst_duty = 0.0f;
    for (int index = -120000; index <= 120000; ++index) {
        const auto angle = P::Angle::fromRatio(index, 1000);
        P::Fraction sine{}, cosine{};
        M::sinCos(angle, sine, cosine);
        const float actual = angle.toFloat();
        worst_sine = std::max(worst_sine, std::abs(sine.toFloat() - std::sin(actual)));
        worst_sine = std::max(worst_sine, std::abs(cosine.toFloat() - std::cos(actual)));
    }
    check(worst_sine < 0.000081f, "Q30 LUT interpolation matches sine/cosine within the float LUT error budget");

    for (int index = 0; index < 3600; ++index) {
        const float theta = float(index) * FOC_TWO_PI / 3600.0f;
        P::Fraction sine{}, cosine{};
        M::sinCos(P::Angle::fromFloat(theta), sine, cosine);
        const auto ab = T::clarke(P::Current::fromFloat(18.0f * std::cos(theta)),
                                  P::Current::fromFloat(18.0f * std::cos(theta - FOC_TWO_PI / 3.0f)),
                                  P::Current::fromFloat(18.0f * std::cos(theta + FOC_TWO_PI / 3.0f)));
        const auto dq = T::park(ab, sine, cosine);
        const auto back = T::invPark(dq, sine, cosine);
        worst_transform = std::max(worst_transform, std::abs(back.alpha.toFloat() - ab.alpha.toFloat()));
        worst_transform = std::max(worst_transform, std::abs(back.beta.toFloat() - ab.beta.toFloat()));
        check(std::abs(dq.q.toFloat()) < 0.002f, "Park aligns an 18 A rotating vector");
        for (float bus : {12.0f, 20.8f, 31.3f, 48.0f}) {
            const float amplitude = bus * FOC_INV_SQRT3 * 0.95f;
            const float alpha = amplitude * std::cos(theta), beta = amplitude * std::sin(theta);
            const auto fixed = T::svpwm(P::Voltage::fromFloat(alpha), P::Voltage::fromFloat(beta),
                                        P::Voltage::fromFloat(bus));
            const auto floating = svpwm(alpha, beta, bus);
            worst_duty = std::max(worst_duty, std::abs(fixed.u.toFloat() - floating.u));
            worst_duty = std::max(worst_duty, std::abs(fixed.v.toFloat() - floating.v));
            worst_duty = std::max(worst_duty, std::abs(fixed.w.toFloat() - floating.w));
        }
    }
    check(worst_transform < 0.003f, "transform round-trip stays within LUT interpolation error");
    check(worst_duty < 0.000002f, "fixed SVPWM differs by much less than a PWM timer count");

    for (float limit : {0.1f, 1.0f, 12.0f, 20.0f, 1000.0f}) {
        const auto bound = P::Voltage::fromFloat(limit);
        for (int index = -2000; index <= 2000; ++index) {
            const auto d = multiplyTo<P::Voltage>(bound, P::Fraction::fromRatio(index, 2000));
            const auto q = M::qVoltageLimit(d, bound);
            const uint64_t used = uint64_t(int64_t(d.raw()) * d.raw()) + uint64_t(int64_t(q.raw()) * q.raw());
            const uint64_t available = uint64_t(int64_t(bound.raw()) * bound.raw());
            check(used <= available, "integer voltage limit always stays inside the circle");
            const uint64_t next = uint64_t(int64_t(d.raw()) * d.raw()) + uint64_t(int64_t(q.raw() + 1) * (q.raw() + 1));
            check(next > available, "integer root leaves less than one voltage LSB of unused q headroom");
        }
    }
    check(M::voltageLimitExceeded(P::VoltageRequest::fromRaw(INT32_MIN),
                                  P::VoltageRequest::fromRaw(INT32_MIN), number<P::Voltage>(20)),
          "wide squared request detects saturation without signed overflow");
    const auto dead_bus = T::svpwm(number<P::Voltage>(10), number<P::Voltage>(10), P::Voltage{});
    check(dead_bus.u == number<P::Duty>(1, 2) && dead_bus.v == dead_bus.u && dead_bus.w == dead_bus.u,
          "dead bus produces equal duties");
    std::printf("Profile math: sin/cos %.8f, transform %.8f A, duty %.9f; %d failures\n",
                worst_sine, worst_transform, worst_duty, failures);
    return failures != 0;
}
