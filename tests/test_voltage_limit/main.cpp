/** @file main.cpp @brief Independent geometry, wide-request and PWM/AW oracles. */
#include "foc_control_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

using namespace libecu;
static int failures = 0;
static unsigned checks = 0;
static constexpr double PI = 3.14159265358979323846;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok && failures++ < 20) std::printf("FAIL: %s\n", message);
}

// Evaluate ALL tangent half-planes independently, without selecting a segment.
static double idealRoom(double d, double limit) {
    d = std::min(std::abs(d), limit);
    double q = limit;
    for (unsigned k = 1; k < 8; ++k) {
        const double theta = PI * k / 16;
        q = std::min(q, (limit - std::sin(theta) * d) / std::cos(theta));
    }
    return q;
}

static int32_t quantizedRoom(int64_t d, int32_t limit) {
    d = std::min(std::abs(d), int64_t(limit));
    int64_t q = limit;
    for (unsigned k = 1; k < 8; ++k) {
        const double theta = PI * k / 16;
        const int64_t a = std::floor((int64_t(1) << 28) / std::cos(theta));
        const int64_t b = std::ceil((int64_t(1) << 28) * std::tan(theta));
        q = std::min(q, (int64_t(limit) * a - d * b) / (int64_t(1) << 28));
    }
    return int32_t(q);
}

static void testFixed() {
    using P = FixedProfile;
    using V = P::Voltage;
    using R = P::VoltageRequest;
    PolygonVoltageLimit<P> limiter;
    check(limiter.apply(number<R>(1), number<R>(-1)).limited, "unprepared limiter has zero headroom");
    const auto exercise = [&](int32_t l, int32_t d) {
        limiter.prepare(V::fromRaw(l));
        const int32_t q = limiter.qLimit(V::fromRaw(d)).raw();
        check(q == quantizedRoom(d, l), "fixed selected face equals full rational envelope");
        const double x = std::min(std::abs(double(d)), double(l));
        const double ideal = idealRoom(x, l);
        check(q <= ideal + 0.00001 && ideal - q < 1.00001 + 2.0 * l / (1u << 28),
              "fixed directed-rounding error stays within analytic bound");
        check(x * x + double(q) * q <= double(l) * l / std::pow(std::cos(PI / 32), 2) + 1,
              "fixed output stays inside ideal circumradius");
        check(limiter.qLimit(V::fromRaw(d > 0 ? -d : d)).raw() == q, "q headroom is even in d");
    };
    for (int32_t l = 0; l <= 128; ++l) {
        for (int32_t d = -l - 1; d <= l + 1; ++d) exercise(l, d);
    }
    std::mt19937 rng(0x51f0c);
    for (unsigned i = 0; i < 60000; ++i) exercise(rng() & INT32_MAX, int32_t(rng()));
    for (int32_t l : {129, 1024, 262144, 5242880, INT32_MAX / 2, INT32_MAX}) {
        // Both sides of every ideal vertex; worst scalar headroom and endpoints.
        for (unsigned k = 0; k < 7; ++k) {
            const double crossing = l * std::sin((k + 0.5) * PI / 16) / std::cos(PI / 32);
            for (int offset = -32; offset <= 32; ++offset) exercise(l, int32_t(crossing) + offset);
        }
        for (int32_t d : {INT32_MIN, -l, -1, 0, 1, l, INT32_MAX}) exercise(l, d);
    }
    // Full-width PI requests cannot silently saturate during Voltage conversion.
    for (unsigned i = 0; i < 60000; ++i) {
        const int32_t l = rng() & INT32_MAX;
        const int32_t rd = i % 2 ? int32_t(rng()) : int32_t(rng() % 67108864) - 33554432;
        const int32_t rq = int32_t(rng());
        limiter.prepare(V::fromRaw(l));
        const int64_t d = int64_t(rd) * 64, q = int64_t(rq) * 64;
        const int32_t expected_d = std::clamp(d, -int64_t(l), int64_t(l));
        const int32_t room = quantizedRoom(expected_d, l);
        const int32_t expected_q = std::clamp(q, -int64_t(room), int64_t(room));
        const auto actual = limiter.apply(R::fromRaw(rd), R::fromRaw(rq));
        check(actual.d.raw() == expected_d && actual.q.raw() == expected_q,
              "wide requests preserve d priority and both output signs");
        check(actual.limited == (d != expected_d || q != expected_q),
              "saturation flag means an actual output clamp, including sub-request-LSB clamps");
    }
    limiter.prepare(number<V>(20));
    auto out = limiter.apply(number<R>(1), number<R>(20));
    check(!out.limited && out.d == number<V>(1) && out.q == number<V>(20),
          "outside circle but inside polygon is not saturated");
    out = limiter.apply(number<R>(20), number<R>(20));
    check(out.limited && out.d == number<V>(20) && out.q > number<V>(196, 100),
          "circumscribed polygon retains q headroom at d=L");
    limiter.prepare(number<V>(-1));
    out = limiter.apply(R::fromRaw(INT32_MIN), R::fromRaw(INT32_MAX));
    check(out.d == V{} && out.q == V{} && out.limited, "nonpositive limit disables both axes");
}

static void testFloat() {
    PolygonVoltageLimit<FloatProfile> limiter;
    for (float l : {0.0f, 1e-6f, 0.1f, 1.0f, 20.0f, 8191.0f}) {
        limiter.prepare(l);
        const auto exercise = [&](float d) {
            const double ideal = idealRoom(d, l);
            const float q = limiter.qLimit(d);
            check(std::abs(q - ideal) <= 0.000002 * l + 1e-12,
                  "float envelope matches independent half-plane geometry");
            check(q >= 0 && q <= l && q == limiter.qLimit(-d), "float range and symmetry");
        };
        for (int i = -5000; i <= 5000; ++i) exercise(l * (i / 5000.0f));
        for (unsigned k = 0; k < 7; ++k) {
            float d = l * std::sin((k + 0.5) * PI / 16) / std::cos(PI / 32);
            for (unsigned i = 0; i < 32; ++i) d = std::nextafter(d, 0.0f);
            for (unsigned i = 0; i < 64; ++i) { exercise(d); d = std::nextafter(d, INFINITY); }
        }
    }
    limiter.prepare(20.0f);
    const auto inside = limiter.apply(1.0f, 20.0f), outside = limiter.apply(25.0f, -25.0f);
    check(!inside.limited && outside.limited && outside.d == 20.0f && outside.q < -1.96f,
          "float flags and d priority use the polygon rather than the circle");
}

template<class P>
static void testCore() {
    using V = typename P::Voltage;
    using C = typename P::Current;
    using D = typename P::Duty;
    using T = FocTransforms<P>;
    using Core = FocControlCore<P, 16000>;
    Core core;
    FocParamsT<P> params;
    params.angle_offset_rad = {};
    core.setCurrentPiGains(number<typename P::Gain>(1), {});
    MotorAlgorithmInputT<P> in;
    in.bus_voltage = number<V>(40);
    in.electric_mode = ElectricMode::CURRENT_MODE;
    in.drive_mode = DriveMode::FORWARD;
    // Large over-demand at all orientations, both PWM modes and all duty spans.
    for (bool low : {false, true}) for (float max_duty : {0.5f, 0.6f, 0.95f, 1.0f}) {
        params.low_side_clamped_pwm = low;
        params.max_duty_cycle = numberCast<D>(max_duty);
        params.max_modulation = number<D>(1);
        params.anti_windup_kb = {};
        for (float d_request : {-100.0f, -10.0f, 0.0f, 10.0f, 100.0f}) {
            params.id_target = numberCast<C>(d_request);
            core.setParams(params);
            for (unsigned angle = 0; angle < 720; ++angle) {
                in.angle_steps = number<typename P::Angle>(angle, 120);
                in.target_current = number<C>(100);
                const auto out = core.update(in);
                const auto rotation = T::rotation(out.angle_e_rad);
                const auto ab = T::invPark(DqVectorT<V>{out.v_d, out.v_q}, rotation);
                // Reconstruct applied voltage solely from the clamped duties.
                const double u = numberCast<float>(out.duty_u), v = numberCast<float>(out.duty_v);
                const double w = numberCast<float>(out.duty_w);
                const double alpha = 40 * (2 * u - v - w) / 3, beta = 40 * (v - w) / std::sqrt(3.0);
                check(std::abs(alpha - numberCast<float>(ab.alpha)) < 0.0001 &&
                      std::abs(beta - numberCast<float>(ab.beta)) < 0.0001,
                      "polygon corners are physically reproduced rather than clipped independently by PWM");
                const float minimum = low ? 0 : 1 - max_duty;
                check(u >= minimum - 1e-7 && v >= minimum - 1e-7 && w >= minimum - 1e-7 &&
                      u <= max_duty + 1e-7 && v <= max_duty + 1e-7 && w <= max_duty + 1e-7,
                      "PWM duty span remains bounded");
                check(out.saturation == 1, "outer-loop saturation sign follows actual clamp");
            }
        }
    }
    // Anti-windup must not alter an attainable point between circle and polygon.
    params.max_modulation = number<D>(1, 5);
    params.max_duty_cycle = number<D>(95, 100);
    params.low_side_clamped_pwm = true;
    params.anti_windup_kb = number<typename P::Fraction>(1);
    params.id_target = number<C>(1, 4);
    core.setParams(params);
    core.reset();
    in.angle_steps = {};
    const V l = multiplyTo<V>(params.max_modulation, T::voltageLimit(in.bus_voltage));
    // Floor the target to the wide-request grid: the voltage must be attainable.
    const float q = std::floor(numberCast<float>(l) * 4096) / 4096;
    in.target_current = numberCast<C>(q);
    for (unsigned i = 0; i < 100; ++i) {
        const auto out = core.update(in);
        check(out.saturation == 0 && std::abs(numberCast<float>(out.v_q) - q) < 0.000001f &&
              out.v_d == number<V>(1, 4), "annulus requests do not trigger false anti-windup");
    }
    // A real clamp back-calculates immediately; release the error without reset.
    in.target_current = number<C>(100);
    const auto saturated = core.update(in);
    check(saturated.saturation == 1, "real over-demand activates anti-windup");
    in.target_current = {};
    const auto recovery = core.update(in);
    check(recovery.v_q < V{}, "back-calculation opposes the previous unreachable q request");
}

int main() {
    testFixed();
    testFloat();
    testCore<FloatProfile>();
    testCore<FixedProfile>();
    std::printf("Polygon voltage limit: %u checks, %d failures\n", checks, failures);
    return failures != 0;
}
