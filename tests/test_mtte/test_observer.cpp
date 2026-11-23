/** @file test_observer.cpp @brief Independent SI/sign, phase, validity and control-bound contracts. */
#include "fixture.hpp"
#include "numeric_ops.hpp"
#include "pi_float_interface.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
using namespace fixture;
using State = MtteObserver::State;
using Out = MtteObserver::Result;
constexpr float TAU = MtteObserver::TWO_PI;

void check(bool ok, const char* detail) {
    if (!ok) { std::fprintf(stderr, "MTTE contract failed: %s\n", detail); std::exit(1); }
}
void near(float a, float b, float tolerance, const char* detail) {
    if (!(std::abs(a - b) <= tolerance)) std::fprintf(stderr, "got %.8g, expected %.8g +/- %.3g: ", a,b,tolerance);
    check(std::abs(a - b) <= tolerance, detail);
}

void mechanics(DriveMode mode, float accel_rps2, float iq_a) {
    // Geometry deliberately different from the model: dropping 2*pi, r², alpha or M is observable.
    auto p = PARAMS;
    p.wheel_inertia_kg_m2 = 0.18f; p.kt_nm_per_a = 0.4f; p.radius_m = 0.15f;
    p.mass_kg = 15.0f; p.relaxation = 0.8f; p.probe_a = 0.0f;
    MtteObserver observer;
    check(observer.configure(p), "valid independent calibration");
    const float sign = mode == DriveMode::REVERSE ? -1.0f : 1.0f;
    Out o;
    for (unsigned k = 0; k <= 300; ++k) {
        const auto f = sample(k, sign * (8.0f + accel_rps2 * (k * 0.01f)), sign * iq_a, mode);
        o = observer.update(k, f, 11.0f, 10.0f);
    }
    // Deliberately use wheel and longitudinal equations rather than the implementation's product for E8.
    const float contact_force = (p.kt_nm_per_a * iq_a - p.wheel_inertia_kg_m2 * TAU * accel_rps2) / p.radius_m;
    const float chassis_accel = contact_force / p.mass_kg; // E2 with zero road resistance.
    const float max_t = p.radius_m * contact_force + p.wheel_inertia_kg_m2 * chassis_accel / (p.relaxation*p.radius_m);
    check(o.state == State::Tracking, "rolled and settled");
    near(o.acceleration_rad_s2, TAU*accel_rps2, 0.001f, "signed mechanical acceleration in radians");
    near(o.motor_nm, p.kt_nm_per_a * iq_a, 0.0002f, "measured, not demanded, motor torque in selected frame");
    near(o.force_n, contact_force, 0.002f, "signed E6 in N");
    near(o.tmax_nm, max_t, 0.001f, "E7 longitudinal acceleration ratio implies E8 in Nm");
    near(o.positive_ceiling_a, std::clamp(max_t / p.kt_nm_per_a, 0.0f, 10.0f), 0.004f,
         "positive-only authority, including hard ceiling");
}

void matchedFreeWheel() {
    // Continuous independent oracle. Free wheel, sinusoidal torque; analytic response of the locked PLL.
    // H(jw)=Ki/(Ki-w²+j*Kp*w). 100-Hz interval-average current and two speed endpoints share a clock.
    constexpr double w = 2.0 * 3.141592653589793 * 3.0, a = 4.0;
    const double re = PARAMS.pll_ki - w*w, im = PARAMS.pll_kp*w;
    const double amp = PARAMS.pll_ki / std::hypot(re, im);
    const double phase = -std::atan2(im, re);
    MtteObserver observer;
    check(observer.configure(PARAMS), "configure sinusoidal case");
    float max_force = 0.0f;
    for (unsigned k = 0; k < 500; ++k) {
        const double t = 0.01*k;
        const float n = 8 + KT*a/(J*TAU*w) * amp * std::sin(w*t + phase);
        const float iq = a * (std::sin(w*t) - std::sin(w*(t-0.01))) / (w*0.01);
        const Out o = observer.update(k, sample(k,n,iq), 10.0f, 20.0f);
        if (k > 130) {
            check(o.state == State::Tracking, "locked sinusoid remains observable");
            max_force = std::max(max_force, std::abs(o.force_n));
        }
    }
    std::printf("Free-wheel 3-Hz phase oracle: max |Fhat| = %.5f N (motor amplitude %.2f N equivalent)\n",
                max_force, KT*a/R);
    check(max_force < 0.12f, "phase-matched torque cancels free-wheel acceleration at 3 Hz");
}

void policy() {
    MtteObserver observer;
    const auto parked = sample(0, 0.0f, 12.0f);
    const auto stop = [](Out o) { return o.state == State::Unavailable && o.positive_ceiling_a == 0.0f; };
    check(stop(observer.update(0, parked, 10, 20)), "default disabled");
    check(observer.configure(PARAMS), "enable explicit calibration");
    check(stop(observer.update(0, parked, 10, 20)), "no inferred standstill grip");
    // Cross uint32 rollover without losing phase or differentiating from zero.
    uint32_t k = UINT32_MAX - 20u;
    for (uint32_t i=0; i<=PARAMS.settling_ticks; ++i, ++k) {
        const auto o = observer.update(k, sample(k,3,0), 4, 20);
        check(o.state == (i == PARAMS.settling_ticks ? State::Tracking : State::Settling), "settle across wrap");
        if (i < PARAMS.settling_ticks) check(o.positive_ceiling_a == 0.0f, "no preload fabricated while settling");
    }
    // Exact zero force and nonzero stationary demand: bounded probes, off the rest of the time.
    unsigned probes = 0;
    for (unsigned i=0; i<128; ++i, ++k) {
        const Out o = observer.update(k, sample(k,3,0), 4, 0.2f);
        check(o.force_n == 0.0f && o.tmax_nm == 0.0f && o.rise_a == 0.0f, "no fake force or E17 on a plateau");
        check(o.positive_ceiling_a == 0.0f || o.positive_ceiling_a == 0.2f, "independent cap clamps probes too");
        probes += o.positive_ceiling_a != 0.0f;
    }
    check(probes == 8, "bounded probe duty even at zero grip");
    while (k % 64u < 4u) { observer.update(k, sample(k,3,0), 0, 20); ++k; }
    auto step = [&](float demand) {
        const auto o = observer.update(k, sample(k,3,0), demand, 20); ++k; return o;
    };
    const Out rise = step(10), flat = step(10), fall = step(8);
    check(rise.rise_a == PARAMS.rise_max_a && flat.rise_a == 0 && fall.rise_a == 0, "bounded E17 only on a rise");
    near(step(8.05f).rise_a, 0.4f, 0.0001f, "E17 time gain times positive slope in A/s");
    for (unsigned i=0; i<65; ++i) {
        const auto o = step(i % 2 ? -4.0f : 0.0f);
        check(o.probe_a == 0 && o.rise_a == 0 && o.positive_ceiling_a == 0, "zero/braking creates no drive");
    }
    // A legal pair after any fault must start over. Include stale predictions with a fresh tick number.
    for (unsigned fault=0; fault<9; ++fault) {
        auto f = sample(k, 3, 10);
        float limit = 20;
        switch (fault) {
        case 0: --f.iq_tick; break;
        case 1: --f.speed_tick; break;
        case 2: f.iq_complete = false; break;
        case 3: f.hall_valid = false; break;
        case 4: f.mean_iq_a = std::numeric_limits<float>::quiet_NaN(); break;
        case 5: f.hall_rps = std::numeric_limits<float>::infinity(); break;
        case 6: f.direction = DriveMode::NEUTRAL; break;
        case 7: f.hall_rps = -3.0f; break; // spinning opposite to requested forward
        case 8: limit = -0.1f; break;
        }
        check(stop(observer.update(k, f, 10, limit)), "absent/inconsistent inputs fail closed");
        ++k;
        for (unsigned i=0; i<=PARAMS.settling_ticks; ++i, ++k) {
            const auto o = observer.update(k, sample(k,3,0), 10, 20);
            check(o.state == (i == PARAMS.settling_ticks ? State::Tracking : State::Settling),
                  "fresh contiguous recovery");
            near(o.force_n, 0, 0, "no stale impulse after reset");
        }
    }
    ++k; // missed a whole interval
    check(observer.update(k, sample(k,4,0), 10,20).state == State::Settling, "gap reseeds speed");
    ++k;
    check(observer.update(k, sample(k,-4,0,DriveMode::REVERSE), 10,20).state == State::Settling, "direction handoff");
    auto p = PARAMS;
    p.wheel_inertia_kg_m2 = 0;
    check(!observer.configure(p) && stop(observer.update(k, sample(k,3,0), 10,20)),
          "reject J=0 and disable old config");
    p = PARAMS; p.kt_nm_per_a = -KT; check(!observer.configure(p), "reject wrong Kt sign in calibration");
    p = PARAMS; p.radius_m = 0; check(!observer.configure(p), "reject zero radius");
    p = PARAMS; p.mass_kg = 0; check(!observer.configure(p), "airborne is zero contact, not zero body mass in E8");
    p = PARAMS; p.relaxation = 0; check(!observer.configure(p), "reject singular alpha");
}

void signedStimulus() {
    MtteObserver obs;
    auto p = PARAMS;
    p.probe_a = 0; // isolate E17 from the rolling probe
    check(obs.configure(p), "E17 fixture");
    for (unsigned k=0; k<180; ++k) obs.update(k, sample(k, 3,-4), 1,20);
    const auto o = obs.update(180, sample(180, 3,-4), 10,20);
    check(o.tmax_nm < -1.0f && o.rise_a == p.rise_max_a && o.positive_ceiling_a==0,
          "apply E17 to signed E8, then clamp; negative evidence is not silently erased");
}

void antiwindup() {
    // Same sustained 1-RPS error under a narrow dynamic ceiling. Negative control: clipping
    // externally masks the applied output from the integral. Only the actual capped PI can unwind it.
    using namespace libecu;
    using Pi = PiFloatInterface<FixedProfile::SpeedPiTypes,100>;
    using I = FixedProfile::Current;
    using N = FixedProfile::Speed;
    Pi capped, unaware;
    check(capped.setParameters(COMS3_PI) && unaware.setParameters(COMS3_PI), "fixed COMS3 PI");
    auto p=PARAMS;
    p.probe_a=0;
    MtteObserver obs;
    check(obs.configure(p),"anti-windup fixture");
    for (unsigned k=0;k<400;++k) {
        auto preview=capped;
        const auto ordinary=preview.updateNativeCapped(number<N>(4),number<N>(3),0,number<I>(45),number<I>(45));
        const auto o=obs.update(k,sample(k,3,0.5f),ordinary.toFloat(),45);
        const auto high=numberCast<I>(o.positive_ceiling_a);
        const auto actual=capped.updateNativeCapped(number<N>(4),number<N>(3),0,number<I>(45),high);
        const auto raw=unaware.updateNativeCapped(number<N>(4),number<N>(3),0,number<I>(45),number<I>(45));
        check(actual==std::min(raw,high),"identical bounded outputs while load ceiling is binding");
    }
    const float first=capped.updateNativeCapped(number<N>(3),number<N>(3),0,number<I>(45),number<I>(45)).toFloat();
    const float hidden=unaware.updateNativeCapped(number<N>(3),number<N>(3),0,number<I>(45),number<I>(45)).toFloat();
    std::printf("Release at zero speed error: actual bounded PI %.3f A; external clip negative control %.3f A\n",
                first,hidden);
    check(first<8.0f && hidden>35.0f,"back-calculation saw the applied MTTE cap, not the unconstrained output");
}
} // namespace

int main() {
    mechanics(DriveMode::FORWARD,  1.0f,  5.0f);
    mechanics(DriveMode::REVERSE,  1.0f,  5.0f);
    mechanics(DriveMode::FORWARD, -0.8f,  2.0f);
    mechanics(DriveMode::FORWARD,  1.0f, -2.0f);
    matchedFreeWheel();
    policy();
    signedStimulus();
    antiwindup();
    std::puts("PASS: SI/sign, PLL phase, data continuity, E17, positive-only policy and actual PI anti-windup");
}
