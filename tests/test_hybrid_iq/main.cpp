/** @file main.cpp
 * @brief Hybrid Iq against a delayed PMSM/inverter, noisy sensing and model error
 */
#include "foc_algorithm.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

using namespace libecu;
using P = ActiveNumericProfile;
static int checks = 0, failures = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}
static constexpr float PI = 3.14159265359f;
static constexpr float R = 0.658f, L = 0.0007f, FLUX = 0.012336f;

// The plant consumes actual duties after a one-frame delay, not requested dq
// voltage. Rotor motion during the delay/integration is included. No PLL here:
// this experiment isolates current regulation, not Hall-estimator quality.
template<uint32_t Hz>
struct Rig {
    using Core = FocControlCore<P, Hz>;
    using I = typename P::Current;
    using V = typename P::Voltage;
    using A = typename P::Angle;
    Core core;
    typename Core::Params params;
    typename Core::Input in{};
    float r = R, l = L, flux = FLUX;
    float id = 0, iq = 0, theta = 0, omega = 0, bus = 40;
    float applied_alpha = 0, applied_beta = 0;
    unsigned tick = 0;

    explicit Rig(bool hybrid = true) {
        params.angle_offset_rad = A{};
        params.low_side_clamped_pwm = true;
        params.flux_linkage_wb = numberCast<typename P::Flux>(FLUX);
        params.l_phase_h = numberCast<typename P::Inductance>(L);
        check(makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER, 1000, Hz,
                                             params.current_filter), "Id/Iq filter design");
        HybridIqParams h;
        h.enabled = hybrid;
        h.r_phase_ohm = R;
        check(makeHybridIqParams<P, Hz>(h, params.hybrid_iq), "hybrid coefficient design");
        core.setParams(params);
        core.setCurrentPiGains(numberCast<typename Core::Gain>(0.549779f),
                               numberCast<typename Core::Gain>(516.792f));
        core.reset();
        in.drive_mode = DriveMode::FORWARD;
        in.electric_mode = ElectricMode::CURRENT_MODE;
    }

    typename Core::Output step(float target, float noise_q = 0, float noise_d = 0) {
        const float s = std::sin(theta), c = std::cos(theta);
        const float alpha = (id + noise_d)*c - (iq + noise_q)*s;
        const float beta = (id + noise_d)*s + (iq + noise_q)*c;
        in.i_u = numberCast<I>(-alpha);
        in.i_v = numberCast<I>(0.5f*alpha - 0.866025404f*beta);
        in.i_w = numberCast<I>(0.5f*alpha + 0.866025404f*beta);
        in.angle_steps = numberCast<A>(theta/(PI/3));
        in.speed_steps_s = numberCast<typename P::PllSpeed>(omega/(PI/3));
        in.bus_voltage = numberCast<V>(bus);
        in.target_current = numberCast<I>(target);
        const auto out = core.update(in);
        for (int k = 0; k < 8; ++k) {
            const float ss = std::sin(theta), cc = std::cos(theta);
            const float vd = applied_alpha*cc + applied_beta*ss;
            const float vq = -applied_alpha*ss + applied_beta*cc;
            const float did = (vd-r*id+omega*l*iq)/l;
            const float diq = (vq-r*iq-omega*l*id-omega*flux)/l;
            id += did/(8*Hz);
            iq += diq/(8*Hz);
            theta += omega/(8*Hz);
        }
        theta = std::remainder(theta, 2*PI);
        const float u = numberCast<float>(out.duty_u);
        const float v = numberCast<float>(out.duty_v);
        const float w = numberCast<float>(out.duty_w);
        applied_alpha = bus*(2*u-v-w)/3;
        applied_beta = bus*(v-w)/std::sqrt(3.0f);
        ++tick;
        return out;
    }
};

static void parameterValidation() {
    HybridIqParams h;
    h.enabled = true;
    h.r_phase_ohm = R;
    HybridIqParamsT<P> out;
    check(makeHybridIqParams<P, 16000>(h, out), "valid hybrid config");
    const auto original = out.alpha;
    for (float invalid : {-1.0f, 0.0f, std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()}) {
        auto bad = h;
        bad.averaging_time_s = invalid;
        check(!makeHybridIqParams<P, 16000>(bad, out) && out.alpha == original,
              "invalid time constant is rejected atomically");
    }
    auto bad = h;
    bad.r_phase_ohm = 0;
    check(!makeHybridIqParams<P, 16000>(bad, out), "enabled model needs positive R");
    bad = h; bad.integral_gain = -1;
    check(!makeHybridIqParams<P, 16000>(bad, out), "negative adaptation rejected");
    bad = h; bad.correction_slew_v_s = 0;
    check(!makeHybridIqParams<P, 16000>(bad, out), "zero slew rejected");
    bad = h; bad.r_phase_ohm = std::numeric_limits<float>::quiet_NaN();
    check(!makeHybridIqParams<P, 16000>(bad, out), "NaN resistance rejected");
}

template<uint32_t Hz>
static void transients() {
    Rig<Hz> rig;
    const auto first = rig.step(1.0f);
    check(std::fabs(numberCast<float>(first.v_q)-R) < 0.001f,
          "first voltage responds to target without waiting for average");
    for (unsigned n=1; n<Hz/200; ++n) rig.step(1.0f);
    check(std::fabs(rig.iq-1.0f) < 0.04f, "nominal current step settles within 5 ms");
    for (unsigned n=0; n<Hz; ++n) rig.step(1.0f);
    check(std::fabs(rig.iq-1.0f) < 0.01f, "slow adaptation retains DC tracking");
    for (unsigned n=0; n<Hz/100; ++n) rig.step(-1.0f);
    check(std::fabs(rig.iq+1.0f) < 0.05f, "braking current responds without slow-feedback delay");

    rig.core.reset();
    const auto spike = rig.step(0.0f, 10.0f);
    check(spike.v_q == typename P::Voltage{}, "first-sample spike cannot seed Uq");
    const auto after = rig.step(0.0f);
    check(std::fabs(numberCast<float>(after.v_q)) < 0.002f, "spike has only bounded slow influence");

    // Model error with persistent offset: exercise both directions of trim.
    for (float actual_r : {0.22f, 0.9f}) {
        Rig<Hz> mismatch;
        mismatch.r = actual_r;
        mismatch.l = actual_r == 0.22f ? 0.000175f : 0.0012f;
        for (unsigned n=0; n<Hz*3; ++n) mismatch.step(1.5f);
        check(std::fabs(mismatch.iq-1.5f) < 0.02f, "trim corrects resistance mismatch");
    }

    // Long impossible demand must not leave a large hidden integrator.
    Rig<Hz> saturated;
    for (unsigned n=0; n<Hz; ++n) saturated.step(100.0f);
    check(saturated.step(100.0f).saturation == 1, "final limiter reports positive saturation");
    for (unsigned n=0; n<Hz; ++n) saturated.step(0.0f);
    check(std::fabs(saturated.iq) < 0.05f, "recovers after prolonged positive saturation");
    for (unsigned n=0; n<Hz; ++n) saturated.step(-100.0f);
    check(saturated.step(-100.0f).saturation == -1, "final limiter reports negative saturation");
    for (unsigned n=0; n<Hz; ++n) saturated.step(0.0f);
    check(std::fabs(saturated.iq) < 0.05f, "recovers after prolonged negative saturation");

    // Id-priority clipping, voltage-mode bypass and state re-entry.
    Rig<Hz> modes;
    modes.params.id_target = number<typename P::Current>(50);
    modes.core.setParams(modes.params);
    for (unsigned n=0; n<Hz/10; ++n) modes.step(1);
    modes.params.id_target = typename P::Current{};
    modes.core.setParams(modes.params);
    for (unsigned n=0; n<Hz; ++n) modes.step(1);
    check(std::fabs(modes.iq-1) < 0.05f && std::fabs(modes.id) < 0.01f,
          "Id-priority saturation and reconfiguration recover");
    modes.in.electric_mode = ElectricMode::VOLTAGE_MODE;
    modes.in.duty_command = number<typename P::Duty>(1,10);
    const auto voltage = modes.step(20,10);
    check(std::fabs(numberCast<float>(voltage.v_q)-2.193f) < 0.02f,
          "voltage mode bypasses both hybrid model and measured current");
    modes.in.electric_mode = ElectricMode::CURRENT_MODE;
    check(std::fabs(numberCast<float>(modes.step(1).v_q)-R) < 0.001f,
          "voltage mode clears adaptation before re-entry");

    Rig<Hz> bus_recovery;
    bus_recovery.params.id_target = number<typename P::Current>(20);
    bus_recovery.core.setParams(bus_recovery.params);
    bus_recovery.bus = 10;
    for (unsigned n=0; n<Hz/2; ++n) bus_recovery.step(1.5f);
    bus_recovery.bus = 40; // No reset/reconfiguration: exercise the actual stored states.
    for (unsigned n=0; n<Hz*2; ++n) bus_recovery.step(1.5f);
    check(std::fabs(bus_recovery.id-20) < 0.02f && std::fabs(bus_recovery.iq-1.5f) < 0.05f,
          "Id-priority clipping recovers when bus headroom returns without state reset");

    Rig<Hz> toggle(false);
    for (unsigned n=0; n<Hz/20; ++n) toggle.step(2);
    toggle.params.hybrid_iq.enabled = true;
    toggle.core.setParams(toggle.params);
    toggle.step(0);
    toggle.params.hybrid_iq.enabled = false;
    toggle.core.setParams(toggle.params);
    toggle.core.setCurrentFilter({}); // Remove the measurement LPF history for this assertion.
    toggle.id = toggle.iq = 0;
    check(toggle.step(0).v_q == typename P::Voltage{}, "strategy toggle cannot revive a stale PI integral");
}

struct Stats {
    float mean = 0, ripple = 0, voltage_ripple = 0;
};
template<uint32_t Hz>
static Stats rotating(float rps, bool hybrid, bool noisy, bool mismatch = false, bool braking = false) {
    Rig<Hz> rig(hybrid);
    rig.omega = 2*PI*20*rps;
    rig.in.drive_mode = rps < 0 ? DriveMode::REVERSE : DriveMode::FORWARD;
    if (mismatch) {
        rig.r = 0.22f;
        rig.l = 0.000175f;
        rig.flux *= 1.15f;
    }
    const float target = braking ? -1.5f : 1.5f;
    const float physical_target = rps < 0 ? -target : target;
    double sum = 0, squares = 0, vsum = 0, vsquares = 0;
    bool finite = true, bounded = true;
    for (unsigned n=0; n<Hz*3; ++n) {
        // Coherent Hall-rate ripple plus bipolar switching outliers; a
        // deliberately synthetic sensor trace, not a recording from HW051.
        const float noise = noisy ? 2*std::sin(6*rig.omega*n/Hz) +
            (n%97 == 0 ? ((n/97)%2 ? 8.0f : -8.0f) : 0.0f) : 0;
        const auto out = rig.step(target, noise);
        finite &= std::isfinite(rig.iq) && std::isfinite(rig.id);
        const float u = numberCast<float>(out.duty_u), v = numberCast<float>(out.duty_v);
        const float w = numberCast<float>(out.duty_w);
        bounded &= u >= 0 && v >= 0 && w >= 0 && u <= 0.951f && v <= 0.951f && w <= 0.951f;
        if (n >= Hz*2) {
            const float voltage = numberCast<float>(out.v_q);
            sum += rig.iq; squares += rig.iq*rig.iq;
            vsum += voltage; vsquares += voltage*voltage;
        }
    }
    check(finite && bounded, "rotating delayed inverter remains finite and within PWM bounds");
    Stats result;
    result.mean = float(sum/Hz);
    result.ripple = float(std::sqrt(std::max(0.0, squares/Hz-(sum/Hz)*(sum/Hz))));
    result.voltage_ripple = float(std::sqrt(std::max(0.0, vsquares/Hz-(vsum/Hz)*(vsum/Hz))));
    check(std::fabs(result.mean-physical_target) < 0.06f, "mean torque current tracks in rotating plant");
    return result;
}

static void correctionBounds() {
    HybridIqParams h;
    h.enabled = true; h.r_phase_ohm = R;
    HybridIqParamsT<P> p;
    check(makeHybridIqParams<P, 16000>(h,p), "bound test configuration");
    HybridIqControl<P> control;
    float previous = 0;
    for (unsigned n=0; n<16000*2; ++n) {
        const auto requested = control.request(number<typename P::Current>(10), typename P::Current{}, p);
        control.integrate(requested, numberCast<typename P::Voltage>(requested), p);
        const float correction = numberCast<float>(control.correction());
        check(correction <= 4.00001f && correction-previous <= 10.0f/16000+0.00001f,
              "correction voltage and slew remain bounded");
        previous = correction;
    }
    check(std::fabs(previous-4.0f) < 0.001f, "persistent bias reaches bounded trim");
    control.reset();
    for (unsigned n=0; n<16000; ++n) {
        const auto requested = control.request(number<typename P::Current>(10), typename P::Current{}, p);
        control.integrate(requested, typename P::Voltage{}, p);
    }
    check(control.correction() == typename P::Accumulator{}, "clipped output cannot wind up trim");
    // Exercise saturated innovations near both Q20 current endpoints. This
    // validates the bounded raw arithmetic under UBSan, not only motor-scale inputs.
    h.averaging_time_s = 0.000001f;
    check(makeHybridIqParams<P, 16000>(h,p), "fast convex filter coefficient remains valid");
    control.reset();
    for (unsigned n=0; n<10000; ++n) {
        const auto target = number<typename P::Current>(n%3 ? 2047 : -2048);
        const auto measured = number<typename P::Current>(n%7 ? -2048 : 2047);
        const auto requested = control.request(target, measured, p);
        control.integrate(requested, numberCast<typename P::Voltage>(requested), p);
        check(std::isfinite(numberCast<float>(requested)) &&
              std::fabs(numberCast<float>(control.correction())) <= 4.00001f,
              "full-range saturated innovations cannot overflow the bounded state");
    }
}

class Pwm final : public PwmInterface {
public:
    bool initialize(uint32_t, uint16_t) override { return true; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float,float,float) override {}
    void enable(bool) override {}
    void apply() override {}
};

static void publicConfiguration() {
    Pwm pwm;
    FocAlgorithm foc(pwm);
    auto params = foc.getParams();
    params.hybrid_iq.enabled = true;
    params.hybrid_iq.r_phase_ohm = R;
    check(!foc.setParams(params), "public API rejects hybrid without back-EMF model");
    params.flux_linkage_wb = FLUX;
    params.l_phase_h = L;
    check(foc.setParams(params), "public API installs hybrid");
    auto bad = params; bad.hybrid_iq.averaging_time_s = -1;
    check(!foc.setParams(bad) && foc.getParams().hybrid_iq.averaging_time_s == 0.02f,
          "failed public configuration preserves old parameters");
}

int main() {
    parameterValidation();
    publicConfiguration();
    correctionBounds();
    transients<8000>();
    transients<16000>();
    transients<20000>();
    for (float speed : {2.0f,3.0f,5.0f,7.0f,9.0f}) {
        const auto pi = rotating<16000>(speed, false, true);
        const auto hybrid = rotating<16000>(speed, true, true);
        std::printf("%s %.0f RPS: synthetic sensor noise -> physical Iq RMS %.4f -> %.4f A, "
                    "Uq RMS %.4f -> %.4f V\n", CONTROL_PROFILE_NAME, speed,
                    pi.ripple, hybrid.ripple, pi.voltage_ripple, hybrid.voltage_ripple);
        check(hybrid.ripple < pi.ripple*0.2f, "hybrid attenuates sensor-induced current ripple");
        check(hybrid.voltage_ripple < pi.voltage_ripple*0.1f, "no fast measured-current P in Uq");
        rotating<16000>(speed, true, false, true);
    }
    rotating<8000>(-5, true, true);
    rotating<20000>(-5, true, true, true);
    rotating<16000>(5, true, false, false, true);
    rotating<16000>(-5, true, false, false, true);
    std::printf("%s: %d checks, %d failures\n", CONTROL_PROFILE_NAME, checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
