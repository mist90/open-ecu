/** @file main.cpp @brief Selected current calibration from Hall speed through the real d/q regulators. */
#include "control_calibration.hpp"
#include "foc_algorithm.hpp"
#include "motor_config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <type_traits>

namespace {
using namespace libecu;
using P = ActiveNumericProfile;
using Core = FocAlgorithm::Core;
static_assert(ActiveTiming::CURRENT_HZ == 16000);

// Physical oracle in Hall sectors/s. One Hall step is not a mechanical revolution.
double weight(bool q_axis, double signed_speed) {
#ifdef MOTOR_1
    const double speed = std::abs(signed_speed);
    constexpr double tail = 1.0 / 16.0;
    if (speed < 75.0)
        return tail + (1.0 - tail) * std::max(0.0, speed - 60.0) / 15.0;
    if (q_axis && speed > 360.0)
        return 1.0 - (1.0 - tail) * std::min(120.0, speed - 360.0) / 120.0;
#else
    (void)q_axis;
    (void)signed_speed;
#endif
    return 1.0;
}

void near(double actual, double expected, double tolerance, const char* signal, double speed) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr, "%s @ %.8g Hall steps/s: got %.10g, expected %.10g +/- %.3g\n",
                     signal, speed, actual, expected, tolerance);
        std::exit(1);
    }
}

// The real weights are dyadic even between knots. A binary64 product has
// room for all 32 current bits and all fractional bits of either ramp.
void checkRawWeights(int32_t velocity) {
    using F = FixedProfile;
    const double speed = double(velocity) / F::PllSpeed::SCALE;
    const auto v = F::PllSpeed::fromRaw(velocity);
    for (int32_t raw : {INT32_MIN, INT32_MIN + 1, -16000001, -24, -9, -8, -7, -1, 0, 1,
                        7, 8, 9, 24, 16000001, INT32_MAX - 1, INT32_MAX}) {
        const auto e = F::Current::fromRaw(raw);
        const auto d = weightedCurrentError<F, motor_config::current_loop, false>(e, v);
        const auto q = weightedCurrentError<F, motor_config::current_loop, true>(e, v);
        near(d.raw(), std::llround(raw * weight(false, speed)), 0, "raw d error", speed);
        near(q.raw(), std::llround(raw * weight(true, speed)), 0, "raw q error", speed);
    }
}

void rawCurve() {
    for (int step = -490; step <= 490; ++step) {
        const auto speed = number<FixedProfile::PllSpeed>(step).raw();
        for (int delta : {-1, 0, 1, 27}) checkRawWeights(speed + delta);
    }
    checkRawWeights(INT32_MIN);
    checkRawWeights(INT32_MAX);
}

struct Pwm final : PwmInterface {
    bool initialize(uint32_t, uint16_t) override { return true; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float, float, float) override {}
    void enable(bool) override {}
};

void activeFoc() {
    Pwm pwm;
    FocAlgorithm foc(pwm);
    auto options = foc.getParams();
    options.use_decoupling = true;
#ifdef MOTOR_1
    if (foc.setParams(options) || foc.getParams().use_decoupling) {
        std::fputs("PI-only scooter accepted decoupling\n", stderr);
        std::exit(1);
    }
#else
    // With zero current error a rotating motor needs only its signed back-EMF.
    options.flux_linkage_wb = 0.02f;
    if (!foc.setParams(options)) std::exit(1);
    for (int direction : {-1, 1}) {
        Core::Input in{};
        in.drive_mode = DriveMode::FORWARD;
        in.bus_voltage = number<P::Voltage>(40);
        in.speed_steps_s = number<P::PllSpeed>(direction * 60); // 10 electrical revolutions/s.
        const auto out = foc.updateNative(in);
        near(numberCast<float>(out.v_d), 0, 0, "zero-current decoupled Vd", direction * 60);
        near(numberCast<float>(out.v_q), direction * 20 * std::acos(-1.0) * options.flux_linkage_wb,
             0.001, "back-EMF compensation", direction * 60);
    }
    options.use_decoupling = false;
    if (!foc.setParams(options)) std::exit(1);
#endif
    // At 16 kHz, Ki=125 integrates one weighted error in exactly 128 frames.
    if (!foc.setAngleOffsetRad(0) || !foc.setCurrentPiGains(1, 125)) {
        std::fputs("FOC fixture setup failed\n", stderr);
        std::exit(1);
    }
    constexpr double v_lsb = 1.0 / 4096.0; // Fixed voltage request is Q12.
    for (float s : {0.f, 20.f, 60.f, 67.5f, 75.f, 180.f, 360.f, 420.f, 480.f, 800.f}) {
        for (int direction : {-1, 1}) for (int sign : {-1, 1}) {
            for (DriveMode mode : {DriveMode::FORWARD, DriveMode::REVERSE}) {
                const float speed = direction * s;
                Core::Input in{};
                in.drive_mode = mode;
                in.bus_voltage = number<Core::Voltage>(40);
                in.speed_steps_s = numberCast<P::PllSpeed>(speed);
                in.i_u = number<Core::Current>(sign);
                in.i_v = -in.i_u;
                in.target_current = number<Core::Current>(2 * sign);
                foc.reset();
                const auto out = foc.updateNative(in);
                // At angle zero, these zero-sum ADC phases yield Id=-sign, Iq=sign/sqrt(3).
                const double iq = sign / std::sqrt(3.0);
                near(numberCast<float>(out.i_d), -sign, 1e-6, "measured Id", speed);
                near(numberCast<float>(out.i_q), iq, 1e-6, "measured Iq", speed);
                const double target_q = (mode == DriveMode::REVERSE ? -2 : 2) * sign;
                const double d = weight(false, speed) * sign;
                const double q = weight(true, speed) * (target_q - iq);
                near(numberCast<float>(out.v_d), d, v_lsb, "d proportional voltage", speed);
                near(numberCast<float>(out.v_q), q, v_lsb, "q proportional voltage", speed);
                for (unsigned k = 1; k < 128; ++k) foc.updateNative(in);
                // Cross into a different band with zero d/q error: accumulated
                // volts survive the handoff without a rescale or reset.
                in.i_u = in.i_v = in.target_current = {};
                in.speed_steps_s = number<P::PllSpeed>(s < 75 ? 200 : 0);
                const auto held = foc.updateNative(in);
                near(numberCast<float>(held.v_d), d, v_lsb, "stored d voltage", speed);
                near(numberCast<float>(held.v_q), q, v_lsb, "stored q voltage", speed);
            }
        }
    }
}

void genericFoc() {
    // Standalone users get the identity curve even in a scooter executable.
    using Generic = FocControlCore<P, 16000>;
    Generic foc;
    Generic::Params p{};
    p.id_target = number<P::Current>(-1);
    foc.setParams(p);
    foc.setCurrentPiGains(number<P::Gain>(1), {});
    Generic::Input in{};
    in.drive_mode = DriveMode::FORWARD;
    in.bus_voltage = number<P::Voltage>(40);
    in.target_current = number<P::Current>(1);
    for (int speed : {-600, 0, 67, 75, 420, 600}) {
        in.speed_steps_s = number<P::PllSpeed>(speed);
        const auto out = foc.update(in);
        near(numberCast<float>(out.v_d), -1, 0, "generic d voltage", speed);
        near(numberCast<float>(out.v_q), 1, 0, "generic q voltage", speed);
    }
}
} // namespace

int main() {
    if constexpr (std::is_same_v<P, FixedProfile>) rawCurve();
    activeFoc();
    genericFoc();
    std::puts("PASS: selected d/q error curves, signed FOC voltage and integral handoff, generic unity");
}
