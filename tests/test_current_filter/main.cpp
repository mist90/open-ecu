/** @file main.cpp
 * @brief Filter frequency response and delayed closed-loop FOC, float and fixed
 */
#include "foc_control_core.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

using namespace libecu;
static int failures = 0, checks = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

template<class P>
static void frequencyResponse() {
    using Current = typename P::Current;
    CurrentFilterCoefficients<P> c;
    check(!makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER, 0, 20000, c), "reject zero cutoff");
    check(!makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER, 10000, 20000, c), "reject Nyquist cutoff");
    check(!makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER, 2000, 0, c), "reject zero sample rate");
    check(!makeCurrentFilterCoefficients(CurrentFilterOrder::FIRST_ORDER,
                                        std::numeric_limits<float>::quiet_NaN(), 20000, c), "reject NaN cutoff");
    check(!makeCurrentFilterCoefficients(static_cast<CurrentFilterOrder>(255), 2000, 20000, c), "reject unknown order");
    check(makeCurrentFilterCoefficients(CurrentFilterOrder::OFF, 0, 20000, c), "explicit bypass");
    CurrentFeedbackFilter<P> filter;
    for (float value : {-20.0f, 0.0f, 0.25f, 18.0f}) {
        const Current input = numberCast<Current>(value);
        check(filter.update(input, c) == input, "bypass preserves raw current exactly");
    }

    for (auto order : {CurrentFilterOrder::FIRST_ORDER, CurrentFilterOrder::BUTTERWORTH_SECOND_ORDER}) {
        for (uint32_t fs : {10000U, 20000U, 40000U}) {
            for (float fc : {1500.0f, 2000.0f, 2500.0f}) {
                check(makeCurrentFilterCoefficients(order, fc, fs, c), "design valid coefficients");
                const auto good = c;
                check(!makeCurrentFilterCoefficients(order, -1, fs, c) && c.b0 == good.b0 && c.a1 == good.a1,
                      "invalid configuration preserves coefficients");
                filter.reset();
                for (int n = 0; n < 2000; ++n) {
                    const float value = numberCast<float>(filter.update(numberCast<Current>(-3.25f), c));
                    if (n == 0 || n == 1999)
                        check(std::fabs(value + 3.25f) < 0.0005f, "warm start and DC gain");
                }
                for (float f : {500.0f, fc, 0.4f*fs}) {
                    filter.reset();
                    std::complex<double> input_sum{}, output_sum{};
                    const int count = int(fs); // integer-Hz frequencies are coherent over one second
                    for (int n = 0; n < count + 1000; ++n) {
                        const double angle = 6.283185307179586 * f * n / fs;
                        const float input = float(std::sin(angle));
                        const float output = numberCast<float>(filter.update(numberCast<Current>(input), c));
                        if (n >= 1000) {
                            const auto weight = std::polar(1.0, -angle);
                            input_sum += double(input) * weight;
                            output_sum += double(output) * weight;
                        }
                    }
                    const auto measured = output_sum / input_sum;
                    const auto z = std::polar(1.0, -6.283185307179586 * f / fs);
                    const auto expected = (double(numberCast<float>(c.b0)) + double(numberCast<float>(c.b1))*z +
                                           double(numberCast<float>(c.b2))*z*z) /
                                          (1.0 + double(numberCast<float>(c.a1))*z +
                                           double(numberCast<float>(c.a2))*z*z);
                    check(std::abs(measured - expected) < 0.0003, "measured gain and phase match transfer function");
                    if (f == fc) check(std::fabs(std::abs(measured) - std::sqrt(0.5)) < 0.0003, "cutoff is -3 dB");
                }
                filter.reset();
                check(std::fabs(numberCast<float>(filter.update(numberCast<Current>(0.5f), c))-0.5f) < 1e-5f,
                      "reset removes previous signal history");
            }
        }
    }
}

template<class P>
static void delayedLoop(const char* profile, float plant_r = 0.658f, float plant_l = 0.0007f,
                        float gain_scale = 1.0f) {
    using C = FocControlCore<P, 20000>;
    for (auto order : {CurrentFilterOrder::OFF, CurrentFilterOrder::FIRST_ORDER,
                      CurrentFilterOrder::BUTTERWORTH_SECOND_ORDER}) {
        for (float fc : {1500.0f, 2000.0f, 2500.0f}) {
            C core;
            typename C::Params params;
            params.angle_offset_rad = typename P::Angle{};
            params.low_side_clamped_pwm = true;
            check(makeCurrentFilterCoefficients(order, fc, 20000, params.current_filter), "loop filter setup");
            core.setParams(params);
            core.setCurrentPiGains(numberCast<typename C::Gain>(gain_scale*0.0007f*3141.59265f),
                                   numberCast<typename C::Gain>(gain_scale*0.658f*3141.59265f));
            core.reset();
            typename C::Input in;
            in.drive_mode = DriveMode::FORWARD;
            in.electric_mode = ElectricMode::CURRENT_MODE;
            in.bus_voltage = numberCast<typename C::Voltage>(40.8f);
            float id = 0.0f, iq = 0.0f, delayed_d = 0.0f, delayed_q = 0.0f, peak = 0.0f;
            const float decay = std::exp(-plant_r * 0.00005f / plant_l);
            for (int n = 0; n < 1800; ++n) {
                const float target = n < 40 ? 0.0f : n < 240 ? 0.3f : n < 440 ? -0.3f :
                                     n < 800 ? 0.0f : n < 1200 ? 100.0f : 0.0f;
                in.target_current = numberCast<typename C::Current>(target);
                in.i_u = numberCast<typename C::Current>(-id);
                in.i_v = numberCast<typename C::Current>(0.5f*id - 0.866025404f*iq);
                in.i_w = numberCast<typename C::Current>(0.5f*id + 0.866025404f*iq);
                const auto out = core.update(in);
                check(std::isfinite(numberCast<float>(out.v_q)), "finite filtered FOC output");
                check(std::fabs(numberCast<float>(out.i_q) - iq) < 0.001f,
                      "FOC telemetry retains the unfiltered measurement");
                id = id*decay + delayed_d/plant_r*(1.0f-decay);
                iq = iq*decay + delayed_q/plant_r*(1.0f-decay);
                delayed_d = numberCast<float>(out.v_d);
                delayed_q = numberCast<float>(out.v_q);
                if (n >= 40 && n < 240) peak = std::max(peak, iq);
                if (n == 239 || n == 439 || n == 799 || n == 1799) {
                    check(std::fabs(iq-target) < 0.01f, "step/reversal/saturation recovery with one-cycle delay");
                    check(std::fabs(id) < 0.01f, "d axis remains regulated");
                }
            }
            check(peak < 0.405f, "step overshoot bounded below 35 percent");
            in.i_u = numberCast<typename C::Current>(-1.0f);
            in.i_v = in.i_w = numberCast<typename C::Current>(0.5f);
            core.update(in);
            core.setAngleOffset(numberCast<typename C::Angle>(0.5f));
            const auto rotated = core.update(in);
            const auto filtered = core.filteredCurrents();
            check(filtered.d == rotated.i_d && filtered.q == rotated.i_q,
                  "angle changes seed filter history in the new dq frame");
            std::printf("%s order=%u fc=%.0f Hz: step peak %.4f A, final %.6f A\n",
                        profile, unsigned(order), fc, peak, iq);
        }
    }
}

template<class P>
static void noiseFloor() {
    using C = FocControlCore<P, 20000>;
    using I = typename C::Current;
    for (auto order : {CurrentFilterOrder::OFF, CurrentFilterOrder::FIRST_ORDER,
                      CurrentFilterOrder::BUTTERWORTH_SECOND_ORDER}) {
        C core;
        typename C::Params p;
        p.angle_offset_rad = typename C::Angle{};
        p.low_side_clamped_pwm = true;
        p.current_noise_floor = number<I>(1, 10);
        check(makeCurrentFilterCoefficients(order, 2000, 20000, p.current_filter), "noise-floor filter setup");
        core.setParams(p);
        core.setCurrentPiGains(numberCast<typename C::Gain>(0.549775f), numberCast<typename C::Gain>(516.792f));
        core.reset();
        typename C::Input in;
        in.drive_mode = DriveMode::FORWARD;
        in.electric_mode = ElectricMode::CURRENT_MODE;
        in.bus_voltage = number<typename C::Voltage>(40);
        auto sample = [&](float d, float q, float target = 0.0f, float speed = 0.0f) {
            in.i_u = numberCast<I>(-d);
            in.i_v = numberCast<I>(0.5f*d-0.866025404f*q);
            in.i_w = numberCast<I>(0.5f*d+0.866025404f*q);
            in.target_current = numberCast<I>(target);
            in.speed_steps_s = numberCast<typename P::PllSpeed>(speed);
            return core.update(in);
        };
        typename C::Output out;
        // A prior active command leaves an integrator that must not survive idle entry.
        for (int n=0; n<200; ++n) out=sample(0,0,0.3f);
        check(numberCast<float>(out.v_q)>0.5f, "precondition: nonzero stored voltage");
        out=sample(0.04f,-0.03f);
        check(out.v_d==typename C::Voltage{} && out.v_q==typename C::Voltage{}, "idle clears stored voltage");
        check(out.duty_u==typename C::Duty{} && out.duty_v==typename C::Duty{} && out.duty_w==typename C::Duty{},
              "idle zero vector stops DPWMMIN pulses");
        check(std::fabs(numberCast<float>(out.i_q)+0.03f)<0.001f, "noise suppression retains raw current diagnostics");
        for (int n=0; n<300; ++n) {
            out=sample(0.07f*std::sin(n*0.41f),0.06f*std::cos(n*0.37f));
            const auto feedback=core.filteredCurrents();
            check(feedback.d==I{} && feedback.q==I{} && out.v_d==typename C::Voltage{} &&
                  out.v_q==typename C::Voltage{}, "sub-threshold noise cannot grow duty or integral");
        }
        for (int n=0; n<100; ++n) out=sample(0,0.125f);
        check(out.v_q==typename C::Voltage{}, "hysteresis retains idle between 0.10 and 0.15 A");
        for (int n=0; n<100; ++n) out=sample(0,0.20f);
        check(numberCast<float>(out.v_q)<-0.01f, "real current above release threshold resumes regulation");
        for (int n=0; n<100; ++n) out=sample(0,0.02f);
        check(out.v_q==typename C::Voltage{}, "return below threshold parks again");
        for (auto direction : {DriveMode::FORWARD,DriveMode::REVERSE}) {
            core.reset();
            in.drive_mode=direction;
            out=sample(0,0,0.02f);
            check(direction==DriveMode::FORWARD ? numberCast<float>(out.v_q)>0 : numberCast<float>(out.v_q)<0,
                  "small nonzero torque requests bypass idle suppression in both directions");
        }
        in.drive_mode=DriveMode::FORWARD;
        core.reset();
        out=sample(0,0.02f,0,1.0f);
        check(core.filteredCurrents().q>I{} && out.v_q<typename C::Voltage{},
              "zero torque at speed still regulates current against back-EMF");
        p.id_target=number<I>(2,100);
        core.setParams(p);core.reset();
        out=sample(0,0);
        check(out.v_d>typename C::Voltage{}, "nonzero d-axis reference bypasses idle suppression");
        p.id_target=I{};
        core.setParams(p);core.reset();
        in.electric_mode=ElectricMode::VOLTAGE_MODE;
        in.duty_command=number<typename C::Duty>(1,10);
        out=sample(0,0);
        check(out.v_q>typename C::Voltage{}, "voltage mode retains its direct command");
        in.electric_mode=ElectricMode::CURRENT_MODE;
        p.current_noise_floor=I{};
        core.setParams(p);core.reset();
        out=sample(0,0.02f);
        check(out.v_q<typename C::Voltage{}, "zero noise-floor setting disables suppression");
    }
}

int main() {
    frequencyResponse<FloatProfile>();
    frequencyResponse<FixedProfile>();
    delayedLoop<FloatProfile>("float");
    delayedLoop<FixedProfile>("fixed");
    // A faster electrical plant needs lower PI gains even at the same PWM rate.
    delayedLoop<FloatProfile>("float lower-L", 0.22f, 0.000175f, 0.25f);
    delayedLoop<FixedProfile>("fixed lower-L", 0.22f, 0.000175f, 0.25f);
    noiseFloor<FloatProfile>();
    noiseFloor<FixedProfile>();
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
