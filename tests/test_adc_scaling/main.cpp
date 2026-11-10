/**
 * @file main.cpp
 * @brief Capture-independent ADC sum scaling, calibration and auxiliary isolation
 */
#include "adc_interface.hpp"
#include <cmath>
#include <cstdio>
#include <initializer_list>

uint32_t time_us() { static uint32_t clock = 0; return ++clock; }

class Adc final : public libecu::AdcInterface {
public:
    uint32_t currents[3]{2000, 2050, 2100};
    uint32_t bus = 2500;
    uint32_t getRawAdcValue(libecu::PwmChannel phase) override {
        return currents[static_cast<unsigned>(phase)];
    }
    uint32_t getRawAdcValue() override { return bus; }
    uint32_t getRawTemperatureAdcValue() override { return 1500; }
};

int main() {
    Adc plain, summed;
    for (unsigned phase = 0; phase < 3; ++phase) summed.currents[phase] *= 2;
    summed.bus *= 2;
    libecu::CurrentSensorCalibration calibration{0.003f, 16.0f, 3.3f, 12, 0, 0, 0};
    libecu::VoltageSensorParameters divider{169000.0f, 18000.0f};
    int failures = 0;
    auto check = [&](bool ok, const char* text) {
        if (!ok) { ++failures; std::printf("FAIL: %s\n", text); }
    };
    check(plain.initialize(calibration, divider), "normal scale initializes");
    calibration.injected_sample_sum = 2;
    check(summed.initialize(calibration, divider), "summed scale initializes");
    check(plain.calibrateZeroOffset() && summed.calibrateZeroOffset(), "zero calibration works at both scales");
    for (auto phase : {libecu::PwmChannel::PHASE_U, libecu::PwmChannel::PHASE_V, libecu::PwmChannel::PHASE_W}) {
        check(std::fabs(plain.readPhaseCurrent(phase)) < 0.0001f, "normal calibrated zero");
        check(std::fabs(summed.readPhaseCurrent(phase)) < 0.0001f, "summed calibrated zero");
        for (int offset : {-50, 0, 50}) {
            const uint32_t raw = plain.getRawAdcValue(phase) + offset;
            check(std::fabs(plain.convertAdcToCurrent(raw, phase) -
                            summed.convertAdcToCurrent(raw * 2, phase)) < 0.00001f,
                  "signed physical current is invariant under ADC sum scaling");
        }
    }
    check(std::fabs(plain.readBusVoltage() - summed.readBusVoltage()) < 0.00001f,
          "bus voltage has the same physical scale");
    check(plain.readTemperature() == summed.readTemperature(), "NTC conversion retains its own scale");
    check(plain.convertAdcToThrottle(1500) == summed.convertAdcToThrottle(1500),
          "throttle conversion retains its own scale");
    const auto phase = libecu::PwmChannel::PHASE_U;
    const uint32_t raw = summed.getRawAdcValue(phase);
    const float zero = summed.convertAdcToCurrent(raw, phase);
    const float half = summed.convertAdcToCurrent(raw + 1, phase) - zero;
    const float full = summed.convertAdcToCurrent(raw + 2, phase) - zero;
    check(half > 0.008f && half < 0.009f, "odd sums retain the 8.4 mA half-LSB");
    check(std::fabs(2.0f * half - full) < 0.00001f, "half-LSB remains linear");

    // Published scale-2 fixtures: a normalized single scan, an unequal pair
    // (also the representation of hardware x2/shift0), then a single scan again.
    // These are port outputs; the common API neither knows nor changes scan count.
    const uint32_t frames[][3] = {
        {2050u * 2, 2000u * 2, 2100u * 2},
        {2050u + 2051u, 2000u + 1999u, 2100u + 2101u},
        {1950u * 2, 2100u * 2, 2100u * 2},
    };
    const float amperes_per_count = 3.3f / 4095.0f / (0.003f * 16.0f);
    const auto bus_before = summed.readNativeBusVoltage();
    for (const auto& frame : frames) {
        for (unsigned i = 0; i < 3; ++i) summed.currents[i] = frame[i];
        libecu::AdcInterface::NativeCurrent native[3]{};
        summed.readNativeCurrents(native[0], native[1], native[2]);
        for (unsigned i = 0; i < 3; ++i) {
            const float expected_current = (float(frame[i]) / 2.0f - float(plain.currents[i])) * amperes_per_count;
            check(std::fabs(libecu::numberCast<float>(native[i]) - expected_current) < 0.0001f,
                  "alternating single/pair frames use the same calibrated current scale");
        }
        check(summed.getCalibration().injected_sample_sum == 2,
              "physical acquisition count does not change calibration");
        check(summed.readNativeBusVoltage() == bus_before,
              "current-frame changes preserve the independently cached bus value");
    }
    calibration.injected_sample_sum = 0;
    check(!summed.initialize(calibration, divider), "zero sample sum is rejected");
    calibration = {0.010f, 57.0f / 11.0f, 3.3f, 12};
    check(plain.initialize(calibration, {169000.0f, 9310.0f}), "fractional IHM08M1 gain initializes");
    check(plain.calibrateZeroOffset(), "IHM08M1 offset calibration");
    // Independent circuit calculation: 1 + 4.7k/1k, attenuated by 6.8k/(6.8k+680).
    const float expected = 100.0f * 3.3f / 4095.0f / (0.010f * 5.7f * 6800.0f / 7480.0f);
    check(std::fabs(plain.convertAdcToCurrent(2100, phase) - expected) < 0.0001f,
          "IHM08M1 current uses the full fractional amplifier gain");
    calibration.opamp_gain = 0.0f;
    check(!plain.initialize(calibration, divider), "zero gain is rejected");
    calibration.opamp_gain = -1.0f;
    check(!plain.initialize(calibration, divider), "negative gain is rejected");
    std::printf("ADC scale: %d failures\n", failures);
    return failures != 0;
}
