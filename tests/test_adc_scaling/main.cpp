/**
 * @file main.cpp
 * @brief Injected ADC sum scaling, calibration and regular-conversion isolation
 */
#include "adc_interface.hpp"
#include <cmath>
#include <cstdio>
#include <initializer_list>

uint32_t time_us() { static uint32_t clock = 0; return ++clock; }

class Adc final : public libecu::AdcInterface {
public:
    uint32_t scale = 1;
    uint32_t getRawAdcValue(libecu::PwmChannel phase) override {
        return (2000u + 50u * static_cast<uint32_t>(phase)) * scale;
    }
    uint32_t getRawAdcValue() override { return 2500u * scale; }
    uint32_t getRawTemperatureAdcValue() override { return 1500; }
};

int main() {
    Adc plain, summed;
    summed.scale = 2;
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
    check(plain.readTemperature() == summed.readTemperature(), "regular NTC conversion retains its own scale");
    const auto phase = libecu::PwmChannel::PHASE_U;
    const uint32_t raw = summed.getRawAdcValue(phase);
    const float zero = summed.convertAdcToCurrent(raw, phase);
    const float half = summed.convertAdcToCurrent(raw + 1, phase) - zero;
    const float full = summed.convertAdcToCurrent(raw + 2, phase) - zero;
    check(half > 0.008f && half < 0.009f, "odd sums retain the 8.4 mA half-LSB");
    check(std::fabs(2.0f * half - full) < 0.00001f, "half-LSB remains linear");
    calibration.injected_sample_sum = 0;
    check(!summed.initialize(calibration, divider), "zero sample sum is rejected");
    std::printf("ADC scale: %d failures\n", failures);
    return failures != 0;
}
