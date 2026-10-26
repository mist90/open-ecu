/** @file main.cpp
 * @brief Complete raw ADC sweeps across float and integer conversion profiles
 */
#include "adc_conversion_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace libecu;
static int failures = 0;
static void check(bool ok, const char* message) {
    if (!ok) { if (failures < 10) std::printf("FAIL: %s\n", message); ++failures; }
}

template<class P>
AdcCalibrationT<P> calibration(uint16_t samples) {
    AdcCalibrationT<P> c;
    c.reference = number<typename P::Voltage>(33, 10);
    c.shunt = number<typename P::ShuntResistance>(3, 1000);
    c.injected_sum = samples;
    return c;
}

int main() {
    AdcConversionCore<FloatProfile> floating;
    AdcConversionCore<FixedProfile> fixed, single;
    check(floating.configure(calibration<FloatProfile>(2)), "float calibration accepted");
    check(fixed.configure(calibration<FixedProfile>(2)), "fixed calibration accepted");
    check(single.configure(calibration<FixedProfile>(1)), "single-sample calibration accepted");
    floating.setZero(0, 500025, 100);
    fixed.setZero(0, 500025, 100);
    float current_error = 0.0f, voltage_error = 0.0f, temperature_error = 0.0f;
    for (uint32_t raw = 0; raw <= 8190; ++raw) {
        current_error = std::max(current_error, std::abs(floating.current(raw, 0) - fixed.current(raw, 0).toFloat()));
        voltage_error = std::max(voltage_error, std::abs(floating.busVoltage(raw) - fixed.busVoltage(raw).toFloat()));
    }
    for (uint32_t raw = 1; raw < 4095; ++raw) {
        temperature_error = std::max(temperature_error,
            std::abs(floating.temperature(raw) - fixed.temperature(raw).toFloat()));
        check(single.temperature(raw) == fixed.temperature(raw), "regular NTC scale is independent of injected sums");
    }
    check(current_error < 0.0001f, "full-range current conversion error is below 0.1 mA");
    check(voltage_error < 0.001f, "full-range voltage conversion error is below 1 mV");
    check(temperature_error < 0.02f, "integer NTC calculation tracks beta equation within 0.02 C");
    check(fixed.temperature(0) == number<FixedProfile::Temperature>(-273), "open NTC sentinel is preserved");
    check(fixed.throttle(0) == FixedProfile::Duty{} && fixed.throttle(4095) == number<FixedProfile::Duty>(1),
          "throttle endpoints are exact");
    auto ihm_float = calibration<FloatProfile>(1);
    auto ihm_fixed = calibration<FixedProfile>(1);
    ihm_float.shunt = 0.010f;
    ihm_fixed.shunt = number<FixedProfile::ShuntResistance>(1, 100);
    ihm_float.opamp_gain = 57.0f / 11.0f;
    ihm_fixed.opamp_gain = number<FixedProfile::Gain>(57, 11);
    AdcConversionCore<FloatProfile> shield_float;
    AdcConversionCore<FixedProfile> shield_fixed;
    check(shield_float.configure(ihm_float) && shield_fixed.configure(ihm_fixed),
          "both profiles support the external fractional-gain amplifier");
    shield_float.setZero(0, 2122, 1);
    shield_fixed.setZero(0, 2122, 1);
    for (unsigned raw = 0; raw <= 4095; ++raw) {
        const float expected = (float(raw) - 2122.0f) * (3.3f / 4095.0f) / (0.010f * 57.0f / 11.0f);
        check(std::abs(shield_float.current(raw, 0) - expected) < 0.0001f,
              "fractional float gain matches the physical circuit");
        check(std::abs(shield_fixed.current(raw, 0).toFloat() - expected) < 0.001f,
              "fractional fixed gain matches the physical circuit within 1 mA");
    }
    auto invalid = calibration<FixedProfile>(2);
    invalid.shunt = number<FixedProfile::ShuntResistance>(1, 100000000);
    check(!fixed.configure(invalid), "unrepresentable sensor gain is rejected");
    check(std::abs(fixed.busVoltage(5000).toFloat() - floating.busVoltage(5000)) < 0.001f,
          "failed configuration preserves the prior calibration");
    for (unsigned i = 0; i <= 10000; ++i) {
        const float duty = float(i) / 10000;
        const int difference = int(dutyToCompare(duty, 4249)) -
                               int(dutyToCompare(FixedProfile::Duty::fromFloat(duty), 4249));
        check(std::abs(difference) <= 1, "fixed PWM conversion stays within one timer count of float");
    }
    std::printf("ADC profile: current %.8f A, bus %.8f V, temperature %.6f C; %d failures\n",
                current_error, voltage_error, temperature_error, failures);
    return failures != 0;
}
