/** @file main.cpp
 * @brief Complete raw ADC sweeps across float and integer conversion profiles
 */
#include "adc_conversion_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

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

template<class T>
void pwmReferenceTests() {
    using Storage = typename T::StorageType;
    constexpr int64_t low = std::numeric_limits<Storage>::min();
    constexpr int64_t high = std::numeric_limits<Storage>::max();
    constexpr int64_t one = int64_t(1) << T::FRACTION_BITS;
    const uint32_t periods[] = {0, 1, 2, 3, 2250, 4249, 4500, 65535, 65536, INT32_MAX, UINT32_MAX};
    auto verify = [&](int64_t raw) {
        const T duty = T::fromRaw(static_cast<Storage>(raw));
        // Wider integer rational oracle, independent of the fixed operators
        // and production multiplication/clamping. It also covers Q0/Q31,
        // where 1.0 is respectively integral or not representable.
        const __int128 numerator = std::max(int64_t(0), std::min(raw, std::min(one, high)));
        for (uint32_t period : periods) {
            const auto expected = static_cast<uint32_t>(numerator * period / one);
            check(dutyToCompare(duty, period) == expected, "PWM matches the exact integer rational reference");
        }
    };
    if constexpr (sizeof(Storage) <= 2) {
        for (int64_t raw = low; raw <= high; ++raw) verify(raw);
    } else {
        for (int64_t raw : {low, low + 1, int64_t(-1), int64_t(0), int64_t(1), high - 1, high}) verify(raw);
        for (int64_t raw : {one - 1, one, one + 1}) if (raw <= high) verify(raw);
        uint32_t random = 0x496fa381u;
        for (unsigned i = 0; i < 100000; ++i) {
            random = random * 1664525u + 1013904223u;
            verify(int64_t(random) - (int64_t(1) << 31));
        }
        // Exercise exact compare boundaries and the raw counts either side.
        for (uint32_t compare = 0; compare <= 4500; ++compare) {
            const int64_t boundary = int64_t((__int128(compare) * one + 4499) / 4500);
            for (int64_t raw : {boundary - 1, boundary, boundary + 1})
                if (raw >= low && raw <= high) verify(raw);
        }
    }
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
    pwmReferenceTests<FixedPointType<0, int8_t>>();
    pwmReferenceTests<FixedPointType<7, int8_t>>();
    pwmReferenceTests<FixedPointType<12, int16_t>>();
    pwmReferenceTests<FixedPointType<15, int16_t>>();
    pwmReferenceTests<FixedPointType<16>>();
    pwmReferenceTests<FixedProfile::Duty>();
    pwmReferenceTests<FixedPointType<31>>();
    pwmReferenceTests<FixedPointType<30, int64_t>>();
    std::printf("ADC profile: current %.8f A, bus %.8f V, temperature %.6f C; %d failures\n",
                current_error, voltage_error, temperature_error, failures);
    return failures != 0;
}
