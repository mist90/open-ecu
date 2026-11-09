/** @file main.cpp
 * @brief Bit-exact bus reciprocals across ADC inputs, normalization seams and signed boundaries.
 */
#include "adc_conversion_core.hpp"
#include <algorithm>
#include <climits>
#include <cstdio>
#include <cstdlib>

using namespace libecu;

static uint64_t checks = 0;

static void check(int32_t bus) {
    int64_t expected = INT32_MAX;
    if (bus) {
        const uint64_t magnitude = bus < 0 ? uint64_t(-int64_t(bus)) : uint64_t(bus);
        // Independent oracle: original rounded integer division, including
        // sign, zero and saturation, without reciprocal normalization.
        const uint64_t quotient = ((uint64_t(1) << 48) + magnitude / 2) / magnitude;
        expected = bus < 0 ? -int64_t(quotient) : int64_t(quotient);
        expected = std::clamp(expected, int64_t(INT32_MIN), int64_t(INT32_MAX));
    }
    const int32_t actual = FixedMath::inverseBusVoltage(FixedProfile::Voltage::fromRaw(bus)).raw();
    if (actual != expected) {
        std::printf("FAIL: bus=%d reciprocal=%d expected=%lld\n", bus, actual,
                    static_cast<long long>(expected));
        std::exit(1);
    }
    ++checks;
}

int main() {
    using P = FixedProfile;
    AdcCalibrationT<P> calibration;
    calibration.shunt = number<P::ShuntResistance>(36, 10000);
    calibration.opamp_gain = number<P::Gain>(49900, 19800);
    calibration.reference = number<P::Voltage>(5);
    calibration.divider_up_ohms = 31809;
    calibration.divider_down_ohms = 1770;
    AdcConversionCore<P> converter;
    // A 12-bit,5-V divider fixture, with both x1 and x2 phase calibration:
    // the single auxiliary sample is scaled into the converter's sum domain.
    for (unsigned sum : {1u, 2u}) {
        calibration.injected_sum = sum;
        if (!converter.configure(calibration)) return 1;
        for (unsigned code = 0; code < 4096; ++code) check(converter.busVoltage(code * sum).raw());
    }
    for (int64_t bus = -524288; bus <= 524288; ++bus) check(int32_t(bus));
    for (unsigned bit = 0; bit <= 31; ++bit) {
        for (int offset = -128; offset <= 128; ++offset) {
            const int64_t value = (int64_t(1) << bit) + offset;
            if (value >= INT32_MIN && value <= INT32_MAX) check(int32_t(value));
            if (-value >= INT32_MIN && -value <= INT32_MAX) check(int32_t(-value));
        }
    }
    // Both sides of EVERY 16-bit normalization bin, not just powers of two.
    for (unsigned shift = 3; shift <= 15; ++shift) {
        for (uint32_t high = 32768; high <= 65535; ++high) {
            const int64_t value = int64_t(high) << shift;
            for (int offset : {-1, 0, 1}) {
                if (value + offset <= INT32_MAX) check(int32_t(value + offset));
            }
        }
    }
    uint64_t random = 0xa1e6bc78329d456fULL;
    for (unsigned i = 0; i < 2000000; ++i) {
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        check(int32_t(random));
    }
    std::printf("Bus reciprocal: %llu ADC/boundary/random inputs match rounded division\n",
                static_cast<unsigned long long>(checks));
}
