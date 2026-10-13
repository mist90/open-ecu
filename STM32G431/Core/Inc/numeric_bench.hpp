/** @file numeric_bench.hpp
 * @brief Bounded DWT benchmark, run before enabling the bridge or PWM interrupts
 */
#ifndef ECU_NUMERIC_BENCH_HPP
#define ECU_NUMERIC_BENCH_HPP

#include "ecu_config.hpp"
#include "adc_conversion_core.hpp"
#include "foc_control_core.hpp"
#include "hall_monitor_core.hpp"
#include "motor_pll_core.hpp"

struct NumericBenchResult {
    uint32_t samples;
    uint32_t total_cycles;
    uint32_t max_cycles;
    uint32_t checksum;
};

extern "C" {
// Order: PLL, Hall, ADC (three currents + bus), FOC normal, FOC saturated.
inline volatile NumericBenchResult numeric_bench_results[5] = {};
}

template<class Value>
uint32_t numericBenchBits(Value value) {
    static_assert(sizeof(Value) == 4, "Benchmark observes a scalar result");
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

template<class Function>
void measureNumericCore(unsigned index, Function tick) {
    uint32_t total = 0, maximum = 0, checksum = 0;
    constexpr unsigned samples = 1000;
    for (unsigned n = 0; n < samples; ++n) {
        unsigned iteration = n;
        const uint32_t start = DWT->CYCCNT;
        // A volatile timer read alone does not constrain pure arithmetic. Tie
        // the input and result to compiler barriers so inlined ADC calculations
        // cannot migrate out of the measured interval.
        __asm volatile ("" : "+r"(iteration) : : "memory");
        const uint32_t value = tick(iteration);
        __asm volatile ("" : : "r"(value) : "memory");
        const uint32_t cycles = DWT->CYCCNT - start;
        total += cycles;
        if (cycles > maximum) maximum = cycles;
        checksum += value;
    }
    numeric_bench_results[index].samples = samples;
    numeric_bench_results[index].total_cycles = total;
    numeric_bench_results[index].max_cycles = maximum;
    numeric_bench_results[index].checksum = checksum;
}

inline void benchmarkNumericCores() {
    using namespace libecu;
    using P = ActiveNumericProfile;
    MotorPLLCore<P, ActiveTiming::CURRENT_HZ> pll(number_v<P::PllSpeed, 6000>);
    HallMonitorCore<P, ActiveTiming::CURRENT_HZ> hall;
    AdcConversionCore<P> adc;
    FocControlCore<P, ActiveTiming::CURRENT_HZ> foc;
    AdcCalibrationT<P> calibration;
    calibration.reference = number_v<P::Voltage, 33, 10>;
    calibration.shunt = number_v<P::ShuntResistance, 3, 1000>;
    calibration.injected_sum = 2;
    adc.configure(calibration);
    for (unsigned phase = 0; phase < 3; ++phase) adc.setZero(phase, 409500, 100);
    foc.setCurrentPiGains(number_v<P::Gain, 2>, P::Gain{});
    MotorAlgorithmInputT<P> input{};
    input.bus_voltage = number_v<P::Voltage, 20>;
    input.drive_mode = DriveMode::FORWARD;
    input.electric_mode = ElectricMode::CURRENT_MODE;
    input.target_current = number_v<P::Current, 1>;
    measureNumericCore(0, [&](unsigned n) {
        pll.updateHall((n / 40) % 6);
        pll.updateTick();
        return numericBenchBits(pll.getAngle());
    });
    measureNumericCore(1, [&](unsigned n) {
        hall.tick(true);
        if (n % 20 == 0) hall.onPosition((n / 20) % 6, true);
        return uint32_t(hall.faultCause());
    });
    measureNumericCore(2, [&](unsigned n) {
        return numericBenchBits(adc.current(4000 + n % 100, 0)) ^
               numericBenchBits(adc.current(4200 + n % 100, 1)) ^
               numericBenchBits(adc.current(4100 + n % 100, 2)) ^
               numericBenchBits(adc.busVoltage(4800 + n % 100));
    });
    const auto foc_tick = [&](unsigned n) {
        input.angle_steps = number<P::Angle>(n % 6);
        const auto out = foc.update(input);
        return numericBenchBits(out.duty_u) ^ numericBenchBits(out.duty_v) ^ numericBenchBits(out.duty_w);
    };
    measureNumericCore(3, foc_tick);
    foc.setCurrentPiGains(number_v<P::Gain, 2>, number_v<P::Gain, 2000>);
    input.target_current = number_v<P::Current, 100>;
    auto params = foc.getParams();
    params.id_target = number_v<P::Current, 3>;
    foc.setParams(params);
    foc.reset();
    measureNumericCore(4, foc_tick);
}

#endif // ECU_NUMERIC_BENCH_HPP
