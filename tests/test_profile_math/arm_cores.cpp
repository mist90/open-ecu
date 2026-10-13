/** @file arm_cores.cpp
 * @brief Compile-only non-FPU target check of the complete numerical kernels
 */
#include "motor_pll_core.hpp"
#include "hall_monitor_core.hpp"
#include "foc_control_core.hpp"
#include "adc_conversion_core.hpp"

using namespace libecu;
using P = FixedProfile;
static MotorPLLCore<P, 20000> pll(number<P::PllSpeed>(4800), true);
static HallMonitorCore<P, 20000> hall;
static FocControlCore<P, 20000> foc;
static AdcConversionCore<P> adc;

extern "C" void configure_cores() {
    pll.setUsePLL(true);
    foc.setCurrentPiGains(number<P::Gain>(22, 10), number<P::Gain>(2067));
    AdcCalibrationT<P> calibration;
    calibration.reference = number<P::Voltage>(33, 10);
    calibration.shunt = number<P::ShuntResistance>(3, 1000);
    calibration.injected_sum = 2;
    adc.configure(calibration);
    adc.setZero(0, 500000, 100);
}

extern "C" uint8_t tick_estimator(uint8_t position) {
    pll.updateHall(position);
    pll.updateTick();
    hall.tick(true);
    hall.onPosition(position, true);
    return pll.getNextHall(DriveMode::FORWARD);
}

extern "C" void tick_foc(const MotorAlgorithmInputT<P>* in, MotorAlgorithmOutputT<P>* out) {
    *out = foc.update(*in);
}

extern "C" int32_t modulation(int32_t d, int32_t q, int32_t bus) {
    return FocControlCore<P, 20000>::modulationMagnitude(P::Voltage::fromRaw(d),
                P::Voltage::fromRaw(q), P::Voltage::fromRaw(bus)).raw();
}

extern "C" int32_t read_current(uint32_t raw) { return adc.current(raw, 0).raw(); }
extern "C" int32_t read_temperature(uint32_t raw) { return adc.temperature(raw).raw(); }
extern "C" uint32_t pwm_compare(int32_t duty) { return dutyToCompare(P::Duty::fromRaw(duty), 4249); }
