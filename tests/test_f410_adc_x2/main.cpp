/** @file main.cpp
 * @brief Exercise the actual F410 ADC HAL against anonymous fake MMIO pages
 */
#include "stm32_adc.hpp"
#include "board_config.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <sys/mman.h>

using namespace libecu;
static int checks = 0, failures = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

uint32_t time_us() { static uint32_t now = 0; return now += 100; }

static void raw(uint32_t u = 2122, uint32_t v = 2120, uint32_t w = 2130, uint32_t bus = 2028) {
    ADC1->JDR1 = u; ADC1->JDR2 = v; ADC1->JDR3 = w; ADC1->JDR4 = bus;
    CLEAR_BIT(ADC1->CR2, ADC_CR2_JSWSTART); // Hardware clears the software-start request.
}

static void timer(uint32_t compare = 500, uint32_t count = 2100, uint32_t period = 2500) {
    TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_DIR | TIM_COUNTERMODE_CENTERALIGNED1;
    TIM1->PSC = 0;
    TIM1->ARR = period;
    TIM1->CCR1 = compare; TIM1->CCR2 = compare; TIM1->CCR3 = compare;
    TIM1->CNT = count;
}

static void checkAcquisition() {
    constexpr uint32_t sum = Stm32Adc::INJECTED_SAMPLE_SUM;
    static_assert(sum == (STM32F4_ADC_SOFTWARE_X2 ? 2 : 1));
    const uint32_t regular_bits = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_EXTEN_0;
    ADC1->CR2 = regular_bits | ADC_EXTERNALTRIGINJECCONV_T1_TRGO |
                ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
    Stm32Adc adc;
    CurrentSensorCalibration calibration{ihm08::SHUNT_OHMS, ihm08::CURRENT_GAIN, ihm08::ADC_REFERENCE, 12};
    calibration.injected_sample_sum = sum;
    check(adc.initialize(calibration, {float(ihm08::BUS_R_UP), float(ihm08::BUS_R_DOWN)}), "initialize SI scaling");
    timer(); raw();
    check(adc.captureInjected(10), "first frame falls back until PWM history is known");
    check(adc.samples() == 1 && adc.frameStartedCycles() == 10, "one frame with matching timestamp");
    check(adc.getRawAdcValue(PwmChannel::PHASE_U) == 2122*sum, "fallback uses the advertised sum scale");
    check(adc.offsetsPlausible() && adc.calibrateZeroOffset(), "zero calibration works in either build mode");
    check(std::fabs(adc.readPhaseCurrent(PwmChannel::PHASE_U)) < .001f, "zero remains zero after scaling");

#if STM32F4_ADC_SOFTWARE_X2
    raw(2122,2120,2130,2028);
    check(!adc.captureInjected(100), "first JEOC starts a second scan without publishing");
    check(adc.samples() == 1 && adc.getRawAdcValue(PwmChannel::PHASE_U) == 4244, "partial frame stays private");
    check((ADC1->CR2 & ADC_CR2_JEXTEN) == 0 && (ADC1->CR2 & ADC_CR2_JSWSTART), "software scan is requested");
    check((ADC1->CR2 & regular_bits) == regular_bits, "regular ADC/DMA configuration is preserved");
    raw(2123,2121,2131,2029);
    check(adc.captureInjected(550), "second JEOC publishes one coherent sum");
    check(adc.samples() == 2 && adc.frameStartedCycles() == 100, "two IRQs are one control frame");
    check(adc.getRawAdcValue(PwmChannel::PHASE_U) == 4245 &&
          adc.getRawAdcValue(PwmChannel::PHASE_V) == 4241 &&
          adc.getRawAdcValue(PwmChannel::PHASE_W) == 4261 && adc.getRawAdcValue() == 4057,
          "all four sums retain odd half-LSB results");
    check((ADC1->CR2 & ADC_CR2_JEXTEN) == ADC_EXTERNALTRIGINJECCONVEDGE_FALLING,
          "PWM trigger is restored after the second scan");
    const float expected_current = 0.5f*3.3f/(4095.0f*ihm08::SHUNT_OHMS*ihm08::CURRENT_GAIN);
    check(std::fabs(adc.readPhaseCurrent(PwmChannel::PHASE_U)-expected_current) < .001f,
          "half-count current is converted with the correct physical gain");
    const float expected_bus = 2028.5f*3.3f/4095.0f*(ihm08::BUS_R_UP+ihm08::BUS_R_DOWN)/ihm08::BUS_R_DOWN;
    check(std::fabs(adc.readBusVoltage()-expected_bus) < .002f, "Vbus uses the same sum scale");

    raw(4000);
    check(!adc.captureInjected(1000), "overcurrent example starts a paired frame");
    const int32_t limit = int32_t(25.0f*ihm08::SHUNT_OHMS*ihm08::CURRENT_GAIN*4095.0f/3.3f*sum);
    const int32_t zero = 2122*sum;
    check(int32_t(adc.getRawProtectionCurrent(PwmChannel::PHASE_U))-zero > limit,
          "first-scan peak remains visible to protection before frame completion");
    raw(2122);
    check(adc.captureInjected(1450), "complete overcurrent example");
    check(int32_t(adc.getRawAdcValue(PwmChannel::PHASE_U))-zero < limit,
          "example average is below the threshold, so checking only the average would miss the peak");
    raw(); check(!adc.captureInjected(1500), "start second-scan negative peak example");
    raw(200); check(adc.captureInjected(1950), "complete second-scan negative peak example");
    check(int32_t(adc.getRawProtectionCurrent(PwmChannel::PHASE_U))-zero < -limit,
          "second-scan negative peak is also visible without averaging");
    check(int32_t(adc.getRawAdcValue(PwmChannel::PHASE_U))-zero > -limit,
          "averaging would also miss the second-scan negative peak");

    timer(1700); raw();
    check(adc.captureInjected(2000), "high duty skips the second scan");
    check(adc.getRawAdcValue() == 2028*sum, "high-duty fallback preserves bus scale");
    timer(500); raw();
    check(adc.captureInjected(3000), "high-to-low transition also requires the previous window to be valid");
    check(!adc.captureInjected(4000), "next low-duty frame can use two scans");
    raw(); check(adc.captureInjected(4450), "finish low-duty frame");
    timer(500,950); raw();
    check(adc.captureInjected(5000), "4.5-us boundary has no spare time and falls back");
    timer(500,951); raw();
    check(!adc.captureInjected(6000), "strictly larger remaining window allows the second scan");
    raw(); check(adc.captureInjected(6450), "finish boundary frame");
    timer(); TIM1->CR1 &= ~TIM_CR1_DIR; raw();
    check(adc.captureInjected(7000), "up-counting is not a second-scan window");
    timer(); TIM1->CR1 &= ~TIM_CR1_CEN; raw();
    check(adc.captureInjected(8000), "stopped PWM timebase falls back");
    timer(); TIM1->PSC = 1; raw();
    check(adc.captureInjected(9000), "unqualified counter prescaler falls back");
    timer(500,2100,2400); raw();
    check(adc.captureInjected(10000), "period changes invalidate window history");
    check(!adc.captureInjected(0xfffffff0U), "timestamp wrap frame starts");
    raw(); check(adc.captureInjected(20), "timestamp wrap frame completes");
    check(adc.frameStartedCycles() == 0xfffffff0U && uint32_t(20-adc.frameStartedCycles()) == 36,
          "whole-frame timestamp survives counter wrap");
    raw(); check(!adc.captureInjected(0), "zero is a valid first timestamp");
    raw(); check(adc.captureInjected(450) && adc.frameStartedCycles() == 0, "zero timestamp is retained");
#else
    for (uint32_t n=0; n<6; ++n) {
        timer(n%2 ? 1700 : 500); raw(2123,2121,2131,2029);
        check(adc.captureInjected(100+n), "disabled x2 publishes on every JEOC");
        check(adc.frameStartedCycles() == 100+n, "disabled x2 keeps the current IRQ timestamp");
        check(adc.getRawAdcValue(PwmChannel::PHASE_U) == 2123 && adc.getRawAdcValue() == 2029,
              "disabled x2 preserves 12-bit samples");
        check(!(ADC1->CR2 & ADC_CR2_JSWSTART), "disabled x2 never requests a second scan");
    }
    check(adc.samples() == 7, "disabled x2 frame counter advances once per conversion sequence");
#endif
    check(adc.getRawTemperatureAdcValue() == 0 && adc.readRawPotentiometer() == 0,
          "injected publication does not overwrite regular-channel storage");
    check(adc.getRawProtectionCurrent(static_cast<PwmChannel>(99)) == 0, "invalid protection channel is bounded");
    check((ADC1->CR2 & regular_bits) == regular_bits, "regular ADC flags survive all paths");
}

int main() {
    // Only anonymous process memory is mapped: no debugger or physical device access.
    constexpr std::size_t bytes = 0x4000;
    void* area = mmap(reinterpret_cast<void*>(TIM1_BASE), bytes, PROT_READ|PROT_WRITE,
                      MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE, -1, 0);
    if (area == MAP_FAILED) { std::perror("fake peripheral mapping"); return EXIT_FAILURE; }
    new (reinterpret_cast<void*>(TIM1_BASE)) TIM_TypeDef{};
    new (reinterpret_cast<void*>(ADC1_BASE)) ADC_TypeDef{};
    checkAcquisition();
    munmap(area,bytes);
    std::printf("F410 ADC %s, x2=%d: %d checks, %d failures\n",
                CONTROL_PROFILE_NAME, STM32F4_ADC_SOFTWARE_X2, checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
