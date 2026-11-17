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
    static_assert(sum == 1);
    const uint32_t regular_bits = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_EXTEN_0;
    ADC1->CR2 = regular_bits | ADC_EXTERNALTRIGINJECCONV_T1_TRGO |
                ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
    Stm32Adc adc;
    CurrentSensorCalibration calibration{ihm08::SHUNT_OHMS, ihm08::CURRENT_GAIN, ihm08::ADC_REFERENCE, 12};
    calibration.injected_sample_sum = sum;
    check(adc.initialize(calibration, {float(ihm08::BUS_R_UP), float(ihm08::BUS_R_DOWN)}), "initialize SI scaling");
    timer(); raw();
    adc.captureInjected();
    check(adc.samples() == 1, "first PWM scan publishes one frame");
    check(adc.getRawAdcValue(PwmChannel::PHASE_U) == 2122, "current retains physical 12-bit units");
    check(adc.offsetsPlausible() && adc.calibrateZeroOffset(), "zero calibration uses single-scan units");
    check(std::fabs(adc.readPhaseCurrent(PwmChannel::PHASE_U)) < .001f, "zero remains zero after scaling");

    const uint32_t trigger_config = ADC1->CR2;
    for (uint32_t n=0; n<6; ++n) {
        timer(n%2 ? 1700 : 500); raw(2123,2121,2131,2029);
        adc.captureInjected();
        check(adc.samples() == n+2, "every JEOC publishes one frame at low and high duty");
        check(adc.getRawAdcValue(PwmChannel::PHASE_U) == 2123 &&
              adc.getRawAdcValue(PwmChannel::PHASE_V) == 2121 &&
              adc.getRawAdcValue(PwmChannel::PHASE_W) == 2131,
              "one scan publishes all three physical samples");
        check(adc.getRawAdcValue() == 0, "obsolete JDR4 voltage cannot overwrite the regular cache");
        check(ADC1->CR2 == trigger_config, "publication preserves the hardware trigger and starts no conversion");
    }
    const float one_count = 3.3f/(4095.0f*ihm08::SHUNT_OHMS*ihm08::CURRENT_GAIN);
    check(std::fabs(adc.readPhaseCurrent(PwmChannel::PHASE_U)-one_count) < .001f,
          "one ADC count has the correct physical current gain");
    raw(2121); adc.captureInjected();
    check(std::fabs(adc.readPhaseCurrent(PwmChannel::PHASE_U)+one_count) < .001f,
          "negative current keeps the same physical gain");
    const int32_t limit = int32_t(25.0f*ihm08::SHUNT_OHMS*ihm08::CURRENT_GAIN*4095.0f/3.3f);
    raw(4000); adc.captureInjected();
    check(int32_t(adc.getRawAdcValue(PwmChannel::PHASE_U))-2122 > limit,
          "positive current peak is visible to raw-count protection");
    raw(200); adc.captureInjected();
    check(int32_t(adc.getRawAdcValue(PwmChannel::PHASE_U))-2122 < -limit,
          "negative current peak is visible to raw-count protection");
    check(!adc.offsetsPlausible(), "out-of-range bias cannot pass zero calibration");
    check(adc.getRawTemperatureAdcValue() == 0 && adc.readRawPotentiometer() == 0,
          "injected publication does not overwrite regular-channel storage");
    check(adc.getRawAdcValue(static_cast<PwmChannel>(99)) == 0, "invalid phase channel is bounded");
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
    std::printf("F410 ADC %s: %d checks, %d failures\n", CONTROL_PROFILE_NAME, checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
