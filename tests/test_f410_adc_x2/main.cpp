/** @file main.cpp
 * @brief F410 real PWM setup and x1/x2 aperture qualification against anonymous MMIO
 */
#include "stm32_adc.hpp"
#include "stm32_pwm.hpp"
#include "pwm_sampling_state.hpp"
#include "board_config.hpp"
#include <cstdio>
#include <cstdlib>
#include <new>
#include <sys/mman.h>

using namespace libecu;
static int checks = 0, failures = 0;
static void check(bool ok, const char* why) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", why); }
}
extern "C" {
void HAL_GPIO_Init(GPIO_TypeDef*, GPIO_InitTypeDef*) {}
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, GPIO_PinState) {}
void HAL_NVIC_SetPriority(IRQn_Type, uint32_t, uint32_t) {}
void HAL_NVIC_EnableIRQ(IRQn_Type) {}
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef* timer) {
    timer->Instance->CR1 = timer->Init.CounterMode;
    timer->Instance->ARR = timer->Init.Period;
    timer->Instance->PSC = timer->Init.Prescaler;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef* timer, const TIM_OC_InitTypeDef* oc, uint32_t channel) {
    (&timer->Instance->CCR1)[channel / 4] = oc->Pulse;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef* timer, const TIM_MasterConfigTypeDef* master) {
    timer->Instance->CR2 = master->MasterOutputTrigger;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef*, const TIM_BreakDeadTimeConfigTypeDef* config) {
    check(config->BreakState == TIM_BREAK_ENABLE && config->BreakPolarity == TIM_BREAKPOLARITY_LOW &&
          config->AutomaticOutput == TIM_AUTOMATICOUTPUT_DISABLE, "asynchronous hardware break remains enabled");
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef*, uint32_t, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
}

static void compares(uint32_t highest) {
    TIM1->CCR1 = highest; TIM1->CCR2 = 300; TIM1->CCR3 = 100;
}
static void timerSetup() {
    Stm32Pwm pwm;
    check(pwm.initialize(20000, ihm08::DEAD_TIME_NS), "production PWM initializes at 20 kHz");
    check((TIM1->CR1 & TIM_CR1_CMS) == TIM_COUNTERMODE_CENTERALIGNED2 && !(TIM1->CR2 & TIM_CR2_CCDS),
          "CC4 DMA is up-count compare only, not both slopes or update");
    check(TIM1->CCR4 == TIM1->ARR - ihm08::ADC_LEAD_TICKS, "source remains 1 us before the PWM peak");
    // RM0401 center-aligned CCxIF direction qualification, applied to the
    // configuration produced by the real HAL. Model 20 complete carrier cycles.
    unsigned requests = 0;
    const uint32_t cms = TIM1->CR1 & TIM_CR1_CMS;
    for (unsigned frame = 0; frame < 20; ++frame) {
        for (uint32_t t = 0; t < 2 * TIM1->ARR; ++t) {
            const bool down = t >= TIM1->ARR;
            const uint32_t count = down ? 2 * TIM1->ARR - t : t;
            if (count == TIM1->CCR4 &&
                (cms == TIM_COUNTERMODE_CENTERALIGNED3 ||
                 (down ? cms == TIM_COUNTERMODE_CENTERALIGNED1 : cms == TIM_COUNTERMODE_CENTERALIGNED2)))
                ++requests;
        }
    }
    check(requests == 20, "one CC4 hardware request per full PWM carrier");
}

static void aperture() {
    // These direct-register fixtures do not emulate UIF/preload transfer. Test
    // their legacy conservative path independently of the production PWM writer.
    pwm_sampling_state = {};
    constexpr unsigned sum = Stm32Adc::INJECTED_SAMPLE_SUM;
    static_assert(sum == (STM32F4_ADC_SOFTWARE_X2 ? 2 : 1));
    // Last-U aperture: subtract its SAR conversion and the two later ranks
    // from the complete current scan. Auxiliary apertures are separate.
    for (uint32_t compare = 700; compare < 2450; compare += 25) {
        compares(compare);
        Stm32Adc adc;
        check(adc.beginFrames(0), "source can arm without depending on a nonzero timestamp");
        const uint32_t closing = sum == 1 ? 60 : 384;
        const bool expected = 2400 > compare + 300 && 5000 > 2400 + closing + compare;
        check(adc.currentWindowValid() == expected, "guard covers settle and final current aperture at each duty");
        check(adc.currentWindowValid(0),
               "x1/x2 can exclude the explicitly reconstructed unique-highest U leg");
        if (!expected) {
            // Even if the controller requests a low duty next, the old active
            // compare represented by the armed frame must still be rejected.
            compares(700);
            check(!adc.currentWindowValid(), "all-leg guard retains old per-phase PWM window history");
        }
    }
    TIM1->CCR1 = 100; TIM1->CCR2 = 300; TIM1->CCR3 = 1999;
    Stm32Adc adc;
    check(adc.beginFrames(0xfffffff0U), "arm timestamp can straddle wrap");
    check(adc.currentWindowValid(), "last count strictly inside x2 aperture accepted");
    TIM1->CCR3 = 2000;
    check(adc.currentWindowValid() == (sum == 1), "exact closing boundary has no slack and fails x2");
    check(adc.currentWindowValid(2), "the exact reconstructed W phase can exclude that single invalid leg");
    check(adc.currentWindowValid(0) == (sum == 1), "a different reconstruction choice cannot exclude W");
    for (unsigned highest = 0; highest < 3; ++highest) {
        uint32_t duties[3] = {0, 0, 0};
        duties[highest] = 2375; // 95% is intentionally reconstructed by FOC
        duties[(highest + 1) % 3] = 1000; // both consumed legs and both x2 samples fit
        TIM1->CCR1 = duties[0]; TIM1->CCR2 = duties[1]; TIM1->CCR3 = duties[2];
        Stm32Adc high_duty;
        check(high_duty.beginFrames(0), "arm a high-duty reconstruction case");
        check(!high_duty.currentWindowValid(), "unqualified/default consumer cannot use a bad shunt sample");
        check(high_duty.currentWindowValid(int(highest)),
               "x1/x2 keep valid 95%-highest reconstruction in every phase/rank");
        check(!high_duty.currentWindowValid(int((highest + 1) % 3)),
               "a different post-reset reconstruction choice cannot exempt the bad leg");
        (&TIM1->CCR1)[(highest + 1) % 3] = 2350;
        check(!high_duty.currentWindowValid(int(highest)),
               "reconstructing the highest leg never excuses a second invalid consumed leg");
    }
    TIM1->CCR1 = TIM1->CCR2 = 2375; TIM1->CCR3 = 0;
    Stm32Adc tied;
    check(tied.beginFrames(0) && !tied.currentWindowValid(0) && !tied.currentWindowValid(1),
           "two narrow high-duty windows cannot be rescued by a tied highest-leg choice");
    check(tied.currentInvalidPhases() == 3,
           "both invalid legs remain explicit: single-phase reconstruction cannot repair two");
    TIM1->CCR1 = TIM1->CCR2 = 1000;
    Stm32Adc wide_tied;
    check(wide_tied.beginFrames(0) && wide_tied.currentWindowValid(),
           "ties are harmless when all three sampled apertures are qualified");
    TIM1->RCR = 1;
    check(!wide_tied.currentWindowValid(), "a skipped peak reload cannot qualify a duty generation");
    Stm32Adc skipped_reload;
    check(!skipped_reload.beginFrames(0), "nonzero repetition counter cannot arm the peak-generation proof");
    TIM1->RCR = 0;
    SET_BIT(TIM1->CR1, TIM_CR1_UDIS);
    check(!wide_tied.currentWindowValid(), "in-progress duty write cannot qualify the sampling window");
    Stm32Adc masked_reload;
    check(!masked_reload.beginFrames(0), "suppressed updates cannot arm the next duty generation");
    CLEAR_BIT(TIM1->CR1, TIM_CR1_UDIS);
    TIM1->PSC = 1;
    Stm32Adc bad_clock;
    check(!bad_clock.beginFrames(0), "unqualified prescaler fails closed rather than changing aperture units");
    TIM1->PSC = 0;
    MODIFY_REG(TIM1->CR1, TIM_CR1_CMS, TIM_COUNTERMODE_CENTERALIGNED3);
    Stm32Adc double_trigger;
    check(!double_trigger.beginFrames(0), "both-slope compare configuration cannot arm acquisition");
    check(!(TIM1->DIER & TIM_DIER_CC4DE), "failed qualification stops the trigger request source");
}

int main() {
    void* mmio = mmap(reinterpret_cast<void*>(0x40010000), 0x20000, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (mmio == MAP_FAILED) { std::perror("anonymous MMIO"); return EXIT_FAILURE; }
    timerSetup(); aperture();
    std::printf("F410 PWM/regular aperture %s x%u: %d checks, %d failures\n",
                CONTROL_PROFILE_NAME, Stm32Adc::INJECTED_SAMPLE_SUM, checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
