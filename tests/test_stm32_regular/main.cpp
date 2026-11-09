/** @file main.cpp
 * @brief Production pipelined ADC drivers against fake HAL and anonymous MMIO.
 * Source events, individual DMA writes and auxiliary completions are independent.
 */
#include "main.h"
#include "stm32_adc.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <sys/mman.h>

using namespace libecu;
static unsigned checks, failures, depth;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}
static ADC_InitTypeDef configs[2]{};
static ADC_ChannelConfTypeDef ranks[2][16]{};
static unsigned rank_count[2]{};
static DMA_InitTypeDef dma_configs[2]{};
static uint16_t* buffers[2]{};
static Stm32Adc* adc;
static uint32_t now, epoch, source;
static bool fail_dma_start;
static void (*on_time)() = nullptr;
static void (*before_lock)() = nullptr;
static void (*after_unlock)() = nullptr;
static constexpr unsigned sum = Stm32Adc::INJECTED_SAMPLE_SUM;
#if defined(STM32F410Rx)
using ChannelArgument = ADC_ChannelConfTypeDef;
static constexpr uint32_t clock_hz = 100000000, period = 2500, source_offset = 24;
static constexpr unsigned current_ranks = Stm32Adc::FRAME_RANKS;
static constexpr uint32_t current_us = (current_ranks * 27 + 24) / 25;
static ADC_InjectionConfTypeDef injected[3]{};
static unsigned injected_count;
static uint32_t* trigger_word;
static uint32_t aux_launch;
#else
using ChannelArgument = const ADC_ChannelConfTypeDef;
static constexpr uint32_t clock_hz = 170000000, period = 4250, source_offset = 25;
static constexpr unsigned current_ranks = Stm32Adc::CURRENT_RANKS;
static constexpr unsigned bank_ranks = Stm32Adc::ADC1_RANKS;
static constexpr uint32_t current_us = (current_ranks * 38 + 84) / 85;
static unsigned adc1_position;
#endif

uint32_t time_us() {
    if (on_time) { auto hook = on_time; on_time = nullptr; hook(); }
    return now;
}
extern "C" void disable_interrupts() {
    if (before_lock && !depth) { auto hook = before_lock; before_lock = nullptr; hook(); }
    ++depth;
}
extern "C" void enable_interrupts() {
    check(depth == 1, "slow snapshot has one bounded mask and no spin");
    --depth;
    if (after_unlock) { auto hook = after_unlock; after_unlock = nullptr; hook(); }
}
extern "C" {
uint32_t SystemCoreClock = clock_hz;
void Error_Handler() { std::abort(); }
void HAL_GPIO_Init(GPIO_TypeDef*, GPIO_InitTypeDef*) {}
void HAL_NVIC_SetPriority(IRQn_Type, uint32_t, uint32_t) {}
void HAL_NVIC_EnableIRQ(IRQn_Type) {}
void HAL_Delay(uint32_t delay) { check(delay == 1, "only initial ADC enable has a stabilization delay"); }
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* handle) {
    configs[handle->Instance == ADC1 ? 0 : 1] = handle->Init;
#if defined(STM32F410Rx)
    handle->Instance->CR2 = ADC_CR2_DDS;
#else
    handle->Instance->CFGR = handle->Init.ExternalTrigConvEdge;
#endif
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* handle, ChannelArgument* config) {
    unsigned i = handle->Instance == ADC1 ? 0 : 1;
    ranks[i][rank_count[i]++] = *config;
    return HAL_OK;
}
#if defined(STM32F410Rx)
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* handle, ADC_InjectionConfTypeDef* config) {
    injected[injected_count++] = *config;
    // Use ST's actual rank-position macro, not a left-aligned host sequence.
    handle->Instance->JSQR &= ~ADC_JSQR_JL;
    handle->Instance->JSQR |= ADC_SQR1(config->InjectedNbrOfConversion);
    handle->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, config->InjectedRank, config->InjectedNbrOfConversion);
    handle->Instance->JSQR |= ADC_JSQR(config->InjectedChannel, config->InjectedRank, config->InjectedNbrOfConversion);
    return HAL_OK;
}
#else
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_OPAMP_Init(OPAMP_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_OPAMP_Start(OPAMP_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef*, const ADC_MultiModeTypeDef* config) {
    check(config->Mode == ADC_MODE_INDEPENDENT && config->DMAAccessMode == ADC_DMAACCESSMODE_DISABLED,
          "independent ADCs cannot stall V2/W2 behind ADC1 auxiliary ranks");
    return HAL_OK;
}
#endif
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef* handle) {
#if defined(STM32F410Rx)
    dma_configs[handle->Instance == DMA2_Stream0 ? 0 : 1] = handle->Init;
#else
    dma_configs[handle->Instance == DMA1_Channel1 ? 0 : 1] = handle->Init;
#endif
    return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef* handle, uint32_t from, uint32_t to, uint32_t length) {
    if (fail_dma_start) return HAL_ERROR;
#if defined(STM32F410Rx)
    handle->Instance->CR = DMA_SxCR_EN;
    handle->Instance->NDTR = length;
    if (handle->Instance == DMA2_Stream4) {
        check(to == uintptr_t(&ADC1->CR2) && length == 1, "one hardware-triggered regular-start word");
        trigger_word = reinterpret_cast<uint32_t*>(uintptr_t(from));
    } else {
        check(from == uintptr_t(&ADC1->DR) && length == current_ranks, "F410 DMA contains current ranks ONLY");
        buffers[0] = reinterpret_cast<uint16_t*>(uintptr_t(to));
    }
#else
    const unsigned i = handle->Instance == DMA1_Channel1 ? 0 : 1;
    check(from == uintptr_t(i ? &ADC2->DR : &ADC1->DR), "each independent DMA reads its actual ADC DR");
    check(length == (i ? current_ranks : 2 * bank_ranks), "ADC1 has two banks; ADC2 one current sequence");
    handle->Instance->CCR = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE |
                           (i ? 0 : DMA_CCR_CIRC);
    handle->Instance->CNDTR = length;
    buffers[i] = reinterpret_cast<uint16_t*>(uintptr_t(to));
#endif
    return HAL_OK;
}
#if !defined(STM32F410Rx)
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* handle, uint32_t* data, uint32_t length) {
    SET_BIT(handle->Instance->CR, ADC_CR_ADEN | ADC_CR_ADSTART);
    SET_BIT(handle->Instance->IER, ADC_IER_OVRIE);
    return HAL_DMA_Start(handle->DMA_Handle, uint32_t(uintptr_t(&handle->Instance->DR)),
                         uint32_t(uintptr_t(data)), length);
}
#endif
}

static void at(uint32_t time) {
    now = time;
    const uint32_t t = (uint32_t(now - epoch) % 50) * (clock_hz / 1000000);
    TIM1->CNT = t < period ? t : 2 * period - t;
    MODIFY_REG(TIM1->CR1, TIM_CR1_DIR, t < period ? 0 : TIM_CR1_DIR);
}
static void ack() {
#if defined(STM32F410Rx)
    DMA2->LISR &= ~DMA2->LIFCR; DMA2->LIFCR = 0;
    DMA2->HISR &= ~DMA2->HIFCR; DMA2->HIFCR = 0;
    ADC1->SR &= ADC_SR_JEOC | ADC_SR_JSTRT; // emulate W0C, preserve owned injected flags
#else
    uint32_t clear = DMA1->IFCR;
    if (clear & DMA_IFCR_CGIF1) clear |= 15u;
    if (clear & DMA_IFCR_CGIF2) clear |= 15u << 4;
    DMA1->ISR &= ~clear; DMA1->IFCR = 0;
    // Rearm writes EOC/EOS W1C on ADC2 only. ADC1's ongoing scan is untouched.
    ADC2->ISR = 0;
#endif
}
static void captured() {
#if defined(STM32F410Rx)
    check((ADC1->CR2 & ADC_CR2_JSWSTART) && !(ADC1->CR2 & ADC_CR2_SWSTART),
          "CPU launches only injected auxiliaries, never a second regular current triple");
    aux_launch = now;
    CLEAR_BIT(ADC1->CR2, ADC_CR2_JSWSTART); // hardware start handshake
    ADC1->SR = ADC_SR_JSTRT;
#endif
    ack();
}
static void setup(uint32_t first = 1000, uint32_t u_compare = 500,
                  uint32_t v_compare = 600, uint32_t w_compare = 700) {
    if (adc) adc->~Stm32Adc();
    adc = new (reinterpret_cast<void*>(0x20000000)) Stm32Adc;
    new (reinterpret_cast<void*>(ADC1_BASE)) ADC_TypeDef{};
    new (reinterpret_cast<void*>(TIM1_BASE)) TIM_TypeDef{};
#if defined(STM32F410Rx)
    new (reinterpret_cast<void*>(DMA2_BASE)) DMA_TypeDef{};
    new (reinterpret_cast<void*>(DMA2_Stream0_BASE)) DMA_Stream_TypeDef{};
    new (reinterpret_cast<void*>(DMA2_Stream4_BASE)) DMA_Stream_TypeDef{};
    TIM1->CR1 = TIM_COUNTERMODE_CENTERALIGNED2;
    TIM1->CCR4 = period - 100;
    injected_count = 0;
#else
    new (reinterpret_cast<void*>(ADC2_BASE)) ADC_TypeDef{};
    new (reinterpret_cast<void*>(DMA1_BASE)) DMA_TypeDef{};
    new (reinterpret_cast<void*>(DMA1_Channel1_BASE)) DMA_Channel_TypeDef{};
    new (reinterpret_cast<void*>(DMA1_Channel2_BASE)) DMA_Channel_TypeDef{};
    TIM1->CR1 = TIM_COUNTERMODE_CENTERALIGNED1;
    TIM1->CCR4 = period - 1;
    adc1_position = 0;
#endif
    TIM1->ARR = period;
    TIM1->CCR1 = u_compare; TIM1->CCR2 = v_compare; TIM1->CCR3 = w_compare;
    now = epoch = first; source = first + source_offset;
    rank_count[0] = rank_count[1] = 0;
    before_lock = after_unlock = on_time = nullptr;
    CurrentSensorCalibration calibration{.01f, 5.f, 3.3f, 12};
    calibration.injected_sample_sum = sum;
    calibration.offset_voltage_u = 2122 * 3.3f / 4095;
    calibration.offset_voltage_v = 2100 * 3.3f / 4095;
    calibration.offset_voltage_w = 2200 * 3.3f / 4095;
    check(adc->initialize(calibration, {169000.f, 18000.f}), "independent phase zeros initialize");
    check(adc->initializeHardware() && adc->beginFrames(now), "arm both pipelines with PWM stopped");
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
#if defined(STM32F410Rx)
    ADC1->SR = 0;
#endif
    ack();
}
#if !defined(STM32F410Rx)
static void adc1Rank(uint16_t value) {
    buffers[0][adc1_position++] = value;
    if (adc1_position == bank_ranks) SET_BIT(DMA1->ISR, DMA_ISR_HTIF1);
    if (adc1_position == 2 * bank_ranks) { adc1_position = 0; SET_BIT(DMA1->ISR, DMA_ISR_TCIF1); }
    DMA1_Channel1->CNDTR = 2 * bank_ranks - adc1_position;
}
#endif
static void currents(uint16_t first_u = 2122, uint16_t second_u = 2123, bool adc1 = true,
                     uint32_t injected_delay_us = 0) {
    at(source);
#if defined(STM32F410Rx)
    check(DMA2_Stream4->NDTR == 1 && (DMA2_Stream4->CR & DMA_SxCR_EN), "one-shot PWM source is armed");
    ADC1->CR2 = *trigger_word & ~ADC_CR2_SWSTART;
    DMA2_Stream4->NDTR = 0; CLEAR_BIT(DMA2_Stream4->CR, DMA_SxCR_EN);
    SET_BIT(DMA2->HISR, DMA_HISR_TCIF4);
    if (injected_delay_us) {
        // The PWM request arrives while the prior injection is still running.
        // Its JEOC/JDRs appear first; only then can regular rank DMA complete.
        at(source + injected_delay_us);
        ADC1->JDR1 = 2029; ADC1->JDR2 = 1309; ADC1->JDR3 = 2048;
        SET_BIT(ADC1->SR, ADC_SR_JEOC);
    }
    for (unsigned s = 0; s < sum; ++s) {
        buffers[0][3 * s] = s ? second_u : first_u;
        buffers[0][3 * s + 1] = 2100 + s;
        buffers[0][3 * s + 2] = 2200 + s;
    }
    DMA2_Stream0->NDTR = 0; CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
    SET_BIT(DMA2->LISR, DMA_LISR_TCIF0);
#else
    if (adc1) for (unsigned s = 0; s < sum; ++s) adc1Rank(s ? second_u : first_u);
    for (unsigned s = 0; s < sum; ++s) {
        buffers[1][2 * s] = 2100 + s;
        buffers[1][2 * s + 1] = 2200 + s;
    }
    DMA1_Channel2->CNDTR = 0;
    SET_BIT(DMA1->ISR, DMA_ISR_TCIF2);
    SET_BIT(ADC2->ISR, ADC_ISR_EOS);
#endif
    at(source + injected_delay_us + current_us + 1);
}
static void auxiliary(uint16_t bus = 2029, uint16_t ntc = 1309, uint16_t gas = 2048) {
#if defined(STM32F410Rx)
    at(aux_launch + 19);
    ADC1->JDR1 = bus; ADC1->JDR2 = ntc; ADC1->JDR3 = gas;
    SET_BIT(ADC1->SR, ADC_SR_JEOC);
#else
    at(source + 9);
    adc1Rank(bus); adc1Rank(ntc); adc1Rank(gas);
#endif
}
static void rearm() {
    at(now + 1);
    check(adc->rearmFrame(now), "normal current DMA rearmed without waiting for auxiliaries");
    ack();
}
static void first() {
    setup(); currents();
    check(adc->captureFrame(now), "first current frame primes the auxiliary pipeline");
    captured();
    check(adc->samples() == 1 && !adc->regularReady() && !adc->regularFresh(now),
          "first currents do not label uninitialized auxiliaries ready");
    check(adc->getRawAdcValue() == 0 && adc->serviceRegular(now) == 0,
          "first SysTick is explicitly pending; it does not trip or date zero data fresh");
    rearm(); auxiliary(); source += 50;
}

static void configuration() {
    setup();
    check(adc->serviceRegular(now) == 0, "pre-trigger slow service is pending");
#if defined(STM32F410Rx)
    check(configs[0].ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV4 &&
          Stm32Adc::CURRENT_SCAN_TICKS == 324 * sum,
          "configured ADC divider matches the datasheet324/648-tick conversion duration");
    check(configs[0].NbrOfConversion == 3 * sum && rank_count[0] == 3 * sum,
          "one regular scan contains exactly x1/x2 current triples");
    const uint32_t expected[] = {ADC_CHANNEL_0, ADC_CHANNEL_11, ADC_CHANNEL_10};
    for (unsigned i = 0; i < 3 * sum; ++i)
        check(ranks[0][i].Channel == expected[i % 3] && ranks[0][i].SamplingTime == ADC_SAMPLETIME_15CYCLES,
              "F410 current rank order and 15-clock aperture retained");
    check(injected_count == 3 && injected[0].InjectedChannel == ADC_CHANNEL_1 &&
          injected[1].InjectedChannel == ADC_CHANNEL_12 && injected[2].InjectedChannel == ADC_CHANNEL_4,
          "injected sequence is Vbus, NTC, throttle only");
    for (auto config : injected)
        check(config.InjectedNbrOfConversion == 3 && config.InjectedSamplingTime == ADC_SAMPLETIME_144CYCLES &&
              config.ExternalTrigInjecConv == ADC_INJECTED_SOFTWARE_START && !config.AutoInjectedConv,
              "long auxiliary apertures and software-only injected start");
    check(ADC1->JSQR == ((2u << 20) | (1u << 5) | (12u << 10) | (4u << 15)),
          "three injected ranks are right-aligned in JSQ2/3/4, not JSQ1/2/3");
    check(!(ADC1->CR1 & (ADC_CR1_JEOCIE | ADC_CR1_JAUTO | ADC_CR1_JDISCEN)) &&
          !(*trigger_word & ADC_CR2_JSWSTART), "injected IRQ/auto mode and trigger-word replay are disabled");
    check(dma_configs[0].Mode == DMA_NORMAL && dma_configs[1].Mode == DMA_NORMAL,
          "current and PWM-start DMAs each stop after one frame");
#else
    check(configs[0].NbrOfConversion == sum + 3 && configs[1].NbrOfConversion == 2 * sum,
          "compact independent lengths avoid all dummy ranks");
    for (unsigned i = 0; i < sum; ++i)
        check(ranks[0][i].Channel == ADC_CHANNEL_VOPAMP1 && ranks[0][i].SamplingTime == ADC_SAMPLETIME_6CYCLES_5,
              "U samples occupy adjacent ADC1 ranks");
    for (unsigned i = 0; i < current_ranks; ++i)
        check(ranks[1][i].Channel == (i % 2 ? ADC_CHANNEL_VOPAMP3_ADC2 : ADC_CHANNEL_VOPAMP2) &&
              ranks[1][i].SamplingTime == ADC_SAMPLETIME_6CYCLES_5, "ADC2 preserves physical V/W OPAMP routing");
    check(ranks[0][sum].Channel == ADC_CHANNEL_1 && ranks[0][sum+1].Channel == ADC_CHANNEL_5 &&
          ranks[0][sum+2].Channel == ADC_CHANNEL_11, "ADC1 tail is Vbus/NTC/throttle");
    for (unsigned i = sum; i < bank_ranks; ++i)
        check(ranks[0][i].SamplingTime == ADC_SAMPLETIME_92CYCLES_5, "G431 auxiliary aperture remains 92.5 clocks");
    for (auto c : configs)
        check(c.ExternalTrigConv == ADC_EXTERNALTRIG_T1_TRGO2 &&
              c.ExternalTrigConvEdge == ADC_EXTERNALTRIGCONVEDGE_RISING && c.OversamplingMode == DISABLE &&
              c.ContinuousConvMode == DISABLE, "both independent ADCs start once on the same PWM edge");
    check(dma_configs[0].Mode == DMA_CIRCULAR && dma_configs[1].Mode == DMA_NORMAL &&
          dma_configs[0].PeriphDataAlignment == DMA_PDATAALIGN_HALFWORD &&
          dma_configs[1].MemDataAlignment == DMA_MDATAALIGN_HALFWORD, "physical halfwords, double ADC1 bank only");
    check(!(DMA1_Channel1->CCR & (DMA_CCR_TCIE | DMA_CCR_HTIE)) &&
          (DMA1_Channel1->CCR & DMA_CCR_TEIE) && (DMA1_Channel2->CCR & DMA_CCR_TCIE) &&
          !(DMA1_Channel2->CCR & DMA_CCR_HTIE), "only ADC2 current TC generates a normal interrupt");
#endif
}

static void pipelineAndProtection() {
    first(); currents();
    const uint32_t current_source = source;
    check(adc->captureFrame(now), "second current frame consumes the prior complete auxiliaries");
    captured();
    check(adc->regularReady() && adc->regularFresh(now), "current+previous-aux epochs are ready");
    check(adc->getRawAdcValue() == 2029 * sum && adc->getRawTemperatureAdcValue() == 0,
          "prior Vbus publishes every control tick; NTC remains SysTick-owned");
    check(adc->auxiliaryStartedUs() < current_source && adc->frameStartedUs() == current_source,
          "auxiliary source is not relabeled as the current trigger");
    check(adc->getRawAdcValue(PwmChannel::PHASE_U) == 2122 * sum + sum - 1 &&
          adc->getRawAdcValue(PwmChannel::PHASE_V) == 2100 * sum + sum - 1 &&
          adc->getRawAdcValue(PwmChannel::PHASE_W) == 2200 * sum + sum - 1,
          "all odd phase sums retain their own original samples and zero identities");
    check(adc->serviceRegular(now) == 1 && adc->getRawTemperatureAdcValue() == 1309 &&
          adc->readRawPotentiometer() == 2048, "slow snapshot copies complete physical NTC/throttle codes");
    check(std::fabs(adc->readTemperature() - 25.f) < 1.f &&
          std::fabs(adc->readBusVoltage() - 2029.f * 3.3f / 4095 * 187000 / 18000) < .002f,
          "calibrated auxiliary units remain correct");
    rearm(); auxiliary(1900, 1450, 2700); source += 50;
    currents(4000, 2122);
    check(adc->getRawAdcValue() == 2029 * sum, "DMA/injected activity cannot mutate the consumed bus cache");
    check(adc->captureFrame(now), "next frame has a positive first-scan transient"); captured();
    check(adc->getRawAdcValue() == 1900 * sum, "previous auxiliary generation advances once per current frame");
    check(adc->convertAdcToCurrent(adc->getRawProtectionCurrent(PwmChannel::PHASE_U, 0), PwmChannel::PHASE_U) > 25,
          "original positive peak remains visible to the existing F410 raw threshold");
    if (sum == 2) check(adc->readPhaseCurrent(PwmChannel::PHASE_U) < 25, "checking only the sum would hide that peak");
    check(adc->getRawTemperatureAdcValue() == 1309, "fast completion does not replace slow snapshot");
    rearm(); auxiliary(); source += 50; currents(sum == 1 ? 200 : 2122, 200);
    check(adc->captureFrame(now), "last-scan negative transient frame"); captured();
    check(adc->convertAdcToCurrent(adc->getRawProtectionCurrent(PwmChannel::PHASE_U, sum-1), PwmChannel::PHASE_U) < -25,
          "negative original peak remains visible before feedback averaging");
    if (sum == 2) check(adc->readPhaseCurrent(PwmChannel::PHASE_U) > -25, "negative peak is also hidden by averaging");
}

static void preemptFrame() {
    rearm(); auxiliary(1800, 1500, 2800); source += 50; currents();
    check(adc->captureFrame(now), "DMA frame preempts slow snapshot caller"); captured();
}
static void preemption() {
    first(); currents(); check(adc->captureFrame(now), "slow preemption initial complete pipeline"); captured();
    const uint32_t old_now = now;
    before_lock = preemptFrame;
    check(adc->serviceRegular(old_now) == 1 && adc->getRawTemperatureAdcValue() == 1500 &&
          adc->readRawPotentiometer() == 2800, "clock and complete raw snapshot survive preemption before mask");
    rearm(); auxiliary(1700, 1600, 2900); source += 50; currents();
    check(adc->captureFrame(now), "next cache source"); captured();
    after_unlock = preemptFrame;
    check(adc->serviceRegular(now) == 1 && adc->getRawTemperatureAdcValue() == 1600 &&
          adc->readRawPotentiometer() == 2900, "DMA after unlock cannot mix the SysTick-owned pair");
    check(depth == 0, "snapshot critical sections restore interrupt state");
}

static void faults() {
    first(); currents();
#if defined(STM32F410Rx)
    DMA2_Stream0->NDTR = 1;
#else
    DMA1_Channel2->CNDTR = 1;
#endif
    check(!adc->captureFrame(now), "partial current transfer fails despite TC");
    check(!adc->rearmFrame(now), "failure cannot rearm acquisition");
    first(); currents();
#if defined(STM32F410Rx)
    SET_BIT(DMA2->LISR, DMA_LISR_TEIF0);
#else
    SET_BIT(DMA1->ISR, DMA_ISR_TEIF2);
#endif
    check(!adc->captureFrame(now), "current DMA error wins over TC");
    first(); currents();
#if defined(STM32F410Rx)
    SET_BIT(ADC1->SR, ADC_SR_OVR);
#else
    SET_BIT(ADC1->ISR, ADC_ISR_OVR);
#endif
    check(!adc->captureFrame(now), "auxiliary ADC overrun also fails the current frame");
    first(); currents(); at(source + 51);
    check(!adc->captureFrame(now), "old current TC cannot be relabeled as a newer source epoch");
    first(); currents(); at(source - 1);
    check(!adc->captureFrame(now), "completion before its owned source cannot certify current data");
    first(); currents(); check(adc->captureFrame(now), "duplicate completion initial frame"); captured();
    check(!adc->captureFrame(now) && adc->samples() == 2, "duplicate TC cannot advance a control generation");
    first(); currents(); check(adc->captureFrame(now), "deadline frame"); captured(); at(source + 47);
    check(!adc->rearmFrame(now), "whole-period deadline is never restarted at current DMA entry");
    first(); at(now + 1000);
    check(adc->serviceRegular(now) < 0, "lost current source fails the independent SysTick watchdog");
    setup(0xfffffff0U); currents(); check(adc->captureFrame(now), "first frame across timestamp wrap"); captured();
    rearm(); auxiliary(); source += 50; currents();
    check(adc->captureFrame(now) && adc->regularFresh(now), "previous auxiliary source survives timestamp wrap");
    captured(); check(adc->serviceRegular(now) == 1, "slow cache survives wrap too");
#if defined(STM32F410Rx)
    first(); CLEAR_BIT(ADC1->SR, ADC_SR_JEOC); currents();
    check(!adc->captureFrame(now), "missing previous injected completion is never used");
    setup(); SET_BIT(ADC1->SR, ADC_SR_JEOC); currents();
    check(!adc->captureFrame(now), "unowned startup JEOC is not a fresh auxiliary frame");
    setup(); currents(); at(source + 27);
    check(!adc->captureFrame(now) && !(ADC1->CR2 & ADC_CR2_JSWSTART),
          "late current IRQ rejects aux launch that would approach the next PWM aperture");
    setup(); currents(); check(adc->captureFrame(now), "aux watchdog priming frame"); captured();
    at(aux_launch + Stm32Adc::AUXILIARY_BUDGET_US + 1);
    check(!adc->rearmFrame(now), "observed over-budget injected scan fails without waiting");
    first(); TIM1->CCR1 = 2375; TIM1->CCR2 = 1800; TIM1->CCR3 = 0; currents();
    check(adc->captureFrame(now), "normal high-duty current completion"); captured();
    check(adc->currentWindowValid(0),
           "x1/x2 high-duty U reconstruction preserves every consumed V/W aperture");
    setup(1000, 2375, 2000, 0); currents();
    check(adc->captureFrame(now), "prime auxiliaries before delayed IRQ delivery"); captured(); rearm(); auxiliary();
    source += 50;
    currents(); at(source + 16);
    check(adc->captureFrame(now), "late IRQ still meets auxiliary launch admission and47us deadline"); captured();
    check(adc->currentWindowValid(0),
           "IRQ delivery latency does not change the datasheet current-aperture geometry");
    check(!(ADC1->CR2 & ADC_CR2_SWSTART), "no software second CURRENT scan exists");
    if (sum == 1) {
        // Old U fails opening while the new preloads tie. Native FOC duties
        // may differ below a CCR count; quantized ties cannot prove its choice.
        setup(1000, 2101, 1856, 0); currents();
        check(adc->captureFrame(now), "prime an old narrow U opening"); captured();
        TIM1->CCR1 = 1856;
        rearm(); auxiliary(); source += 50; currents();
        check(adc->captureFrame(now), "capture a quantized tie after the old U generation"); captured();
        check(!adc->currentWindowValid(0), "one invalid phase plus a quantized maximum tie is rejected");
    }
    setup(1000, 1000, 2375, 0); currents();
    check(adc->captureFrame(now) && !adc->regularReady(),
          "bridge-off priming still succeeds even when retained duty history is unqualified"); captured();
    TIM1->CCR1 = 2375; TIM1->CCR2 = 1000;
    rearm(); auxiliary(); source += 50; currents();
    check(adc->captureFrame(now), "capture after changing the highest phase across preload generations"); captured();
    check(!adc->currentWindowValid(0), "old-high V cannot disappear inside a scalar highest-duty history");
    rearm(); auxiliary(); source += 50; currents();
    check(adc->captureFrame(now), "next stable highest-U carrier"); captured();
    check(adc->currentWindowValid(0), "stable x1/x2 generation regains qualified reconstruction");
    if (sum == 2) {
        // Captured hardware failure: both original U/V samples fit. Delaying
        // IRQ processing to8.30us must not extend physical ADC apertures.
        setup(1000, 984, 0, 1776); currents(); at(source + 8);
        TIM1->CNT -= 30;
        check(adc->captureFrame(now), "replay the physical x2 frame with830-tick IRQ arrival"); captured();
        check(adc->currentWindowValid(2), "reconstructed W no longer stops the captured hardware operating point");
        // An exact physical W closing boundary remains invalid even though IRQ
        // latency no longer participates. U and W cannot both be reconstructed.
        setup(1000, 2375, 0, 2000); currents(); at(source + 8);
        check(adc->captureFrame(now), "late x2 current still fits auxiliary launch admission"); captured();
        check(!adc->currentWindowValid(0), "second x2 W aperture at its closing boundary still rejects the frame");

        // A late duty write owns closing only after the peak. Old W2020 is
        // too late for nominal x2 closing, while the newly loaded1712 fits.
        setup(1000, 1730, 0, 2020); currents();
        check(adc->captureFrame(now), "prime old-W-high preload history"); captured(); auxiliary();
        TIM1->CCR3 = 1712;
        at(source + 32); rearm();
        source += 50; currents(); at(source + 8); TIM1->CNT -= 5;
        check(adc->captureFrame(now), "capture the805-tick old-W/new-U transition"); captured();
        check(adc->currentWindowValid(0), "x2 uses the peak-loaded W closing duty while reconstructing new-high U");
        TIM1->CCR3 = 1700;
        check(!adc->currentWindowValid(0),
              "a post-arm foreground preload write cannot masquerade as the peak-loaded generation");
        TIM1->CCR3 = 1712;
        TIM1->RCR = 1;
        check(!adc->currentWindowValid(0), "repetition-counter change invalidates the peak reload proof");
        TIM1->RCR = 0;
        SET_BIT(TIM1->CR1, TIM_CR1_UDIS);
        check(!adc->currentWindowValid(0), "suppressed peak reload invalidates the generation proof");
        CLEAR_BIT(TIM1->CR1, TIM_CR1_UDIS);

        // Real10-RPS failure: old-high U lacks opening settlement, while the
        // new highest duty is V. V/W are both valid and can reconstruct U.
        setup(1000, 2134, 1762, 0); currents();
        check(adc->captureFrame(now), "prime the old-U-high opening generation"); captured(); auxiliary();
        TIM1->CCR1 = 1507; TIM1->CCR2 = 1541;
        at(source + 32); rearm();
        source += 50; currents(); at(source + 7); TIM1->CNT -= 50;
        check(adc->captureFrame(now), "capture the750-tick old-U/new-V transition"); captured();
        check(adc->currentInvalidPhases() == 1, "only U is invalid; both V/W pairs remain qualified");
        check(!adc->currentWindowValid(1), "automatic new-high V reconstruction would consume invalid U");
        check(!adc->currentWindowValid(0), "legacy highest-only exemption requires an explicit FOC override here");

        // V's first aperture starts1.08us after U, so old V2135 has settled by
        // its own rank. W2023 alone loses the last aperture's closing window.
        setup(1000, 0, 2135, 2023); currents();
        check(adc->captureFrame(now), "prime the old-V-high datasheet-opening replay"); captured(); auxiliary();
        TIM1->CCR2 = 1837;
        at(source + 32); rearm(); source += 50; currents();
        check(adc->captureFrame(now), "capture the old-V/new-W transition"); captured();
        check(adc->currentInvalidPhases() == 4 && adc->currentWindowValid(2),
              "rank-specific first-aperture timing keeps valid V and reconstructs W alone");
    }
#else
    first(); currents(2122, 2123, false);
    check(!adc->captureFrame(now), "missing ADC1 trigger leaves no current-bank U generation");
    if (sum == 2) {
        first(); currents(2122, 2123, false); adc1Rank(2122);
        check(!adc->captureFrame(now), "one current U cannot be combined with a stale second U sample");
    }
    first(); currents(); CLEAR_BIT(DMA1->ISR, DMA_ISR_HTIF1);
    check(!adc->captureFrame(now), "NDTR alone cannot replace the previous bank's completion flag");
    first(); currents(); SET_BIT(DMA1->ISR, DMA_ISR_TCIF1);
    check(!adc->captureFrame(now), "two-bank traversal flags with partial current position are rejected");
    first(); currents(); SET_BIT(ADC1->CFGR, ADC_CFGR_CONT);
    check(!adc->captureFrame(now), "unexpected continuous ADC1 cannot silently traverse the circular banks");
    first(); currents(); SET_BIT(DMA1->ISR, DMA_ISR_TEIF1);
    check(!adc->captureFrame(now), "ADC1 DMA transfer error invalidates both current U and prior auxiliaries");
    first(); currents(); CLEAR_BIT(ADC2->ISR, ADC_ISR_EOS);
    check(!adc->captureFrame(now), "ADC2 EOS is required for current TC ownership");
    first(); currents(); auxiliary(1700, 1600, 2900);
    check(adc->captureFrame(now) && adc->getRawAdcValue() == 2029 * sum,
          "late IRQ after current ADC1 tail still consumes PREVIOUS bank's bus");
    captured();
    check((DMA1->ISR & DMA_ISR_TCIF1) && !(DMA1->ISR & DMA_ISR_HTIF1),
          "consuming prior HT leaves the current bank TC for the next epoch");
    rearm(); source += 50; currents();
    check(adc->captureFrame(now) && adc->getRawAdcValue() == 1700 * sum,
          "alternation consumes the other bank exactly once"); captured();
#endif
}
#if defined(STM32F410Rx)
static void perScanReconstruction() {
    using Current = Stm32Adc::NativeCurrent;
    Current u{}, v{}, w{};
    if (sum == 1) {
        first(); currents();
        check(adc->captureFrame(now), "x1 reconstruction compatibility frame"); captured();
        check(!adc->setCurrentReconstruction(0, 1), "x1 cannot enable per-triple x2 reconstruction");
        adc->readNativeCurrents(u, v, w);
        check(u == adc->convertNativeAdcToCurrent(adc->getRawAdcValue(PwmChannel::PHASE_U), PwmChannel::PHASE_U) &&
              v == adc->convertNativeAdcToCurrent(adc->getRawAdcValue(PwmChannel::PHASE_V), PwmChannel::PHASE_V) &&
              w == adc->convertNativeAdcToCurrent(adc->getRawAdcValue(PwmChannel::PHASE_W), PwmChannel::PHASE_W),
              "x1 native feedback keeps its original calibrated samples");
        check(adc->currentInvalidPhases(0) == adc->currentInvalidPhases() && adc->currentInvalidPhases(1) == 7,
              "x1 mask API accepts only its one physical triple");
        return;
    }

    const float amperes_per_count = 3.3f / (4095.0f * 0.01f * 5.0f);
    const float expected[3] = {20 * amperes_per_count, -30 * amperes_per_count, 10 * amperes_per_count};
    Current reference[3]{};
    bool have_reference = false;
    for (uint16_t poison_v : {uint16_t(0), uint16_t(4095)}) {
        for (uint16_t poison_w : {uint16_t(0), uint16_t(4095)}) {
            // Physical 10.5-RPS failure: V1 opens 33 ticks too early, W2
            // closes 21 ticks too late. Each complete triple has two good legs.
            setup(1000, 0, 2241, 2021); currents();
            check(adc->captureFrame(now), "prime old-V-high per-scan history"); captured(); auxiliary();
            TIM1->CCR2 = 1837;
            at(source + 32); rearm(); source += 50; currents();
            // Independent phase zeros are U2122/V2100/W2200. Retained deltas:
            // first U=120, W=-40 => V=-80; second U=-80, V=20 => W=60.
            const uint16_t physical[6] = {2242, poison_v, 2160, 2042, 2120, poison_w};
            for (unsigned i = 0; i < 6; ++i) buffers[0][i] = physical[i];
            check(adc->captureFrame(now), "capture independently poisoned V1 and W2"); captured();
            check(adc->currentInvalidPhases(0) == 2 && adc->currentInvalidPhases(1) == 4 &&
                  adc->currentInvalidPhases() == 6,
                  "captured operating point has one invalid phase in EACH triple, not two in either");
            check(!adc->currentWindowValid(1) && !adc->currentWindowValid(2),
                  "one reconstruction after averaging cannot repair the V1/W2 union");

            AdcInterface& feedback = *adc;
            feedback.readNativeCurrents(u, v, w);
            check(std::fabs(numberCast<float>(v) - expected[1]) > 5.0f,
                  "negative control: original phase averaging retains poisoned V1 without explicit reconstruction");
            const uint32_t cr2 = ADC1->CR2, dier = TIM1->DIER;
            check(adc->setCurrentReconstruction(1, 2), "select different discarded phases for the two physical triples");
            check(!adc->setCurrentReconstruction(3, 2) && !adc->setCurrentReconstruction(0, 3) &&
                  !adc->setCurrentReconstruction(256, 2) &&
                  !adc->setCurrentReconstruction(1, static_cast<unsigned>(-1)),
                  "invalid first/second indexes reject transactionally without narrowing into valid phases");
            feedback.readNativeCurrents(u, v, w);
            const Current actual[3] = {u, v, w};
            for (unsigned phase = 0; phase < 3; ++phase) {
                check(std::fabs(numberCast<float>(actual[phase]) - expected[phase]) < 0.0002f,
                      "native mean matches independently calibrated physical per-triple KCL vectors");
                check(adc->getRawAdcValue(static_cast<PwmChannel>(phase)) == physical[phase] + physical[phase + 3],
                      "legacy phase-sum getter retains both original samples, including discarded poison");
                for (unsigned scan = 0; scan < 2; ++scan) {
                    const unsigned index = 3 * scan + phase;
                    check(adc->getRawProtectionCurrent(static_cast<PwmChannel>(phase), scan) == 2u * physical[index] &&
                          buffers[0][index] == physical[index],
                          "per-scan protection and DMA storage retain every original raw value");
                }
            }
            check(std::fabs(numberCast<float>(u) + numberCast<float>(v) + numberCast<float>(w)) < 0.000004f,
                  "completed averaged vector remains zero-sum within native rounding");
            if (have_reference) {
                check(u == reference[0] && v == reference[1] && w == reference[2],
                      "independent positive/negative poison in either discarded sample cannot affect native feedback");
            } else {
                reference[0] = u; reference[1] = v; reference[2] = w; have_reference = true;
            }
            check(std::fabs(adc->convertAdcToCurrent(adc->getRawProtectionCurrent(PwmChannel::PHASE_V, 0),
                                                    PwmChannel::PHASE_V)) > 25.0f &&
                  std::fabs(adc->convertAdcToCurrent(adc->getRawProtectionCurrent(PwmChannel::PHASE_W, 1),
                                                    PwmChannel::PHASE_W)) > 25.0f,
                  "original discarded peaks still reach the unchanged raw-current cutout");
            check(ADC1->CR2 == cr2 && TIM1->DIER == dier,
                  "selection and native reconstruction never launch acquisition or touch PWM requests");
            feedback.readNativeCurrents(u, v, w);
            check(u == actual[0] && v == actual[1] && w == actual[2],
                  "same accepted frame can be read repeatedly without consuming its selections");

            TIM1->CCR2 = 1800;
            check(adc->currentInvalidPhases(1) == 6,
                  "post-arm preload changes cannot falsely prove the newer per-scan closing generation");
            TIM1->CCR2 = 1837;
            TIM1->RCR = 1;
            check(adc->currentInvalidPhases(0) == 7 && adc->currentInvalidPhases(1) == 7,
                  "per-scan masks retain repetition-counter guard");
            TIM1->RCR = 0; SET_BIT(TIM1->CR1, TIM_CR1_UDIS);
            check(adc->currentInvalidPhases(0) == 7 && adc->currentInvalidPhases(1) == 7,
                  "per-scan masks retain suppressed-update guard");
            CLEAR_BIT(TIM1->CR1, TIM_CR1_UDIS);
            check(adc->currentInvalidPhases(2) == 7, "out-of-range scan fails closed");

            rearm(); auxiliary(); source += 50; currents(2242, 2242);
            check(adc->captureFrame(now), "accept a new frame with no reconstruction selection"); captured();
            feedback.readNativeCurrents(u, v, w);
            check(u == adc->convertNativeAdcToCurrent(adc->getRawAdcValue(PwmChannel::PHASE_U), PwmChannel::PHASE_U) &&
                  v == adc->convertNativeAdcToCurrent(adc->getRawAdcValue(PwmChannel::PHASE_V), PwmChannel::PHASE_V) &&
                  w == adc->convertNativeAdcToCurrent(adc->getRawAdcValue(PwmChannel::PHASE_W), PwmChannel::PHASE_W),
                  "each accepted current frame clears both selections and restores original native sums");
        }
    }
    setup(1000, 2375, 2375, 0); currents();
    check(adc->captureFrame(now), "capture two invalid legs within the SAME physical triple"); captured();
    check(adc->currentInvalidPhases(0) == 3 && adc->currentInvalidPhases(1) == 3 &&
          !adc->currentWindowValid(0) && !adc->currentWindowValid(1),
          "same-triple two-bad-phase masks cannot qualify any single-phase reconstruction");
}

static void wideReconstructionMean() {
    if (sum != 2) return;
    using Current = Stm32Adc::NativeCurrent;
    Current u{}, v{}, w{};
    for (bool negative : {false, true}) {
        setup();
        auto calibration = adc->getCalibration();
        calibration.shunt_resistance_ohms = 0.002f;
        calibration.opamp_gain = 1.0f;
        calibration.offset_voltage_u = negative ? 3.3f : 0.0f;
        calibration.offset_voltage_v = calibration.offset_voltage_w = 0.0f;
        check(adc->initialize(calibration, {169000.f, 18000.f}), "configure representable high-range native inputs");
        currents();
        for (unsigned scan = 0; scan < 2; ++scan) {
            buffers[0][3 * scan] = negative ? 0 : 4095;
            buffers[0][3 * scan + 1] = 0;
            buffers[0][3 * scan + 2] = 1000;
        }
        check(adc->captureFrame(now), "capture equal large currents whose unhalved sum exceeds Q20 range"); captured();
        check(adc->setCurrentReconstruction(2, 2), "reconstruct W in both large-current triples");
        adc->readNativeCurrents(u, v, w);
        const float expected = negative ? -1650.0f : 1650.0f;
        check(std::fabs(numberCast<float>(u) - expected) < 0.01f &&
              std::fabs(numberCast<float>(v)) < 0.01f && std::fabs(numberCast<float>(w) + expected) < 0.01f,
              "signed x2 averaging never saturates the current-format sum before halving");
    }
    setup();
    auto calibration = adc->getCalibration();
    calibration.shunt_resistance_ohms = 0.002f; calibration.opamp_gain = 1.0f;
    calibration.offset_voltage_u = calibration.offset_voltage_v = 0.0f;
    calibration.offset_voltage_w = 3.3f;
    check(adc->initialize(calibration, {169000.f, 18000.f}), "configure wide reconstructed intermediate");
    currents();
    const uint16_t physical[6] = {4095, 4095, 1000, 0, 1000, 4095};
    for (unsigned i = 0; i < 6; ++i) buffers[0][i] = physical[i];
    check(adc->captureFrame(now), "capture first W=-3300A and second W=0A with representable retained samples"); captured();
    check(adc->setCurrentReconstruction(2, 1), "reconstruct W1 and V2 before averaging");
    adc->readNativeCurrents(u, v, w);
    check(std::fabs(numberCast<float>(u) - 825.0f) < 0.01f &&
          std::fabs(numberCast<float>(v) - 825.0f) < 0.01f && std::fabs(numberCast<float>(w) + 1650.0f) < 0.01f,
          "per-triple reconstruction also stays wide until the final representable mean");
}
#endif

#if !defined(STM32F410Rx)
static void finishTailDuringCopy() { auxiliary(1710, 1610, 2910); }
static void overwritePastBankDuringCopy() {
    auxiliary();
    at(source + 51); // next PWM begins overwriting the formerly previous bank
    adc1Rank(3000);
}
static void dmaInterleavings() {
    first(); currents(); on_time = finishTailDuringCopy;
    check(adc->captureFrame(now) && adc->getRawAdcValue() == 2029 * sum,
          "ADC1 continues its tail during CPU copy without mixing bus epochs"); captured();
    check(DMA1->ISR & DMA_ISR_TCIF1, "boundary arriving during copy is retained for next frame");
    first(); currents(); on_time = overwritePastBankDuringCopy;
    check(!adc->captureFrame(now) && adc->samples() == 1,
          "DMA overwrite across a new PWM source during CPU copy fails BEFORE publication");
}
#endif
static void map(uintptr_t address, size_t size) {
    if (mmap(reinterpret_cast<void*>(address), size, PROT_READ | PROT_WRITE,
             MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0) == MAP_FAILED) {
        std::perror("anonymous MMIO"); std::exit(EXIT_FAILURE);
    }
}
int main() {
    map(0x20000000, 0x10000); map(0x40010000, 0x20000);
#if !defined(STM32F410Rx)
    map(0x48000000, 0x10000); map(0x50000000, 0x10000);
#endif
    configuration(); pipelineAndProtection(); preemption(); faults();
#if defined(STM32F410Rx)
    perScanReconstruction(); wideReconstructionMean();
#endif
#if !defined(STM32F410Rx)
    dmaInterleavings();
#endif
    fail_dma_start = true; rank_count[0] = rank_count[1] = 0;
#if defined(STM32F410Rx)
    injected_count = 0;
#endif
    check(!adc->initializeHardware(), "DMA init failure propagates");
    std::printf("STM32 pipelined ADC %s x%u: %u checks, %u failures\n", CONTROL_PROFILE_NAME, sum, checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
