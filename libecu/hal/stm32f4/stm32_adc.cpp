/** @file stm32_adc.cpp
 * @brief One PWM event -> U,V,W[,U,V,W] -> DMA IRQ, injected auxiliaries alongside FOC
 */
#include "stm32_adc.hpp"
#include "board_config.hpp"
#include "pwm_timing_bench.hpp"
#include "critical_section.hpp"
#include "pwm_sampling_state.hpp"

namespace libecu {
namespace {
constexpr uint32_t RESULT_ERRORS = DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0;
constexpr uint32_t TRIGGER_ERRORS = DMA_HISR_TEIF4 | DMA_HISR_DMEIF4 | DMA_HISR_FEIF4;
constexpr uint32_t RESULT_FLAGS = RESULT_ERRORS | DMA_LISR_TCIF0 | DMA_LISR_HTIF0;
constexpr uint32_t TRIGGER_FLAGS = TRIGGER_ERRORS | DMA_HISR_TCIF4 | DMA_HISR_HTIF4;
}

bool Stm32Adc::initializeHardware() noexcept {
    // A complete current aperture + auxiliary scan must fit in one carrier.
    if ((PHASE_RANKS * 27 + 24) / 25 + AUXILIARY_BUDGET_US + 3 >= FRAME_PERIOD_US) return false;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &gpio);

    adc_.Instance = ADC1;
    adc_.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 100 / 4 = 25 MHz
    adc_.Init.Resolution = ADC_RESOLUTION_12B;
    adc_.Init.ScanConvMode = ENABLE;
    adc_.Init.ContinuousConvMode = DISABLE;
    adc_.Init.DiscontinuousConvMode = DISABLE;
    adc_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc_.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    adc_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_.Init.NbrOfConversion = FRAME_RANKS;
    adc_.Init.DMAContinuousRequests = ENABLE;
    adc_.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&adc_) != HAL_OK) return false;
    ADC_ChannelConfTypeDef channel{};
    const uint32_t phases[] = {ADC_CHANNEL_0, ADC_CHANNEL_11, ADC_CHANNEL_10};
    const uint32_t auxiliary[] = {ADC_CHANNEL_1, ADC_CHANNEL_12, ADC_CHANNEL_4};
    for (unsigned i = 0; i < FRAME_RANKS; ++i) {
        channel.Rank = i + 1;
        channel.Channel = phases[i % 3];
        channel.SamplingTime = ADC_SAMPLETIME_15CYCLES;
        if (HAL_ADC_ConfigChannel(&adc_, &channel) != HAL_OK) return false;
    }
    ADC_InjectionConfTypeDef injected{};
    injected.InjectedNbrOfConversion = 3;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_144CYCLES;
    injected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    for (unsigned i = 0; i < 3; ++i) {
        injected.InjectedRank = i + 1;
        injected.InjectedChannel = auxiliary[i];
        if (HAL_ADCEx_InjectedConfigChannel(&adc_, &injected) != HAL_OK) return false;
    }
    // HAL right-aligns three channels in JSQ2..4; results still appear in
    // JDR1..3 (RM0401). Injected completion is polled once by the NEXT current
    // DMA IRQ, never via another ADC IRQ or a busy-wait.
    CLEAR_BIT(ADC1->CR1, ADC_CR1_JEOCIE | ADC_CR1_JAUTO | ADC_CR1_JDISCEN);

    dma_.Instance = DMA2_Stream0;
    dma_.Init.Channel = DMA_CHANNEL_0;
    dma_.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma_.Init.MemInc = DMA_MINC_ENABLE;
    dma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma_.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_.Init.Mode = DMA_NORMAL;
    dma_.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&dma_) != HAL_OK ||
        HAL_DMA_Start(&dma_, uint32_t(uintptr_t(&ADC1->DR)),
                      uint32_t(uintptr_t(dma_raw_)), FRAME_RANKS) != HAL_OK) return false;
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    SET_BIT(DMA2_Stream0->FCR, DMA_SxFCR_FEIE);
    SET_BIT(ADC1->CR2, ADC_CR2_DMA);
    __HAL_ADC_ENABLE(&adc_);
    HAL_Delay(1); // Power-up stabilization is initialization-only, never an ISR wait.

    // RM0401 table 30: TIM1_CH4 -> DMA2 stream4/channel6. Unlike the injected
    // group, regular EXTSEL cannot select TIM1_TRGO or CH4. A hardware DMA word
    // write bridges that event to SWSTART, with no start interrupt or extra scan.
    const uint32_t stable = ADC1->CR2 & ~(ADC_CR2_SWSTART | ADC_CR2_JSWSTART);
    trigger_word_ = stable | ADC_CR2_SWSTART;
    auxiliary_word_ = stable | ADC_CR2_JSWSTART;
    ADC1->SR = ~uint32_t(ADC_SR_JEOC | ADC_SR_JSTRT);
    trigger_dma_.Instance = DMA2_Stream4;
    trigger_dma_.Init.Channel = DMA_CHANNEL_6;
    trigger_dma_.Init.Direction = DMA_MEMORY_TO_PERIPH;
    trigger_dma_.Init.MemInc = DMA_MINC_DISABLE;
    trigger_dma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    trigger_dma_.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    trigger_dma_.Init.Mode = DMA_NORMAL;
    trigger_dma_.Init.Priority = DMA_PRIORITY_HIGH;
    trigger_dma_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&trigger_dma_) != HAL_OK ||
        HAL_DMA_Start(&trigger_dma_, uint32_t(uintptr_t(&trigger_word_)),
                      uint32_t(uintptr_t(&ADC1->CR2)), 1) != HAL_OK) return false;
    SET_BIT(DMA2_Stream4->CR, DMA_SxCR_TEIE | DMA_SxCR_DMEIE); // no trigger TC/HT IRQ
    SET_BIT(DMA2_Stream4->FCR, DMA_SxFCR_FEIE);
    __HAL_ADC_ENABLE_IT(&adc_, ADC_IT_OVR);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    return true;
}

bool Stm32Adc::nextTrigger(uint32_t now_us) noexcept {
    const uint32_t control = TIM1->CR1, count = TIM1->CNT, period = TIM1->ARR;
    if (TIM1->PSC || TIM1->RCR || (control & TIM_CR1_UDIS) || !period || count > period ||
        (control & TIM_CR1_CMS) != TIM_COUNTERMODE_CENTERALIGNED2 ||
        ((control ^ TIM1->CR1) & TIM_CR1_DIR) || TIM1->CCR4 >= period) return false;
    const uint32_t phase = control & TIM_CR1_DIR ? 2 * period - count : count;
    const uint32_t trigger = TIM1->CCR4;
    const uint32_t distance = trigger > phase ? trigger - phase : 2 * period - phase + trigger;
    expected_trigger_us_ = now_us + distance / (ihm08::TIMER_CLOCK_HZ / 1000000);
    const PwmSamplingState::Compare next_compare{{TIM1->CCR1, TIM1->CCR2, TIM1->CCR3}};
    const auto window = pwm_sampling_state.armWindow((TIM1->SR & TIM_SR_UIF) != 0,
                                                     phase >= trigger, next_compare);
    // Only the PWM writer clears UIF. An update arriving after this observation
    // remains visible; ADC timing itself is still the fixed datasheet schedule.
    if (!window.known && pwm_sampling_state.generation) return false;
    pwm_window_exact_ = window.known;
    pwm_generation_ = window.generation;
    for (unsigned leg = 0; leg < 3; ++leg) {
        armed_compare_[leg] = window.known ? window.opening[leg] :
            (next_compare[leg] > previous_compare_[leg] ? next_compare[leg] : previous_compare_[leg]);
        previous_compare_[leg] = next_compare[leg];
    }
    return true;
}

bool Stm32Adc::beginFrames(uint32_t now_us) noexcept {
    if (failed_ || !nextTrigger(now_us)) { failFrame(); return false; }
    armed_ = true;
    CLEAR_BIT(TIM1->CR2, TIM_CR2_CCDS); // CC4 compare event, never an update request
    TIM1->SR = ~uint32_t(TIM_SR_CC4IF);
    SET_BIT(TIM1->DIER, TIM_DIER_CC4DE);
    return true;
}

void Stm32Adc::failFrame() noexcept {
    failed_ = true;
    armed_ = false;
    CLEAR_BIT(TIM1->DIER, TIM_DIER_CC4DE);
    CLEAR_BIT(DMA2_Stream4->CR, DMA_SxCR_EN | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_EN | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    CLEAR_BIT(DMA2_Stream0->FCR, DMA_SxFCR_FEIE);
    CLEAR_BIT(DMA2_Stream4->FCR, DMA_SxFCR_FEIE);
    CLEAR_BIT(ADC1->CR1, ADC_CR1_OVRIE);
    DMA2->LIFCR = RESULT_FLAGS;
    DMA2->HIFCR = TRIGGER_FLAGS;
}

bool Stm32Adc::captureFrame(uint32_t now_us) noexcept {
    const uint32_t flags = DMA2->LISR, source = DMA2->HISR;
    constexpr uint32_t scan_us = PHASE_RANKS * 27 / 25;
    const uint32_t age = now_us - expected_trigger_us_;
    if (failed_ || !armed_ || (flags & RESULT_ERRORS) || (source & TRIGGER_ERRORS) ||
        !(flags & DMA_LISR_TCIF0) || !(source & DMA_HISR_TCIF4) ||
        DMA2_Stream0->NDTR || DMA2_Stream4->NDTR ||
        (DMA2_Stream0->CR & DMA_SxCR_EN) || (DMA2_Stream4->CR & DMA_SxCR_EN) ||
        !(TIM1->CR1 & TIM_CR1_CEN) || (ADC1->SR & ADC_SR_OVR) ||
        age + 2 < scan_us || age >= FRAME_PERIOD_US - 3) {
        failFrame(); return false;
    }
    // Normal DMA is stopped by hardware at NDTR=0. Neither DMA can overwrite
    // this frame or start another scan until rearmFrame, after control returns.
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    for (unsigned i = 0; i < FRAME_RANKS; ++i) raw_[i] = dma_raw_[i];
    for (unsigned i = 0; i < FRAME_RANKS; ++i) {
        if (raw_[i] > 4095) { failFrame(); return false; }
    }
    if (auxiliary_pending_) {
        if (auxiliary_generation_ != samples_ || !(ADC1->SR & ADC_SR_JEOC) ||
            uint32_t(now_us - auxiliary_launch_us_) > REGULAR_MAX_AGE_US ||
            uint32_t(now_us - auxiliary_launch_us_) < 18) { failFrame(); return false; }
        const Auxiliary next{ADC1->JDR1, ADC1->JDR2, ADC1->JDR3, auxiliary_launch_us_};
        if (next.bus > 4095 || next.temperature > 4095 || next.potentiometer > 4095) {
            failFrame(); return false;
        }
        latest_ = next;
        auxiliary_ready_ = true;
    } else if (have_frame_ || (ADC1->SR & ADC_SR_JEOC)) { failFrame(); return false; }
    // Admission is source-relative, including all copying/IRQ latency. Preserve
    // the long 144-cycle auxiliary apertures and leave 3 us before next CC4.
    // Never launch an injected conversion that could ordinarily preempt it.
    const uint32_t launch_us = time_us();
    if (uint32_t(launch_us - expected_trigger_us_) >= FRAME_PERIOD_US - AUXILIARY_BUDGET_US - 3 ||
        (ADC1->SR & ADC_SR_OVR) || (ADC1->CR2 & (ADC_CR2_SWSTART | ADC_CR2_JSWSTART))) {
        failFrame(); return false;
    }
    ADC1->SR = ~uint32_t(ADC_SR_JEOC | ADC_SR_JSTRT);
    auxiliary_launch_us_ = launch_us;
    auxiliary_generation_ = samples_ + 1;
    auxiliary_pending_ = true;
    // Constant command: do not RMW CR2 and replay a hardware-set SWSTART.
    // Only auxiliaries use CPU JSWSTART; no CPU launch of a second current triple.
    ADC1->CR2 = auxiliary_word_;
    for (unsigned phase = 0; phase < 3; ++phase) {
        phase_sum_[phase] = raw_[phase];
        if (INJECTED_SAMPLE_SUM == 2) phase_sum_[phase] += raw_[phase + 3];
    }
    reconstructed_phase_[0] = reconstructed_phase_[1] = 3;
    frame_started_us_ = expected_trigger_us_;
    armed_ = false;
    have_frame_ = true;
    ++samples_;
    return true;
}

bool Stm32Adc::rearmFrame(uint32_t now_us) noexcept {
    if (failed_ || armed_ || !have_frame_ || !(TIM1->CR1 & TIM_CR1_CEN) ||
        uint32_t(now_us - frame_started_us_) >= FRAME_PERIOD_US - 3 ||
        (uint32_t(now_us - auxiliary_launch_us_) > AUXILIARY_BUDGET_US && !(ADC1->SR & ADC_SR_JEOC)) ||
        (ADC1->SR & ADC_SR_OVR) || !nextTrigger(now_us)) { failFrame(); return false; }
    DMA2->LIFCR = RESULT_FLAGS;
    DMA2->HIFCR = TRIGGER_FLAGS;
    ADC1->SR = ~uint32_t(ADC_SR_EOC | ADC_SR_STRT);
    DMA2_Stream0->NDTR = FRAME_RANKS;
    DMA2_Stream4->NDTR = 1;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
    armed_ = true;
    SET_BIT(DMA2_Stream4->CR, DMA_SxCR_EN);
    return true;
}

unsigned Stm32Adc::currentInvalidPhases() const noexcept {
    const unsigned invalid = currentInvalidScans();
    return (invalid | (invalid >> 3)) & 7;
}

unsigned Stm32Adc::currentInvalidPhases(unsigned scan) const noexcept {
    return scan < INJECTED_SAMPLE_SUM ? (currentInvalidScans() >> (3 * scan)) & 7 : 7;
}

unsigned Stm32Adc::currentInvalidScans() const noexcept {
    constexpr unsigned all_invalid = INJECTED_SAMPLE_SUM == 2 ? 63 : 7;
    if (TIM1->RCR || (TIM1->CR1 & TIM_CR1_UDIS)) return all_invalid;
    // Datasheet timing, independent of IRQ scheduling: x1=3.24us, x2=6.48us.
    // Trigger-DMA/start latency is treated as negligible by this port contract;
    // admission keeps injected auxiliaries separate from the current scan.
    constexpr uint32_t rank_ticks = (15 + 12) * 4;
    constexpr uint32_t sample_ticks = 15 * 4;
    constexpr uint32_t settle_ticks = uint64_t(ihm08::TIMER_CLOCK_HZ) *
        (ihm08::CURRENT_SETTLING_NS + ihm08::DEAD_TIME_NS) / 1000000000;
    const PwmSamplingState::Compare compare{{TIM1->CCR1, TIM1->CCR2, TIM1->CCR3}};
    const uint32_t period = TIM1->ARR, trigger = TIM1->CCR4;
    const PwmSamplingState::Window window{{{armed_compare_[0], armed_compare_[1], armed_compare_[2]}},
        {{previous_compare_[0], previous_compare_[1], previous_compare_[2]}}, pwm_generation_, pwm_window_exact_};
    const bool exact = pwm_window_exact_ && pwm_sampling_state.matches(window, compare);
    if (pwm_window_exact_ && !pwm_sampling_state.known) return all_invalid;
    // Standalone/untracked users retain the previous conservative qualification.
    const bool legacy_peak = !pwm_window_exact_ && INJECTED_SAMPLE_SUM == 2 &&
        compare[0] == previous_compare_[0] && compare[1] == previous_compare_[1] &&
        compare[2] == previous_compare_[2];
    unsigned invalid = 0;
    for (unsigned phase = 0; phase < 3; ++phase) {
        uint32_t maximum = compare[phase] > armed_compare_[phase] ? compare[phase] : armed_compare_[phase];
        if (pwm_window_exact_ && !exact) {
            if (previous_compare_[phase] > maximum) maximum = previous_compare_[phase];
            if (pwm_sampling_state.max_since_arm[phase] > maximum) maximum = pwm_sampling_state.max_since_arm[phase];
        }
        // Qualify this physical rank before any x2 averaging. A pre-peak
        // aperture cannot reach the down-count closing edge for CCR < ARR;
        // only a proved peak reload may select the newer closing duty.
        const uint32_t opening_compare = exact ? armed_compare_[phase] : maximum;
        for (unsigned scan = 0; scan < INJECTED_SAMPLE_SUM; ++scan) {
            const uint32_t start = trigger + (3 * scan + phase) * rank_ticks;
            const uint32_t end = start + sample_ticks;
            const uint32_t closing_compare = exact ? previous_compare_[phase] :
                (legacy_peak && end >= period ? compare[phase] : maximum);
            if (maximum >= period || start <= opening_compare + settle_ticks ||
                2 * period <= end + closing_compare) invalid |= 1u << (3 * scan + phase);
        }
    }
    return invalid;
}

bool Stm32Adc::currentWindowValid(int reconstructed_phase) const noexcept {
    const unsigned invalid = currentInvalidPhases();
    if (!invalid) return true;
    if (reconstructed_phase < 0 || reconstructed_phase >= 3 ||
        invalid != (1u << reconstructed_phase)) return false;
    // Native duty ordering can differ inside one quantized timer count. Do not
    // guess on ties, nor infer FOC's private post-reset previous_ from CCRs.
    const uint32_t compare[3] = {TIM1->CCR1, TIM1->CCR2, TIM1->CCR3};
    for (unsigned phase = 0; phase < 3; ++phase)
        if (phase != unsigned(reconstructed_phase) && compare[phase] >= compare[reconstructed_phase]) return false;
    return true;
}

int Stm32Adc::serviceRegular(uint32_t now_us) noexcept {
    CriticalSection lock; // bounded copy: DMA IRQ cannot replace latest_ halfway through
    now_us = time_us(); // fresh after any IRQ preempting the caller's timestamp read
    if (failed_) return -1;
    if (!have_frame_) return 0;
    if (uint32_t(now_us - frame_started_us_) > REGULAR_MAX_AGE_US || samples_ == slow_generation_) return -1;
    slow_generation_ = samples_;
    if (!auxiliary_ready_) return 0;
    if (!regularFresh(now_us)) return -1;
    slow_ = latest_;
    return 1;
}

uint32_t Stm32Adc::getRawProtectionCurrent(PwmChannel phase, unsigned scan) const noexcept {
    const unsigned index = static_cast<unsigned>(phase);
    if (index >= 3 || scan >= INJECTED_SAMPLE_SUM) return 0;
#ifdef PWM_TIMING_BENCH
    return INJECTED_SAMPLE_SUM * f410_bench_inputs.phase_adc[index];
#else
    return INJECTED_SAMPLE_SUM * raw_[3 * scan + index];
#endif
}
uint32_t Stm32Adc::getRawAdcValue(PwmChannel phase) {
    const unsigned index = static_cast<unsigned>(phase);
#ifdef PWM_TIMING_BENCH
    return index < 3 ? INJECTED_SAMPLE_SUM * f410_bench_inputs.phase_adc[index] : 0;
#else
    return index < 3 ? phase_sum_[index] : 0;
#endif
}
void Stm32Adc::readNativeCurrents(NativeCurrent& u, NativeCurrent& v, NativeCurrent& w) noexcept {
    if (INJECTED_SAMPLE_SUM != 2 || reconstructed_phase_[0] > 2 || reconstructed_phase_[1] > 2) {
        AdcInterface::readNativeCurrents(u, v, w);
        return;
    }
    // Fixed Current is Q20/int32 (+/-2048 A). A reconstructed sample and the
    // sum of two triples may exceed that even when the final mean fits. Keep
    // both operations wide, then round/saturate only once after halving.
    using Wide = ActiveNumericProfile::Accumulator;
    static_assert(31 - FixedProfile::Current::FRACTION_BITS + 2 +
                  FixedProfile::Accumulator::FRACTION_BITS < 63, "Four retained currents must fit the accumulator");
    Wide total_u{}, total_v{}, total_w{};
    for (unsigned scan = 0; scan < 2; ++scan) {
        const unsigned discarded = reconstructed_phase_[scan];
        const auto read = [&](PwmChannel channel) {
            return numberCast<Wide>(
                convertNativeAdcToCurrent(getRawProtectionCurrent(channel, scan), channel));
        };
        Wide cu{}, cv{}, cw{};
        if (discarded == 0) {
            cv = read(PwmChannel::PHASE_V); cw = read(PwmChannel::PHASE_W); cu = -(cv + cw);
        } else if (discarded == 1) {
            cu = read(PwmChannel::PHASE_U); cw = read(PwmChannel::PHASE_W); cv = -(cu + cw);
        } else {
            cu = read(PwmChannel::PHASE_U); cv = read(PwmChannel::PHASE_V); cw = -(cu + cv);
        }
        total_u += cu; total_v += cv; total_w += cw;
    }
    u = divideByConstant<NativeCurrent, 2>(total_u);
    v = divideByConstant<NativeCurrent, 2>(total_v);
    w = divideByConstant<NativeCurrent, 2>(total_w);
}
uint32_t Stm32Adc::getRawAdcValue() {
#ifdef PWM_TIMING_BENCH
    return INJECTED_SAMPLE_SUM * f410_bench_inputs.bus_adc;
#else
    return INJECTED_SAMPLE_SUM * latest_.bus;
#endif
}
bool Stm32Adc::offsetsPlausible() const noexcept {
#ifdef PWM_TIMING_BENCH
    return samples_ != 0;
#else
    for (unsigned i = 0; i < 3; ++i)
        if (phase_sum_[i] < INJECTED_SAMPLE_SUM * 1600 || phase_sum_[i] > INJECTED_SAMPLE_SUM * 2600) return false;
    return samples_ != 0;
#endif
}
} // namespace libecu
