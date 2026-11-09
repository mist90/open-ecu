/**
 * @file stm32_adc.cpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 * @note One TIM1_TRGO2 edge independently triggers BOTH ADCs:
 *       ADC1: U[,U],Vbus,NTC,throttle -> two-bank circular DMA1 channel1.
 *       ADC2: V,W[,V,W] -> normal DMA1 channel2, the only normal-path IRQ.
 */

#include "stm32_adc.hpp"
#include "main.h"  // board pin map, supplied by the port's include path
#include "critical_section.hpp"

// Hardware handles (defined in main.cpp)
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;
static DMA_HandleTypeDef adc1_dma, adc2_dma;

namespace libecu {

bool Stm32Adc::initializeHardware() noexcept {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Analog inputs: PB0(OPAMP3), PB2, PB12(Pot), PB14(NTC) */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize OPAMPs first (analog front-end)
    initOPAMP1();
    initOPAMP2();
    initOPAMP3();

    // Start OPAMPs for current sensing
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) {
        return false;
    }
    if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) {
        return false;
    }
    if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
        return false;
    }

    // Initialize ADC peripherals
    initADC1();
    initADC2();

    // Calibrate ADCs (must be done before starting conversions)
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        return false;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        return false;
    }

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    adc1_dma.Instance = DMA1_Channel1;
    adc1_dma.Init.Request = DMA_REQUEST_ADC1;
    adc1_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    adc1_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    adc1_dma.Init.MemInc = DMA_MINC_ENABLE;
    adc1_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    adc1_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    adc1_dma.Init.Mode = DMA_CIRCULAR;
    adc1_dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    adc2_dma.Instance = DMA1_Channel2;
    adc2_dma.Init = adc1_dma.Init;
    adc2_dma.Init.Request = DMA_REQUEST_ADC2;
    adc2_dma.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&adc1_dma) != HAL_OK || HAL_DMA_Init(&adc2_dma) != HAL_OK) return false;
    __HAL_LINKDMA(&hadc1, DMA_Handle, adc1_dma);
    __HAL_LINKDMA(&hadc2, DMA_Handle, adc2_dma);
    // TIM1 is stopped. HAL only enables/arms the external triggers here; no
    // software conversion or runtime sequence change is needed on either ADC.
    if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(adc1_dma_)),
                          ADC1_DMA_LENGTH) != HAL_OK ||
        HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(adc2_dma_)),
                          CURRENT_RANKS) != HAL_OK) return false;
    __HAL_DMA_DISABLE_IT(&adc1_dma, DMA_IT_HT | DMA_IT_TC); // flags still retain bank provenance
    __HAL_DMA_DISABLE_IT(&adc2_dma, DMA_IT_HT);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    return true;
}

void Stm32Adc::initADC1() noexcept {
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;  // EOC after sequence
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = ADC1_RANKS;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    // Independent lengths/sample times: dual CDR would make V2/W2 wait for
    // ADC1's long auxiliary ranks. The common PWM edge synchronizes the starts.
    multimode.Mode = ADC_MODE_INDEPENDENT;
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    const uint32_t ranks[] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3,
                             ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5, ADC_REGULAR_RANK_6,
                             ADC_REGULAR_RANK_7};
    const uint32_t auxiliary[] = {ADC_CHANNEL_1, ADC_CHANNEL_5, ADC_CHANNEL_11};
    for (unsigned i = 0; i < ADC1_RANKS; ++i) {
        sConfig.Rank = ranks[i];
        sConfig.Channel = i < INJECTED_SAMPLE_SUM ? ADC_CHANNEL_VOPAMP1 : auxiliary[i - INJECTED_SAMPLE_SUM];
        sConfig.SamplingTime = i < INJECTED_SAMPLE_SUM ? ADC_SAMPLETIME_6CYCLES_5 : ADC_SAMPLETIME_92CYCLES_5;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    }

}

void Stm32Adc::initADC2() noexcept {
    /** Common config
     */
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation = 0;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = CURRENT_RANKS;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    ADC_ChannelConfTypeDef channel{};
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    const uint32_t ranks[] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3,
                             ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5, ADC_REGULAR_RANK_6,
                             ADC_REGULAR_RANK_7};
    for (unsigned i = 0; i < CURRENT_RANKS; ++i) {
        channel.Rank = ranks[i];
        channel.Channel = i % 2 ? ADC_CHANNEL_VOPAMP3_ADC2 : ADC_CHANNEL_VOPAMP2;
        channel.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
        if (HAL_ADC_ConfigChannel(&hadc2, &channel) != HAL_OK) Error_Handler();
    }
}

void Stm32Adc::initOPAMP1() noexcept {
    hopamp1.Instance = OPAMP1;
    hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
    hopamp1.Init.Mode = OPAMP_PGA_MODE;
    hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;  // Not used in PGA mode
    hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
    hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    hopamp1.Init.InternalOutput = ENABLE;
    hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp1.Init.InvertingInputSecondary = OPAMP_SEC_INVERTINGINPUT_IO0;
    hopamp1.Init.NonInvertingInputSecondary = OPAMP_SEC_NONINVERTINGINPUT_IO0;
    hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    hopamp1.Init.TrimmingValueP = 0;
    hopamp1.Init.TrimmingValueN = 0;
    if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
    {
        Error_Handler();
    }
}

void Stm32Adc::initOPAMP2() noexcept {
    hopamp2.Instance = OPAMP2;
    hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
    hopamp2.Init.Mode = OPAMP_PGA_MODE;
    hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;  // Not used in PGA mode
    hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
    hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    hopamp2.Init.InternalOutput = ENABLE;
    hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp2.Init.InvertingInputSecondary = OPAMP_SEC_INVERTINGINPUT_IO0;
    hopamp2.Init.NonInvertingInputSecondary = OPAMP_SEC_NONINVERTINGINPUT_IO0;
    hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    hopamp2.Init.TrimmingValueP = 0;
    hopamp2.Init.TrimmingValueN = 0;
    if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Stm32Adc::initOPAMP3() noexcept {
    hopamp3.Instance = OPAMP3;
    hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
    hopamp3.Init.Mode = OPAMP_PGA_MODE;
    hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;  // Not used in PGA mode
    hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
    hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    hopamp3.Init.InternalOutput = ENABLE;
    hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp3.Init.InvertingInputSecondary = OPAMP_SEC_INVERTINGINPUT_IO0;
    hopamp3.Init.NonInvertingInputSecondary = OPAMP_SEC_NONINVERTINGINPUT_IO0;
    hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    hopamp3.Init.TrimmingValueP = 0;
    hopamp3.Init.TrimmingValueN = 0;
    if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
    {
        Error_Handler();
    }
}

uint32_t Stm32Adc::getRawAdcValue(PwmChannel channel) {
    const unsigned phase = static_cast<unsigned>(channel);
    return phase < 3 ? phase_sum_[phase] : 0;
}

uint32_t Stm32Adc::getRawAdcValue() {
    return INJECTED_SAMPLE_SUM * latest_.bus;
}

uint32_t Stm32Adc::getRawProtectionCurrent(PwmChannel channel, unsigned scan) const noexcept {
    if (scan >= INJECTED_SAMPLE_SUM) return 0;
    const unsigned phase = static_cast<unsigned>(channel);
    return phase < 3 ? INJECTED_SAMPLE_SUM * raw_[3 * scan + phase] : 0;
}

bool Stm32Adc::nextTrigger(uint32_t now_us) noexcept {
    const uint32_t control = TIM1->CR1, count = TIM1->CNT, period = TIM1->ARR;
    if (TIM1->PSC || !period || count > period || TIM1->CCR4 >= period ||
        (control & TIM_CR1_CMS) != TIM_COUNTERMODE_CENTERALIGNED1 ||
        ((control ^ TIM1->CR1) & TIM_CR1_DIR)) return false;
    const uint32_t phase = control & TIM_CR1_DIR ? 2 * period - count : count;
    // PWM1 OC4REF rises on the DOWN-counting compare, just after the peak.
    const uint32_t trigger = 2 * period - TIM1->CCR4;
    const uint32_t distance = trigger > phase ? trigger - phase : 2 * period - phase + trigger;
    expected_trigger_us_ = now_us + distance / (SystemCoreClock / 1000000);
    return true;
}

bool Stm32Adc::beginFrames(uint32_t now_us) noexcept {
    if (failed_ || !nextTrigger(now_us)) { failFrame(); return false; }
    armed_ = true;
    return true;
}

void Stm32Adc::failFrame() noexcept {
    failed_ = true;
    armed_ = false;
    CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    CLEAR_BIT(ADC1->IER, ADC_IER_OVRIE);
    CLEAR_BIT(ADC2->IER, ADC_IER_OVRIE);
    // ADC stop is asynchronous; never HAL-stop/poll a converter from an IRQ.
    // Keeping the failed DMA disabled also prevents a stale buffer publication.
    if (ADC1->CR & ADC_CR_ADSTART) SET_BIT(ADC1->CR, ADC_CR_ADSTP);
    if (ADC2->CR & ADC_CR_ADSTART) SET_BIT(ADC2->CR, ADC_CR_ADSTP);
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
}

namespace {
struct BankProgress { uint32_t flags, written; };
bool adc1Progress(unsigned bank, bool previous, BankProgress& result) noexcept {
    constexpr uint32_t length = Stm32Adc::ADC1_RANKS;
    constexpr uint32_t flags_mask = DMA_ISR_HTIF1 | DMA_ISR_TCIF1 | DMA_ISR_TEIF1;
    uint32_t flags = DMA1->ISR & flags_mask;
    uint32_t remaining = DMA1_Channel1->CNDTR;
    const uint32_t flags_after = DMA1->ISR & flags_mask;
    if (flags != flags_after) {
        // One bounded resnapshot if a bank boundary coincided with the reads.
        // Never spin on a DMA producer, and never clear a boundary not consumed.
        flags = flags_after;
        remaining = DMA1_Channel1->CNDTR;
        if ((DMA1->ISR & flags_mask) != flags) return false;
    }
    if (remaining > 2 * length || (flags & DMA_ISR_TEIF1) ||
        (DMA1_Channel1->CCR & (DMA_CCR_EN | DMA_CCR_CIRC)) != (DMA_CCR_EN | DMA_CCR_CIRC) ||
        (ADC1->CFGR & (ADC_CFGR_CONT | ADC_CFGR_DISCEN))) return false;
    uint32_t written;
    if (!bank) {
        if (remaining < length) return false;
        written = 2 * length - remaining;
    } else {
        if (remaining == 2 * length) written = length; // circular reload after TC
        else {
            if (remaining > length) return false;
            written = length - remaining;
        }
    }
    const uint32_t current_flag = bank ? DMA_ISR_TCIF1 : DMA_ISR_HTIF1;
    const uint32_t previous_flag = bank ? DMA_ISR_HTIF1 : DMA_ISR_TCIF1;
    if (written < Stm32Adc::INJECTED_SAMPLE_SUM || written > length ||
        bool(flags & previous_flag) != previous ||
        bool(flags & current_flag) != (written == length)) return false;
    result = {flags, written};
    return true;
}
}

bool Stm32Adc::captureFrame(uint32_t now_us) noexcept {
    const uint32_t flags = DMA1->ISR;
    const uint32_t age = now_us - expected_trigger_us_;
    if (failed_ || !armed_ || !(TIM1->CR1 & TIM_CR1_CEN) ||
        (flags & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) || !(flags & DMA_ISR_TCIF2) ||
        DMA1_Channel2->CNDTR || (ADC1->ISR & ADC_ISR_OVR) || (ADC2->ISR & ADC_ISR_OVR) ||
        !(ADC2->ISR & ADC_ISR_EOS) || age >= FRAME_PERIOD_US - 3) {
        failFrame(); return false;
    }
    const unsigned bank = samples_ & 1u;
    const bool previous = have_frame_;
    BankProgress before{}, after{};
    if (!adc1Progress(bank, previous, before)) { failFrame(); return false; }
    // ADC2 is stable at normal TC. ADC1 continues writing its auxiliary tail.
    // IRQ masking could NOT protect this buffer: source epoch + progress flags
    // and checks on both sides of the copy protect current U and the past bank.
    CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    for (unsigned scan = 0; scan < INJECTED_SAMPLE_SUM; ++scan) {
        raw_[3 * scan] = adc1_dma_[bank * ADC1_RANKS + scan];
        raw_[3 * scan + 1] = adc2_dma_[2 * scan];
        raw_[3 * scan + 2] = adc2_dma_[2 * scan + 1];
    }
    Auxiliary next{};
    if (previous) {
        const unsigned base = (bank ^ 1u) * ADC1_RANKS + INJECTED_SAMPLE_SUM;
        next = {adc1_dma_[base], adc1_dma_[base + 1], adc1_dma_[base + 2], frame_started_us_};
    }
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    const uint32_t after_us = time_us();
    if (uint32_t(after_us - expected_trigger_us_) >= FRAME_PERIOD_US - 3 ||
        !adc1Progress(bank, previous, after) || after.written < before.written ||
        (DMA1->ISR & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) ||
        ((ADC1->ISR | ADC2->ISR) & ADC_ISR_OVR)) { failFrame(); return false; }
    for (uint16_t value : raw_) if (value > 4095) { failFrame(); return false; }
    if (previous && (next.bus > 4095 || next.temperature > 4095 || next.potentiometer > 4095 ||
        uint32_t(after_us - next.started_us) > REGULAR_MAX_AGE_US)) { failFrame(); return false; }
    for (unsigned phase = 0; phase < 3; ++phase) {
        uint32_t sum = 0;
        for (unsigned scan = 0; scan < INJECTED_SAMPLE_SUM; ++scan)
            sum += raw_[3 * scan + phase];
        phase_sum_[phase] = sum;
    }
    if (previous) {
        latest_ = next;
        auxiliary_ready_ = true;
        // Consume ONLY the previous bank's completion. If ADC1's current tail
        // finished during the copy, its distinct boundary flag survives.
        DMA1->IFCR = bank ? DMA_IFCR_CHTIF1 : DMA_IFCR_CTCIF1;
    }
    frame_started_us_ = expected_trigger_us_;
    armed_ = false;
    have_frame_ = true;
    ++samples_;
    return true;
}

bool Stm32Adc::rearmFrame(uint32_t now_us) noexcept {
    if (failed_ || armed_ || !have_frame_ || !(TIM1->CR1 & TIM_CR1_CEN) ||
        uint32_t(now_us - frame_started_us_) >= FRAME_PERIOD_US - 3 ||
        (DMA1->ISR & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) ||
        (ADC1->ISR & ADC_ISR_OVR) || (ADC2->ISR & ADC_ISR_OVR) || !nextTrigger(now_us)) {
        failFrame(); return false;
    }
    ADC2->ISR = ADC_ISR_EOS | ADC_ISR_EOC;
    DMA1->IFCR = DMA_IFCR_CGIF2;
    DMA1_Channel2->CNDTR = CURRENT_RANKS;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    armed_ = true;
    SET_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    // External ADSTART remains armed across non-continuous sequences (RM0440).
    // No SW start or trigger configuration changes in the completion IRQ.
    return true;
}

int Stm32Adc::serviceRegular(uint32_t now_us) noexcept {
    CriticalSection lock;
    // The DMA IRQ may have preempted the caller after it read now_us.
    now_us = time_us();
    if (failed_) return -1;
    if (!have_frame_) return 0;
    if (uint32_t(now_us - frame_started_us_) > REGULAR_MAX_AGE_US || samples_ == slow_generation_) return -1;
    slow_generation_ = samples_;
    if (!auxiliary_ready_) return 0;
    if (!regularFresh(now_us)) return -1;
    slow_ = latest_;
    return 1;
}

float Stm32Adc::readPotentiometer(float max_value)
{
    using Speed = ActiveNumericProfile::Speed;
    return numberCast<float>(multiplyTo<Speed>(convertNativeThrottle(readRawPotentiometer()),
                                              numberCast<Speed>(max_value)));
}

} // namespace libecu
