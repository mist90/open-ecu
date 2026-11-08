/**
 * @file stm32_adc.cpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 * @note Uses ADC injected channels triggered by TIM1_TRGO2 for PWM-synchronized sampling
 *       ADC1 JDR1: Phase U current (OPAMP1_OUT via VOPAMP1)
 *       ADC1 JDR2: Phase U padding, keeping both injected sequences equal
 *       ADC2 JDR1: Phase V current (OPAMP2_OUT via VOPAMP2)
 *       ADC2 JDR2: Phase W current (OPAMP3_OUT via VOPAMP3_ADC2)
 *
 *       ADC1 regular DMA: Vbus (PA0 = IN1), NTC (PB14 = IN5), pot (PB12 = IN11).
 *       SysTick at 1 kHz is the sole owner of scan launch and publication.
 */

#include "stm32_adc.hpp"
#include "main.h"  // board pin map, supplied by the port's include path

// Hardware handles (defined in main.cpp)
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;
static DMA_HandleTypeDef regular_dma;

namespace libecu {

Stm32Adc::Stm32Adc() noexcept {
}

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

    /* Analog inputs: PA0(Vbus), PA1(OPAMP1), PA3, PA5, PA7(OPAMP2) */
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
    regular_dma.Instance = DMA1_Channel1;
    regular_dma.Init.Request = DMA_REQUEST_ADC1;
    regular_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    regular_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    regular_dma.Init.MemInc = DMA_MINC_ENABLE;
    regular_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    regular_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    regular_dma.Init.Mode = DMA_CIRCULAR;
    regular_dma.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&regular_dma) != HAL_OK ||
        HAL_DMA_Start(&regular_dma, uint32_t(uintptr_t(&ADC1->DR)),
                      uint32_t(uintptr_t(regular_dma_)), 3) != HAL_OK) return false;
    // Only injected conversions are dual-mode. Regular ADC1 is independent
    // (RM0440, injected simultaneous mode), with its own DR DMA request.
    SET_BIT(ADC1->CFGR, ADC_CFGR_DMAEN);

    // Start ADC injected conversions (triggered by TIM1_TRGO2)
    // In dual-mode simultaneous: START SLAVE FIRST, THEN MASTER
    // ADC2 (slave): Phase V (VOPAMP2) + Phase W (VOPAMP3_ADC2)
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) {
        return false;
    }

    // ADC1 (master): Phase U (VOPAMP1)
    // Starting master will trigger both ADCs simultaneously
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
        return false;
    }

    return true;
}

void Stm32Adc::initADC1() noexcept {
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    // Keep both samples' sum. Board calibration accounts for the factor of two,
    // preserving half-LSB current information at the original sampling latency.
    static_assert(INJECTED_SAMPLE_SUM == 2, "Update the hardware oversampling ratio with the calibration scale");
    ADC_InjOversamplingTypeDef sConfigOversampling = {
        .Ratio = ADC_OVERSAMPLING_RATIO_2, .RightBitShift = ADC_RIGHTBITSHIFT_NONE};

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
    hadc1.Init.NbrOfConversion = 3;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure the ADC multi-mode for simultaneous injected conversions
     */
    multimode.Mode = ADC_DUALMODE_INJECSIMULT;  // Simultaneous injected mode
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    // Fixed three-rank regular scan. DMA drains every DR result even when
    // the current IRQ preempts SysTick, so no software read can lose a rank.
    // 92.5 cycles at 42.5 MHz also gives the high-impedance Vbus divider time
    // to settle. Configure SMPR/SQR here, before arming injected conversions.
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    sConfig.Channel = ADC_CHANNEL_1;   // PA0 Vbus
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.Channel = ADC_CHANNEL_5;   // PB14 NTC thermistor input
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Rank = ADC_REGULAR_RANK_3;
    sConfig.Channel = ADC_CHANNEL_11;  // PB12 potentiometer input
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel 1: Phase U current (OPAMP1)
     */
    ADC_InjectionConfTypeDef sConfigInjected = {0};
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP1;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;  // Fast sampling for PWM frequency
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedNbrOfConversion = 2;  // Phase U + U padding, matching ADC2's V/W
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.QueueInjectedContext = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;  // TIM1 TRGO2 trigger
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode = ENABLE;
    sConfigInjected.InjecOversampling = sConfigOversampling;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    // RM0440 Rev7 p666: equal sequence lengths (or a trigger interval longer
    // than both sequences) are required in simultaneous mode. Equal duration
    // permits one completion point for both ADCs. Keep U/U versus V/W so the
    // existing current synchronization and x2 timing remain exactly the same;
    // the padding result is never consumed and Vbus is regular-only.
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
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
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;  // Enable scan for 2 injected channels (OPAMP2 + OPAMP3)
    hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel 1: Phase V current (OPAMP2)
     */
    ADC_InjectionConfTypeDef sConfigInjected = {0};
    static_assert(INJECTED_SAMPLE_SUM == 2, "Update the hardware oversampling ratio with the calibration scale");
    ADC_InjOversamplingTypeDef sConfigOversampling = {
        .Ratio = ADC_OVERSAMPLING_RATIO_2, .RightBitShift = ADC_RIGHTBITSHIFT_NONE};
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP2;  // OPAMP2_OUT internally connected
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;  // Match ADC1 timing
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedNbrOfConversion = 2;  // 2 injected channels on ADC2 (V current + W current)
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.QueueInjectedContext = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;  // TIM1 TRGO2 trigger
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode = ENABLE;
    sConfigInjected.InjecOversampling = sConfigOversampling;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel 2: Phase W current (OPAMP3)
     */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP3_ADC2;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
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
    switch (channel) {
        case PwmChannel::PHASE_U:
            return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        case PwmChannel::PHASE_V:
            return HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        case PwmChannel::PHASE_W:
            return HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
        default:
            return 0;
    }
}

uint32_t Stm32Adc::getRawAdcValue() {
    // Preserve the public current/bus sum scale; regular DMA is physical 12-bit.
    return INJECTED_SAMPLE_SUM * regularFrame().bus;
}

uint32_t Stm32Adc::getRawTemperatureAdcValue() {
    return regularFrame().temperature;
}

bool Stm32Adc::serviceRegular(uint32_t now_us) noexcept {
    const uint32_t dma_flags = DMA1->ISR;
    if ((dma_flags & DMA_ISR_TEIF1) || (ADC1->ISR & ADC_ISR_OVR) ||
        !(DMA1_Channel1->CCR & DMA_CCR_EN)) regular_failed_ = true;
    if (regular_failed_) return false;

    if (regular_pending_) {
        if (!(dma_flags & DMA_ISR_TCIF1)) {
            // Next scheduled SysTick: timestamp deltas can be slightly below
            // 1 ms because of IRQ jitter, so the deadline is the call itself.
            regular_failed_ = true;
            return false;
        }
        if ((ADC1->CR & ADC_CR_ADSTART) || DMA1_Channel1->CNDTR != 3 ||
            uint32_t(now_us - regular_started_us_) > REGULAR_MAX_AGE_US) {
            regular_failed_ = true;
            return false;
        }
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const uint32_t next = regular_index_ ^ 1U;
        regular_frames_[next] = {regular_dma_[0], regular_dma_[1], regular_dma_[2], regular_started_us_};
        __atomic_store_n(&regular_index_, next, __ATOMIC_RELEASE);
        regular_ready_ = true;
    }

    DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1;
    ADC1->ISR = ADC_ISR_EOS | ADC_ISR_EOC;
    regular_started_us_ = now_us;
    regular_pending_ = true;
    // ADCs are already enabled by injected-start during initialization. No HAL
    // enable/poll/stop path belongs in SysTick; this just requests a new scan.
    SET_BIT(ADC1->CR, ADC_CR_ADSTART);
    return true;
}

float Stm32Adc::readPotentiometer(float max_value)
{
    using Speed = ActiveNumericProfile::Speed;
    return numberCast<float>(multiplyTo<Speed>(convertNativeThrottle(readRawPotentiometer()),
                                              numberCast<Speed>(max_value)));
}

uint32_t Stm32Adc::readRawPotentiometer() {
    return regularFrame().potentiometer;
}

} // namespace libecu
