/**
 * @file stm32_adc.cpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 * @note Uses ADC injected channels triggered by TIM1_TRGO2 for PWM-synchronized sampling
 *       ADC1 JDR1: Phase U (OPAMP1_OUT via VOPAMP1)
 *       ADC1 JDR2: Vbus voltage divider (PA0 = ADC1_IN1)
 *       ADC2 JDR1: Phase V (OPAMP2_OUT via VOPAMP2)
 *       ADC2 JDR2: Phase W (OPAMP3_OUT via VOPAMP3_ADC2)
 */

#include "stm32_adc.hpp"
#include "../../Core/Inc/main.h"

// Hardware handles (defined in main.cpp)
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

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

    /* Analog inputs: PA0(Vbus), PA1(OPAMP1), PA3, PA4(BEMF_U), PA5, PA7(OPAMP2) */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Analog inputs: PB0(OPAMP3), PB2, PB11(BEMF_W), PB12(Pot) */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Analog input: PC4(BEMF_V) */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PB5: BEMF divider mode control (output, default LOW = through divider) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

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
    ADC_InjOversamplingTypeDef sConfigOversampling = {.Ratio = ADC_OVERSAMPLING_RATIO_2, .RightBitShift = ADC_RIGHTBITSHIFT_1};

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
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
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

    /** Configure Regular Channel for Potentiometer (PB12 = ADC1_IN11)
     */
    sConfig.Channel = ADC_CHANNEL_11;  // PB12 potentiometer input
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;  // Slower sampling for stable reading
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
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
    sConfigInjected.InjectedNbrOfConversion = 3;  // 3 injected channels on ADC1 (Phase U current + Vbus + Phase W voltage)
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

    /** Configure Injected Channel 2: Vbus voltage divider (PA0 = ADC1_IN1)
     */
    ADC_InjectionConfTypeDef sConfigInjectedVbus = {0};
    sConfigInjectedVbus.InjectedChannel = ADC_CHANNEL_1;
    sConfigInjectedVbus.InjectedRank = ADC_INJECTED_RANK_2;
    sConfigInjectedVbus.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    sConfigInjectedVbus.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjectedVbus.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjectedVbus.InjectedOffset = 0;
    sConfigInjectedVbus.InjectedNbrOfConversion = 3;  // 3 injected channels: Phase U current + Vbus + Phase W voltage
    sConfigInjectedVbus.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjectedVbus.AutoInjectedConv = DISABLE;
    sConfigInjectedVbus.QueueInjectedContext = DISABLE;
    sConfigInjectedVbus.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
    sConfigInjectedVbus.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjectedVbus.InjecOversamplingMode = ENABLE;
    sConfigInjectedVbus.InjecOversampling = sConfigOversampling;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjectedVbus) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel 3: Phase W voltage BEMF (PB11 = ADC1_IN14)
     */
    ADC_InjectionConfTypeDef sConfigInjectedBemfW = {0};
    sConfigInjectedBemfW.InjectedChannel = ADC_CHANNEL_14;
    sConfigInjectedBemfW.InjectedRank = ADC_INJECTED_RANK_3;
    sConfigInjectedBemfW.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    sConfigInjectedBemfW.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjectedBemfW.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjectedBemfW.InjectedOffset = 0;
    sConfigInjectedBemfW.InjectedNbrOfConversion = 3;
    sConfigInjectedBemfW.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjectedBemfW.AutoInjectedConv = DISABLE;
    sConfigInjectedBemfW.QueueInjectedContext = DISABLE;
    sConfigInjectedBemfW.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
    sConfigInjectedBemfW.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjectedBemfW.InjecOversamplingMode = ENABLE;
    sConfigInjectedBemfW.InjecOversampling = sConfigOversampling;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjectedBemfW) != HAL_OK)
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
    ADC_InjOversamplingTypeDef sConfigOversampling = {.Ratio = ADC_OVERSAMPLING_RATIO_2, .RightBitShift = ADC_RIGHTBITSHIFT_1};
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP2;  // OPAMP2_OUT internally connected
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;  // Match ADC1 timing
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedNbrOfConversion = 4;  // 4 injected channels on ADC2 (V current + W current + U voltage + V voltage)
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

    /** Configure Injected Channel 3: Phase U voltage BEMF (PA4 = ADC2_IN17)
     */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_17;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel 4: Phase V voltage BEMF (PC4 = ADC2_IN5)
     */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
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
    // Vbus voltage is on ADC1_IN1, injected channel rank 2
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

uint32_t Stm32Adc::getRawPhaseVoltage(PwmChannel channel) {
    switch (channel) {
        case PwmChannel::PHASE_U:
            return HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);  // PA4, ADC2_IN17
        case PwmChannel::PHASE_V:
            return HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);  // PC4, ADC2_IN5
        case PwmChannel::PHASE_W:
            return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);  // PB11, ADC1_IN14
        default:
            return 0;
    }
}

void Stm32Adc::setBemfDividerMode(bool direct_mode) noexcept {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                      direct_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

float Stm32Adc::readPotentiometer(float max_value)
{
    // Start ADC regular conversion
    HAL_ADC_Start(&hadc1);

    // Wait for conversion complete (should be very fast)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

        // Convert ADC value (0-4095) to (0-max_value)
        // Linear mapping: output = (adc_value / 4095.0) * max_value
        float ret_value = (static_cast<float>(adc_value) / 4095.0f) * max_value;

        return ret_value;
    }

    // If conversion failed, return 0
    return 0.0f;
}

} // namespace libecu
