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
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

namespace libecu {

Stm32Adc::Stm32Adc() noexcept {
}

bool Stm32Adc::initializeHardware() noexcept {
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
    sConfigInjected.InjectedNbrOfConversion = 2;  // 2 injected channels on ADC1 (Phase U + Vbus)
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
    sConfigInjectedVbus.InjectedNbrOfConversion = 2;  // 2 injected channels: Phase U + Vbus
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
    sConfigInjected.InjectedNbrOfConversion = 2;  // 2 injected channels on ADC2 (OPAMP2 + OPAMP3)
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
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP3_ADC2;  // OPAMP3_OUT internally connected
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
    // Vbus voltage is on ADC1_IN1, injected channel rank 2
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

} // namespace libecu
