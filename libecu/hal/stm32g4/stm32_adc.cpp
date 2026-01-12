/**
 * @file stm32_adc.cpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 * @note Uses ADC injected channels triggered by TIM1_TRGO2 for PWM-synchronized sampling
 *       ADC1 JDR1: Phase U (OPAMP1_OUT via VOPAMP1)
 *       ADC1 JDR2: Phase W (OPAMP3_OUT via ADC1_IN12)
 *       ADC2 JDR1: Phase V (OPAMP2_OUT via VOPAMP2)
 */

#include "stm32_adc.hpp"
#include "../../Core/Inc/main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

namespace libecu {

Stm32Adc::Stm32Adc() : adc_buffer_{0, 0, 0}
{
}

uint32_t Stm32Adc::getRawAdcValue(PwmChannel channel) {
    switch (channel) {
        case PwmChannel::PHASE_U:
            return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        case PwmChannel::PHASE_V:
            return HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        case PwmChannel::PHASE_W:
            return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        default:
            return 0;
    }
}

} // namespace libecu
