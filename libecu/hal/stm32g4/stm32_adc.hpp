/**
 * @file stm32_adc.hpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 */

#ifndef LIBECU_STM32_ADC_HPP
#define LIBECU_STM32_ADC_HPP

#include "../../include/interfaces/adc_interface.hpp"

namespace libecu {

/**
 * @brief STM32G4 ADC implementation using ADC1 with DMA
 *
 * Reads 3 channels (OPAMP1/2/3 outputs) via DMA for efficient current sensing.
 * Supports PWM-synchronized sampling via TIM1 TRGO trigger.
 */
class Stm32Adc : public AdcInterface {
public:
    /**
     * @brief Constructor
     */
    explicit Stm32Adc();

    // AdcInterface implementation
    uint32_t getRawAdcValue(PwmChannel channel) override;

private:
    // DMA buffer for 3-channel ADC readings (Phase U, V, W)
    volatile uint32_t adc_buffer_[3];
};

} // namespace libecu

#endif // LIBECU_STM32_ADC_HPP
