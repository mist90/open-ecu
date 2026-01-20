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

    /**
     * @brief Initialize ADC and OPAMP hardware
     *
     * Initializes ADC1, ADC2 in dual simultaneous injected mode and all 3 OPAMPs (PGA mode).
     * Configures ADC to be triggered by TIM1_TRGO2 for PWM-synchronized current sampling.
     *
     * @return true if initialization successful, false otherwise
     */
    bool initializeHardware();

    // AdcInterface implementation
    uint32_t getRawAdcValue(PwmChannel channel) override;

private:
    /**
     * @brief Initialize ADC1 peripheral
     * Configures ADC1 with injected channel for Phase U (OPAMP1)
     * and regular channel for potentiometer
     */
    void initADC1();

    /**
     * @brief Initialize ADC2 peripheral
     * Configures ADC2 with injected channels for Phase V (OPAMP2) and Phase W (OPAMP3)
     */
    void initADC2();

    /**
     * @brief Initialize OPAMP1 peripheral (Phase U current amplifier)
     */
    void initOPAMP1();

    /**
     * @brief Initialize OPAMP2 peripheral (Phase V current amplifier)
     */
    void initOPAMP2();

    /**
     * @brief Initialize OPAMP3 peripheral (Phase W current amplifier)
     */
    void initOPAMP3();
};

} // namespace libecu

#endif // LIBECU_STM32_ADC_HPP
