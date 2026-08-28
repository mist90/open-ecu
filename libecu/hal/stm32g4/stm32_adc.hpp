/**
 * @file stm32_adc.hpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 */

#ifndef LIBECU_STM32_ADC_HPP
#define LIBECU_STM32_ADC_HPP

#include "../../include/adc_interface.hpp"

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
    explicit Stm32Adc() noexcept;

    /**
     * @brief Initialize ADC and OPAMP hardware
     *
     * Initializes ADC1, ADC2 in dual simultaneous injected mode and all 3 OPAMPs (PGA mode).
     * Configures ADC to be triggered by TIM1_TRGO2 for PWM-synchronized current sampling.
     *
     * @return true if initialization successful, false otherwise
     */
    bool initializeHardware() noexcept;

    // AdcInterface implementation
    uint32_t getRawAdcValue(PwmChannel channel) override;
    uint32_t getRawAdcValue() override;
    uint32_t getRawTemperatureAdcValue() override;

    /**
     * @brief Read potentiometer and convert to output
     * @param max_value Maximum speed corresponding to 3.3V
     * @return output (0 to max_value)
     */
    float readPotentiometer(float max_value);

private:
    /**
     * @brief Run one regular conversion on ADC1 and return the result
     *
     * ADC1's regular group is shared between the potentiometer (PB12) and the
     * NTC (PB14), one channel at a time: the sequence is a single conversion
     * long, and this points it at the requested channel before starting it.
     * That is deliberate - see the implementation for why a two-rank sequence
     * is the wrong shape here.
     *
     * @param channel HAL/LL channel identifier (ADC_CHANNEL_x)
     * @return Raw conversion result, or 0 if the conversion did not complete
     */
    uint32_t readRegularChannel(uint32_t channel) noexcept;

    /**
     * @brief Initialize ADC1 peripheral
     * Configures ADC1 with injected channel for Phase U (OPAMP1)
     * and regular channel for potentiometer
     */
    void initADC1() noexcept;

    /**
     * @brief Initialize ADC2 peripheral
     * Configures ADC2 with injected channels for Phase V (OPAMP2) and Phase W (OPAMP3)
     */
    void initADC2() noexcept;

    /**
     * @brief Initialize OPAMP1 peripheral (Phase U current amplifier)
     */
    void initOPAMP1() noexcept;

    /**
     * @brief Initialize OPAMP2 peripheral (Phase V current amplifier)
     */
    void initOPAMP2() noexcept;

    /**
     * @brief Initialize OPAMP3 peripheral (Phase W current amplifier)
     */
    void initOPAMP3() noexcept;
};

} // namespace libecu

#endif // LIBECU_STM32_ADC_HPP
