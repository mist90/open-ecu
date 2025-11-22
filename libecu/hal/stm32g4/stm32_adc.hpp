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
     * @param hadc ADC handle (ADC1) - pass as void* to avoid header dependencies
     * @param hdma DMA handle for ADC - pass as void* to avoid header dependencies
     */
    explicit Stm32Adc(void* hadc, void* hdma = nullptr);

    // AdcInterface implementation
    bool initialize(const CurrentSensorCalibration& calibration) override;
    void startConversion() override;
    bool isConversionComplete() override;
    float readPhaseCurrent(PwmChannel channel) override;
    void readAllCurrents(float& i_u, float& i_v, float& i_w) override;
    uint32_t getRawAdcValue(PwmChannel channel) override;
    bool calibrateZeroOffset() override;
    const CurrentSensorCalibration& getCalibration() const override;

private:
    void* hadc_;
    void* hdma_;
    CurrentSensorCalibration calibration_;
    bool initialized_;

    // DMA buffer for 3-channel ADC readings (Phase U, V, W)
    volatile uint32_t adc_buffer_[3];

    // Zero-current offset voltages for each phase (calibrated)
    float offset_voltage_u_;
    float offset_voltage_v_;
    float offset_voltage_w_;

    /**
     * @brief Convert raw ADC value to current in Amperes
     * @param adc_raw Raw ADC reading (0 to 4095 for 12-bit)
     * @param channel PWM channel to use correct offset
     * @return Current in Amperes
     */
    float convertAdcToCurrent(uint32_t adc_raw, PwmChannel channel);

    /**
     * @brief Get ADC channel index for PWM channel
     * @param channel PWM channel (PHASE_U/V/W)
     * @return Array index (0/1/2)
     */
    uint32_t getChannelIndex(PwmChannel channel);
};

} // namespace libecu

#endif // LIBECU_STM32_ADC_HPP
