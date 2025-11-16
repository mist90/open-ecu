/**
 * @file stm32_adc.cpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 */

#include "stm32_adc.hpp"
#include "../../Core/Inc/main.h"

namespace libecu {

Stm32Adc::Stm32Adc(void* hadc, void* hdma)
    : hadc_(hadc)
    , hdma_(hdma)
    , calibration_()
    , initialized_(false)
    , adc_buffer_{0, 0, 0}
{
}

bool Stm32Adc::initialize(const CurrentSensorCalibration& calibration) {
    calibration_ = calibration;

#ifdef STM32G4
    ADC_HandleTypeDef* adc_handle = static_cast<ADC_HandleTypeDef*>(hadc_);

    // Calibrate ADC (should be done after clock and power are stable)
    if (HAL_ADCEx_Calibration_Start(adc_handle, ADC_SINGLE_ENDED) != HAL_OK) {
        return false;
    }

    // Start ADC with DMA for continuous 3-channel scanning
    if (HAL_ADC_Start_DMA(adc_handle, (uint32_t*)adc_buffer_, 3) != HAL_OK) {
        return false;
    }

    initialized_ = true;
    return true;
#else
    // Mock implementation for testing
    initialized_ = true;
    return true;
#endif
}

void Stm32Adc::startConversion() {
#ifdef STM32G4
    ADC_HandleTypeDef* adc_handle = static_cast<ADC_HandleTypeDef*>(hadc_);
    HAL_ADC_Start_DMA(adc_handle, (uint32_t*)adc_buffer_, 3);
#endif
}

bool Stm32Adc::isConversionComplete() {
#ifdef STM32G4
    ADC_HandleTypeDef* adc_handle = static_cast<ADC_HandleTypeDef*>(hadc_);
    return (HAL_ADC_GetState(adc_handle) & HAL_ADC_STATE_REG_EOC) != 0;
#else
    return true;
#endif
}

float Stm32Adc::convertAdcToCurrent(uint32_t adc_raw) {
    // Convert ADC raw value to voltage
    float adc_max_value = (1 << calibration_.adc_resolution_bits) - 1;  // e.g., 4095 for 12-bit
    float v_adc = (adc_raw * calibration_.adc_reference_voltage) / adc_max_value;

    // Remove offset (zero-current voltage)
    float v_shunt_amplified = v_adc - calibration_.offset_voltage;

    // Convert back to shunt voltage (before OPAMP)
    float v_shunt = v_shunt_amplified / calibration_.opamp_gain;

    // Calculate current using Ohm's law: I = V / R
    float current = v_shunt / calibration_.shunt_resistance_ohms;

    return current;
}

uint32_t Stm32Adc::getChannelIndex(PwmChannel channel) {
    switch (channel) {
        case PwmChannel::PHASE_U: return 0;
        case PwmChannel::PHASE_V: return 1;
        case PwmChannel::PHASE_W: return 2;
        default: return 0;
    }
}

float Stm32Adc::readPhaseCurrent(PwmChannel channel) {
    uint32_t index = getChannelIndex(channel);
    uint32_t adc_raw = adc_buffer_[index];
    return convertAdcToCurrent(adc_raw);
}

void Stm32Adc::readAllCurrents(float& i_u, float& i_v, float& i_w) {
    i_u = convertAdcToCurrent(adc_buffer_[0]);
    i_v = convertAdcToCurrent(adc_buffer_[1]);
    i_w = convertAdcToCurrent(adc_buffer_[2]);
}

uint32_t Stm32Adc::getRawAdcValue(PwmChannel channel) {
    uint32_t index = getChannelIndex(channel);
    return adc_buffer_[index];
}

bool Stm32Adc::calibrateZeroOffset() {
    if (!initialized_) {
        return false;
    }

    // Wait for stable readings
#ifdef STM32G4
    HAL_Delay(100);
#endif

    // Average multiple samples for better accuracy
    const uint32_t num_samples = 100;
    float sum_u = 0.0f, sum_v = 0.0f, sum_w = 0.0f;

    for (uint32_t i = 0; i < num_samples; i++) {
#ifdef STM32G4
        // Wait for new conversion
        HAL_Delay(1);
#endif
        float adc_max = (1 << calibration_.adc_resolution_bits) - 1;
        sum_u += (adc_buffer_[0] * calibration_.adc_reference_voltage) / adc_max;
        sum_v += (adc_buffer_[1] * calibration_.adc_reference_voltage) / adc_max;
        sum_w += (adc_buffer_[2] * calibration_.adc_reference_voltage) / adc_max;
    }

    // Calculate average voltage (should be around mid-supply, e.g., 1.65V)
    float avg_offset = (sum_u + sum_v + sum_w) / (3.0f * num_samples);
    calibration_.offset_voltage = avg_offset;

    return true;
}

const CurrentSensorCalibration& Stm32Adc::getCalibration() const {
    return calibration_;
}

} // namespace libecu
