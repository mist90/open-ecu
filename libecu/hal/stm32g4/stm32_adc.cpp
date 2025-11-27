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

Stm32Adc::Stm32Adc(void* hadc, void* hdma)
    : hadc_(hadc)
    , hdma_(hdma)
    , calibration_()
    , initialized_(false)
    , adc_buffer_{0, 0, 0}
    , offset_voltage_u_(0.0f)
    , offset_voltage_v_(0.0f)
    , offset_voltage_w_(0.0f)
{
}

bool Stm32Adc::initialize(const CurrentSensorCalibration& calibration) {
    calibration_ = calibration;

    // ADC calibration and injected channel startup is done in main.cpp
    // before calling this function. Here we just store the calibration.
    initialized_ = true;
    return true;
}

void Stm32Adc::startConversion() {
    ADC_HandleTypeDef* adc_handle = static_cast<ADC_HandleTypeDef*>(hadc_);
    HAL_ADC_Start_DMA(adc_handle, (uint32_t*)adc_buffer_, 3);
}

bool Stm32Adc::isConversionComplete() {
    ADC_HandleTypeDef* adc_handle = static_cast<ADC_HandleTypeDef*>(hadc_);
    return (HAL_ADC_GetState(adc_handle) & HAL_ADC_STATE_REG_EOC) != 0;
}

float Stm32Adc::convertAdcToCurrent(uint32_t adc_raw, PwmChannel channel) {
    // Convert ADC raw value to voltage
    float adc_max_value = (1 << calibration_.adc_resolution_bits) - 1;  // e.g., 4095 for 12-bit
    float v_adc = (adc_raw * calibration_.adc_reference_voltage) / adc_max_value;

    // Select offset voltage for this specific phase
    float offset_voltage = 0.0f;
    switch (channel) {
        case PwmChannel::PHASE_U:
            offset_voltage = offset_voltage_u_;
            break;
        case PwmChannel::PHASE_V:
            offset_voltage = offset_voltage_v_;
            break;
        case PwmChannel::PHASE_W:
            offset_voltage = offset_voltage_w_;
            break;
        default:
            offset_voltage = calibration_.offset_voltage;  // Fallback
            break;
    }

    // Remove offset (zero-current voltage)
    float v_shunt_amplified = v_adc - offset_voltage;

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
    uint32_t adc_raw = 0;

    switch (channel) {
        case PwmChannel::PHASE_U:
            // Phase U: ADC1 JDR1 (OPAMP1)
            adc_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            break;
        case PwmChannel::PHASE_V:
            // Phase V: ADC2 JDR1 (OPAMP2)
            adc_raw = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            break;
        case PwmChannel::PHASE_W:
            // Phase W: ADC1 JDR2 (OPAMP3 via ADC1_IN12)
            adc_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            break;
        default:
            return 0.0f;
    }

    return convertAdcToCurrent(adc_raw, channel);
}

void Stm32Adc::readAllCurrents(float& i_u, float& i_v, float& i_w) {
    // Read from injected data registers (automatically triggered by TIM1_TRGO2)
    uint32_t adc_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);  // Phase U
    uint32_t adc_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);  // Phase V
    uint32_t adc_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);  // Phase W

    i_u = convertAdcToCurrent(adc_u, PwmChannel::PHASE_U);
    i_v = convertAdcToCurrent(adc_v, PwmChannel::PHASE_V);
    i_w = convertAdcToCurrent(adc_w, PwmChannel::PHASE_W);
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

#include <stdio.h>

bool Stm32Adc::calibrateZeroOffset() {
    if (!initialized_) {
        return false;
    }

    // Wait for stable readings
    HAL_Delay(100);

    // Average multiple samples for better accuracy
    const uint32_t num_samples = 100;
    float sum_u = 0.0f, sum_v = 0.0f, sum_w = 0.0f;

    for (uint32_t i = 0; i < num_samples; i++) {
        // Wait for new injected conversion (triggered by TIM1 at 20kHz = 50μs)
        HAL_Delay(1);  // 1ms delay >> 50μs, ensures new data

        // Read from injected data registers
        uint32_t adc_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint32_t adc_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        uint32_t adc_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

        float adc_max = (1 << calibration_.adc_resolution_bits) - 1;
        sum_u += (adc_u * calibration_.adc_reference_voltage) / adc_max;
        sum_v += (adc_v * calibration_.adc_reference_voltage) / adc_max;
        sum_w += (adc_w * calibration_.adc_reference_voltage) / adc_max;
    }

    // Calculate average offset voltage for each phase separately
    offset_voltage_u_ = sum_u / num_samples;
    offset_voltage_v_ = sum_v / num_samples;
    offset_voltage_w_ = sum_w / num_samples;

    printf("zero offsets: %f %f %f\n", offset_voltage_u_, offset_voltage_v_, offset_voltage_w_);

    // Also store average in calibration structure for backward compatibility
    calibration_.offset_voltage = (offset_voltage_u_ + offset_voltage_v_ + offset_voltage_w_) / 3.0f;

    return true;
}

const CurrentSensorCalibration& Stm32Adc::getCalibration() const {
    return calibration_;
}

} // namespace libecu
