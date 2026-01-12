/**
 * @file adc_interface.hpp
 * @brief Platform-independent ADC interface for motor current sensing
 */

#ifndef LIBECU_ADC_INTERFACE_HPP
#define LIBECU_ADC_INTERFACE_HPP

#include "pwm_interface.hpp"
#include <cstdint>

uint32_t time_us();

namespace libecu {

/**
 * @brief Current sensor calibration parameters
 */
struct CurrentSensorCalibration {
    float shunt_resistance_ohms;   ///< Shunt resistor value in Ohms (e.g., 0.003)
    float opamp_gain;              ///< OPAMP amplification gain (e.g., 16.0)
    float adc_reference_voltage;   ///< ADC reference voltage in Volts (e.g., 3.3)
    uint32_t adc_resolution_bits;  ///< ADC resolution in bits (e.g., 12)
    float offset_voltage_u;          ///< Zero-current offset voltage in Volts for U phase (e.g., 1.65)
    float offset_voltage_v;          ///< Zero-current offset voltage in Volts for V phase (e.g., 1.65)
    float offset_voltage_w;          ///< Zero-current offset voltage in Volts for W phase (e.g., 1.65)
};

/**
 * @brief Abstract interface for ADC-based current sensing
 *
 * This interface provides platform-independent current measurement
 * for 3-phase BLDC motor control. Implementations handle hardware-specific
 * ADC configuration, triggering, and conversion.
 *
 * Current Measurement Strategy:
 * - Synchronized with PWM for accurate sampling
 * - Sample during low-side switch conduction (PwmState::DOWN)
 * - Convert ADC reading through OPAMP gain and shunt resistance
 * - Support for bidirectional current measurement
 */
class AdcInterface {
public:
    virtual ~AdcInterface() = default;

    /**
     * @brief Initialize ADC hardware and calibration
     * @param calibration Sensor calibration parameters
     * @return true if initialization successful
     */
    bool initialize(const CurrentSensorCalibration& calibration) {
        calibration_ = calibration;

        // ADC calibration and injected channel startup is done in main.cpp
        // before calling this function. Here we just store the calibration.
        initialized_ = true;
        return true;
    }

    /**
     * @brief Get raw ADC value for a channel (for debugging/calibration)
     * @param channel Phase channel
     * @return Raw ADC value (0 to 2^resolution - 1)
     */
    virtual uint32_t getRawAdcValue(PwmChannel channel) = 0;

    /**
     * @brief Convert raw ADC value to current in Amperes
     * @param adc_raw Raw ADC reading (0 to 4095 for 12-bit)
     * @param channel PWM channel to use correct offset
     * @return Current in Amperes
     */
    float convertAdcToCurrent(uint32_t adc_raw, PwmChannel channel) {
        // Convert ADC raw value to voltage
        float adc_max_value = (1 << calibration_.adc_resolution_bits) - 1;  // e.g., 4095 for 12-bit
        float v_adc = (adc_raw * calibration_.adc_reference_voltage) / adc_max_value;

        // Select offset voltage for this specific phase
        float offset_voltage = 0.0f;
        switch (channel) {
            case PwmChannel::PHASE_U:
                offset_voltage = calibration_.offset_voltage_u;
                break;
            case PwmChannel::PHASE_V:
                offset_voltage = calibration_.offset_voltage_v;
                break;
            case PwmChannel::PHASE_W:
                offset_voltage = calibration_.offset_voltage_w;
                break;
            default:
                offset_voltage = 0.0f;
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

    /**
     * @brief Read current from a specific phase
     * @param channel Phase channel (U/V/W)
     * @return Phase current in Amperes (negative = reverse direction)
     */
    float readPhaseCurrent(PwmChannel channel) {
        return convertAdcToCurrent(getRawAdcValue(channel), channel);
    }

    /**
     * @brief Read all phase currents at once
     * @param i_u Reference to store Phase U current (A)
     * @param i_v Reference to store Phase V current (A)
     * @param i_w Reference to store Phase W current (A)
     */
    void readAllCurrents(float& i_u, float& i_v, float& i_w) {
        i_u = convertAdcToCurrent(getRawAdcValue(PwmChannel::PHASE_U), PwmChannel::PHASE_U);
        i_v = convertAdcToCurrent(getRawAdcValue(PwmChannel::PHASE_V), PwmChannel::PHASE_V);
        i_w = convertAdcToCurrent(getRawAdcValue(PwmChannel::PHASE_W), PwmChannel::PHASE_W);
    }

    /**
     * @brief Calibrate zero-current offset
     * Measures ADC reading when motor is stationary to determine V_offset
     * @return true if calibration successful
     */
    bool calibrateZeroOffset() {
        if (!initialized_) {
            return false;
        }

        // Average multiple samples for better accuracy
        const uint32_t num_samples = 100;
        float sum_u = 0.0f, sum_v = 0.0f, sum_w = 0.0f;

        calibration_.offset_voltage_u = 0.0f;
        calibration_.offset_voltage_v = 0.0f;
        calibration_.offset_voltage_w = 0.0f;

        for (uint32_t i = 0; i < num_samples; i++) {
            // Wait for new injected conversion (triggered by TIM1 at 20kHz = 50μs)
            uint32_t start_time = time_us();

            while (time_us() - start_time < 100) {
            }

            // Read from injected data registers
            uint32_t adc_u = getRawAdcValue(PwmChannel::PHASE_U);
            uint32_t adc_v = getRawAdcValue(PwmChannel::PHASE_V);
            uint32_t adc_w = getRawAdcValue(PwmChannel::PHASE_W);

            float adc_max = (1 << calibration_.adc_resolution_bits) - 1;
            sum_u += (adc_u * calibration_.adc_reference_voltage) / adc_max;
            sum_v += (adc_v * calibration_.adc_reference_voltage) / adc_max;
            sum_w += (adc_w * calibration_.adc_reference_voltage) / adc_max;
        }

        // Calculate average offset voltage for each phase separately
        calibration_.offset_voltage_u = sum_u / num_samples;
        calibration_.offset_voltage_v = sum_v / num_samples;
        calibration_.offset_voltage_w = sum_w / num_samples;

        return true;
    }

    /**
     * @brief Get current calibration parameters
     * @return Current calibration struct
     */
    const CurrentSensorCalibration& getCalibration() const {
        return calibration_;
    }

private:
    CurrentSensorCalibration calibration_;
    bool initialized_ = false;
};

} // namespace libecu

#endif // LIBECU_ADC_INTERFACE_HPP
