/**
 * @file adc_interface.hpp
 * @brief Platform-independent ADC interface for motor current sensing
 */

#ifndef LIBECU_ADC_INTERFACE_HPP
#define LIBECU_ADC_INTERFACE_HPP

#include "pwm_interface.hpp"
#include <cmath>
#include <cstdint>

uint32_t time_us();

namespace libecu {

/**
 * @brief Temperature reported when the NTC reading cannot be trusted
 *
 * Deliberately absurd rather than plausible: it is below any limit, so it
 * never trips the thermal cut-out, but nobody can mistake it for a
 * measurement in telemetry. See convertAdcToTemperature() for when it is
 * returned and why an open sensor cannot be distinguished from a cold one.
 */
constexpr float TEMPERATURE_INVALID_C = -273.0f;

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
 * @brief Voltage sensor parameters for bus voltage measurement
 */
struct VoltageSensorParameters {
    float r_up;    // Upper resistor of voltage divider (Ohms)
    float r_down;  // Lower resistor of voltage divider (Ohms)
};

/**
 * @brief NTC thermistor divider parameters
 *
 * The divider on this board is
 *
 *     +3.3V --- [ NTC ] --- signal --- [ r_pulldown ] --- GND
 *
 * so the signal *rises* with temperature (a hot NTC has a low resistance),
 * and the thermistor resistance follows from the reading as
 *
 *     R_ntc = r_pulldown * (adc_max - raw) / raw
 *
 * The defaults describe the fitted part: 10 k at 25 C, B25/85 = 3435 K,
 * against a 4.7 k pull-down.
 */
struct TemperatureSensorParameters {
    float r_pulldown_ohms = 4700.0f;   ///< Fixed resistor from signal to GND (Ohms)
    float ntc_r25_ohms    = 10000.0f;  ///< NTC nominal resistance at 25 C (Ohms)
    float ntc_beta_k      = 3435.0f;   ///< NTC beta constant B25/85 (Kelvin)
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
    bool initialize(const CurrentSensorCalibration& calibration,
                    const VoltageSensorParameters& params,
                    const TemperatureSensorParameters& temp_params = TemperatureSensorParameters{}) noexcept {
        calibration_ = calibration;
        voltage_params_ = params;
        temp_params_ = temp_params;

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
     * @brief Get raw ADC value for bus voltage channel
     * @return Raw ADC value for Vbus (0 to 2^resolution - 1)
     */
    virtual uint32_t getRawAdcValue() = 0;

    /**
     * @brief Get raw ADC value for the NTC temperature channel
     *
     * Unlike the current and Vbus channels this one is *not* injected: it is a
     * regular conversion started on demand, because a thermal mass measured at
     * 100 Hz has nothing to gain from PWM-synchronous sampling and the injected
     * sequence is on the critical path of the 20 kHz current loop.
     *
     * @return Raw ADC value (0 to 2^resolution - 1), 0 if the conversion failed
     */
    virtual uint32_t getRawTemperatureAdcValue() = 0;

    /**
     * @brief Convert raw ADC value to current in Amperes
     * @param adc_raw Raw ADC reading (0 to 4095 for 12-bit)
     * @param channel PWM channel to use correct offset
     * @return Current in Amperes
     */
    float convertAdcToCurrent(uint32_t adc_raw, PwmChannel channel) noexcept {
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
     * @brief Convert raw ADC value to bus voltage
     * @param adc_raw Raw ADC reading
     * @return Bus voltage in Volts, accounting for resistor divider ratio
     */
    float convertAdcToVoltage(uint32_t adc_raw) noexcept {
        float adc_max_value = (1 << calibration_.adc_resolution_bits) - 1;
        float divider_ratio = (voltage_params_.r_up + voltage_params_.r_down) / voltage_params_.r_down;
        return (adc_raw * calibration_.adc_reference_voltage * divider_ratio) / adc_max_value;
    }

    /**
     * @brief Convert a raw NTC divider reading to degrees Celsius
     *
     * Beta (B-parameter) equation:
     *
     *     1/T = 1/T25 + ln(R_ntc / R25) / B      (T in Kelvin)
     *
     * Good to a degree or so over 0..120 C, which is all a thermal cut-out
     * needs; a Steinhart-Hart fit would only matter if this number were used
     * for compensation rather than protection.
     *
     * Failure modes are asymmetric, and worth knowing before trusting this:
     *
     * - Signal shorted to 3V3, or the NTC shorted out: raw saturates high,
     *   R_ntc goes to zero and the result reads several hundred degrees, so
     *   the cut-out trips. Safe direction.
     * - NTC open circuit, or the signal shorted to GND: the pull-down drags
     *   the pin to 0 V and the reading is indistinguishable from an
     *   arbitrarily cold sensor. There is nothing in the divider that can
     *   separate the two, so raw == 0 returns TEMPERATURE_INVALID_C - visible
     *   in telemetry, but it does NOT trip. A lost sensor wire silently
     *   removes thermal protection; that is a property of where the fixed
     *   resistor sits, not of this code.
     *
     * @param adc_raw Raw ADC reading of the divider mid-point
     * @return Temperature in degrees Celsius, or TEMPERATURE_INVALID_C
     */
    float convertAdcToTemperature(uint32_t adc_raw) const noexcept {
        const float adc_max = static_cast<float>((1u << calibration_.adc_resolution_bits) - 1u);

        if (adc_raw == 0u) {
            return TEMPERATURE_INVALID_C;  // open NTC or signal shorted to GND
        }
        if (temp_params_.r_pulldown_ohms <= 0.0f ||
            temp_params_.ntc_r25_ohms <= 0.0f ||
            temp_params_.ntc_beta_k <= 0.0f) {
            return TEMPERATURE_INVALID_C;  // unconfigured sensor
        }

        // Keep one count below full scale: at raw == adc_max the divider says
        // R_ntc == 0, and ln(0) is not a temperature.
        float raw = static_cast<float>(adc_raw);
        if (raw > adc_max - 1.0f) {
            raw = adc_max - 1.0f;
        }

        const float r_ntc = temp_params_.r_pulldown_ohms * (adc_max - raw) / raw;

        const float inv_t = 1.0f / 298.15f
                          + std::log(r_ntc / temp_params_.ntc_r25_ohms) / temp_params_.ntc_beta_k;

        return 1.0f / inv_t - 273.15f;
    }

    /**
     * @brief Read bus voltage from ADC
     * @return Bus voltage in Volts
     */
    float readBusVoltage() noexcept {
        return convertAdcToVoltage(getRawAdcValue());
    }

    /**
     * @brief Read the NTC temperature
     * @return Temperature in degrees Celsius, or TEMPERATURE_INVALID_C
     */
    float readTemperature() noexcept {
        return convertAdcToTemperature(getRawTemperatureAdcValue());
    }

    /**
     * @brief Read current from a specific phase
     * @param channel Phase channel (U/V/W)
     * @return Phase current in Amperes (negative = reverse direction)
     */
    float readPhaseCurrent(PwmChannel channel) noexcept {
        return convertAdcToCurrent(getRawAdcValue(channel), channel);
    }

    /**
     * @brief Read all phase currents at once
     * @param i_u Reference to store Phase U current (A)
     * @param i_v Reference to store Phase V current (A)
     * @param i_w Reference to store Phase W current (A)
     */
    void readAllCurrents(float& i_u, float& i_v, float& i_w) noexcept {
        i_u = convertAdcToCurrent(getRawAdcValue(PwmChannel::PHASE_U), PwmChannel::PHASE_U);
        i_v = convertAdcToCurrent(getRawAdcValue(PwmChannel::PHASE_V), PwmChannel::PHASE_V);
        i_w = convertAdcToCurrent(getRawAdcValue(PwmChannel::PHASE_W), PwmChannel::PHASE_W);
    }

    /**
     * @brief Calibrate zero-current offset
     * Measures ADC reading when motor is stationary to determine V_offset
     * @return true if calibration successful
     */
    bool calibrateZeroOffset() noexcept {
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

    const CurrentSensorCalibration& getCalibration() const noexcept {
        return calibration_;
    }

    const TemperatureSensorParameters& getTemperatureParams() const noexcept {
        return temp_params_;
    }

private:
    CurrentSensorCalibration calibration_;
    bool initialized_ = false;
    VoltageSensorParameters voltage_params_;
    TemperatureSensorParameters temp_params_;
};

} // namespace libecu

#endif // LIBECU_ADC_INTERFACE_HPP
