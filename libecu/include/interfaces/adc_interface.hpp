/**
 * @file adc_interface.hpp
 * @brief Platform-independent ADC interface for motor current sensing
 */

#ifndef LIBECU_ADC_INTERFACE_HPP
#define LIBECU_ADC_INTERFACE_HPP

#include "pwm_interface.hpp"
#include <cstdint>

namespace libecu {

/**
 * @brief Current sensor calibration parameters
 */
struct CurrentSensorCalibration {
    float shunt_resistance_ohms;   ///< Shunt resistor value in Ohms (e.g., 0.003)
    float opamp_gain;              ///< OPAMP amplification gain (e.g., 16.0)
    float adc_reference_voltage;   ///< ADC reference voltage in Volts (e.g., 3.3)
    uint32_t adc_resolution_bits;  ///< ADC resolution in bits (e.g., 12)
    float offset_voltage;          ///< Zero-current offset voltage in Volts (e.g., 1.65)
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
    virtual bool initialize(const CurrentSensorCalibration& calibration) = 0;

    /**
     * @brief Start ADC conversion (single or continuous)
     * Typically triggered by PWM timer for synchronized sampling
     */
    virtual void startConversion() = 0;

    /**
     * @brief Check if ADC conversion is complete
     * @return true if conversion ready to read
     */
    virtual bool isConversionComplete() = 0;

    /**
     * @brief Read current from a specific phase
     * @param channel Phase channel (U/V/W)
     * @return Phase current in Amperes (negative = reverse direction)
     */
    virtual float readPhaseCurrent(PwmChannel channel) = 0;

    /**
     * @brief Read all phase currents at once
     * @param i_u Reference to store Phase U current (A)
     * @param i_v Reference to store Phase V current (A)
     * @param i_w Reference to store Phase W current (A)
     */
    virtual void readAllCurrents(float& i_u, float& i_v, float& i_w) = 0;

    /**
     * @brief Get raw ADC value for a channel (for debugging/calibration)
     * @param channel Phase channel
     * @return Raw ADC value (0 to 2^resolution - 1)
     */
    virtual uint32_t getRawAdcValue(PwmChannel channel) = 0;

    /**
     * @brief Calibrate zero-current offset
     * Measures ADC reading when motor is stationary to determine V_offset
     * @return true if calibration successful
     */
    virtual bool calibrateZeroOffset() = 0;

    /**
     * @brief Get current calibration parameters
     * @return Current calibration struct
     */
    virtual const CurrentSensorCalibration& getCalibration() const = 0;
};

} // namespace libecu

#endif // LIBECU_ADC_INTERFACE_HPP
