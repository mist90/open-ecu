/**
 * @file pwm_interface.hpp
 * @brief Platform-independent PWM interface for 3-phase motor control
 */

#ifndef LIBECU_PWM_INTERFACE_HPP
#define LIBECU_PWM_INTERFACE_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief PWM channel enumeration for 3-phase motor
 */
enum class PwmChannel : uint8_t {
    PHASE_U = 0,
    PHASE_V = 1,
    PHASE_W = 2,
    CHANNEL_COUNT = 3
};

/**
 * @brief PWM state for high/low side switching
 */
enum class PwmState : uint8_t {
    OFF = 0,        ///< Both high and low side off
    HIGH_SIDE = 1,  ///< High side on, low side off
    LOW_SIDE = 2,   ///< High side off, low side on (for braking)
    FLOATING = 3    ///< High impedance state
};

/**
 * @brief Abstract interface for 3-phase PWM control
 */
class PwmInterface {
public:
    virtual ~PwmInterface() = default;

    /**
     * @brief Initialize PWM hardware
     * @param frequency PWM frequency in Hz
     * @return true if initialization successful
     */
    virtual bool initialize(uint32_t frequency) = 0;

    /**
     * @brief Set PWM duty cycle for a channel
     * @param channel PWM channel
     * @param duty_cycle Duty cycle (0.0 to 1.0)
     */
    virtual void setDutyCycle(PwmChannel channel, float duty_cycle) = 0;

    /**
     * @brief Set PWM state for a channel
     * @param channel PWM channel
     * @param state PWM state
     */
    virtual void setState(PwmChannel channel, PwmState state) = 0;

    /**
     * @brief Enable/disable PWM output
     * @param enable true to enable, false to disable
     */
    virtual void enable(bool enable) = 0;

    /**
     * @brief Emergency stop - immediately disable all outputs
     */
    virtual void emergencyStop() = 0;

    /**
     * @brief Get current PWM frequency
     * @return PWM frequency in Hz
     */
    virtual uint32_t getFrequency() const = 0;
};

} // namespace libecu

#endif // LIBECU_PWM_INTERFACE_HPP