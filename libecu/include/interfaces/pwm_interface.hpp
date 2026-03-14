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
    PHASE_W = 2
};

/**
 * @brief PWM state for 3-phase complementary switching control
 */
enum class PwmState : uint8_t {
    OFF = 0,  ///< High impedance - both high and low side switches disabled
    UP = 1,   ///< Non-inverse PWM - high-side active, low-side complementary
    DOWN = 2  ///< Low-side always ON, high-side always OFF
};

/**
 * @brief Abstract interface for 3-phase complementary PWM control
 *
 * This interface provides complementary PWM control for 3-phase BLDC motors.
 * Each phase has a high-side and low-side switch that operate complementarily
 * with dead-time protection to prevent shoot-through current.
 *
 * PWM Operation:
 * - UP state: High-side switches at duty_cycle, low-side complementary (phase → V+)
 * - DOWN state: Low-side switches at duty_cycle, high-side complementary (phase → GND)
 * - OFF state: Both switches disabled, phase floating (high-Z)
 * - Dead-time ensures switches never conduct simultaneously
 */
class PwmInterface {
public:
    virtual ~PwmInterface() = default;

    /**
     * @brief Initialize PWM hardware with complementary outputs and dead-time
     * @param frequency PWM frequency in Hz
     * @param dead_time_ns Dead-time in nanoseconds to prevent shoot-through
     * @return true if initialization successful
     */
    virtual bool initialize(uint32_t frequency, uint16_t dead_time_ns) = 0;

    /**
     * @brief Set PWM state and modulation for a channel
     * @param channel PWM channel
     * @param state PWM state (OFF/UP/DOWN)
     * @param duty_cycle PWM duty cycle (0.0 to 1.0)
     *                   For UP: High-side active for duty_cycle
     *                   For DOWN: Low-side active for duty_cycle
     *                   For OFF: duty_cycle is ignored (both switches disabled)
     */
    virtual void setChannelState(PwmChannel channel, PwmState state, float duty_cycle = 0.0f) = 0;

    /**
     * @brief Set PWM state for a channel (legacy method)
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

    /**
     * @brief Set all phases to neutral (50% duty cycle)
     * Used for motor startup and balanced operation
     */
    virtual void setNeutral() {
        setChannelState(PwmChannel::PHASE_U, PwmState::OFF, 0.0f);
        setChannelState(PwmChannel::PHASE_V, PwmState::OFF, 0.0f);
        setChannelState(PwmChannel::PHASE_W, PwmState::OFF, 0.0f);
    }
};

} // namespace libecu

#endif // LIBECU_PWM_INTERFACE_HPP