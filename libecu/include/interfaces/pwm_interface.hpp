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
 * @brief PWM state for 3-phase complementary switching control
 */
enum class PwmState : uint8_t {
    OFF = 0,  ///< Both high and low side switches disabled (motor disabled)
    UP = 1,   ///< Complementary PWM at 50% + delta (positive direction)
    DOWN = 2  ///< Complementary PWM at 50% - delta (negative direction)
};

/**
 * @brief Abstract interface for 3-phase complementary PWM control
 * 
 * This interface provides complementary PWM control for 3-phase BLDC motors.
 * Each phase has a high-side and low-side switch that operate complementarily
 * with dead-time protection to prevent shoot-through current.
 * 
 * PWM Operation:
 * - 50% duty cycle = neutral position (balanced)
 * - >50% duty cycle = positive torque direction  
 * - <50% duty cycle = negative torque direction
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
     * @param delta Modulation delta (0.0 to 0.5)
     *              For UP: actual_duty = 0.5 + delta (high-side dominant)
     *              For DOWN: actual_duty = 0.5 - delta (low-side dominant)  
     *              For OFF: delta is ignored (both switches disabled)
     */
    virtual void setChannelState(PwmChannel channel, PwmState state, float delta = 0.0f) = 0;

    /**
     * @brief Set PWM duty cycle for a channel (legacy method)
     * @param channel PWM channel
     * @param duty_cycle Duty cycle (0.0 to 1.0)
     *                   0.5 = neutral (50% base)
     *                   >0.5 = positive torque direction
     *                   <0.5 = negative torque direction
     */
    virtual void setDutyCycle(PwmChannel channel, float duty_cycle) = 0;

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
        setDutyCycle(PwmChannel::PHASE_U, 0.5f);
        setDutyCycle(PwmChannel::PHASE_V, 0.5f);
        setDutyCycle(PwmChannel::PHASE_W, 0.5f);
    }
};

} // namespace libecu

#endif // LIBECU_PWM_INTERFACE_HPP