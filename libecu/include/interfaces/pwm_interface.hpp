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
     * @brief Set the switching state of a channel
     *
     * Carries no duty cycle: which switches conduct and how long they conduct
     * for are deliberately separate. A six-step commutation changes only the
     * former, the current loop changes only the latter, and on hardware that
     * latches them on different events (STM32 COM event vs. update event) any
     * coupling between the two shows up as a glitched PWM period.
     *
     * @param channel PWM channel
     * @param state PWM state (OFF/UP/DOWN)
     */
    virtual void setChannelState(PwmChannel channel, PwmState state) = 0;

    /**
     * @brief Set the modulation depth of all three phases as one atomic update
     *
     * Takes all three phases together rather than one at a time: the compare
     * registers must reload as a set, otherwise an update event landing between
     * two of them leaves the bridge running one phase on the new duty and
     * another on the old one for a period.
     *
     * Six-step passes the same value three times - the per-phase switching state
     * decides which phase actually modulates it. Independent values are what
     * sinusoidal/space-vector modulation needs, which is why the three-phase
     * form is the primitive.
     *
     * @param duty_u,duty_v,duty_w Per-phase duty cycle (0.0 to 1.0)
     */
    virtual void updateDutyCycle(float duty_u, float duty_v, float duty_w) = 0;

    /**
     * @brief Set the same modulation depth on all three phases (six-step)
     */
    void updateDutyCycle(float duty_cycle) noexcept {
        updateDutyCycle(duty_cycle, duty_cycle, duty_cycle);
    }

/**
     * @brief Enable/disable PWM output
     * @param enable true to enable, false to disable
     */
    virtual void enable(bool enable) = 0;

    /**
     * @brief Apply all pending channel state changes atomically
     * @note Default implementation is a no-op. Override in platform-specific
     *       implementations that support atomic commutation (e.g., STM32 COMG).
     */
    virtual void apply() {}

    /**
     * @brief Set all phases to neutral (50% duty cycle)
     * Used for motor startup and balanced operation
     */
    void setNeutral() noexcept {
        setChannelState(PwmChannel::PHASE_U, PwmState::OFF);
        setChannelState(PwmChannel::PHASE_V, PwmState::OFF);
        setChannelState(PwmChannel::PHASE_W, PwmState::OFF);
        updateDutyCycle(0.0f);
        apply();
    }

    uint32_t getFrequency() noexcept {
        return frequency_;
    }
protected:
    uint32_t frequency_ = 0;
};

} // namespace libecu

#endif // LIBECU_PWM_INTERFACE_HPP
