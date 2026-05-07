/**
 * @file commutation_controller.hpp
 * @brief 6-step commutation algorithm for BLDC motors
 */

#ifndef LIBECU_COMMUTATION_CONTROLLER_HPP
#define LIBECU_COMMUTATION_CONTROLLER_HPP

#include "../interfaces/pwm_interface.hpp"
#include "../interfaces/hall_interface.hpp"
#include <cstdint>

namespace libecu {

/**
 * @brief Commutation step for 6-step algorithm
 */
struct CommutationStep {
    PwmState phase_u;    ///< Phase U PWM state
    PwmState phase_v;    ///< Phase V PWM state
    PwmState phase_w;    ///< Phase W PWM state
};

/**
 * @brief 6-step commutation controller for BLDC motors
 */
class CommutationController {
public:
    /**
     * @brief Constructor
     * @param pwm_interface PWM interface
     * @param hall_interface Hall sensor interface
     * @param num_poles Number of motor pole pairs (default: 2 for 4-pole motor)
     */
    CommutationController(PwmInterface& pwm_interface, HallInterface& hall_interface, uint8_t num_poles = 2) noexcept;

    /**
     * @brief Initialize commutation controller
     * @param pwm_frequency PWM frequency in Hz
     * @return true if initialization successful
     */
    bool initialize(uint32_t pwm_frequency = 20000) noexcept;

    /**
     * @brief Update commutation based on rotor position
     * @param position Rotor position (0-5 range, corresponding to 6-step commutation)
     * @param duty_cycle Motor duty cycle (0.0 to 1.0)
     *                   0.0 = 0V output (neutral, no torque)
     *                   1.0 = maximum voltage output (full torque)
     * @param direction Rotation direction
     * @return true if commutation updated successfully, false if position is invalid
     */
    bool update(uint8_t position, float duty_cycle) noexcept;

    /**
     * @brief Get current motor position (0-5 range)
     * @return Current motor position as step index (0-5), or 0xFF if invalid
     */
    uint8_t getCurrentPosition() noexcept;

    /**
     * @brief Update duty cycle without changing commutation step
     * @param duty_cycle Motor duty cycle (0.0 to 1.0)
     */
    void updateDutyCycle(float duty_cycle) noexcept;

    /**
     * @brief Get cached phase state for a given channel
     * @param channel PWM channel
     * @return Cached phase state
     */
    PwmState getPhaseState(PwmChannel channel) const noexcept;

    /**
     * @brief Get number of motor pole pairs
     * @return Number of pole pairs
     */
    uint8_t getNumPoles() const noexcept { return num_poles_; }

private:
    PwmInterface& pwm_interface_;
    HallInterface& hall_interface_;

    uint8_t current_step_;
    uint8_t num_poles_;  ///< Number of motor pole pairs

    // Cached phase states for fast access (updated in update())
    PwmState cached_phase_u_state_;
    PwmState cached_phase_v_state_;
    PwmState cached_phase_w_state_;

    static const CommutationStep COMMUTATION_TABLE[6];

    void applyCommutationStep(const CommutationStep& step, float duty_cycle) noexcept;
};

} // namespace libecu

#endif // LIBECU_COMMUTATION_CONTROLLER_HPP