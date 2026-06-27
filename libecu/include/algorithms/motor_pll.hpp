/**
 * @file motor_pll.hpp
 * @brief Phase-Locked Loop (PLL) for motor rotor angle estimation and virtual commutation
 *
 * Implements a PLL that tracks the motor rotor angle from Hall sensor interrupts,
 * providing smooth angle estimation at PWM frequency (40 kHz) and generating
 * virtual commutation steps with optional 90-degree field offset for maximum torque.
 * 
 * The angle is measured in "steps" where one electrical period = 6.0 steps.
 * The working range is expanded to 60.0 steps (10 electrical periods) to prevent
 * phase wrap-around issues during rapid acceleration/deceleration.
 */

#ifndef LIBECU_MOTOR_PLL_HPP
#define LIBECU_MOTOR_PLL_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Motor rotation mode
 */
enum class DriveMode : uint8_t {
    FORWARD = 0,
    REVERSE = 1,
    NEUTRAL = 2
    // PARKING = 3
};

/**
 * @brief Motor rotor angle PLL (Phase-Locked Loop)
 *
 * Tracks rotor angle from discrete Hall sensor events using a PI controller.
 * The virtual angle is integrated at PWM frequency (40 kHz) and can generate
 * commutation steps with a 90-degree stator field offset for maximum torque.
 *
 * Usage:
 *  - Call updateHall() from Hall sensor EXTI interrupt
 *  - Call updateTick() from TIM1 Update interrupt (40 kHz PWM)
 *  - Call getNextHall() to get the next commutation step
 */
class MotorPLL {
public:
    static constexpr float HALL_TIMEOUT_SEC = 5.0f;
    static constexpr uint8_t ANGLE_MAX = 60;

    /**
     * @brief Constructor
     * @param freq_pwm PWM frequency in Hz (e.g., 40000.0f for 40 kHz)
     * @param is_inverse_commutation_table Use inverse six-step commutation table
     */
    explicit MotorPLL(float freq_pwm, float max_electrical_speed, bool is_inverse_commutation_table = false) noexcept;

    /**
     * @brief Hall sensor interrupt handler (EXTI)
     *
     * Must be called from the real Hall sensor GPIO interrupt.
     * Updates the PLL with the actual rotor position and timing information.
     *
     * @param hall_state Current Hall step (0-5).
     */
    void updateHall(uint8_t hall_state) noexcept;

    /**
     * @brief Angle integrator
     *
     * Must be called from TIM1 Update interrupt (every PWM tick, 40 kHz)
     * when PLL is enabled. Integrates the virtual rotor angle.
     */
    void updateTick() noexcept;

    /**
     * @brief Get next commutation step
     *
     * Call from PWM loop to update the inverter switches.
     * When PLL is active, applies a 90-degree stator field offset
     * relative to the rotor for maximum torque production.
     *
     * @param mode Current drive mode (FORWARD/REVERSE/NEUTRAL)
     * @return Next commutation step (0-5)
     */
    uint8_t getNextHall(const volatile DriveMode &mode) noexcept;

    /**
     * @brief Enable or disable PLL mode
     * @param use true = PLL-based commutation, false = discrete Hall table
     */
    void setUsePLL(bool use) noexcept;

    /// @return true if PLL mode is active
    bool isUsingPLL() const noexcept;

    /// @brief Reset PLL state (angle, speed, integrator)
    void reset() noexcept;

    /// @return Current estimated rotor angle in steps (0-5)
    float getAngle() const noexcept;

    /// @return Current estimated electrical speed in steps/sec
    float getSpeedStepsSec() const noexcept;

private:
    /// @brief Adapt PLL PI gains based on current speed
    void updateAdaptiveGains() noexcept;

    uint8_t hall_state_ = 0x0;   // [0..ANGLE_MAX]
    float angle_ = 0.0f;
    float angle_per_second_ = 0.0f;
    bool is_locked_ = false;
    bool is_inverse_commutation_table_ = false;

    bool use_pll_ = false;
    uint32_t last_timestamp_us_ = 0;
    float time_since_last_hall_ = 0.0f;
    float pll_kp_ = 0.0f;
    float pll_ki_ = 0.0f;
    float pll_integral_ = 0.0f;
    float DT_;
    float max_electrical_speed_;
};

} // namespace libecu

#endif // LIBECU_MOTOR_PLL_HPP
