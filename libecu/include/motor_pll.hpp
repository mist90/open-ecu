/**
 * @file motor_pll.hpp
 * @brief Phase-Locked Loop (PLL) for motor rotor angle estimation and virtual commutation
 *
 * Implements a PLL that tracks the motor rotor angle from Hall sensor interrupts,
 * providing smooth angle estimation at the board's current-loop frequency and generating
 * virtual commutation steps with optional 90-degree field offset for maximum torque.
 *
 * The angle is measured in "steps" where one electrical period = 6.0 steps.
 * The phase is wrapped into [0, 6) at every sample.
 */

#ifndef LIBECU_MOTOR_PLL_HPP
#define LIBECU_MOTOR_PLL_HPP

#include <cstdint>
#include "ecu_config.hpp"
#include "motor_pll_core.hpp"

namespace libecu {

/**
 * @brief Motor rotor angle PLL (Phase-Locked Loop)
 *
 * Tracks rotor angle from discrete Hall sensor events using a PI controller.
 * The virtual angle is integrated at the current-loop frequency and can generate
 * commutation steps with a 90-degree stator field offset for maximum torque.
 *
 * Usage:
 *  - Call updateHall() from Hall sensor EXTI interrupt
 *  - Call updateTick() from TIM1 Update interrupt (20 kHz on STM32G431)
 *  - Call getNextHall() to get the next commutation step
 */
class MotorPLL {
public:
    using Core = MotorPLLCore<ActiveNumericProfile, ActiveTiming::CURRENT_HZ>;
    Core::Angle getNativeAngle() const noexcept { return core_.getAngle(); }
    Core::Speed getNativeSpeedStepsSec() const noexcept { return core_.getSpeedStepsSec(); }
    uint8_t getHallStateRaw() const noexcept { return core_.getInfo().hall_state_raw; }
    static constexpr float LIMIT_ANGLE_ERROR = 0.5f;
    static constexpr float ANGLE_MAX = 6.0f;
    static constexpr float SYNC_SPEED = 5.0f;

    /**
     * @name Lock detector
     *
     * The Hall sensor only reports which 60-degree sector the rotor is in, so
     * even a perfectly locked PLL sees a sawtooth angle error that ramps from
     * +0.5 to -0.5 steps once per sector: mean |error| is 0.25 at every speed,
     * peak |error| is 0.5 at every speed.  Loss of lock is what pins |error|
     * at the 0.5 clamp, driving the mean towards 0.5 - hence the 0.4 trip
     * level, which sits three quarters of the way from locked to unlocked.
     *
     * That only holds if the averaging window spans at least one sawtooth
     * period, and the sawtooth period is 1/speed.  A fixed-tick filter cannot
     * do that: sized for high speed it follows the sawtooth peaks at low
     * speed and trips every sector.  So the window is specified in electrical
     * revolutions and the smoothing factor is derived from the current speed.
     */
    ///@{
    static constexpr float LOCK_WINDOW_EREV = 1.0f;   ///< Averaging window, electrical revolutions
    static constexpr float LOCK_ALPHA_MIN = 1e-4f;    ///< Floor: keeps the metric alive at standstill
    static constexpr float LOCK_ALPHA_MAX = 0.05f;    ///< Ceiling: >= 20 ticks of averaging at any speed
    static constexpr float LOCK_TRIP = LIMIT_ANGLE_ERROR * 0.8f; ///< Mean |error| that means "lost"
    ///@}

    /**
     * @brief Constructor
     * @param max_electrical_speed Maximum signed speed magnitude, steps/sec
     * @param is_inverse_commutation_table Use inverse six-step commutation table
     */
    explicit MotorPLL(float max_electrical_speed, bool is_inverse_commutation_table = false) noexcept;

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
     * Must be called from TIM1 Update interrupt (every PWM tick)
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

    /**
     * @brief Current estimated electrical speed in steps/sec
     *
     * Returns the PI *integrator*, not the full PI output.  The proportional
     * term is fed by the Hall quantisation sawtooth (see LOCK_WINDOW_EREV), so
     * the PI output carries a kp*0.5 = +-50 steps/sec ripple at every speed -
     * 5% of the reading at 8 RPS but 28% of it at 1.5 RPS.  The integrator
     * carries the same information without that ripple, because the sawtooth
     * is symmetric about zero and integrates away.  The angle is still
     * integrated with the full PI output; only the reported speed differs.
     *
     * @return Estimated electrical speed in steps/sec, signed (negative in REVERSE)
     */
    float getSpeedStepsSec() const noexcept;

    /**
     * @brief Snapshot of PLL internal state for telemetry
     */
    struct PllInfo {
        bool use_pll;              ///< PLL enabled flag
        uint8_t hall_state_raw;    ///< Actual Hall sensor reading [0..5]
        float angle;               ///< PLL-estimated angle in steps [0..ANGLE_MAX)
        float angle_per_second;    ///< PLL-estimated speed in steps/sec
        float pll_integral;        ///< PI integrator term in steps/sec
        float kp;                  ///< Proportional gain
        float ki;                  ///< Continuous integral gain
        bool is_sync;              ///< PLL synchronized with Hall sensor flag
    };

    /**
     * @brief Get a snapshot of PLL internal state (for telemetry)
     * @return PllInfo struct with current values
     * @note Caller must hold a CriticalSection if reading from a different interrupt priority
     */
    PllInfo getInfo() const noexcept;

    /**
     * @brief Set continuous PI gains and cache Ki/sample_frequency
     * @param kp_base Proportional gain
     * @param ki_base Integral gain
     * @return false if a coefficient does not fit the active profile
     */
    bool setGains(float kp_base, float ki_base) noexcept;

    /**
     * @brief Get base PI gains
     * @param kp_base Output: base proportional gain
     * @param ki_base Output: base integral gain
     */
    void getBaseGains(float& kp_base, float& ki_base) const noexcept;

private:
    Core core_;
};

} // namespace libecu

#endif // LIBECU_MOTOR_PLL_HPP
