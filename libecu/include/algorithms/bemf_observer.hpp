/**
 * @file bemf_observer.hpp
 * @brief BEMF zero-crossing observer for sensorless 6-step BLDC commutation
 *
 * Detects back-EMF zero-crossings on the floating phase during 6-step
 * trapezoidal commutation and generates synthetic Hall sensor events
 * for the MotorPLL. Supports demagnetization blanking, 30-degree
 * commutation delay, and hybrid Hall/BEMF transition with hysteresis.
 */

#ifndef LIBECU_BEMF_OBSERVER_HPP
#define LIBECU_BEMF_OBSERVER_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Configuration parameters for the BEMF observer
 */
struct BemfObserverParams {
    float blanking_cycles;        ///< PWM cycles to blank after commutation (demagnetization)
    float zc_threshold;           ///< BEMF zero-crossing threshold as fraction of Vbus (default 0.5)
    float transition_speed_low;   ///< Speed below which Hall sensors are used (steps/sec)
    float transition_speed_high;  ///< Speed above which BEMF is used exclusively (steps/sec)
};

/**
 * @brief BEMF zero-crossing observer for sensorless commutation
 *
 * Monitors the floating phase voltage during 6-step commutation to detect
 * BEMF zero-crossings. After detecting a ZC, applies a 30-degree delay
 * (half of one commutation step) before emitting a synthetic Hall event.
 *
 * Supports hybrid mode with configurable speed thresholds and hysteresis
 * for smooth transition between Hall-based and BEMF-based commutation.
 *
 * Usage:
 *  - Call onCommutation() on each commutation event to reset blanking
 *  - Call update() from PWM ISR with the floating phase voltage
 *  - When update() returns true, call getSyntheticHallStep() for the next step
 *  - Use isBemfModeActive() / shouldIgnoreHall() for hybrid mode decisions
 */
class BemfObserver {
public:
    /**
     * @brief Constructor
     * @param pwm_frequency PWM frequency in Hz (e.g., 40000.0f for 40 kHz)
     */
    explicit BemfObserver(float pwm_frequency) noexcept;

    /**
     * @brief Set observer parameters
     * @param params BEMF observer configuration
     */
    void setParameters(const BemfObserverParams& params) noexcept;

    /**
     * @brief Process one BEMF sample from the floating phase
     *
     * Call from PWM ISR at pwm_frequency rate. Handles blanking,
     * zero-crossing detection, and 30-degree commutation delay.
     *
     * @param floating_voltage ADC reading of the floating phase voltage
     * @param bus_voltage Current DC bus voltage
     * @param current_step Current commutation step (0-5)
     * @param speed_steps_per_sec Current motor speed in steps/sec
     * @return true when a synthetic Hall event is ready
     */
    bool update(float floating_voltage, float bus_voltage,
                uint8_t current_step, float speed_steps_per_sec) noexcept;

    /**
     * @brief Get the synthetic commutation step
     * @return Commutation step (0-5) to feed to MotorPLL
     */
    uint8_t getSyntheticHallStep() const noexcept;

    /**
     * @brief Notify observer of a commutation event
     *
     * Resets the blanking counter and clears pending ZC state.
     * Must be called on every commutation to start the demagnetization
     * blanking window.
     *
     * @param new_step The commutation step that was just applied (0-5)
     */
    void onCommutation(uint8_t new_step) noexcept;

    /**
     * @brief Check if BEMF mode is active at the given speed
     * @param speed_steps_per_sec Current motor speed in steps/sec
     * @return true if speed is above the transition threshold (with hysteresis)
     */
    bool isBemfModeActive(float speed_steps_per_sec) const noexcept;

    /**
     * @brief Check if Hall sensor input should be suppressed
     * @param speed_steps_per_sec Current motor speed in steps/sec
     * @return true if speed is above transition_speed_high
     */
    bool shouldIgnoreHall(float speed_steps_per_sec) const noexcept;

    /**
     * @brief Telemetry snapshot of observer internal state
     */
    struct BemfInfo {
        bool bemf_active;          ///< BEMF mode currently active
        bool zc_detected;          ///< Zero-crossing detected, waiting for delay
        float blanking_counter;    ///< Remaining blanking cycles
        uint8_t synthetic_step;    ///< Last synthetic commutation step
        float floating_voltage;    ///< Last floating phase voltage reading
    };

    /**
     * @brief Get a snapshot of observer internal state (for telemetry)
     * @return BemfInfo struct with current values
     */
    BemfInfo getInfo() const noexcept;

private:
    float pwm_frequency_;          ///< PWM frequency in Hz
    BemfObserverParams params_;    ///< Observer configuration

    float blanking_counter_;       ///< Remaining blanking cycles (decremented each update)
    bool zc_detected_;             ///< Zero-crossing detected, waiting for 30° delay
    float delay_counter_;          ///< Remaining delay ticks after ZC
    bool event_pending_;           ///< Synthetic Hall event ready for consumption
    uint8_t synthetic_step_;       ///< Computed next commutation step
    bool prev_above_threshold_;    ///< Previous comparison result for edge detection
    float last_floating_voltage_;  ///< Last floating phase voltage (for telemetry)
    mutable bool bemf_was_active_; ///< Hysteresis state for BEMF/Hall transition
};

} // namespace libecu

#endif // LIBECU_BEMF_OBSERVER_HPP
