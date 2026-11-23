/**
 * @file motor_algorithm.hpp
 * @brief Common interface for the electrical control algorithms
 *
 * Both algorithms regulate a signed torque-current demand. Six-step uses the
 * conducting shunt and commits a new set of phase states on commutation. FOC
 * regulates d/q current and writes three independent PWM duties. On the
 * carrier they work directly on the controller's electrical storage; the
 * value types below serve standalone callers.
 */

#ifndef LIBECU_MOTOR_ALGORITHM_HPP
#define LIBECU_MOTOR_ALGORITHM_HPP

#include <cstdint>

#include "motor_types.hpp"

namespace libecu {

/**
 * @brief One standalone sample and its torque-current command.
 */
struct MotorAlgorithmInput {
    /**
     * @name Phase currents, in the low-side shunt's sign convention
     *
     * Straight from `AdcInterface::readAllCurrents()`, and **positive means
     * current flowing OUT of the phase**, down through the low-side switch and
     * its shunt to ground. That is the opposite of the usual motor convention
     * (positive into the phase) that the Clarke transform expects, so anything
     * doing vector maths on these has to negate them first.
     *
     * Measured, not assumed: six-step driving forward at 3.0 RPS against a
     * +0.525 A target reads +0.538 A on the DOWN phase, 99.1 % of samples
     * positive. The DOWN phase is by definition the one current leaves
     * through, so a positive reading there is current leaving.
     *
     * Six-step wants exactly this convention - it reads the DOWN phase and
     * compares against a positive current command - which is why the raw
     * values are carried here rather than being negated at the source.
     */
    ///@{
    float i_u;              ///< Phase U shunt current (A), signed
    float i_v;              ///< Phase V shunt current (A), signed
    float i_w;              ///< Phase W shunt current (A), signed
    ///@}
    float bus_voltage;      ///< Measured DC bus voltage (V)
    float angle_steps;      ///< Rotor electrical angle from the PLL, [0, 6) steps
    float speed_steps_s;    ///< Rotor electrical speed (steps/s), signed
    float target_current;   ///< Torque-current command (A) relative to drive_mode
    uint8_t step;           ///< Six-step target commutation step, 0-5
    DriveMode drive_mode;   ///< FORWARD / REVERSE / NEUTRAL
};

/**
 * @brief What the algorithm produced, for telemetry and the outer loop
 *
 * The duties have already been written to the PWM interface by the time this
 * is returned; they are reported so the oscilloscope capture and the telemetry
 * can see them without asking the timer.
 */
struct MotorAlgorithmOutput {
    float duty_u;           ///< Phase U duty actually applied
    float duty_v;           ///< Phase V duty actually applied
    float duty_w;           ///< Phase W duty actually applied
    float duty;             ///< Six-step duty; FOC modulation magnitude is computed by the slow controller tick
    float measured_current; ///< Torque-relevant current (A) fed to the outer loop
    float i_d;              ///< Measured d-axis current (A), FOC only
    float i_q;              ///< Measured q-axis current (A), FOC only
    float v_d;              ///< Applied d-axis voltage (V), FOC only
    float v_q;              ///< Applied q-axis voltage (V); six-step reports duty × bus voltage
    float angle_e_rad;      ///< Electrical angle the algorithm used (rad)
    int8_t saturation;      ///< +1/-1 if the actuator is clamped high/low, else 0
};

/**
 * @brief Common lifecycle for electrical control algorithms
 *
 * Implementations hold a reference to the PWM interface and are the only thing
 * in the system that writes to it while the drive is running.
 */
class MotorAlgorithm {
public:
    virtual ~MotorAlgorithm() = default;

    /**
     * @brief Clear all internal state (integrators, cached steps, counters)
     *
     * Called when the drive goes to NEUTRAL or the gains change. Must not touch
     * hardware - the bridge may be disabled at this point.
     */
    virtual void reset() noexcept = 0;

    /**
     * @brief Take ownership of the bridge
     *
     * Called when this algorithm becomes the selected one, before its first
     * update(). This is where an algorithm establishes the switching
     * configuration it assumes for the rest of its life: FOC puts all three
     * phases in UP once here and never raises another COM event, six-step
     * invalidates its cached step so the next update() re-commutates from
     * scratch.
     */
    virtual void onEnter() noexcept = 0;

#if !defined(LIBECU_NATIVE_CARRIER_ONLY) || !LIBECU_NATIVE_CARRIER_ONLY
    /** Value-facing compatibility path on ports using erased algorithms. */
    virtual MotorAlgorithmOutput update(const MotorAlgorithmInput& in) noexcept = 0;
#endif
    // Native-only ports do not need to retain a value adapter in the vtable.
};

} // namespace libecu

#endif // LIBECU_MOTOR_ALGORITHM_HPP
