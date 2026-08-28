/**
 * @file motor_algorithm.hpp
 * @brief Common interface for the electrical control algorithms
 *
 * Everything between "here are the measurements" and "here are the PWM duties"
 * lives behind this interface. `BldcController` gathers the inputs once per
 * PWM interrupt - phase currents, bus voltage, rotor angle, commands - hands
 * them to whichever algorithm is selected, and does nothing else with the
 * bridge. The algorithms own the PWM interface outright: six-step needs
 * per-phase switching states and a COM event, FOC needs three independent
 * duties and no COM event at all, and pushing that difference up into the
 * controller was what made the old handler hard to follow.
 *
 * The two axes of mode are deliberately orthogonal:
 *
 * | DriveAlgorithm | ElectricMode  | what runs                                   |
 * |----------------|---------------|---------------------------------------------|
 * | SIX_STEP       | VOLTAGE_MODE  | trapezoidal, duty commanded directly         |
 * | SIX_STEP       | CURRENT_MODE  | trapezoidal, inner current PI on the shunt   |
 * | FOC            | VOLTAGE_MODE  | SVPWM, Vq commanded directly, no current loop|
 * | FOC            | CURRENT_MODE  | SVPWM, d/q current PI - full field-oriented  |
 *
 * The two voltage-mode rows exist for bring-up: they exercise the modulator
 * and the angle without a current loop in the way, which is the only sane way
 * to find a wrong angle offset.
 */

#ifndef LIBECU_MOTOR_ALGORITHM_HPP
#define LIBECU_MOTOR_ALGORITHM_HPP

#include <cstdint>

#include "motor_pll.hpp"   // DriveMode

namespace libecu {

/**
 * @brief Electric drive mode (electrical control strategy)
 */
enum class ElectricMode : uint8_t {
    VOLTAGE_MODE = 0,  ///< Direct voltage/duty cycle control
    CURRENT_MODE = 1   ///< Current control with an inner PI loop at PWM rate
};

/**
 * @brief Which electrical algorithm drives the bridge
 */
enum class DriveAlgorithm : uint8_t {
    SIX_STEP = 0,  ///< Trapezoidal six-step commutation (the long-standing mode)
    FOC = 1        ///< Field-oriented control with space-vector modulation
};

/**
 * @brief Everything an algorithm is allowed to know, gathered once per ISR
 *
 * Sampled by BldcController at the top of the PWM interrupt so that both
 * algorithms see exactly the same snapshot and neither one reaches into
 * hardware behind the other's back.
 */
struct MotorAlgorithmInput {
    /**
     * @name Phase currents, in the low-side shunt's sign convention
     *
     * Straight from `AdcInterface::readAllCurrents()`, and **positive means
     * current flowing OUT of the phase**, down through the low-side switch and
     * its shunt to ground. That is the opposite of the usual motor convention
     * (positive into the phase) that the Clarke transform expects, so anything
     * doing vector maths on these has to negate them first - see
     * FocAlgorithm::reconstructCurrents().
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
    float target_current;   ///< Torque command (A); magnitude, sign from drive_mode
    float duty_command;     ///< Voltage-mode command, 0.0 to 1.0
    uint8_t step;           ///< Six-step target commutation step, 0-5
    DriveMode drive_mode;   ///< FORWARD / REVERSE / NEUTRAL
    ElectricMode electric_mode; ///< Voltage or current control
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
    float duty;             ///< Scalar modulation depth for telemetry (see note)
    float measured_current; ///< Torque-relevant current (A) fed to the outer loop
    float i_d;              ///< Measured d-axis current (A), FOC only
    float i_q;              ///< Measured q-axis current (A), FOC only
    float v_d;              ///< Applied d-axis voltage (V), FOC only
    float v_q;              ///< Applied q-axis voltage (V), FOC only
    float angle_e_rad;      ///< Electrical angle the algorithm used (rad)
    int8_t saturation;      ///< +1/-1 if the actuator is clamped high/low, else 0
};

/**
 * @brief Base class for an electrical control algorithm
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

    /**
     * @brief Run one control cycle and drive the PWM outputs
     * @param in Measurements and commands for this cycle
     * @return Duties applied plus diagnostics
     */
    virtual MotorAlgorithmOutput update(const MotorAlgorithmInput& in) noexcept = 0;
};

} // namespace libecu

#endif // LIBECU_MOTOR_ALGORITHM_HPP
