/**
 * @file current_loop_tuning.hpp
 * @brief PI gains for the inner current loop, derived from the motor model
 *
 * In 6-step drive two phases sit in series across the bridge, so the
 * small-signal response of current to duty, about any operating point (the
 * back-EMF is a slowly varying disturbance, not part of this dynamic), is
 *
 *      dI/dduty = Vbus / (2R + 2L*s)
 *
 * A PI controller `kp + ki/s` has a zero at `1/Ti` with `Ti = kp/ki`. Placing
 * that zero on the plant pole (`Ti = L/R`) cancels it, and the open loop
 * collapses to a pure integrator:
 *
 *      Lopen(s) = kp * Vbus / (2*L*s)
 *
 * whose crossover is therefore `w_c = kp*Vbus/(2L)`. Solving for the gains at a
 * chosen bandwidth:
 *
 *      kp = 2*L*w_c / Vbus
 *      ki = 2*R*w_c / Vbus          (which makes Ti = L/R automatically)
 *
 * Both gains scale with 1/Vbus, so they are strictly correct only at the bus
 * voltage they were computed for; a battery sagging 20% detunes the loop by
 * 20%, which is usually acceptable.
 *
 * @note L is best measured from the *closed-loop current step response*
 *       (L = kp*Vbus / (2*w_measured)), not from the demagnetisation window -
 *       the latter quantises to whole PWM cycles and on this hardware
 *       over-estimated L by roughly an order of magnitude.
 */

#ifndef LIBECU_CURRENT_LOOP_TUNING_HPP
#define LIBECU_CURRENT_LOOP_TUNING_HPP

#include "pid_controller.hpp"

namespace libecu {

/**
 * @brief Electrical model of one driven phase pair
 */
struct CurrentLoopModel {
    float l_phase_h;     ///< Per-phase inductance (H)
    float r_phase_ohm;   ///< Per-phase resistance (ohm)
    float bus_voltage;   ///< Nominal DC bus voltage the gains are computed for (V)
    float bandwidth_hz;  ///< Desired closed-loop crossover; keep below f_pwm/10
};

/**
 * @brief Compute current-loop PI gains from the model
 *
 * Fills `kp`, `ki` and clears `kd`. Output and integral limits, and the sample
 * time, are left to the caller.
 *
 * @return PidParameters with kp/ki set; zeroed if the model is not usable.
 */
inline PidParameters tuneCurrentPi(const CurrentLoopModel& m) noexcept {
    PidParameters p{};
    if (m.l_phase_h <= 0.0f || m.r_phase_ohm <= 0.0f ||
        m.bus_voltage <= 0.0f || m.bandwidth_hz <= 0.0f) {
        return p;
    }
    const float w_c = 6.28318531f * m.bandwidth_hz;
    p.kp = 2.0f * m.l_phase_h   * w_c / m.bus_voltage;
    p.ki = 2.0f * m.r_phase_ohm * w_c / m.bus_voltage;
    p.kd = 0.0f;
    return p;
}

/**
 * @brief Compute d/q current-loop PI gains for FOC from the same model
 *
 * FOC differs from six-step in two ways that both change the gains:
 *
 *  1. In the rotor frame each axis sees **one** phase, not two in series, so
 *     the plant is `1/(R + L*s)` rather than `1/(2R + 2L*s)`.
 *  2. The regulator output is a **voltage in volts**, not a duty cycle, so
 *     Vbus does not appear in the gains at all. That is the better place to
 *     be: the loop no longer detunes when the battery sags, and the
 *     modulation limit becomes a physical voltage rather than a number
 *     between 0 and 1.
 *
 * Placing the PI zero on the plant pole (`Ti = L/R`) again collapses the open
 * loop to `kp/(L*s)`, so `w_c = kp/L` and
 *
 *      kp = L * w_c     (V/A)
 *      ki = R * w_c     (V/A/s)
 *
 * `bus_voltage` in the model is ignored here; it is kept in the struct so one
 * CurrentLoopModel can feed both tuners.
 *
 * @return PidParameters with kp/ki set; zeroed if the model is not usable.
 */
inline PidParameters tuneFocCurrentPi(const CurrentLoopModel& m) noexcept {
    PidParameters p{};
    if (m.l_phase_h <= 0.0f || m.r_phase_ohm <= 0.0f || m.bandwidth_hz <= 0.0f) {
        return p;
    }
    const float w_c = 6.28318531f * m.bandwidth_hz;
    p.kp = m.l_phase_h   * w_c;
    p.ki = m.r_phase_ohm * w_c;
    p.kd = 0.0f;
    return p;
}

/// @return Integral time constant implied by the gains (s); should equal L/R
inline float currentPiTi(const PidParameters& p) noexcept {
    return (p.ki > 0.0f) ? (p.kp / p.ki) : 0.0f;
}

} // namespace libecu

#endif // LIBECU_CURRENT_LOOP_TUNING_HPP
