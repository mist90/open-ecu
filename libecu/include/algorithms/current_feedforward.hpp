/**
 * @file current_feedforward.hpp
 * @brief Model-based feed-forward duty command for the inner current loop
 *
 * A PI current regulator on its own has to discover the operating point from
 * scratch every time: at 8 RPS roughly 0.82 of the duty cycle is just cancelling
 * back-EMF, and all of it has to come out of integral action.  That is what
 * makes the loop slow to recover from saturation and prone to windup.
 *
 * The plant is known, so most of that duty can be computed instead.  In 6-step
 * drive two phases sit in series across the bridge, so
 *
 *      duty * Vbus = 2*E + 2*R*I + 2*L*dI/dt + V0
 *
 * with E the line-to-neutral back-EMF plateau, R and L the per-phase winding
 * values, and V0 the roughly constant dead-time and diode drop.  Rearranged for
 * a target current this gives a feed-forward duty, and the PI regulator is left
 * to trim only the model error - resistance drift with temperature, magnet
 * temperature, and whatever the model misses.
 *
 * E comes from either of two places (see `use_observer_bemf`):
 *  - measured, from BemfObserver::getAmplitude(), whenever that is valid
 *  - modelled, as ke * omega_e, which works at any speed including standstill
 *
 * Parameters can be identified from captured logs with tests/test_bemf_replay.
 */

#ifndef LIBECU_CURRENT_FEEDFORWARD_HPP
#define LIBECU_CURRENT_FEEDFORWARD_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Motor electrical constants and feed-forward configuration
 */
struct CurrentFeedforwardParams {
    bool  enabled;              ///< false = update() returns 0 and the PI runs alone

    // ---- motor electrical model -------------------------------------------
    float ke_v_s_per_rad;       ///< Back-EMF constant, V*s per *electrical* radian
    float r_phase_ohm;          ///< Per-phase resistance (ohm)
    float l_phase_h;            ///< Per-phase inductance (H)
    float v_offset;             ///< Constant bridge loss: dead time + diode drops (V)

    // ---- back-EMF source ---------------------------------------------------
    bool  use_observer_bemf;    ///< Prefer the observer's measured E when it is valid

    /**
     * Low-pass on the back-EMF term, applied per PWM cycle (0 < a <= 1; 1 = off).
     *
     * This is not optional in practice.  `MotorPLL::getSpeedStepsSec()` is
     * `angle_error * kp + integral`, and with kp = 100 against an error clamped
     * to +-0.5 steps the proportional part alone swings +-50 steps/s at the PWM
     * rate, stepping at every Hall edge.  At `2*ke*(pi/3)/Vbus` that is of order
     * +-4% duty poured straight into the bridge every cycle - measured on
     * hardware as worse current tracking than PI-only.
     *
     * The back-EMF is a mechanical quantity, so filtering it hard costs nothing:
     * 0.002 at 20 kHz is a 25 ms time constant, far faster than any real speed
     * change and far slower than the PLL ripple.
     */
    float bemf_lpf_alpha;

    // ---- inductive (deadbeat) term ----------------------------------------
    /**
     * Scales the `2*L*dI/dt` term.  Class default 0; set it per motor.
     *
     * Note what this term actually is.  It keys off `target - measured`, not
     * off the setpoint derivative, so it is *feedback*, not feed-forward: a
     * clamped extra proportional gain of `di_dt_gain * 2*L*f_pwm / Vbus` duty
     * per amp.  That is also why it helps - the current PI's kp is low.
     *
     * Sizing matters more than the value of L.  On MOTOR_1 (L = 5.2 mH, 31 V,
     * 20 kHz) the raw gain is 6.7 duty/A, so gain 1.0 reaches the 0.10 clamp at
     * 15 mA of error and becomes a bang-bang relay - measured on hardware as a
     * PWM-rate limit cycle with 23% duty chatter and *worse* tracking.  Scale
     * it so the clamp corresponds to the real transient you want to cover
     * (~0.4 A of commutation ripple here), which gave 0.02.
     */
    float di_dt_gain;
    float max_di_dt_duty;       ///< Clamp on the inductive contribution, in duty

    // ---- output ------------------------------------------------------------
    float min_duty;             ///< Lower clamp on the feed-forward duty
    float max_duty;             ///< Upper clamp on the feed-forward duty
};

/**
 * @brief One PWM-synchronous input set
 */
struct CurrentFeedforwardInput {
    float target_current;       ///< Commanded phase current (A), signed - negative = regen
    float measured_current;     ///< Measured phase current (A)
    float bus_voltage;          ///< DC bus voltage (V)
    float speed_steps_per_sec;  ///< Electrical speed, signed; magnitude is used
    float bemf_peak_volts;      ///< Measured |e| plateau from BemfObserver (V)
    bool  bemf_valid;           ///< Whether bemf_peak_volts is usable
};

/**
 * @brief Feed-forward duty command for the inner current loop
 *
 * Usage:
 *  - Call setParameters() (or setMotorConstants()) once at startup
 *  - Call update() from the PWM ISR and add the result to the PI output
 *
 * @note The PI regulator that trims this must be allowed a *negative* output,
 *       otherwise it cannot correct an over-prediction. See the note on
 *       `pid_current_regulator` in main.cpp.
 */
class CurrentFeedforward {
public:
    /**
     * @brief Constructor
     * @param pwm_frequency PWM frequency in Hz - sets dt for the inductive term
     */
    explicit CurrentFeedforward(float pwm_frequency) noexcept;

    void setParameters(const CurrentFeedforwardParams& params) noexcept;
    const CurrentFeedforwardParams& getParameters() const noexcept { return params_; }

    /**
     * @brief Set the electrical constants from an identification run
     * @param ke_v_s_per_rad Back-EMF constant, V*s per electrical radian
     * @param r_phase_ohm    Per-phase resistance (ohm)
     * @param l_phase_h      Per-phase inductance (H); pass 0 if unknown
     * @param v_offset       Constant bridge loss (V); pass 0 if R already absorbs it
     */
    void setMotorConstants(float ke_v_s_per_rad, float r_phase_ohm,
                           float l_phase_h, float v_offset) noexcept;

    /**
     * @brief Compute the feed-forward duty for this PWM cycle
     * @return Duty in [min_duty, max_duty]; 0 when disabled
     */
    float update(const CurrentFeedforwardInput& in) noexcept;

    /// @brief Clear the telemetry snapshot (no internal state is carried)
    void reset() noexcept;

    /// @return Back-EMF the model predicts at this speed (V), ignoring the observer
    float predictBemfVolts(float speed_steps_per_sec) const noexcept;

    /**
     * @brief Telemetry snapshot, broken down by contribution
     */
    struct Info {
        float duty;             ///< Total feed-forward duty actually returned
        float duty_bemf;        ///< Share from 2*E
        float duty_resistive;   ///< Share from 2*R*I
        float duty_inductive;   ///< Share from 2*L*dI/dt (after clamping)
        float duty_offset;      ///< Share from V0
        float bemf_volts;       ///< E used this cycle (V)
        bool  bemf_measured;    ///< E came from the observer rather than ke*omega
        bool  saturated;        ///< Output hit min_duty or max_duty
        float bemf_raw_volts;   ///< E before the low-pass, for diagnosing noise
    };

    Info getInfo() const noexcept { return info_; }

private:
    float pwm_frequency_;
    CurrentFeedforwardParams params_;
    Info  info_;
    float bemf_filtered_;      ///< LPF state for the back-EMF term (V)
    bool  bemf_primed_;        ///< false until the filter has been seeded
};

} // namespace libecu

#endif // LIBECU_CURRENT_FEEDFORWARD_HPP
