/**
 * @file bemf_observer.hpp
 * @brief BEMF zero-crossing observer for sensorless 6-step BLDC commutation
 *
 * Detects back-EMF zero-crossings on the floating phase during 6-step
 * trapezoidal commutation and generates synthetic Hall sensor events
 * for the MotorPLL.
 *
 * The detector is polarity-normalised: the raw floating-phase voltage is
 * referenced to the motor neutral and multiplied by a per-step sign so that
 * the resulting signal (`v_diff`) is always negative before the zero-crossing
 * and positive after it, on every one of the six steps and in both directions.
 * A single "crosses zero upwards" test then covers all cases, which removes
 * the amplitude-dependent bias of a fixed +-5%-of-Vbus threshold pair.
 *
 * Two commutation-timing strategies are available (see BemfTimingMode):
 * a speed-derived 30-degree countdown, and a VESC-style flux integration
 * that needs no speed estimate at all.
 */

#ifndef LIBECU_BEMF_OBSERVER_HPP
#define LIBECU_BEMF_OBSERVER_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Strategy used to place the commutation event after a zero-crossing
 */
enum class BemfTimingMode : uint8_t {
    /**
     * Count down 30 electrical degrees using the PLL speed estimate.
     * Simple, but the delay depends on a speed estimate that is itself
     * driven by this observer's own events (positive feedback), and it
     * degrades whenever the rotor accelerates inside a step.
     */
    DELAY_30DEG = 0,

    /**
     * Integrate the polarity-normalised BEMF from the zero-crossing and
     * commutate when the integral reaches `integrator_limit_vs`.
     *
     * The BEMF amplitude is proportional to speed and the duration of the
     * 30-degree arc is inversely proportional to speed, so the integral over
     * that arc is speed-independent - one constant covers the whole speed
     * range.  Integration is also a low-pass filter, so per-sample ADC noise
     * averages out instead of directly displacing the commutation instant.
     * This is the scheme VESC uses (mcpwm.c, COMM_MODE_INTEGRATE).
     */
    FLUX_INTEGRATE = 1,
};

/**
 * @brief Configuration parameters for the BEMF observer
 */
struct BemfObserverParams {
    // ---- hybrid Hall/BEMF mode gating -------------------------------------
    float min_duty;               ///< Minimum duty for ON-time BEMF sensing; below this, Hall is used
    float transition_speed_low;   ///< |speed| below which Hall is used (steps/sec)
    float transition_speed_high;  ///< |speed| above which BEMF is used exclusively (steps/sec)
    bool  is_inverse_commutation; ///< Motor uses the inverse commutation table

    // ---- demagnetisation blanking -----------------------------------------
    float blanking_cycles;        ///< Hard floor of the blanking window, in PWM cycles
    float blanking_fraction;      ///< Extra blanking as a fraction of the step period (0..0.45)

    // ---- zero-crossing detection ------------------------------------------
    float   zc_deadband_volts;    ///< |v_diff| below this counts as zero (ADC/EMI noise floor)
    uint8_t zc_confirm_samples;   ///< Consecutive positive samples required to accept a crossing
    bool    use_virtual_neutral;  ///< Reference = (Vu+Vv+Vw)/3 instead of Vbus/2 (needs unclipped dividers)
    bool    auto_polarity;        ///< Learn the global BEMF slope sign at runtime
    float   rail_margin;          ///< Discard samples this fraction of Vbus away from either rail

    // ---- commutation timing ------------------------------------------------
    BemfTimingMode timing_mode;   ///< 30-degree countdown or flux integration
    float integrator_limit_vs;    ///< Volt-seconds from ZC to commutation (FLUX_INTEGRATE)
    float phase_advance;          ///< 0..0.9: fraction of the 30-degree arc removed (advance)
    bool  auto_learn_limit;       ///< Learn integrator_limit_vs from Hall-driven steps
    float learn_alpha;            ///< LPF coefficient for the learned limit (0..1)

    // ---- loss of lock ------------------------------------------------------
    float max_step_periods;       ///< Declare loss of lock if no event within this many step periods

    // ---- BEMF amplitude estimator ------------------------------------------
    float amplitude_lpf_alpha;    ///< LPF coefficient for the amplitude estimate (0..1)
};

/**
 * @brief Back-EMF amplitude estimate, for model-based (feed-forward) control
 *
 * Recovered from a least-squares fit of the polarity-normalised BEMF ramp
 * across one commutation step.  Within a step the floating winding sits in the
 * linear transition of the trapezoid, sweeping from -E to +E, so the fitted
 * slope `s` and the measured step period `T` give the plateau directly:
 *
 *      E = s * T / 2
 *
 * The fit uses every sample that survived the rail guard, so its noise scales
 * as 1/sqrt(n) instead of taking a single peak reading.
 *
 * Intended use - the 6-step voltage loop with two phases in series is
 *
 *      duty * Vbus = 2*E + 2*R_phase*I + 2*L_phase*dI/dt
 *
 * so a feed-forward duty for a target current is
 *
 *      duty_ff = (2*E + 2*R_phase*I_target) / Vbus
 *
 * leaving the PID to trim only the model error.  `ke_v_s_per_rad` is the term
 * that makes this work below the BEMF transition speed too: E is only measured
 * while the observer runs, but ke is a motor constant, so once learned the
 * model can predict E = ke * omega from the PLL speed at any speed.
 */
struct BemfAmplitude {
    float    peak_volts;          ///< |e| plateau, line-to-neutral (V)
    float    line_to_line_volts;  ///< 2 * peak_volts - what the driven pair sees (V)
    float    slope_v_per_s;       ///< Fitted dv/dt of the BEMF ramp (V/s)
    float    ke_v_s_per_rad;      ///< peak_volts / omega_electrical (V*s/rad)
    float    step_period_s;       ///< Step period the last fit was scaled by
    uint32_t fit_samples;         ///< Samples in the last fit (0 = never fitted)
    bool     valid;               ///< Last fit met the sample count and slope-sign checks
};

/**
 * @brief One PWM-synchronous measurement set handed to the observer
 */
struct BemfObserverInput {
    float   v_float;              ///< Floating-phase voltage (V, referred to inverter ground)
    float   v_u;                  ///< Phase U voltage (V) - only used for the virtual neutral
    float   v_v;                  ///< Phase V voltage (V)
    float   v_w;                  ///< Phase W voltage (V)
    float   bus_voltage;          ///< DC bus voltage (V)
    uint8_t step;                 ///< Commutation step currently applied to the bridge (0-5)
    float   speed_steps_per_sec;  ///< PLL speed estimate, signed (negative in REVERSE)
};

/**
 * @brief BEMF zero-crossing observer for sensorless commutation
 *
 * Usage:
 *  - Call onCommutation() whenever the applied commutation step changes
 *  - Call update() from the PWM ISR with a fresh BemfObserverInput
 *  - When update() returns true, feed getSyntheticHallStep() to MotorPLL::updateHall()
 *  - Use isBemfModeActive() / shouldIgnoreHall() for the hybrid mode decision
 */
class BemfObserver {
public:
    /**
     * @brief Constructor
     * @param pwm_frequency PWM frequency in Hz (e.g. 20000.0f)
     */
    explicit BemfObserver(float pwm_frequency) noexcept;

    /// @brief Set observer parameters
    void setParameters(const BemfObserverParams& params) noexcept;

    /// @return Current observer parameters
    const BemfObserverParams& getParameters() const noexcept { return params_; }

    /**
     * @brief Process one PWM-synchronous BEMF measurement
     * @param in Measurement set for this PWM cycle
     * @return true when a synthetic Hall event is ready
     */
    bool update(const BemfObserverInput& in) noexcept;

    /// @return Synthetic Hall position (0-5) to feed to MotorPLL::updateHall()
    uint8_t getSyntheticHallStep() const noexcept;

    /**
     * @brief Notify the observer that a commutation has been applied
     *
     * Restarts the blanking window and the per-step detection state.  Also
     * where the measured step period and the learned integrator limit are
     * updated.
     *
     * @param new_step The commutation step that was just applied (0-5)
     */
    void onCommutation(uint8_t new_step) noexcept;

    /**
     * @brief Check whether BEMF mode is active at the given speed and duty
     * @param speed_steps_per_sec PLL speed estimate, signed - magnitude is used
     * @param duty_cycle Current PWM duty cycle (0.0-1.0)
     */
    bool isBemfModeActive(float speed_steps_per_sec, float duty_cycle) noexcept;

    /**
     * @brief Check whether real Hall events should be suppressed
     *
     * Returns false while the observer has lost lock, so the Hall sensors
     * automatically take over again until the observer re-synchronises.
     */
    bool shouldIgnoreHall(float speed_steps_per_sec, float duty_cycle) const noexcept;

    /// @return true if no zero-crossing has arrived within max_step_periods
    bool isSyncLost() const noexcept { return sync_lost_; }

    /// @return Integrator limit learned from Hall-driven steps (V*s), 0 if none yet
    float getLearnedIntegratorLimit() const noexcept { return learned_limit_; }

    /**
     * @brief Filtered back-EMF amplitude estimate
     *
     * Updated once per commutation step, from a least-squares fit of the BEMF
     * ramp over that step.  Values are low-pass filtered with
     * `amplitude_lpf_alpha` and retain their last good value when a step
     * produces no usable fit, so the caller can read this every cycle.
     *
     * @note Only updated while the observer is receiving samples, i.e. above
     *       `min_duty` and while a phase is floating.  Use `ke_v_s_per_rad`
     *       together with the PLL speed to predict the BEMF outside that range.
     */
    BemfAmplitude getAmplitude() const noexcept;

    /// @return Filtered |e| plateau, line-to-neutral (V); 0 if never fitted
    float getBemfPeakVolts() const noexcept { return amp_peak_; }

    /// @return Filtered BEMF constant (V*s per electrical radian); 0 if never fitted
    float getBemfConstant() const noexcept { return amp_ke_; }

    /**
     * @brief Predict the BEMF plateau at an arbitrary speed from the learned constant
     * @param speed_steps_per_sec Electrical speed, signed - magnitude is used
     * @return |e| plateau in Volts, or 0 if the constant has not been learned
     * @note One step is 60 electrical degrees, so omega_e = (pi/3) * steps_per_sec.
     */
    float predictBemfVolts(float speed_steps_per_sec) const noexcept;

    /**
     * @brief Telemetry snapshot of observer internal state
     */
    struct BemfInfo {
        bool    bemf_active;        ///< BEMF mode currently active
        bool    zc_detected;        ///< Zero-crossing confirmed, waiting to fire
        bool    sync_lost;          ///< No zero-crossing within max_step_periods
        bool    blanked;            ///< Currently inside the blanking window
        uint8_t synthetic_step;     ///< Last synthetic Hall position emitted
        int8_t  polarity;           ///< Learned global BEMF slope sign (+1 / -1)
        float   floating_voltage;   ///< Last floating phase voltage (V)
        float   v_ref;              ///< Neutral reference used for the last sample (V)
        float   v_diff;             ///< Last polarity-normalised BEMF error (V)
        float   integrator_vs;      ///< Flux integral accumulated since the ZC (V*s)
        float   integrator_limit;   ///< Limit currently in force (V*s)
        float   zc_fraction;        ///< Sub-sample position of the ZC inside the last PWM tick
        float   last_step_period_s; ///< Measured duration of the previous commutation step
    };

    /// @return Snapshot of observer internal state (for telemetry)
    BemfInfo getInfo() const noexcept;

private:
    /// @return Effective flux-integral limit in V*s, after learning and phase advance
    float effectiveLimit() const noexcept;

    /// @return Expected duration of one commutation step, in seconds
    float expectedStepPeriod(float speed_steps_per_sec) const noexcept;

    /// @return Nominal BEMF slope sign for the step, before the learned flip
    int8_t baseSlopeSign(uint8_t step, float speed_steps_per_sec) const noexcept;

    /// Accumulate one sample into the per-step least-squares slope fit
    void accumulateFit(float v_diff_raw) noexcept;

    /// Solve the fit for the step that just ended and fold it into the estimate
    void finishAmplitudeFit(float step_period) noexcept;

    void resetStepState() noexcept;

    float pwm_frequency_;          ///< PWM frequency in Hz
    float dt_;                     ///< 1 / pwm_frequency_
    BemfObserverParams params_;

    // --- per-step detection state ---
    float   time_since_comm_;      ///< Seconds since the last commutation
    int32_t samples_valid_;        ///< Post-blanking samples seen in this step
    float   v_diff_prev_;          ///< Previous polarity-normalised BEMF error (V)
    int32_t confirm_count_;        ///< Consecutive positive v_diff samples
    bool    pending_;              ///< A crossing candidate is being confirmed
    float   pending_integral_;     ///< Flux accumulated since the candidate crossing (V*s)
    float   zc_fraction_;          ///< Sub-sample position of the crossing in its PWM tick
    bool    zc_active_;            ///< Crossing confirmed, waiting for the commutation instant
    float   integrator_;           ///< Flux integral since the crossing (V*s)
    float   delay_ticks_;          ///< Remaining ticks in DELAY_30DEG mode
    bool    fired_;                ///< A synthetic event was already emitted for this step
    uint8_t synthetic_step_;       ///< Synthetic Hall position to report

    // --- cross-step state ---
    float   last_step_period_;     ///< Measured duration of the previous step (s)
    float   learned_limit_;        ///< Flux limit learned from Hall-driven steps (V*s)
    int8_t  polarity_;             ///< Learned global slope sign (+1 / -1)
    int32_t polarity_disagree_;    ///< Consecutive steps whose first sample contradicts polarity_
    bool    sync_lost_;            ///< No zero-crossing within max_step_periods
    bool    bemf_was_active_;      ///< Hysteresis state for the BEMF/Hall handover

    // --- telemetry ---
    float   last_v_float_;
    float   last_v_ref_;
    float   last_v_diff_;

    // --- crossing bracket (sub-sample interpolation) ---
    float   v_neg_last_;           ///< Last strictly-negative v_diff sample (V)
    int32_t neg_age_;              ///< PWM ticks between that sample and the current one
    bool    have_neg_;             ///< v_neg_last_ holds a usable value
    float   t_since_zc_;           ///< Seconds since the interpolated crossing instant

    // --- BEMF amplitude estimator ---
    // Least-squares accumulators over the current step.  The abscissa is the
    // PWM tick index rather than seconds: the sums then stay O(1..1e3) instead
    // of O(1e-8), which keeps the normal equations well away from float
    // cancellation.
    int32_t ticks_since_comm_;     ///< PWM ticks since the last commutation
    int32_t fit_n_;                ///< Samples accumulated this step
    float   fit_sx_;               ///< sum(k)
    float   fit_sxx_;              ///< sum(k^2)
    float   fit_sy_;               ///< sum(v)
    float   fit_sxy_;              ///< sum(k*v)
    float   amp_peak_;             ///< Filtered |e| plateau (V)
    float   amp_slope_;            ///< Filtered BEMF ramp slope (V/s)
    float   amp_ke_;               ///< Filtered BEMF constant (V*s/rad electrical)
    float   amp_step_period_;      ///< Step period the last fit used (s)
    int32_t amp_fit_samples_;      ///< Samples in the last attempted fit
    bool    amp_valid_;            ///< Last attempted fit succeeded
};

} // namespace libecu

#endif // LIBECU_BEMF_OBSERVER_HPP
