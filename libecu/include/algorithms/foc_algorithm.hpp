/**
 * @file foc_algorithm.hpp
 * @brief Field-oriented control with space-vector modulation
 *
 * One cycle, start to finish:
 *
 *     i_u,i_v,i_w  ->  reject worst shunt  ->  Clarke  ->  Park  ->  Id, Iq
 *                                                                     |
 *                                          d/q current PI (volts) <----+
 *                                                    |
 *                        vector magnitude limit  ->  inverse Park  ->  Valpha, Vbeta
 *                                                                     |
 *                                                    SVPWM  ->  duty_u, duty_v, duty_w
 *
 * The maths is in foc_math.hpp and is pure; this class holds the state -
 * regulator integrators, the previous duties, the angle offset - and owns the
 * PWM interface.
 *
 * ## What FOC assumes about the bridge
 *
 * All three phases sit in PwmState::UP permanently. onEnter() sets that once
 * and raises the single COM event needed to latch it; after that nothing but
 * updateDutyCycle() is ever called, which is exactly the split the PWM driver
 * was restructured for (see the preload note in Stm32Pwm::initialize()). There
 * are no commutation events in FOC, so the six-step post-commutation blanking
 * window has nothing to blank and does not exist here.
 *
 * ## Current sampling, which is the part that bites
 *
 * The ADC is triggered at the counter peak, in the middle of the window where
 * all three low-side switches are on, so all three shunts carry phase current
 * there. That window is (1 - duty) * PWM period wide, and SVPWM drives the
 * highest phase duty towards 1 as the modulation index rises: at 0.95 duty it
 * is 2.5 us, and the injected ADC sequence alone takes 1.79 us. The sample from
 * that one phase lands in or near a switching transient and is not to be
 * trusted.
 *
 * The fix used here needs no hardware change and no moving ADC trigger: all
 * three shunts are read, the phase whose duty was highest during the sampled
 * period is discarded, and it is reconstructed from the other two using
 * i_u + i_v + i_w = 0. Two good samples are all a three-phase system needs.
 * The alternative - tracking CCR4 with the duty, as VESC does - remains open if
 * two shunts are ever not enough.
 *
 * Note the sampling *order* is a separate, unfixed hazard: on this board U is
 * ADC1 rank 1, V is ADC2 rank 1 and W is ADC2 rank 2, so W is converted
 * roughly 0.9 us later than the other two and is the most exposed to a
 * shrinking window regardless of its duty. If bench data shows W misbehaving
 * at high modulation while U and V are clean, that ordering is the reason.
 *
 * ## The angle offset, and why it is tunable
 *
 * The PLL reports angle in "steps" of 60 electrical degrees, and its zero is
 * defined by the Hall sector boundaries, not by the rotor d axis. Working the
 * offset out from the six-step table: row k of COMMUTATION_TABLE puts the
 * stator field at (k*60 - 30) electrical degrees, and MotorPLL::getNextHall()
 * applies step round(angle + 1) in FORWARD, so the field sits at
 * (angle*60 + 30) degrees. Six-step is at its torque optimum when the field
 * leads the rotor by 90 degrees, so
 *
 *     angle*60 + 30 = theta_d + 90   =>   theta_d = (angle - 1) * 60 degrees
 *
 * hence the default offset of -1 step = -60 electrical degrees. That is a
 * derivation, not a measurement: if it is wrong it is wrong by a whole
 * multiple of 60 degrees and the motor will produce little or no torque, so
 * the offset is a runtime parameter (AT+FANG) to be trimmed on the bench. An
 * inverse commutation table (MotorControlParams::useInverseCommTable) reverses
 * the step sequence and needs its own value.
 */

#ifndef LIBECU_FOC_ALGORITHM_HPP
#define LIBECU_FOC_ALGORITHM_HPP

#include <cstdint>

#include "../interfaces/pwm_interface.hpp"
#include "foc_math.hpp"
#include "motor_algorithm.hpp"

namespace libecu {

/**
 * @brief Tuning for the FOC algorithm
 */
struct FocParams {
    /**
     * @brief Fraction of the SVM linear range to use, 0.0 to 1.0
     *
     * The requested voltage vector is limited to
     * max_modulation * Vbus / sqrt(3). Leaving headroom below 1.0 keeps the
     * modulator out of overmodulation, where the output stops being sinusoidal
     * and the current loops start seeing harmonics they cannot reject.
     */
    float max_modulation;

    /**
     * @brief Upper duty clamp, matching MotorControlParams::max_duty_cycle
     *
     * A backstop only - the voltage limit above should keep the duties in
     * range - but it also enforces a minimum low-side on-time, which is what
     * keeps the two surviving shunt samples valid.
     */
    float max_duty_cycle;

    /**
     * @brief Electrical angle offset from the PLL step angle, in radians
     *
     * Default -FOC_RAD_PER_STEP (-60 electrical degrees); see the derivation
     * in the file comment.
     */
    float angle_offset_rad;

    /// @brief d-axis current reference (A). 0 for a surface-magnet motor.
    float id_target;

    /**
     * @name Cross-coupling and back-EMF feed-forward
     *
     * At speed the d and q axes are coupled through omega_e * L, and the q axis
     * additionally has to overcome the back-EMF omega_e * flux_linkage. The PI
     * loops can reject both as disturbances, but only within their bandwidth,
     * so feeding them forward buys accuracy at high speed for a few flops.
     *
     * Off by default: it depends on L and ke being right, and a wrong
     * feed-forward is harder to diagnose than none at all. Bring the drive up
     * without it, then enable it.
     */
    ///@{
    bool  use_decoupling;    ///< Enable the terms below
    float l_phase_h;         ///< Per-phase inductance (H)
    float flux_linkage_wb;   ///< ke in V.s/rad_e - the back-EMF constant
    ///@}

    /**
     * @brief Back-calculation anti-windup gain for the d/q regulators
     *
     * **Dimensionless** - the fraction of the clipped voltage excess folded
     * back into the integrator each sample, not scaled by dt. 1.0 removes the
     * whole excess in one sample and is the sensible default; 0 disables
     * back-calculation and leaves only the hard clamp at the voltage limit.
     *
     * Note this is a different quantity from PidParameters::kb, which *is*
     * scaled by dt. At a 50 us sample time the dt-scaled form produces a
     * correction some three orders of magnitude too small to oppose the
     * integration it is meant to cancel.
     */
    float anti_windup_kb;
};

/**
 * @brief Field-oriented control with space-vector modulation
 */
class FocAlgorithm final : public MotorAlgorithm {
public:
    /**
     * @brief Constructor
     * @param pwm_interface PWM interface this algorithm drives
     * @param pwm_frequency_hz Control frequency, used as the regulator sample rate
     */
    FocAlgorithm(PwmInterface& pwm_interface, uint32_t pwm_frequency_hz) noexcept;

    /// @brief Set modulation limits, angle offset and feed-forward configuration
    void setParams(const FocParams& params) noexcept;

    /// @brief Get the current configuration
    const FocParams& getParams() const noexcept { return params_; }

    /**
     * @brief Set the d/q current regulator gains
     *
     * Both axes share one set of gains, which is correct for a motor with
     * Ld == Lq. The output of these regulators is in **volts**, not duty, so
     * the gains are not interchangeable with the six-step current PI: see
     * tuneFocCurrentPi() in current_loop_tuning.hpp.
     *
     * @param kp Proportional gain (V/A)
     * @param ki Integral gain (V/A/s)
     */
    void setCurrentPiGains(float kp, float ki) noexcept;

    /// @brief Get the d/q current regulator gains
    void getCurrentPiGains(float& kp, float& ki) const noexcept;

    /// @brief Set the electrical angle offset (radians)
    void setAngleOffsetRad(float offset_rad) noexcept;

    /// @brief Get the electrical angle offset (radians)
    float getAngleOffsetRad() const noexcept { return params_.angle_offset_rad; }

    void reset() noexcept override;
    void onEnter() noexcept override;
    MotorAlgorithmOutput update(const MotorAlgorithmInput& in) noexcept override;

private:
    /**
     * @brief Phase currents with the least trustworthy shunt reconstructed
     *
     * See the file comment. Uses the duties that were in force during the
     * sampled period - last cycle's, not this cycle's.
     */
    void reconstructCurrents(const MotorAlgorithmInput& in,
                             float& i_u, float& i_v, float& i_w) const noexcept;

    PwmInterface& pwm_interface_;
    FocParams params_;

    float dt_;              ///< Regulator sample time (s)
    float kp_;              ///< d/q proportional gain (V/A)
    float ki_;              ///< d/q integral gain (V/A/s)

    float integral_d_;      ///< d-axis integrator (V)
    float integral_q_;      ///< q-axis integrator (V)

    /// Duties applied on the previous cycle - the ones the ADC sample was taken
    /// under, which is what decides which shunt to distrust.
    float prev_duty_u_;
    float prev_duty_v_;
    float prev_duty_w_;
};

} // namespace libecu

#endif // LIBECU_FOC_ALGORITHM_HPP
