/**
 * @file six_step_algorithm.hpp
 * @brief Trapezoidal six-step commutation, self-contained
 *
 * This is the whole six-step drive: the commutation table, the atomic
 * application of a step to the bridge, the choice of which shunt is carrying
 * phase current, the post-commutation blanking window, and the inner current
 * PI. It replaces the former CommutationController plus the six-step-specific
 * half of BldcController::pwmInterruptHandler(), which between them had the
 * commutation table in one file, the shunt selection in another, and the
 * blanking counter in a third.
 */

#ifndef LIBECU_SIX_STEP_ALGORITHM_HPP
#define LIBECU_SIX_STEP_ALGORITHM_HPP

#include <cstdint>

#include "pwm_interface.hpp"
#include "motor_algorithm.hpp"
#include "pi_float_interface.hpp"
#include "ecu_config.hpp"
#include "motor_samples.hpp"

namespace libecu {
template<class P> struct MotorStatusT;

/**
 * @brief One row of the six-step commutation table
 */
struct CommutationStep {
    PwmState phase_u;    ///< Phase U PWM state
    PwmState phase_v;    ///< Phase V PWM state
    PwmState phase_w;    ///< Phase W PWM state
};

/**
 * @brief Tuning for the six-step algorithm
 */
struct SixStepParams {
    float max_duty_cycle;   ///< Upper duty clamp (0.0 to 1.0)

    /**
     * @brief PWM cycles to blank the current measurement for after a commutation
     *
     * For the first cycles after the phases change, the outgoing phase is still
     * freewheeling through the body diodes, so the shunt of the nominal DOWN
     * phase is not carrying the phase current. Those samples are wrong by up to
     * an amp, and handing them to the current PI turns a measurement artefact
     * into real duty modulation - which is torque ripple the motor can be heard
     * and felt making. During the blanking window the duty is held and the PI
     * is not updated, so the integrator cannot wind up on the bad samples.
     *
     * Measured on MOTOR_1 at 3.9 RPS: residual spread of the current reading is
     * 0.60 A one cycle after commutation, 0.28 A at three, 0.17 A at six and
     * 0.02 A - the noise floor - from nine onwards. Six covers the bulk of the
     * transient without holding the duty for much of a short sector.
     *
     * 0 disables blanking.
     */
    uint8_t blanking_cycles;
};

/**
 * @brief Six-step trapezoidal commutation with an inner current loop
 */
class SixStepAlgorithm final : public MotorAlgorithm {
public:
    using Profile = ActiveNumericProfile;
    using Current = Profile::Current;
    using Duty = Profile::Duty;
    using Input = MotorAlgorithmInputT<Profile>;
    using Output = MotorAlgorithmOutputT<Profile>;
    using Frame = MotorAlgorithmFrameT<Profile>;
    Output updateNative(const Input& input) noexcept;
    /** @brief PWM path: overwrite and apply one caller-owned current frame. */
    void updateNative(Frame& frame) noexcept;
    /** Serialized carrier path: read commands and publish results in the motor status. */
    void updateOwnedCarrier(const Input& in, MotorStatusT<Profile>& status) noexcept;
    /**
     * @brief Constructor
     * @param pwm_interface PWM interface this algorithm drives
     */
    explicit SixStepAlgorithm(PwmInterface& pwm_interface) noexcept;

    /// @brief Set duty clamp and blanking window
    void setParams(const SixStepParams& params) noexcept;

    /// @brief Set the inner current-loop PI parameters (output is duty, 0 to 1)
    /// @return false on unrepresentable tuning, retaining the previous configuration
    bool setCurrentPi(const PiParameters& params) noexcept;
    using CurrentPi = PiFloatInterface<ActivePiProfile::CurrentPiTypes, ActiveTiming::CURRENT_HZ>;
    /** @brief Install a validated, reset regulator under the caller's publication mask. */
    void installCurrentPi(const CurrentPi& next) noexcept { current_pi_ = next; }

    /// @brief Get the inner current-loop PI parameters
    const PiParameters& getCurrentPi() const noexcept;

    /// @return Whether the current PI has a representable configuration.
    bool isCurrentPiConfigured() const noexcept { return current_pi_.isConfigured(); }

    void reset() noexcept override;
    void onEnter() noexcept override;
    MotorAlgorithmOutput update(const MotorAlgorithmInput& in) noexcept;

    /**
     * @brief Switching state currently applied to a phase
     * @param channel PWM channel
     * @return Cached phase state, OFF before the first commutation
     */
    PwmState getPhaseState(PwmChannel channel) const noexcept;

private:
    /**
     * @brief Apply one commutation step to the bridge
     *
     * Switching states and duty travel on separate paths - states are
     * preloaded against the COM event raised at the end of this function, duty
     * is preloaded against the timer's update event - and neither has to wait
     * for the other, because no phase state is encoded in a compare register.
     * See the preload note in Stm32Pwm::initialize(); getting this wrong cost
     * two measured regressions.
     */
    template<bool Owned> void applyCommutationStep(const CommutationStep& step, Duty duty_cycle) noexcept;
    template<bool Owned> void writeDuty(Duty duty) noexcept;
    template<bool Owned, class Commands, class Result>
    void updateInto(const Input& in, const Commands& commands, Result& out) noexcept;

    /**
     * @brief Phase current from the shunt that is actually carrying it
     *
     * In six-step two phases conduct and one floats. The low-side switch of the
     * DOWN phase is on for the whole period, so its shunt sees the full phase
     * current at the sampling instant; the UP phase only conducts through its
     * shunt while its low side is on, and the OFF phase carries nothing.
     *
     * The cached states are the ones that were in force *during* the period
     * that was just sampled, not the ones about to be applied - which is why
     * the cache is read before this cycle's commutation, never after.
     */
    Current currentFromActivePhase(const Input& in) const noexcept;

    static const CommutationStep COMMUTATION_TABLE[6];

    PwmInterface& pwm_interface_;
    CurrentPi current_pi_;
    SixStepParams params_;
    Duty max_duty_ = number_v<Duty, 95, 100>;

    uint8_t current_step_;          ///< Step currently applied, 0xFF if none
    PwmState cached_phase_u_state_;
    PwmState cached_phase_v_state_;
    PwmState cached_phase_w_state_;

    /// PWM cycles since the last commutation; gates the blanking window.
    /// Saturates rather than wrapping, so a long sector cannot re-enter blanking.
    uint8_t cycles_since_commutation_;

    Duty last_duty_;                ///< Duty held across the blanking window
    int8_t last_saturation_;        ///< Saturation flag held across the blanking window
};

} // namespace libecu

#endif // LIBECU_SIX_STEP_ALGORITHM_HPP
