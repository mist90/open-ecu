/** @file foc_control_core.hpp
 * @brief Typed FOC current control, vector limiting and SVPWM
 */
#ifndef LIBECU_FOC_CONTROL_CORE_HPP
#define LIBECU_FOC_CONTROL_CORE_HPP

#include "motor_samples.hpp"
#include "profile_math.hpp"
#include "polygon_voltage_limit.hpp"
#include "control_optimization.hpp"
#include "control_calibration.hpp"
#include "current_feedback_filter.hpp"
#include "loop_profile.hpp"
#include <atomic>

namespace libecu {

template<class P>
struct FocParamsT {
    typename P::Duty max_modulation = number<typename P::Duty>(95, 100);
    typename P::Duty max_duty_cycle = number<typename P::Duty>(95, 100);
    typename P::Angle angle_offset_rad = number<typename P::Angle>(-10471975511966LL, 10000000000000LL);
    typename P::Current id_target{};
    bool use_decoupling = false;
    typename P::Inductance l_phase_h{};
    typename P::Flux flux_linkage_wb{};
    typename P::Fraction anti_windup_kb = number<typename P::Fraction>(1);
    bool low_side_clamped_pwm = false;
    CurrentFilterCoefficients<P> current_filter{};
    typename P::Current current_noise_floor{};
};

/** @brief Pure, allocation-free electrical FOC loop with profile-owned math. */
template<class Profile, uint32_t FrequencyHz, const CurrentLoopCalibration& Calibration = unweighted_current_loop>
class FocControlCore {
    static_assert(FrequencyHz > 0, "FOC requires a positive sample frequency");
public:
    using Current = typename Profile::Current;
    using Voltage = typename Profile::Voltage;
    using Request = typename Profile::VoltageRequest;
    using Angle = typename Profile::Angle;
    using PllSpeed = typename Profile::PllSpeed;
    using Duty = typename Profile::Duty;
    using Unit = typename Profile::Fraction;
    using Gain = typename Profile::Gain;
    using Integral = typename Profile::Accumulator;
    using StepGain = typename Profile::FocStepGain;
    using Math = typename Profile::Math;
    using Transforms = FocTransforms<Profile>;
    using VoltageLimiter = PolygonVoltageLimit<Profile>;
    using Params = FocParamsT<Profile>;
    using Input = MotorAlgorithmInputT<Profile>;
    using Output = MotorAlgorithmOutputT<Profile>;
    inline static constexpr Angle RAD_PER_STEP = number<Angle>(10471975511966LL, 10000000000000LL);

    void setParams(const Params& params) noexcept {
        params_ = params;
        const Duty ceiling = VoltageLimiter::modulationCeiling(params_.max_duty_cycle,
                                                               params_.low_side_clamped_pwm);
        modulation_ = params_.max_modulation < ceiling ? params_.max_modulation : ceiling;
        bus_valid_ = false;
        filter_d_.reset();
        filter_q_.reset();
        zero_current_idle_ = false;
        next_reconstructed_phase_ = 3;
    }
    const Params& getParams() const noexcept { return params_; }
#if defined(LIBECU_PWM_BUS_LOOKAHEAD) && LIBECU_PWM_BUS_LOOKAHEAD
    /** @brief Lower-priority, single-writer lookahead. The PWM reader may preempt any instruction.
     * Only an exact (calibrated bus, modulation) match can ever be used by the current frame. */
    void prepareBus(Voltage voltage) noexcept {
        const unsigned at = published_;
        const Duty modulation = modulation_;
        if (prepared_bus_[at].voltage == voltage && prepared_bus_[at].modulation == modulation) return;
        prepared_bus_[1 - at] = {voltage, modulation,
            multiplyTo<Voltage>(modulation, Transforms::voltageLimit(voltage)), Transforms::modulationBus(voltage)};
        std::atomic_signal_fence(std::memory_order_release);
        published_ = 1 - at;
    }
#endif
    /** @brief Set precomputed feedback coefficients, resetting both axes' histories. */
    void setCurrentFilter(const CurrentFilterCoefficients<Profile>& coefficients) noexcept {
        params_.current_filter = coefficients;
        filter_d_.reset();
        filter_q_.reset();
        zero_current_idle_ = false;
    }
    /** @brief Measured d/q currents used by the PIs after filtering and idle suppression. */
    DqVectorT<Current> filteredCurrents() const noexcept {
        return zero_current_idle_ ? DqVectorT<Current>{} : DqVectorT<Current>{filter_d_.value(), filter_q_.value()};
    }
    void setCurrentPiGains(Gain kp, Gain ki) noexcept {
        setCurrentPiGains(kp, ki, divideByInteger<StepGain>(ki, FrequencyHz));
    }
    void setCurrentPiGains(Gain kp, Gain ki, StepGain step) noexcept {
        kp_ = kp;
        if constexpr (std::is_same<Profile, FixedProfile>::value)
            narrow_gain_ = uint32_t(kp.raw()) + (1u << 23) < (1u << 24);
        ki_ = ki;
        ki_step_ = step;
        integral_d_ = integral_q_ = Integral{};
    }
    void getCurrentPiGains(Gain& kp, Gain& ki) const noexcept { kp = kp_; ki = ki_; }
    void setAngleOffset(Angle angle) noexcept {
        params_.angle_offset_rad = angle;
        filter_d_.reset();
        filter_q_.reset();
        zero_current_idle_ = false;
    }
    void reset() noexcept {
        integral_d_ = integral_q_ = Integral{};
        filter_d_.reset();
        filter_q_.reset();
        zero_current_idle_ = false;
        next_reconstructed_phase_ = 3;
        automatic_reconstructed_phase_ = 0; // All three reset duties tie.
    }

    /** @brief Override one update's discarded phase after the board qualifies the other two.
     *  @return false for a phase outside U/V/W (0/1/2), leaving any pending choice intact.
     *  @note Caller owns the sampling ISR. Consumed by update(); reset/setParams cancel it.
     */
    bool setNextReconstructedPhase(unsigned phase) noexcept {
        if (phase > 2) return false;
        next_reconstructed_phase_ = static_cast<uint8_t>(phase);
        return true;
    }

    /** @brief Next discarded phase: one-frame override, otherwise highest previous duty (U/V/W ties). */
    unsigned reconstructedPhase() const noexcept {
        return next_reconstructed_phase_ < 3 ? next_reconstructed_phase_ : automatic_reconstructed_phase_;
    }

    /** @brief Standalone value-facing interface (physical SI units). */
    LIBECU_HOT_LOOP Output update(const Input& sample) noexcept {
        Output out{};
        updateKernel(sample, sample, out, sample.angle_steps, sample.speed_steps_s, out.saturation);
        return out;
    }

    /** @brief Read this carrier's acquired phases/bus and write its electrical results in place. */
    template<class Commands, class Signals>
    [[gnu::always_inline]] void update(const Commands& in, Signals& out, Angle angle_steps,
                                       PllSpeed speed_steps_s) noexcept {
        updateKernel(in, out, out, angle_steps, speed_steps_s, out.current_pi_saturation);
    }

    /** @brief Scalar modulation depth for a slow telemetry snapshot. */
    static Duty modulationMagnitude(Voltage d, Voltage q, Voltage bus) noexcept {
        const Voltage limit = Transforms::voltageLimit(bus);
        return limit > Voltage{} ? divideTo<Duty>(Math::voltageMagnitude(d, q), limit) : Duty{};
    }

private:
    // The carrier uses one Signals object for both samples and results. The
    // standalone interface supplies those separately; neither copies the samples.
    template<class Commands, class Samples, class Result>
    [[gnu::always_inline]] void updateKernel(const Commands& in, const Samples& sampled, Result& out,
          Angle angle_steps, PllSpeed speed_steps_s, int8_t& saturation) noexcept {
        const Angle angle = multiplyTo<Angle>(angle_steps, RAD_PER_STEP) + params_.angle_offset_rad;
        out.angle_e_rad = angle;
        const auto rotation = Transforms::rotation(angle);
        recordLoopStage(LoopStage::FocTrig);

        const unsigned reconstructed = reconstructedPhase();
        next_reconstructed_phase_ = 3;
        const auto dq = Transforms::parkPhaseCurrents(sampled.i_u, sampled.i_v, sampled.i_w, reconstructed, rotation);
        out.i_d = dq.d;
        out.i_q = dq.q;
        recordLoopStage(LoopStage::FocPark);
        const Current filtered_d = filter_d_.update(dq.d, params_.current_filter);
        const Current filtered_q = filter_q_.update(dq.q, params_.current_filter);
        const bool reverse = in.drive_mode == DriveMode::REVERSE;
        out.measured_current = reverse ? -dq.q : dq.q;
        // Auxiliary ADC samples commonly span several current-loop ticks. Keep
        // their derived values exact, and recompute before using ANY changed
        // bus sample (including a collapse) or changed modulation parameters.
        if (!bus_valid_ || sampled.bus_voltage != cached_bus_) {
            cached_bus_ = sampled.bus_voltage;
#if defined(LIBECU_PWM_BUS_LOOKAHEAD) && LIBECU_PWM_BUS_LOOKAHEAD
            const auto& entry = prepared_bus_[published_];
            std::atomic_signal_fence(std::memory_order_acquire);
            if (entry.voltage == cached_bus_ && entry.modulation == modulation_) {
                modulation_bus_ = entry.bus;
                voltage_limiter_.prepare(entry.limit);
            } else
#endif
            {
                voltage_limiter_.prepare(modulatedLimit(Transforms::voltageLimit(cached_bus_)));
                modulation_bus_ = Transforms::modulationBus(cached_bus_);
            }
            bus_valid_ = true;
        }
        const Voltage limit = voltage_limiter_.inradius();
        // Only the zero-demand standstill state suppresses feedback. At speed,
        // even Iq*=0 needs voltage to reject back-EMF; small nonzero demands must
        // also retain a real feedback path instead of winding up into a dead band.
        const bool idle_candidate = params_.current_noise_floor > Current{} &&
            in.target_current == Current{} && params_.id_target == Current{} &&
            numericAbs(speed_steps_s) <= number_v<PllSpeed, 1, 10>;
        if (idle_candidate) {
            const Current threshold = zero_current_idle_
                ? multiplyTo<Current>(params_.current_noise_floor, number_v<Unit, 3, 2>)
                : params_.current_noise_floor;
            zero_current_idle_ = numericAbs(filtered_d) <= threshold && numericAbs(filtered_q) <= threshold;
        } else {
            zero_current_idle_ = false;
        }
        const Current feedback_d = zero_current_idle_ ? Current{} : filtered_d;
        const Current feedback_q = zero_current_idle_ ? Current{} : filtered_q;
        if (zero_current_idle_) integral_d_ = integral_q_ = Integral{};
        recordLoopStage(LoopStage::FocFilter);
        const Current target_q = reverse ? -in.target_current : in.target_current;
        const Current error_d = weightedCurrentError<Profile, Calibration, false>(params_.id_target - feedback_d,
                                                                                 speed_steps_s);
        Request requested_d = piRequest(error_d, integral_d_);
        const Current error_q = weightedCurrentError<Profile, Calibration, true>(target_q - feedback_q, speed_steps_s);
        Request requested_q = piRequest(error_q, integral_q_);
        if (!Calibration.disable_decoupling && params_.use_decoupling && !zero_current_idle_) {
            const auto omega = multiplyTo<PllSpeed>(speed_steps_s, RAD_PER_STEP);
            const Gain omega_l = multiplyTo<Gain>(omega, params_.l_phase_h);
            requested_d -= multiplyTo<Request>(omega_l, dq.q);
            const Request coupling_q = multiplyTo<Request>(omega_l, dq.d);
            requested_q += coupling_q + multiplyTo<Request>(omega, params_.flux_linkage_wb);
        }

        recordLoopStage(LoopStage::FocPi);
        const auto applied = voltage_limiter_.apply(requested_d, requested_q);
        const bool limited = applied.limited;
        const Voltage v_d = applied.d, v_q = applied.q;
        out.v_d = v_d;
        out.v_q = v_q;
        saturation = limited ? ((reverse ? -requested_q : requested_q) > Request{} ? 1 : -1) : 0;
        recordLoopStage(LoopStage::FocLimit);
        const Integral bound = numberCast<Integral>(limit);
        // An unclipped axis is exactly the original Request promoted to Voltage;
        // its back-calculation error is zero even if the other axis clipped.
        const auto integrate = [&](Integral state, Current error, Voltage voltage, Request requested,
                                   bool clipped) __attribute__((always_inline)) {
            addIntegral(state, multiplyTo<Integral>(ki_step_, error));
            if (clipped && params_.anti_windup_kb > Unit{})
                addIntegral(state, backCalculation(voltage, requested));
            if constexpr (std::is_same<Profile, FixedProfile>::value) {
                return Integral::fromRaw(fixed_detail::clampWideState(state.raw(), bound.raw()));
            } else {
                return numericClamp(state, -bound, bound);
            }
        };
        integral_d_ = integrate(integral_d_, error_d, v_d, requested_d, applied.d_limited);
        integral_q_ = integrate(integral_q_, error_q, v_q, requested_q, applied.q_limited);

        recordLoopStage(LoopStage::FocIntegral);
        const auto duty = Transforms::limitedDuties({v_d, v_q}, rotation, limit, modulation_bus_,
                                                   params_.low_side_clamped_pwm, params_.max_duty_cycle);
        recordLoopStage(LoopStage::FocSvpwm);
        // Only the identity of the highest duty is needed on the next tick.
        // Keep the old U, then V, then W tie order even at the duty clamps.
        automatic_reconstructed_phase_ = duty.u >= duty.v && duty.u >= duty.w ? 0 : duty.v >= duty.w ? 1 : 2;
        recordLoopStage(LoopStage::FocOutput);
        out.duty_u = duty.u;
        out.duty_v = duty.v;
        out.duty_w = duty.w;
    }

    Voltage modulatedLimit(Voltage base) const noexcept {
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // Modulation in [0,1] cannot widen the base. Keep the saturating
            // calculation for configurations passed directly to the public core.
            if (uint32_t(modulation_.raw()) <= Unit::SCALE)
                return Voltage::fromRaw(fixed_detail::boundedRoundedProduct<Unit::FRACTION_BITS>(
                    modulation_.raw(), base.raw()));
        }
        return multiplyTo<Voltage>(modulation_, base);
    }

    Request piRequest(Current error, Integral integral) const noexcept {
        const Request state = integralRequest(integral);
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // A full-scale Current and |kp.raw|<=2^23 give |P.raw|<=2^30;
            // the previous voltage-clamped integral contributes at most 2^25.
            constexpr unsigned shift = Gain::FRACTION_BITS + Current::FRACTION_BITS - Request::FRACTION_BITS;
            static_assert(shift == 24 && Voltage::FRACTION_BITS - Request::FRACTION_BITS == 6);
            if (narrow_gain_) return Request::fromRaw(state.raw() +
                fixed_detail::boundedRoundedProduct<shift>(kp_.raw(), error.raw()));
        }
        return multiplyTo<Request>(kp_, error) + state;
    }
    Integral backCalculation(Voltage applied, Request requested) const noexcept {
        const auto error = numberCast<Request>(applied) - requested;
        if (params_.anti_windup_kb == number_v<Unit, 1>) return numberCast<Integral>(error);
        return multiplyTo<Integral>(params_.anti_windup_kb, error);
    }
    static Request integralRequest(Integral value) noexcept {
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // Both states start at zero and end each sample clamped to a
            // representable Voltage. Narrowing to the wider-range Request
            // cannot saturate, or overflow the 64-bit rounding addition.
            constexpr unsigned shift = Integral::FRACTION_BITS - Request::FRACTION_BITS;
            static_assert(shift > 0 && Voltage::FRACTION_BITS > Request::FRACTION_BITS &&
                          Integral::FRACTION_BITS - Voltage::FRACTION_BITS < 31);
            return Request::fromRaw(int32_t(fixed_detail::roundedLowWord<shift>(value.raw())));
        } else {
            return numberCast<Request>(value);
        }
    }
    static void addIntegral(Integral& state, Integral delta) noexcept {
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            // Each sample begins voltage-clamped, then adds at most one Ki
            // term and one back-calculation term. The full operand ranges,
            // not merely typical motor values, leave ample int64 headroom.
            constexpr int bound_bit = 31 + Integral::FRACTION_BITS - Voltage::FRACTION_BITS;
            constexpr int ki_bit = 62 + Integral::FRACTION_BITS - StepGain::FRACTION_BITS - Current::FRACTION_BITS;
            constexpr int aw_bit = 62 + Integral::FRACTION_BITS - Unit::FRACTION_BITS - Request::FRACTION_BITS;
            static_assert(sizeof(typename Integral::StorageType) == 8 &&
                          bound_bit < 61 && ki_bit < 61 && aw_bit < 61,
                          "Three bounded FOC terms must fit the wide accumulator");
            state = Integral::fromRaw(state.raw() + delta.raw());
        } else {
            state += delta;
        }
    }

    Params params_{};
    Duty modulation_ = VoltageLimiter::modulationCeiling(number<Duty>(95, 100), false);
    Gain kp_{}, ki_{};
    StepGain ki_step_{};
    Integral integral_d_{}, integral_q_{};
    CurrentFeedbackFilter<Profile> filter_d_, filter_q_;
    Voltage cached_bus_{};
    VoltageLimiter voltage_limiter_{};
    typename Transforms::ModulationBus modulation_bus_{};
#if defined(LIBECU_PWM_BUS_LOOKAHEAD) && LIBECU_PWM_BUS_LOOKAHEAD
    struct BusEntry { Voltage voltage{}; Duty modulation{}; Voltage limit{}; typename Transforms::ModulationBus bus{}; };
    BusEntry prepared_bus_[2]{};
    volatile uint8_t published_ = 0;
#endif
    bool bus_valid_ = false;
    bool zero_current_idle_ = false;
    bool narrow_gain_ = false;
    uint8_t next_reconstructed_phase_ = 3; // 3 selects the automatic duty-history rule.
    uint8_t automatic_reconstructed_phase_ = 0;
};

} // namespace libecu
#endif // LIBECU_FOC_CONTROL_CORE_HPP
