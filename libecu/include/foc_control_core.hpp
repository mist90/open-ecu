/** @file foc_control_core.hpp
 * @brief Typed FOC current control, vector limiting and SVPWM
 */
#ifndef LIBECU_FOC_CONTROL_CORE_HPP
#define LIBECU_FOC_CONTROL_CORE_HPP

#include "motor_samples.hpp"
#include "profile_math.hpp"
#include "polygon_voltage_limit.hpp"
#include "control_optimization.hpp"
#include "current_feedback_filter.hpp"
#include "hybrid_iq_control.hpp"
#include "loop_profile.hpp"
#include <atomic>
#if defined(LIBECU_MOTOR1_T60_LOAD)
#include "motor1_t60_load.hpp"
#endif
#if defined(LIBECU_MOTOR1_SMOOTH_BENCH) && LIBECU_MOTOR1_SMOOTH_BENCH
#include "motor1_bench_compensation.hpp"
#endif

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
    HybridIqParamsT<P> hybrid_iq{};
};

/** @brief Pure, allocation-free electrical FOC loop with profile-owned math. */
template<class Profile, uint32_t FrequencyHz>
class FocControlCore {
    static_assert(FrequencyHz > 0, "FOC requires a positive sample frequency");
public:
    using Current = typename Profile::Current;
    using Voltage = typename Profile::Voltage;
    using Request = typename Profile::VoltageRequest;
    using Angle = typename Profile::Angle;
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
    using Frame = MotorAlgorithmFrameT<Profile>;
    inline static constexpr Angle RAD_PER_STEP = number<Angle>(10471975511966LL, 10000000000000LL);

    void setParams(const Params& params) noexcept {
        if (params.hybrid_iq.enabled != params_.hybrid_iq.enabled) integral_q_ = Integral{};
        params_ = params;
        bemf_per_step_ = multiplyTo<typename Profile::Flux>(params.flux_linkage_wb, RAD_PER_STEP);
        const Duty ceiling = VoltageLimiter::modulationCeiling(params_.max_duty_cycle,
                                                               params_.low_side_clamped_pwm);
        modulation_ = params_.max_modulation < ceiling ? params_.max_modulation : ceiling;
        bus_valid_ = false;
        filter_d_.reset();
        filter_q_.reset();
        hybrid_iq_.reset();
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
#if defined(LIBECU_MOTOR1_SMOOTH_BENCH) && LIBECU_MOTOR1_SMOOTH_BENCH
    /** @brief Publish a precomputed point. Caller serializes this copy with the current ISR. */
#if defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH
    void applyBenchTrims(Motor1BenchTrims trims) noexcept { held_ = trims; }
#endif
    void applyBenchPoint(const Motor1BenchPoint& point) noexcept { bench_ = point; }
#endif
    /** @brief Set precomputed feedback coefficients, resetting both axes' histories. */
    void setCurrentFilter(const CurrentFilterCoefficients<Profile>& coefficients) noexcept {
        params_.current_filter = coefficients;
        filter_d_.reset();
        filter_q_.reset();
        hybrid_iq_.reset();
        zero_current_idle_ = false;
    }
    /** @brief PI feedback values; hybrid without idle suppression publishes raw q here. */
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
        hybrid_iq_.reset();
    }
    void getCurrentPiGains(Gain& kp, Gain& ki) const noexcept { kp = kp_; ki = ki_; }
    void setAngleOffset(Angle angle) noexcept {
        params_.angle_offset_rad = angle;
        filter_d_.reset();
        filter_q_.reset();
        hybrid_iq_.reset();
        zero_current_idle_ = false;
    }
    void reset() noexcept {
        integral_d_ = integral_q_ = Integral{};
        filter_d_.reset();
        filter_q_.reset();
        hybrid_iq_.reset();
        zero_current_idle_ = false;
        next_reconstructed_phase_ = 3;
        const Duty zero = params_.low_side_clamped_pwm ? Duty{} : number<Duty>(1, 2);
        previous_ = {zero, zero, zero};
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
        if (next_reconstructed_phase_ < 3) return next_reconstructed_phase_;
        if (previous_.u >= previous_.v && previous_.u >= previous_.w) return 0;
        return previous_.v >= previous_.w ? 1u : 2u;
    }

    /** @brief Value-facing adapter; the PWM path uses caller-owned frame storage. */
    Output update(const Input& in) noexcept {
        Frame frame{in, {}};
        update(frame);
        return frame.output;
    }

    /** @brief Run one sample in place. Voltages and currents are physical SI values. */
    LIBECU_HOT_LOOP void update(Frame& frame) noexcept {
        update(frame.input, frame.input, frame.output);
        frame.output.duty = Duty{};
    }

    /** Write electrical results directly to the owner's storage. Command fields
     * may reside in the same status object; they are distinct from every output. */
    template<class Commands, class Result>
#if defined(LIBECU_PWM_DEADLINE_ONLY) && LIBECU_PWM_DEADLINE_ONLY
    [[gnu::always_inline]] inline
#else
    [[gnu::noinline]] LIBECU_HOT_LOOP
#endif
    void update(const Input& in, const Commands& commands, Result& out) noexcept {
        Angle park_steps = in.angle_steps;
#if defined(LIBECU_MOTOR1_SMOOTH_BENCH) && LIBECU_MOTOR1_SMOOTH_BENCH
        if constexpr (std::is_same<Profile, FixedProfile>::value) {
            park_steps = motor1BenchParkAngle(park_steps, bench_.angle_gain);
        }
#endif
        const Angle angle = multiplyTo<Angle>(park_steps, RAD_PER_STEP) + params_.angle_offset_rad;
        out.angle_e_rad = angle;
        const auto rotation = Transforms::rotation(angle);
        recordLoopStage(LoopStage::FocTrig);

        const unsigned reconstructed = reconstructedPhase();
        next_reconstructed_phase_ = 3;
        const auto dq = Transforms::parkPhaseCurrents(in.i_u, in.i_v, in.i_w, reconstructed, rotation);
        out.i_d = dq.d;
        out.i_q = dq.q;
        recordLoopStage(LoopStage::FocPark);
        const Current filtered_d = filter_d_.update(dq.d, params_.current_filter);
        // Hybrid filters the error itself. Do not also pay for an unused q PI
        // filter unless the optional standstill noise-floor detector needs it.
        const Current filtered_q = filter_q_.update(dq.q,
            (!BENCH_PI_ONLY && params_.hybrid_iq.enabled) && params_.current_noise_floor == Current{}
                ? CurrentFilterCoefficients<Profile>{} : params_.current_filter);
        const bool reverse = in.drive_mode == DriveMode::REVERSE;
        out.measured_current = reverse ? -dq.q : dq.q;
        // Auxiliary ADC samples commonly span several current-loop ticks. Keep
        // their derived values exact, and recompute before using ANY changed
        // bus sample (including a collapse) or changed modulation parameters.
        if (!bus_valid_ || in.bus_voltage != cached_bus_) {
            cached_bus_ = in.bus_voltage;
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
        const bool current_mode = commands.electric_mode == ElectricMode::CURRENT_MODE;
        // Only the zero-demand standstill state suppresses feedback. At speed,
        // even Iq*=0 needs voltage to reject back-EMF; small nonzero demands must
        // also retain a real feedback path instead of winding up into a dead band.
        const bool idle_candidate = current_mode && params_.current_noise_floor > Current{} &&
            commands.target_current == Current{} && params_.id_target == Current{} &&
            numericAbs(in.speed_steps_s) <= number_v<typename Profile::PllSpeed, 1, 10>;
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
        if (zero_current_idle_) {
            integral_d_ = integral_q_ = Integral{};
            hybrid_iq_.reset();
        }
        recordLoopStage(LoopStage::FocFilter);
        Current error_d{}, error_q{};
        Request requested_d{}, requested_q{};
        if (current_mode) {
            const Current target_q = reverse ? -commands.target_current : commands.target_current;
            error_d = params_.id_target - feedback_d;
#if defined(LIBECU_MOTOR1_SMOOTH_BENCH) && LIBECU_MOTOR1_SMOOTH_BENCH
            error_d = Motor1BenchPoint::weight(error_d, bench_.d);
#elif defined(LIBECU_MOTOR1_T60_LOAD)
            error_d = motor1T60Error<false>(error_d, in.speed_steps_s);
#endif
            requested_d = piRequest(error_d, integral_d_);
            if (!BENCH_PI_ONLY && params_.hybrid_iq.enabled) {
                requested_q = hybrid_iq_.request(target_q, zero_current_idle_ ? Current{} : dq.q, params_.hybrid_iq);
                if (!zero_current_idle_) requested_q += multiplyTo<Request>(in.speed_steps_s, bemf_per_step_);
            } else {
                error_q = target_q - feedback_q;
#if defined(LIBECU_MOTOR1_SMOOTH_BENCH) && LIBECU_MOTOR1_SMOOTH_BENCH
#if defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH
                const bool allow_trim = !reverse && in.drive_mode==DriveMode::FORWARD && in.speed_steps_s.raw()>0 &&
                    target_q.raw()>0 && target_q.raw()<3*int32_t(Current::SCALE);
                if (allow_trim) error_q += held_.current;
#else
                if constexpr (std::is_same<Profile, FixedProfile>::value) {
                    error_q += motor1BenchRipple(in, target_q, bench_.ripple_gain);
                }
#endif
                error_q = Motor1BenchPoint::weight(error_q, bench_.q);
#elif defined(LIBECU_MOTOR1_T60_LOAD)
                error_q = motor1T60Error<true>(error_q, in.speed_steps_s);
#endif
                requested_q = piRequest(error_q, integral_q_);
#if defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH
                if (allow_trim) requested_q += held_.voltage;
#endif
            }
            if (!BENCH_PI_ONLY && params_.use_decoupling && !zero_current_idle_) {
                const auto omega = multiplyTo<typename Profile::PllSpeed>(in.speed_steps_s, RAD_PER_STEP);
                const Gain omega_l = multiplyTo<Gain>(omega, params_.l_phase_h);
                // Hybrid feed-forward must not bypass its slow error filter
                // through a raw-current cross-coupling term.
                requested_d -= multiplyTo<Request>(omega_l, (!BENCH_PI_ONLY && params_.hybrid_iq.enabled) ? target_q : dq.q);
                const Request coupling_q = multiplyTo<Request>(
                    omega_l, (!BENCH_PI_ONLY && params_.hybrid_iq.enabled) ? params_.id_target : dq.d);
                requested_q += (!BENCH_PI_ONLY && params_.hybrid_iq.enabled) ? coupling_q
                    : coupling_q + multiplyTo<Request>(omega, params_.flux_linkage_wb);
            }
        } else {
            requested_q = multiplyTo<Request>(in.duty_command, limit);
            if (reverse) requested_q = -requested_q;
            integral_d_ = integral_q_ = Integral{};
            hybrid_iq_.reset();
        }

        recordLoopStage(LoopStage::FocPi);
        const auto applied = voltage_limiter_.apply(requested_d, requested_q);
        const bool limited = applied.limited;
        const Voltage v_d = applied.d, v_q = applied.q;
        out.v_d = v_d;
        out.v_q = v_q;
        out.setSaturation(limited ? ((reverse ? -requested_q : requested_q) > Request{} ? 1 : -1) : 0);
        recordLoopStage(LoopStage::FocLimit);
        if (current_mode) {
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
            if (!BENCH_PI_ONLY && params_.hybrid_iq.enabled) {
                hybrid_iq_.integrate(requested_q, v_q, params_.hybrid_iq);
            } else {
                integral_q_ = integrate(integral_q_, error_q, v_q, requested_q, applied.q_limited);
            }
        }

        recordLoopStage(LoopStage::FocIntegral);
        const auto duty = Transforms::limitedDuties({v_d, v_q}, rotation, limit, modulation_bus_,
                                                   params_.low_side_clamped_pwm, params_.max_duty_cycle);
        recordLoopStage(LoopStage::FocSvpwm);
        previous_ = duty;
        recordLoopStage(LoopStage::FocOutput);
        out.duty_u = duty.u;
        out.duty_v = duty.v;
        out.duty_w = duty.w;
    }

    /** @brief Scalar modulation depth for a slow telemetry snapshot. */
    static Duty modulationMagnitude(Voltage d, Voltage q, Voltage bus) noexcept {
        const Voltage limit = Transforms::voltageLimit(bus);
        return limit > Voltage{} ? divideTo<Duty>(Math::voltageMagnitude(d, q), limit) : Duty{};
    }
private:
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
#if (defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH) || defined(LIBECU_MOTOR1_T60_LOAD)
    static constexpr bool BENCH_PI_ONLY = true;
#else
    static constexpr bool BENCH_PI_ONLY = false;
#endif
#if defined(LIBECU_MOTOR1_SMOOTH_BENCH) && LIBECU_MOTOR1_SMOOTH_BENCH
    static_assert(std::is_same<Profile, FixedProfile>::value, "The MOTOR_1 bench profile is fixed-point");
    Motor1BenchPoint bench_{};
#if defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH
    Motor1BenchTrims held_{};
#endif
#endif
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
    typename Profile::Flux bemf_per_step_{};
    StepGain ki_step_{};
    Integral integral_d_{}, integral_q_{};
    CurrentFeedbackFilter<Profile> filter_d_, filter_q_;
    HybridIqControl<Profile> hybrid_iq_;
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
    PhaseDutiesT<Duty> previous_{number<Duty>(1, 2), number<Duty>(1, 2), number<Duty>(1, 2)};
};

} // namespace libecu
#endif // LIBECU_FOC_CONTROL_CORE_HPP
