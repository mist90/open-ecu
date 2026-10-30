/** @file foc_control_core.hpp
 * @brief Typed FOC current control, vector limiting and SVPWM
 */
#ifndef LIBECU_FOC_CONTROL_CORE_HPP
#define LIBECU_FOC_CONTROL_CORE_HPP

#include "motor_samples.hpp"
#include "profile_math.hpp"
#include "control_optimization.hpp"
#include "current_feedback_filter.hpp"

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
    using Params = FocParamsT<Profile>;
    using Input = MotorAlgorithmInputT<Profile>;
    using Output = MotorAlgorithmOutputT<Profile>;
    inline static constexpr Angle RAD_PER_STEP = number<Angle>(10471975511966LL, 10000000000000LL);

    void setParams(const Params& params) noexcept {
        params_ = params;
        filter_d_.reset();
        filter_q_.reset();
        zero_current_idle_ = false;
    }
    const Params& getParams() const noexcept { return params_; }
    /** @brief Set precomputed feedback coefficients, resetting both axes' histories. */
    void setCurrentFilter(const CurrentFilterCoefficients<Profile>& coefficients) noexcept {
        params_.current_filter = coefficients;
        filter_d_.reset();
        filter_q_.reset();
        zero_current_idle_ = false;
    }
    /** @brief Feedback values; the main output retains unfiltered dq measurements. */
    DqVectorT<Current> filteredCurrents() const noexcept {
        return zero_current_idle_ ? DqVectorT<Current>{} : DqVectorT<Current>{filter_d_.value(), filter_q_.value()};
    }
    void setCurrentPiGains(Gain kp, Gain ki) noexcept {
        kp_ = kp;
        ki_ = ki;
        ki_step_ = divideByInteger<StepGain>(ki, FrequencyHz);
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
        const Duty zero = params_.low_side_clamped_pwm ? Duty{} : number<Duty>(1, 2);
        previous_ = {zero, zero, zero};
    }

    /** @brief Run one sample. Voltages and current magnitudes are physical SI values. */
    LIBECU_HOT_LOOP Output update(const Input& in) noexcept {
        const Angle angle = multiplyTo<Angle>(in.angle_steps, RAD_PER_STEP) + params_.angle_offset_rad;
        const auto rotation = Transforms::rotation(angle);

        Current u = -in.i_u, v = -in.i_v, w = -in.i_w;
        if (previous_.u >= previous_.v && previous_.u >= previous_.w) u = -(v + w);
        else if (previous_.v >= previous_.w) v = -(u + w);
        else w = -(u + v);
        const auto dq = Transforms::park(Transforms::clarkeZeroSum(u, v, w), rotation);
        const Current filtered_d = filter_d_.update(dq.d, params_.current_filter);
        const Current filtered_q = filter_q_.update(dq.q, params_.current_filter);
        const bool reverse = in.drive_mode == DriveMode::REVERSE;
        const Current measured_current = reverse ? -dq.q : dq.q;
        const Voltage limit = multiplyTo<Voltage>(params_.max_modulation, Transforms::voltageLimit(in.bus_voltage));
        const bool current_mode = in.electric_mode == ElectricMode::CURRENT_MODE;
        // Only the zero-demand standstill state suppresses feedback. At speed,
        // even Iq*=0 needs voltage to reject back-EMF; small nonzero demands must
        // also retain a real feedback path instead of winding up into a dead band.
        const bool idle_candidate = current_mode && params_.current_noise_floor > Current{} &&
            in.target_current == Current{} && params_.id_target == Current{} &&
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
        if (zero_current_idle_) integral_d_ = integral_q_ = Integral{};
        Current error_d{}, error_q{};
        Request requested_d{}, requested_q{};
        if (current_mode) {
            error_d = params_.id_target - feedback_d;
            error_q = (reverse ? -in.target_current : in.target_current) - feedback_q;
            requested_d = multiplyTo<Request>(kp_, error_d) + numberCast<Request>(integral_d_);
            requested_q = multiplyTo<Request>(kp_, error_q) + numberCast<Request>(integral_q_);
            if (params_.use_decoupling && !zero_current_idle_) {
                const auto omega = multiplyTo<typename Profile::PllSpeed>(in.speed_steps_s, RAD_PER_STEP);
                const Gain omega_l = multiplyTo<Gain>(omega, params_.l_phase_h);
                requested_d -= multiplyTo<Request>(omega_l, dq.q);
                requested_q += multiplyTo<Request>(omega_l, dq.d) +
                               multiplyTo<Request>(omega, params_.flux_linkage_wb);
            }
        } else {
            requested_q = multiplyTo<Request>(in.duty_command, limit);
            if (reverse) requested_q = -requested_q;
            integral_d_ = integral_q_ = Integral{};
        }

        const bool limited = Math::voltageLimitExceeded(requested_d, requested_q, limit);
        const Voltage v_d = numericClamp(numberCast<Voltage>(requested_d), -limit, limit);
        Voltage v_q = numberCast<Voltage>(requested_q);
        if (limited) {
            const Voltage room = Math::qVoltageLimit(v_d, limit);
            v_q = numericClamp(v_q, -room, room);
        }
        if (current_mode) {
            addIntegral(integral_d_, multiplyTo<Integral>(ki_step_, error_d));
            addIntegral(integral_q_, multiplyTo<Integral>(ki_step_, error_q));
            if (limited && params_.anti_windup_kb > Unit{}) {
                addIntegral(integral_d_, multiplyTo<Integral>(params_.anti_windup_kb,
                                                            numberCast<Request>(v_d) - requested_d));
                addIntegral(integral_q_, multiplyTo<Integral>(params_.anti_windup_kb,
                                                            numberCast<Request>(v_q) - requested_q));
            }
            const Integral bound = numberCast<Integral>(limit);
            integral_d_ = numericClamp(integral_d_, -bound, bound);
            integral_q_ = numericClamp(integral_q_, -bound, bound);
        }

        const auto voltage = Transforms::invPark(DqVectorT<Voltage>{v_d, v_q}, rotation);
        auto duty = Transforms::svpwm(voltage.alpha, voltage.beta, in.bus_voltage,
                                     params_.low_side_clamped_pwm);
        const Duty low = params_.low_side_clamped_pwm ? Duty{} : number_v<Duty, 1> - params_.max_duty_cycle;
        duty.u = numericClamp(duty.u, low, params_.max_duty_cycle);
        duty.v = numericClamp(duty.v, low, params_.max_duty_cycle);
        duty.w = numericClamp(duty.w, low, params_.max_duty_cycle);
        previous_ = duty;
        const int8_t saturation = limited ? ((reverse ? -requested_q : requested_q) > Request{} ? 1 : -1) : 0;
        return {duty.u, duty.v, duty.w, Duty{}, measured_current, dq.d, dq.q, v_d, v_q, angle, saturation};
    }

    /** @brief Scalar modulation depth for a slow telemetry snapshot. */
    static Duty modulationMagnitude(Voltage d, Voltage q, Voltage bus) noexcept {
        const Voltage limit = Transforms::voltageLimit(bus);
        return limit > Voltage{} ? divideTo<Duty>(Math::voltageMagnitude(d, q), limit) : Duty{};
    }

private:
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
    Gain kp_{}, ki_{};
    StepGain ki_step_{};
    Integral integral_d_{}, integral_q_{};
    CurrentFeedbackFilter<Profile> filter_d_, filter_q_;
    bool zero_current_idle_ = false;
    PhaseDutiesT<Duty> previous_{number<Duty>(1, 2), number<Duty>(1, 2), number<Duty>(1, 2)};
};

} // namespace libecu
#endif // LIBECU_FOC_CONTROL_CORE_HPP
