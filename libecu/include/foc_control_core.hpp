/** @file foc_control_core.hpp
 * @brief Typed FOC current control, vector limiting and SVPWM
 */
#ifndef LIBECU_FOC_CONTROL_CORE_HPP
#define LIBECU_FOC_CONTROL_CORE_HPP

#include "motor_samples.hpp"
#include "profile_math.hpp"
#include "control_optimization.hpp"

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

    void setParams(const Params& params) noexcept { params_ = params; }
    const Params& getParams() const noexcept { return params_; }
    void setCurrentPiGains(Gain kp, Gain ki) noexcept {
        kp_ = kp;
        ki_ = ki;
        ki_step_ = divideByInteger<StepGain>(ki, FrequencyHz);
        integral_d_ = integral_q_ = Integral{};
    }
    void getCurrentPiGains(Gain& kp, Gain& ki) const noexcept { kp = kp_; ki = ki_; }
    void setAngleOffset(Angle angle) noexcept { params_.angle_offset_rad = angle; }
    void reset() noexcept {
        integral_d_ = integral_q_ = Integral{};
        const Duty zero = params_.low_side_clamped_pwm ? Duty{} : number<Duty>(1, 2);
        previous_ = {zero, zero, zero};
    }

    /** @brief Run one sample. Voltages and current magnitudes are physical SI values. */
    LIBECU_HOT_LOOP Output update(const Input& in) noexcept {
        Output out{};
        out.angle_e_rad = multiplyTo<Angle>(in.angle_steps, RAD_PER_STEP) + params_.angle_offset_rad;
        Unit sine{}, cosine{};
        Math::sinCos(out.angle_e_rad, sine, cosine);

        Current u = -in.i_u, v = -in.i_v, w = -in.i_w;
        if (previous_.u >= previous_.v && previous_.u >= previous_.w) u = -(v + w);
        else if (previous_.v >= previous_.w) v = -(u + w);
        else w = -(u + v);
        const auto dq = Transforms::park(Transforms::clarkeZeroSum(u, v, w), sine, cosine);
        out.i_d = dq.d;
        out.i_q = dq.q;
        const bool reverse = in.drive_mode == DriveMode::REVERSE;
        out.measured_current = reverse ? -dq.q : dq.q;
        const Voltage limit = multiplyTo<Voltage>(params_.max_modulation, Transforms::voltageLimit(in.bus_voltage));
        const bool current_mode = in.electric_mode == ElectricMode::CURRENT_MODE;
        Current error_d{}, error_q{};
        Request requested_d{}, requested_q{};
        if (current_mode) {
            error_d = params_.id_target - dq.d;
            error_q = (reverse ? -in.target_current : in.target_current) - dq.q;
            requested_d = multiplyTo<Request>(kp_, error_d) + numberCast<Request>(integral_d_);
            requested_q = multiplyTo<Request>(kp_, error_q) + numberCast<Request>(integral_q_);
            if (params_.use_decoupling) {
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
        out.v_d = numericClamp(numberCast<Voltage>(requested_d), -limit, limit);
        out.v_q = numberCast<Voltage>(requested_q);
        if (limited) {
            const Voltage room = Math::qVoltageLimit(out.v_d, limit);
            out.v_q = numericClamp(out.v_q, -room, room);
        }
        if (current_mode) {
            integral_d_ += multiplyTo<Integral>(ki_step_, error_d);
            integral_q_ += multiplyTo<Integral>(ki_step_, error_q);
            if (limited && params_.anti_windup_kb > Unit{}) {
                integral_d_ += multiplyTo<Integral>(params_.anti_windup_kb,
                                                    numberCast<Request>(out.v_d) - requested_d);
                integral_q_ += multiplyTo<Integral>(params_.anti_windup_kb,
                                                    numberCast<Request>(out.v_q) - requested_q);
            }
            const Integral bound = numberCast<Integral>(limit);
            integral_d_ = numericClamp(integral_d_, -bound, bound);
            integral_q_ = numericClamp(integral_q_, -bound, bound);
        }

        const auto voltage = Transforms::invPark(DqVectorT<Voltage>{out.v_d, out.v_q}, sine, cosine);
        auto duty = Transforms::svpwm(voltage.alpha, voltage.beta, in.bus_voltage,
                                     params_.low_side_clamped_pwm);
        const Duty low = params_.low_side_clamped_pwm ? Duty{} : number_v<Duty, 1> - params_.max_duty_cycle;
        out.duty_u = numericClamp(duty.u, low, params_.max_duty_cycle);
        out.duty_v = numericClamp(duty.v, low, params_.max_duty_cycle);
        out.duty_w = numericClamp(duty.w, low, params_.max_duty_cycle);
        previous_ = {out.duty_u, out.duty_v, out.duty_w};
        out.saturation = limited ? ((reverse ? -requested_q : requested_q) > Request{} ? 1 : -1) : 0;
        return out;
    }

    /** @brief Scalar modulation depth for a slow telemetry snapshot. */
    static Duty modulationMagnitude(Voltage d, Voltage q, Voltage bus) noexcept {
        const Voltage limit = Transforms::voltageLimit(bus);
        return limit > Voltage{} ? divideTo<Duty>(Math::voltageMagnitude(d, q), limit) : Duty{};
    }

private:
    Params params_{};
    Gain kp_{}, ki_{};
    StepGain ki_step_{};
    Integral integral_d_{}, integral_q_{};
    PhaseDutiesT<Duty> previous_{number<Duty>(1, 2), number<Duty>(1, 2), number<Duty>(1, 2)};
};

} // namespace libecu
#endif // LIBECU_FOC_CONTROL_CORE_HPP
