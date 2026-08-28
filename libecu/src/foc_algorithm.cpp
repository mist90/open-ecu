/**
 * @file foc_algorithm.cpp
 * @brief Implementation of field-oriented control with space-vector modulation
 */

#include "../include/foc_algorithm.hpp"

#include <algorithm>

namespace libecu {

FocAlgorithm::FocAlgorithm(PwmInterface& pwm_interface, uint32_t pwm_frequency_hz) noexcept
    : pwm_interface_(pwm_interface)
    , params_{}
    , dt_(pwm_frequency_hz > 0 ? 1.0f / static_cast<float>(pwm_frequency_hz) : 0.0f)
    , kp_(0.0f)
    , ki_(0.0f)
    , integral_d_(0.0f)
    , integral_q_(0.0f)
    , prev_duty_u_(0.5f)
    , prev_duty_v_(0.5f)
    , prev_duty_w_(0.5f)
{
    params_.max_modulation   = 0.95f;
    params_.max_duty_cycle   = 0.95f;
    params_.angle_offset_rad = -FOC_RAD_PER_STEP;
    params_.id_target        = 0.0f;
    params_.use_decoupling   = false;
    params_.l_phase_h        = 0.0f;
    params_.flux_linkage_wb  = 0.0f;
    params_.anti_windup_kb   = 1.0f;
}

void FocAlgorithm::setParams(const FocParams& params) noexcept
{
    params_ = params;
}

void FocAlgorithm::setCurrentPiGains(float kp, float ki) noexcept
{
    kp_ = kp;
    ki_ = ki;
    integral_d_ = 0.0f;
    integral_q_ = 0.0f;
}

void FocAlgorithm::getCurrentPiGains(float& kp, float& ki) const noexcept
{
    kp = kp_;
    ki = ki_;
}

void FocAlgorithm::setAngleOffsetRad(float offset_rad) noexcept
{
    params_.angle_offset_rad = offset_rad;
}

void FocAlgorithm::reset() noexcept
{
    integral_d_ = 0.0f;
    integral_q_ = 0.0f;
    prev_duty_u_ = 0.5f;
    prev_duty_v_ = 0.5f;
    prev_duty_w_ = 0.5f;
}

void FocAlgorithm::onEnter() noexcept
{
    reset();

    // All three phases modulate for the rest of this algorithm's life. This is
    // the only COM event FOC ever raises: from here on the duties alone carry
    // the whole waveform.
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, PwmState::UP);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, PwmState::UP);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, PwmState::UP);
    pwm_interface_.updateDutyCycle(0.5f, 0.5f, 0.5f);
    pwm_interface_.apply();
}

void FocAlgorithm::reconstructCurrents(const MotorAlgorithmInput& in,
                                       float& i_u, float& i_v, float& i_w) const noexcept
{
    // Flip into the motor convention. The shunts sit in the low-side legs and
    // read current leaving the phase as positive; Clarke, Park and every
    // torque sign downstream assume positive is current entering the phase.
    //
    // Getting this wrong does not fail loudly - it inverts Id and Iq, which
    // looks exactly like a 180-degree angle error in the telemetry but is not
    // one, and must not be "fixed" by adding 180 to the angle offset. That
    // would correct the measurement and simultaneously point the applied
    // voltage vector the wrong way, because the offset rotates both. It has to
    // be fixed here, on the measurement path alone.
    //
    // Caught on the bench: voltage-mode FOC spinning forward and consuming
    // power reported Iq = -0.49 A averaged over a full electrical revolution.
    // A driving machine's current must have a positive projection on the
    // back-EMF, which lies on +q, so the sign could not have been right.
    i_u = -in.i_u;
    i_v = -in.i_v;
    i_w = -in.i_w;

    // Highest duty == shortest low-side on-time == least valid sample.
    if (prev_duty_u_ >= prev_duty_v_ && prev_duty_u_ >= prev_duty_w_) {
        i_u = -(i_v + i_w);
    } else if (prev_duty_v_ >= prev_duty_w_) {
        i_v = -(i_u + i_w);
    } else {
        i_w = -(i_u + i_v);
    }
}

MotorAlgorithmOutput FocAlgorithm::update(const MotorAlgorithmInput& in) noexcept
{
    MotorAlgorithmOutput out{};

    // ---- Angle -------------------------------------------------------------
    const float theta = in.angle_steps * FOC_RAD_PER_STEP + params_.angle_offset_rad;
    float sin_theta, cos_theta;
    sinCos(theta, sin_theta, cos_theta);

    // ---- Measurement -------------------------------------------------------
    float i_u, i_v, i_w;
    reconstructCurrents(in, i_u, i_v, i_w);

    const AlphaBeta i_ab = clarke(i_u, i_v, i_w);
    const DqVector i_dq = park(i_ab, sin_theta, cos_theta);

    // ---- References --------------------------------------------------------
    // The torque command is a magnitude; direction lives in the drive mode.
    // Reverse torque is negative q-axis current at the same rotor angle - the
    // 180-degree field flip that six-step gets by shifting the commutation
    // step by three falls out of the sign here.
    const float sign = (in.drive_mode == DriveMode::REVERSE) ? -1.0f : 1.0f;
    const float id_ref = params_.id_target;
    const float iq_ref = sign * in.target_current;

    // ---- Regulators --------------------------------------------------------
    // The voltage limit is the SVM inscribed circle, scaled by the configured
    // headroom. It moves with the measured bus voltage, so a sagging battery
    // narrows the limit rather than silently overmodulating.
    const float v_limit = params_.max_modulation * svpwmVoltageLimit(in.bus_voltage);

    const bool current_mode = (in.electric_mode == ElectricMode::CURRENT_MODE);

    float v_d, v_q;
    float v_d_req, v_q_req;
    float err_d = 0.0f;
    float err_q = 0.0f;

    if (!current_mode) {
        // Bring-up path: no current loop at all. The duty command drives the
        // q axis directly as a fraction of the available voltage, so a wrong
        // angle offset shows up as missing torque rather than as a current
        // loop fighting itself.
        v_d_req = 0.0f;
        v_q_req = sign * in.duty_command * v_limit;
        integral_d_ = 0.0f;
        integral_q_ = 0.0f;
    } else {
        err_d = id_ref - i_dq.d;
        err_q = iq_ref - i_dq.q;

        // Proportional term plus the integrator as it stands. The integration
        // itself happens *after* the vector limit below, so that only the part
        // of the demand that was actually applied gets integrated.
        v_d_req = kp_ * err_d + integral_d_;
        v_q_req = kp_ * err_q + integral_q_;

        if (params_.use_decoupling) {
            // omega_e in electrical rad/s: the PLL speed is in steps/s and one
            // step is 60 electrical degrees.
            const float omega_e = in.speed_steps_s * FOC_RAD_PER_STEP;
            v_d_req -= omega_e * params_.l_phase_h * i_dq.q;
            v_q_req += omega_e * (params_.l_phase_h * i_dq.d + params_.flux_linkage_wb);
        }
    }

    // ---- Vector magnitude limit -------------------------------------------
    // Clipping the axes independently would rotate the voltage vector, which
    // is a torque-angle error rather than an amplitude error. Limit the
    // magnitude instead, and give d priority: the d-axis voltage is what holds
    // the current where the field expects it, and losing orientation is worse
    // than losing some torque.
    v_d = v_d_req;
    v_q = v_q_req;

    const float v_mag_sq = v_d * v_d + v_q * v_q;
    bool limited = false;
    if (v_mag_sq > v_limit * v_limit) {
        limited = true;
        if (v_d > v_limit)       v_d =  v_limit;
        else if (v_d < -v_limit) v_d = -v_limit;

        const float q_room_sq = v_limit * v_limit - v_d * v_d;
        // sqrtf via the FPU instruction on this core; q_room_sq >= 0 by
        // construction because v_d has just been clamped to +-v_limit.
        float q_room = 0.0f;
        if (q_room_sq > 0.0f) {
            q_room = __builtin_sqrtf(q_room_sq);
        }
        if (v_q > q_room)       v_q =  q_room;
        else if (v_q < -q_room) v_q = -q_room;
    }

    // ---- Integration, with anti-windup ------------------------------------
    if (current_mode) {
        integral_d_ += ki_ * err_d * dt_;
        integral_q_ += ki_ * err_q * dt_;

        // Back-calculation. The excess that the limiter clipped away never
        // reached the motor, so integrating as though it had is exactly what
        // windup is: the integrator climbs against a ceiling it cannot pass and
        // the drive stays pinned long after the demand drops.
        //
        // kb is dimensionless - the fraction of the clipped excess folded back
        // per sample - and is deliberately NOT scaled by dt. Scaling it by dt
        // is the obvious-looking form and it is useless here: at 50 us the
        // correction comes out some three orders of magnitude smaller than the
        // ki*err*dt it is supposed to oppose. kb = 1 removes the whole excess
        // in one sample, which is what a voltage-limited FOC loop wants.
        if (limited && params_.anti_windup_kb > 0.0f) {
            integral_d_ += params_.anti_windup_kb * (v_d - v_d_req);
            integral_q_ += params_.anti_windup_kb * (v_q - v_q_req);
        }

        // Hard bound as a second line of defence. The integrator's job is to
        // supply the steady-state voltage, and it can never usefully exceed
        // what the bus can produce - so the same limit the vector is held to is
        // the natural clamp, and it scales itself with the measured Vbus.
        if (integral_d_ >  v_limit) integral_d_ =  v_limit;
        if (integral_d_ < -v_limit) integral_d_ = -v_limit;
        if (integral_q_ >  v_limit) integral_q_ =  v_limit;
        if (integral_q_ < -v_limit) integral_q_ = -v_limit;
    }

    // ---- Modulation --------------------------------------------------------
    const AlphaBeta v_ab = invPark(DqVector{v_d, v_q}, sin_theta, cos_theta);
    PhaseDuties duties = svpwm(v_ab.alpha, v_ab.beta, in.bus_voltage);

    // Backstop clamp. The voltage limit above should already keep these in
    // range; what this genuinely enforces is the minimum low-side on-time the
    // shunts need, by never letting a duty reach 1.
    const float d_max = params_.max_duty_cycle;
    const float d_min = 1.0f - params_.max_duty_cycle;
    duties.u = std::max(d_min, std::min(duties.u, d_max));
    duties.v = std::max(d_min, std::min(duties.v, d_max));
    duties.w = std::max(d_min, std::min(duties.w, d_max));

    pwm_interface_.updateDutyCycle(duties.u, duties.v, duties.w);

    prev_duty_u_ = duties.u;
    prev_duty_v_ = duties.v;
    prev_duty_w_ = duties.w;

    // ---- Report ------------------------------------------------------------
    out.duty_u = duties.u;
    out.duty_v = duties.v;
    out.duty_w = duties.w;
    // Scalar modulation depth for telemetry: the fraction of the linear SVM
    // range in use, which is the closest FOC analogue of the six-step duty.
    out.duty = (v_limit > 0.0f)
             ? (__builtin_sqrtf(v_d * v_d + v_q * v_q) / v_limit) * params_.max_modulation
             : 0.0f;
    // The outer speed loop and the telemetry both want "the current that is
    // making torque", which in the rotor frame is Iq. Reported signed and
    // referred back to the commanded direction so it compares like for like
    // with the six-step reading, which is always a magnitude.
    out.measured_current = sign * i_dq.q;
    out.i_d = i_dq.d;
    out.i_q = i_dq.q;
    out.v_d = v_d;
    out.v_q = v_q;
    out.angle_e_rad = theta;
    // Which way the actuator is stuck, for the speed loop's anti-windup.
    out.saturation = limited ? ((v_q_req * sign > 0.0f) ? 1 : -1) : 0;
    return out;
}

} // namespace libecu
