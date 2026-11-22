/** @file foc_algorithm.cpp
 * @brief PWM ownership and float-facing boundary of the typed FOC kernel
 */
#include "../include/foc_algorithm.hpp"
#if defined(LIBECU_PWM_DEADLINE_ONLY) && LIBECU_PWM_DEADLINE_ONLY
#include "../include/bldc_controller.hpp"
#endif

namespace libecu {
#if defined(LIBECU_PWM_DEADLINE_ONLY) && LIBECU_PWM_DEADLINE_ONLY
// A non-template definition keeps its named SRAM section through GCC LTO.
LIBECU_RAM_CODE("drive_foc") LIBECU_HOT_LOOP
void FocAlgorithm::updateDriveStatus(const Core::Input& in, NativeMotorStatus& status) noexcept {
    core_.update(in, status, status);
}
#endif

#if defined(LIBECU_PWM_RAM) && LIBECU_PWM_RAM
// Unlike an inline constexpr COMDAT, this definition retains its section
// through GCC LTO. Only the load image is in flash; interpolation reads SRAM.
LIBECU_RAM_DATA("sine") const std::array<int32_t, 129> FixedMath::RAM_SINE = FixedMath::SIN_TABLE;
LIBECU_RAM_CODE("rotation") LIBECU_HOT_LOOP
void FixedMath::sinCos(Angle angle, Fraction& sine, Fraction& cosine) noexcept {
    sinCosKernel(angle, sine, cosine);
}
#endif

FocAlgorithm::FocAlgorithm(PwmInterface& pwm) noexcept : pwm_interface_(pwm) {
    setParams(FocParams{0.95f, 0.95f, -FOC_RAD_PER_STEP, 0.0f, false, 0.0f, 0.0f, 1.0f});
}

bool FocAlgorithm::setParams(const FocParams& params) noexcept {
#if (defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH) || defined(LIBECU_MOTOR1_T60_LOAD)
    if (params.hybrid_iq.enabled || params.use_decoupling) return false;
#endif
    Core::Params typed{};
    if (!tryNumber(params.max_modulation, typed.max_modulation) ||
        !tryNumber(params.max_duty_cycle, typed.max_duty_cycle) ||
        !tryNumber(params.angle_offset_rad, typed.angle_offset_rad) ||
        !tryNumber(params.id_target, typed.id_target) || !tryNumber(params.l_phase_h, typed.l_phase_h) ||
        !tryNumber(params.flux_linkage_wb, typed.flux_linkage_wb) ||
        !tryNumber(params.anti_windup_kb, typed.anti_windup_kb) ||
        !tryNumber(params.current_noise_floor_a, typed.current_noise_floor)) return false;
    if (params.max_modulation < 0.0f || params.max_modulation > 1.0f ||
        params.max_duty_cycle < 0.5f || params.max_duty_cycle > 1.0f ||
        params.anti_windup_kb < 0.0f || params.anti_windup_kb > 1.0f ||
        params.current_noise_floor_a < 0.0f) return false;
    typed.use_decoupling = params.use_decoupling;
    typed.low_side_clamped_pwm = params.low_side_clamped_pwm;
    if (!makeCurrentFilterCoefficients(params.current_filter_order, params.current_filter_cutoff_hz,
                                       ActiveTiming::CURRENT_HZ, typed.current_filter)) return false;
    if (!makeHybridIqParams<ActiveNumericProfile, ActiveTiming::CURRENT_HZ>(
            params.hybrid_iq, typed.hybrid_iq)) return false;
    if (params.hybrid_iq.enabled && (params.flux_linkage_wb <= 0.0f || params.l_phase_h < 0.0f)) return false;
    ActiveNumericProfile::Flux bemf_per_step{};
    if (params.hybrid_iq.enabled &&
        !tryNumber(params.flux_linkage_wb * FOC_RAD_PER_STEP, bemf_per_step)) return false;
    core_.setParams(typed);
    params_ = params;
    return true;
}

bool FocAlgorithm::prepareCurrentPiGains(float kp, float ki, PreparedGains& out) noexcept {
    Core::Gain p{}, i{};
    Core::StepGain step{};
    if (!tryNumber(kp, p) || !tryNumber(ki, i) ||
        !tryNumber(numberCast<float>(i) / ActiveTiming::CURRENT_HZ, step)) return false;
    out = {p, i, divideByInteger<Core::StepGain>(i, ActiveTiming::CURRENT_HZ)};
    return true;
}

bool FocAlgorithm::setCurrentPiGains(float kp, float ki) noexcept {
    PreparedGains gains;
    if (!prepareCurrentPiGains(kp, ki, gains)) return false;
    applyCurrentPiGains(gains);
    return true;
}

void FocAlgorithm::getCurrentPiGains(float& kp, float& ki) const noexcept {
    Core::Gain p{}, i{};
    core_.getCurrentPiGains(p, i);
    kp = numberCast<float>(p);
    ki = numberCast<float>(i);
}

bool FocAlgorithm::setAngleOffsetRad(float angle) noexcept {
    Core::Angle typed{};
    if (!tryNumber(angle, typed)) return false;
    applyAngleOffset(angle, typed);
    return true;
}

void FocAlgorithm::reset() noexcept { core_.reset(); }

void FocAlgorithm::onEnter() noexcept {
    core_.reset();
    pwm_interface_.setChannelState(PwmChannel::PHASE_U, PwmState::UP);
    pwm_interface_.setChannelState(PwmChannel::PHASE_V, PwmState::UP);
    pwm_interface_.setChannelState(PwmChannel::PHASE_W, PwmState::UP);
    const Core::Duty zero = params_.low_side_clamped_pwm ? Core::Duty{} : number_v<Core::Duty, 1, 2>;
    pwm_interface_.updateNativeDutyCycle(zero);
    pwm_interface_.apply();
}

MotorAlgorithmOutput FocAlgorithm::update(const MotorAlgorithmInput& in) noexcept {
    Core::Input input{};
    input.i_u = numberCast<Core::Current>(in.i_u);
    input.i_v = numberCast<Core::Current>(in.i_v);
    input.i_w = numberCast<Core::Current>(in.i_w);
    input.bus_voltage = numberCast<Core::Voltage>(in.bus_voltage);
    input.angle_steps = numberCast<Core::Angle>(in.angle_steps);
    input.speed_steps_s = numberCast<ActiveNumericProfile::PllSpeed>(in.speed_steps_s);
    input.target_current = numberCast<Core::Current>(in.target_current);
    input.duty_command = numberCast<Core::Duty>(in.duty_command);
    input.step = in.step;
    input.drive_mode = in.drive_mode;
    input.electric_mode = in.electric_mode;
    const auto result = updateNative(input);
    MotorAlgorithmOutput out{};
    out.duty_u = numberCast<float>(result.duty_u);
    out.duty_v = numberCast<float>(result.duty_v);
    out.duty_w = numberCast<float>(result.duty_w);
    out.measured_current = numberCast<float>(result.measured_current);
    out.i_d = numberCast<float>(result.i_d);
    out.i_q = numberCast<float>(result.i_q);
    out.v_d = numberCast<float>(result.v_d);
    out.v_q = numberCast<float>(result.v_q);
    out.angle_e_rad = numberCast<float>(result.angle_e_rad);
    out.saturation = result.saturation;
    return out;
}

FocAlgorithm::Core::Output FocAlgorithm::updateNative(const Core::Input& input) noexcept {
    Core::Frame frame{input, {}};
    updateNative(frame);
    return frame.output;
}

LIBECU_HOT_LOOP void FocAlgorithm::updateNative(Core::Frame& frame) noexcept {
    updateNative(frame, pwm_interface_);
}

float FocAlgorithm::modulationMagnitude(float d, float q, float bus) noexcept {
    return numberCast<float>(Core::modulationMagnitude(numberCast<Core::Voltage>(d),
                            numberCast<Core::Voltage>(q), numberCast<Core::Voltage>(bus)));
}

} // namespace libecu
