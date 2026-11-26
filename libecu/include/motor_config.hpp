/**
 * @file motor_config.hpp
 * @brief Per-motor configuration for the ECU
 *
 * Properties of the machine on the shaft: pole count, electrical model,
 * limits and loop tuning. Properties of the board it is driven from - PWM and
 * control-loop rates, dead time, DC link voltage, shunt and divider values -
 * belong to the port and reach the ECU through libecu::BoardInfo.
 */

#ifndef LIBECU_MOTOR_CONFIG_HPP
#define LIBECU_MOTOR_CONFIG_HPP

#include "control_calibration.hpp"
#include "current_loop_tuning.hpp"

namespace libecu::motor_config {

// The target selects the machine. Each machine has one calibration.
#ifdef MOTOR_1
#define BLDC_NUM_POLES 40
#define BLDC_MAX_CURRENT 45.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 12.0f
#define BLDC_MAX_ACCELERATION 4.0f
// Thermal cut-out, degrees C, measured by the NTC on the board. Above this the
// drive goes to NEUTRAL, the same way over-voltage does. Not yet calibrated
// against a reference thermometer - see the NTC note in stm32_adc.cpp.
#define BLDC_MAX_TEMPERATURE 100.0f
#define BLDC_INVERTION false
// Velocity PI at standstill and at 1 mechanical RPS. This calibration follows the motor across ports.
#define BLDC_SPEED_CURRENT_KP 27.5f
#define BLDC_SPEED_CURRENT_KI 12.0f
inline constexpr float running_speed_kp = 11.0f, running_speed_ki = 3.0f;
#define BLDC_PLL_KP 50.0f
#define BLDC_PLL_KI 1250.0f
// Mechanical velocity PI feedback only. Set to 0 for a stock-PLL comparison build.
#ifndef LIBECU_HALL_PERIOD_SPEED
#define LIBECU_HALL_PERIOD_SPEED 1
#endif

// Hall sectors/s (120 per mechanical revolution here). Both axes recover full
// PI authority after the low-speed ramp; only Iq fades again at high speed.
inline constexpr ErrorWeightRamp low_speed_weight{60, 75, 1.0f / 16};
inline constexpr CurrentLoopCalibration current_loop{
    {low_speed_weight, {}}, {low_speed_weight, {360, 480, 1.0f / 16}}, true
};
// The integral earns 16 A/(rev/s) above 7.5 RPM, within the configured current limit.
inline constexpr RollingIntegralLimit rolling_integral{12.0f, 0.125f, 16.0f};
inline constexpr float speed_integral_max_a = BLDC_MAX_CURRENT;
inline constexpr float soft_start_rps2 = BLDC_MAX_ACCELERATION; // Mechanical speed command ramp.
// Retain 1/8 of fast speed error within a 0.25 RPS band around its moving mean.
// Sustained error and large transients still reach the PI (no steady-state deadband).
inline constexpr float velocity_error_band_rps = .25f, velocity_fast_weight = .125f;
inline constexpr float velocity_error_alpha = 1.f/80; // 100 Hz service, ~0.8 s.
#define BLDC_SIX_STEP_USE_MOTOR_MODEL true
#define BLDC_CURRENT_FALLBACK_KP 0.1f
#define BLDC_CURRENT_FALLBACK_KI 50.0f
#define BLDC_START_WITH_FOC true
#define BLDC_FOC_ANGLE_OFFSET_DEG -60.0f
// Electrical model of one phase, used for model-based six-step PI tuning.
//
// ke and R were identified from a captured no-load speed sweep over 14
// operating points (0.30 V rms fit).
// L is NOT the demag-window figure (5.2 mH) - that quantises to whole PWM
// cycles and over-estimated by an order of magnitude. This value is backed out
// of the measured closed-loop current step: L = kp*Vbus/(2*w_measured), with
// the step settling in ~1.05 ms at kp = 0.1.
//
// R is an upper bound: a no-load speed sweep cannot separate winding resistance
// from the dead-time and diode drops, so ki (which scales with R) is the less
// trustworthy of the two gains.
#define BLDC_KE_V_S_PER_RAD_E   1.2336e-2f
#define BLDC_R_PHASE_OHM        0.658f
#define BLDC_L_PHASE_H          0.0007f
// Six-step current-loop bandwidth. Measured sweep against the old hand-tuned gains
// (which are themselves ~356 Hz by the same formula):
//   500 Hz: mean |Ierr| -15%/-9% at 5/8 RPS, worst case and commutation swing
//           unchanged, duty activity 1.4x
//   800 Hz: mean |Ierr| -35%/-24%, but at 8 RPS the commutation swing grows
//           26% and p95 11%, and duty activity is 2.3x
// 500 Hz ships because it never regresses; raise it if mean accuracy matters
// more than worst-case excursion.
#define BLDC_ILOOP_BW_HZ        500.0f
// Measured drive's d/q gains (125-Hz model tuning with filtered feedback).
inline constexpr PiParameters foc_current_pi{0.549775f, 516.792f};
#else
// JK42BLS01, FOC qualified unloaded on STM32G431 at 20 V.
#define BLDC_NUM_POLES 8
#define BLDC_MAX_CURRENT 6.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 200.0f
#define BLDC_MAX_ACCELERATION 20.0f
#define BLDC_MAX_TEMPERATURE 100.0f
#define BLDC_INVERTION true
#define BLDC_SPEED_CURRENT_KP 0.025f
#define BLDC_SPEED_CURRENT_KI 0.025f
inline constexpr float running_speed_kp = BLDC_SPEED_CURRENT_KP, running_speed_ki = BLDC_SPEED_CURRENT_KI;
inline constexpr CurrentLoopCalibration current_loop{};
inline constexpr RollingIntegralLimit rolling_integral{};
inline constexpr float speed_integral_max_a = 12.0f;
inline constexpr float soft_start_rps2 = BLDC_MAX_ACCELERATION;
inline constexpr float velocity_error_band_rps = 0.f, velocity_fast_weight = 1.f, velocity_error_alpha = 1.f;
// Six-step uses its independently tuned duty-output current PI.
#define BLDC_SIX_STEP_USE_MOTOR_MODEL false
#define BLDC_CURRENT_FALLBACK_KP 0.15f
#define BLDC_CURRENT_FALLBACK_KI 200.0f
#define BLDC_PLL_KP 100.0f
#define BLDC_PLL_KI 5000.0f
#ifndef LIBECU_HALL_PERIOD_SPEED
#define LIBECU_HALL_PERIOD_SPEED 0
#endif
#define BLDC_START_WITH_FOC true
// The inverse commutation alignment needs the opposite vector to MOTOR_1.
#define BLDC_FOC_ANGLE_OFFSET_DEG 120.0f
// Effective FOC model from biased fixed-field voltage steps at 20 kHz:
// tau ~= 1.067 ms, R ~= 1.6 ohm, L ~= 1.7 mH. Includes inverter/current-scale
// effects; these are not bare-winding datasheet values (nominal R_LL = 1.8 ohm).
// At 500 Hz: FPID ~= 5.340708 V/A, 5026.548 V/(A*s). Hardware current steps
// 0.2..0.6 A settle to 95% in about 0.85..0.90 ms; see docs/JK42BLS01_FOC.md.
// Back-EMF remains unidentified; feed-forward stays disabled.
#define BLDC_KE_V_S_PER_RAD_E   0.0f
#define BLDC_R_PHASE_OHM        1.6f
#define BLDC_L_PHASE_H          0.0017f
#define BLDC_ILOOP_BW_HZ        500.0f
inline constexpr PiParameters foc_current_pi =
    tuneFocCurrentPi({BLDC_L_PHASE_H, BLDC_R_PHASE_OHM, 0, BLDC_ILOOP_BW_HZ});
#endif

inline constexpr bool schedule_speed_pi =
    BLDC_SPEED_CURRENT_KP != running_speed_kp || BLDC_SPEED_CURRENT_KI != running_speed_ki;

/** @brief Interpolate speed gains from the magnitude of filtered mechanical rev/s, in native PI formats. */
template<class Kernel>
constexpr typename Kernel::Gains scheduledSpeedGains(typename Kernel::Input feedback) noexcept {
    using Speed = typename Kernel::Input;
    using Gain = typename Kernel::Gain;
    constexpr auto low = number_v<Speed, 1, 8>, high = number_v<Speed, 1>;
    constexpr auto kp_slope = divideTo<Gain>(constant<Gain>(BLDC_SPEED_CURRENT_KP - running_speed_kp), high - low);
    constexpr auto ki_slope = divideTo<Gain>(constant<Gain>(BLDC_SPEED_CURRENT_KI - running_speed_ki), high - low);
    const auto distance = numericClamp(numericAbs(feedback), low, high) - low;
    const auto kp = constant<Gain>(BLDC_SPEED_CURRENT_KP) - multiplyTo<Gain>(distance, kp_slope);
    const auto ki = constant<Gain>(BLDC_SPEED_CURRENT_KI) - multiplyTo<Gain>(distance, ki_slope);
    return {kp, ki, divideByInteger<typename Kernel::StepGain>(ki, 2u * Kernel::FREQUENCY_HZ)};
}

} // namespace libecu::motor_config
#endif // LIBECU_MOTOR_CONFIG_HPP
