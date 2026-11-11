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

// #define MOTOR_1  // Scooter motor; leave disabled for JK42BLS01.

#ifdef MOTOR_1
#define BLDC_NUM_POLES 40
#define BLDC_MAX_CURRENT 20.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 12.0f
#define BLDC_MAX_ACCELERATION 6.0f
// Thermal cut-out, degrees C, measured by the NTC on the board. Above this the
// drive goes to NEUTRAL, the same way over-voltage does. Not yet calibrated
// against a reference thermometer - see the NTC note in stm32_adc.cpp.
#define BLDC_MAX_TEMPERATURE 100.0f
#define BLDC_INVERTION false
#define BLDC_SPEED_VOLTAGE_KP 0.1f
#define BLDC_SPEED_VOLTAGE_KI 0.2f
// Mechanical-RPS -> current PI: 10-inch wheel, 80 kg effective road mass.
// Kp is limited by the unloaded wheel/PLL delay; Ki restores load-step response.
// Shared by every MOTOR_1 port; calculation and bench checks: docs/MOTOR_1_SPEED_PI.md.
#define BLDC_SPEED_CURRENT_KP 3.0f
#define BLDC_SPEED_CURRENT_KI 2.0f
#define BLDC_SIX_STEP_USE_MOTOR_MODEL true
#define BLDC_CURRENT_FALLBACK_KP 0.1f
#define BLDC_CURRENT_FALLBACK_KI 50.0f
#define BLDC_PLL_KP 100.0f
#define BLDC_PLL_KI 5000.0f
#define BLDC_START_WITH_FOC true
#define BLDC_FOC_ANGLE_OFFSET_DEG -60.0f
// Electrical model of one phase, used to compute the current-loop PI gains
// (see libecu/include/current_loop_tuning.hpp).
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
// Current-loop bandwidth. Measured sweep against the old hand-tuned gains
// (which are themselves ~356 Hz by the same formula):
//   500 Hz: mean |Ierr| -15%/-9% at 5/8 RPS, worst case and commutation swing
//           unchanged, duty activity 1.4x
//   800 Hz: mean |Ierr| -35%/-24%, but at 8 RPS the commutation swing grows
//           26% and p95 11%, and duty activity is 2.3x
// 500 Hz ships because it never regresses; raise it if mean accuracy matters
// more than worst-case excursion.
#define BLDC_ILOOP_BW_HZ        500.0f
// FOC uses lower gains with dq filtering: FPID ~= 0.549779 / 516.792.
// This is the model's tuning input, not a measured closed-loop bandwidth.
// F410 noise/transient comparison: docs/MOTOR_1_CURRENT_FILTER.md.
#define BLDC_FOC_ILOOP_BW_HZ    125.0f
#else
// JK42BLS01, FOC qualified unloaded on STM32G431 at 20 V.
#define BLDC_NUM_POLES 8
#define BLDC_MAX_CURRENT 6.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 200.0f
#define BLDC_MAX_ACCELERATION 20.0f
#define BLDC_MAX_TEMPERATURE 100.0f
#define BLDC_INVERTION true
#define BLDC_SPEED_VOLTAGE_KP 0.005f
#define BLDC_SPEED_VOLTAGE_KI 0.02f
#define BLDC_SPEED_CURRENT_KP 0.025f
#define BLDC_SPEED_CURRENT_KI 0.025f
// Six-step uses its independently tuned duty-output current PI.
#define BLDC_SIX_STEP_USE_MOTOR_MODEL false
#define BLDC_CURRENT_FALLBACK_KP 0.15f
#define BLDC_CURRENT_FALLBACK_KI 200.0f
#define BLDC_PLL_KP 100.0f
#define BLDC_PLL_KI 5000.0f
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
#define BLDC_FOC_ILOOP_BW_HZ    500.0f
#endif

#endif // LIBECU_MOTOR_CONFIG_HPP
