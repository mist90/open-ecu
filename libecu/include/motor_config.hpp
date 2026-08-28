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

#define MOTOR_1

#ifdef MOTOR_1
#define BLDC_NUM_POLES 40
#define BLDC_MAX_CURRENT 18.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 20.0f
#define BLDC_MAX_ACCELERATION 2.0f
// Over-voltage cut-out: above this the bus is being pushed by regeneration
// hard enough to damage the battery, and the drive goes to NEUTRAL.
#define BLDC_MAX_VOLTAGE 36.0f
// Thermal cut-out, degrees C, measured by the NTC on the board. Above this the
// drive goes to NEUTRAL, the same way over-voltage does. Not yet calibrated
// against a reference thermometer - see the NTC note in stm32_adc.cpp.
#define BLDC_MAX_TEMPERATURE 100.0f
#define BLDC_INVERTION false
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
#else
#define BLDC_NUM_POLES 8
#define BLDC_MAX_CURRENT 6.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 200.0f
#define BLDC_MAX_ACCELERATION 100.0f
#define BLDC_MAX_VOLTAGE 36.0f
#define BLDC_MAX_TEMPERATURE 100.0f
#define BLDC_INVERTION true
// Not identified for this motor - run a no-load speed sweep against captures
// from it before trusting these. Zero L or R falls back to the hand-tuned gains.
#define BLDC_KE_V_S_PER_RAD_E   0.0f
#define BLDC_R_PHASE_OHM        0.0f
#define BLDC_L_PHASE_H          0.0f
#define BLDC_ILOOP_BW_HZ        500.0f
#endif

#endif // LIBECU_MOTOR_CONFIG_HPP
