# F410/IHM08M1 dq feedback filtering with MOTOR_1

The subsequent acquisition-only comparison, including the final post-refactor
firmware hash, is in [F410_ADC_X1_X2.md](F410_ADC_X1_X2.md).

## Selected configuration, 2026-11-11

| Setting | Value |
|---|---:|
| Feedback filter | Independent first-order low-pass filters on Id and Iq |
| Digital -3 dB cutoff | 2000 Hz |
| FOC current PI Kp | 0.549779 V/A |
| FOC current PI Ki | 516.792 V/(A s) |
| Speed PI (`SPID`) | 3 / 2 |
| Current limits | -6 / +20 A |

The filter belongs to the **board's analog front end**, not to the motor:
`STM32F410/Core/Inc/board_config.hpp` sets
`ihm08::CURRENT_FEEDBACK_FILTER_HZ=2000`. The port publishes it as
`BoardInfo::current_feedback_filter_hz`, which the shared main uses for any
motor attached to that board. Zero disables filtering; the generic board
default, G431 and both ES32 ports use zero. Only first order and bypass are
implemented; the former second-order implementation has been removed.

`motor_config.hpp` selects the separate
`BLDC_FOC_ILOOP_BW_HZ=125` model-tuning input for MOTOR_1. With the existing
L=0.7 mH / R=0.658 ohm model this produces the gains above. **125 Hz is not a
measured bandwidth:** the effective current plant differs from that model,
as already documented in `PORTING_GUIDE.md`. Six-step retains its own 500-Hz
tuning input; the JK42BLS01 retains its previous FOC configuration.

The shared application loads these defaults without UART overrides. Filtering
is after current reconstruction and Clarke/Park, before the d/q PI errors.
Coefficients use the port's actual current-loop frequency. At 20 kHz the
first-order coefficient is approximately 0.455887. Raw dq values remain in
OSC telemetry. The existing alpha=0.005 reported-current filter is separate.
The optional zero-current noise-floor suppression remains disabled.

## Hardware comparison

NUCLEO-F410RB + IHM08M1, unloaded 10-inch MOTOR_1, approximately 40.2 V,
float arithmetic, 20-kHz FOC and window-qualified ADC x2. Identified ST-Link:
`066EFF535752877167082532`. Original speed PI 3/2 was used throughout.

Compared the original unfiltered current PI 2.199115/2067.168 with first-order
2-kHz filtering at 25%, 50%, 75% and 100% of both gains. Also tried 25%/50% Kp
with the original Ki. Fixed-field 0.4-to-0.8-A and 0.8-to-0.4-A steps were
repeated three times for each setting; the rotor was parked between settings.

Simply enabling the filter at the original gains was stable in these runs,
but increased the upward-step overshoot. The selected quarter-gain setting
had the most favorable noise/transient compromise among the tested settings.
The user explicitly reported that its rotation run was **much quieter**.
This is a subjective observation; no calibrated acoustic recording was made.

Indicative step metrics, using a centered five-sample (0.25-ms) offline average:

| Setting | Up-step t90 | Down-step t90 | Up-step overshoot | Down-step overshoot |
|---|---:|---:|---:|---:|
| Original, filter off | 0.20 ms | 0.50 ms | 16% | 19% |
| 2 kHz, original PI | 0.10 ms | 0.75 ms | 52% | 22% |
| 2 kHz, quarter PI | 0.25 ms | 1.50 ms | 21% | 31% |

Values are medians of three steps. Sub-0.25-ms estimates are resolution-limited;
these are comparative diagnostics, not an identified bandwidth. Lower current
gains slow the downward step. Small-current dead-time nonlinearities remain.
The human's earlier rejection of **speed-loop** Ki=0.15 is independent: the
speed-loop Ki remains 2, while this experiment changes the inner current loop.

### Noise is not the same as total current ripple

Complete 1024-sample OSC blocks were analyzed separately at 20 kHz, without
joining UART transmission gaps. Entries are median per-block values in mA.
High-frequency RMS uses a Hann-window spectrum from 1 kHz to Nyquist.

| Speed | Raw Id standard deviation, off -> selected | Raw Iq tracking-error standard deviation, off -> selected | Iq error HF RMS, off -> selected |
|---|---:|---:|---:|
| 1 RPS | 65 -> 86 | 72 -> 64 | 72 -> 58 |
| 3 RPS | 79 -> 157 | 73 -> 86 | 72 -> 60 |
| 6 RPS | 96 -> 202 | 79 -> 127 | 73 -> 64 |
| 8 RPS | 112 -> 211 | 100 -> 152 | 92 -> 79 |

Thus the quieter setting reduces the high-frequency Iq component by about
11-21% in this comparison, **but increases lower-frequency current ripple**.
In particular, Id has a strong approximately 960-Hz component at 8 RPS
(six times the 160-Hz electrical rotation frequency). Lower PI gains reject
that component less strongly. A smoother filtered signal must not be reported
as proof of lower raw current ripple or better loaded torque regulation.

Forward 0.3, 1, 3, 6, 8, 3, 0 RPS, reverse 1, 3, 0 RPS and commanded stops
passed without Hall faults. At 8 RPS the selected setting averaged 7.9986 RPS.
The speed response and sound were checked unloaded; loaded operation was not
physically qualified by this experiment.

### Execution time

The selected float configuration captured 40000 active-drive timing frames at
each of ten operating points, with OSC and telemetry enabled: 400000 frames
total. There were no overruns or board faults. Maximum reported time from PWM
peak through control/OSC completion was **33.89 us**, within the 50-us period.
This metric excludes the recorder bookkeeping and ISR tail; the live overrun
guard includes recorder bookkeeping. Both paired and single-scan acquisition
paths were exercised (only 794/40000 frames were paired at 8 RPS).

A rebuilt **full-fixed** image then loaded the selected gains from its boot
defaults and passed 1 -> 3 -> 8 -> 0 RPS. Its 160000 captured timing frames
had no overruns/faults; the maximum reported peak-to-completion time was
**37.88 us**. Capture: `08-final-fixed-defaults.json`.

The final float image was flashed with verification and then passed a
0.3 -> 1 -> 3 -> 0-RPS boot-default check without an FPID override. Readback
was `FPID=0.5498,516.7920`, `SPID=3.000,2.000`; 160000 timing samples had no
faults/overruns and a 33.01-us maximum reported peak-to-completion time.
Capture: `09-final-float-defaults.json`. It was left in MODE=1, EMODE=1,
DMODE=2, SPD=0, with current/duty zero. Final BIN SHA-256:
`1f5f96b96953a9931f1e57b31639d106f10112f2f638d340d7a99df9c8d5d016`.

## Reproduction and checks

Artifacts and bounded probes: `/tmp/opencode/motor1-dq-filter-20261111/`.

- `01-off-speed.json`, `02-off-steps.json`: original configuration.
- `03-first-steps.json`, `04-first-full-speed.json`: filter/gain comparisons.
- `05-first-keep-ki-steps.json`, `06-first-half-kp-speed.json`: independent Kp trial.
- `07-first-quarter-speed.json`: selected tuning and non-halting SWD timing.
- `probe.py`, `speed.py`, `analyze.py`: capture and analysis; every drive probe
  parks with DMODE=2/SPD=0 on completion or error.

Host filter checks cover float/fixed frequency response at 8/10/16/20/40 kHz,
and the selected first-order reduced-gain delayed-loop case at 8/16/20 kHz.
Shared-main tests verify MOTOR_1 filter/gain boot defaults and the separate
JK42BLS01 configuration. Hardware captures identify firmware binaries by
SHA-256 because the standard version string alone does not distinguish builds.

Filter tests cover float/fixed arithmetic. Shared-main boot tests combine
both motors with board cutoffs 0/1250/2000 Hz, both numeric profiles and manual
input on/off, establishing that the filter follows the board independently
of the motor. The float, fixed-PI and full-fixed control integration suites
also passed during the hardware comparison.

After moving the cutoff to BoardInfo and removing the second-order code,
the first-order/bypass suite passed **140330 checks**, all 24 board-cutoff /
motor / numeric-profile / manual-input startup combinations passed, and all
three control integration profiles passed again. Firmware builds passed for
F410 and G431 in float/fixed and for both ES32 ports in fixed arithmetic.
Build/test logs: `/tmp/opencode/motor1-board-filter-20261111/`.
