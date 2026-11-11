# MOTOR_1 speed PI: 10-inch wheel and 80 kg road mass

The inner FOC current-loop filter/gain experiment is documented separately in
[MOTOR_1_CURRENT_FILTER.md](MOTOR_1_CURRENT_FILTER.md). The F410 board's 2-kHz
first-order filter and MOTOR_1's FPID=0.549779/516.792 retain the outer SPID=3/2 below.

## Shared settings

`libecu/include/motor_config.hpp`, under `MOTOR_1`:

| Setting | Value | Unit |
|---|---:|---|
| `BLDC_SPEED_CURRENT_KP` | 3.0 | A / mechanical RPS |
| `BLDC_SPEED_CURRENT_KI` | 2.0 | A / (mechanical RPS · s) |
| `BLDC_MAX_CURRENT` | 20 | A |
| `BLDC_MIN_CURRENT` | -6 | A |

These are the outer **VELOCITY / CURRENT_MODE** gains (`AT+MODE=1`,
`AT+EMODE=1`). `AT+SPID=3,2` selects them at runtime in that mode.
The shared parameter factory loads them for every `MOTOR_1` build: F410,
G431 when that motor is selected, and both ES32 ports. The outer loop is
100 Hz on all of them; current/PLL rates are 20 kHz on STM32 and 8/16 kHz
on ES32. The voltage-output speed PI has different units (duty/RPS).

The current-speed PI uses trapezoidal integration, feedback LPF alpha 0.5,
integral limits -4/+12 A and anti-windup gain 2/s. Its output limits are
-6/+20 A. The existing zero-speed standstill release still clears residual
current after stopping. JK42BLS01 has its own motor configuration.

Ki was restored from the experimental 0.15 to **2.0 at the user's request**:
the user found the response to actual wheel-load changes much worse at 0.15.
Kp remains 3.0 and the current ceiling remains 20 A. The calculations and
qualification below record the earlier 3/0.15 experiment, not validation of
the current 3/2 loaded transient. Its ideal 80-kg damping estimate is about 0.23.

The current 3/2 model run passes all 30 unloaded cases. The 24 loaded cases
settle, but exceed the earlier experiment's startup-overshoot/rebound budgets;
the comparison therefore returns nonzero. Those budgets have not been relaxed.
At 80 kg, the modeled load-step dip decreases from 0.735 to **0.358 RPS**,
while startup overshoot increases from 0.591 to **0.986 RPS**. Shared-main boot
tests pass for both motors and float/fixed arithmetic with the restored Ki.

The updated F410 float image was flashed with verification and reported
`+SPID:3.000,2.000` from its boot defaults. A 0.3→1→3→1→0-RPS bench cycle
completed without Hall faults; measured averages at 1 and 3 RPS were 1.002
and 3.007 RPS. It was left at `DMODE=2`, `SPD=0`, current/duty zero.
Capture: `/tmp/opencode/motor1-ki-restore-hardware.json`.

## Earlier 3/0.15 calculation

The user specified a **10-inch outside tire diameter** and **80 kg** to be
accelerated. Treat 80 kg as the effective translating mass:

```
r = 0.254 / 2 = 0.127 m
J_load = m r² = 80 × 0.127² = 1.29032 kg·m²
```

With the existing identified one-phase flux linkage and 20 pole pairs,
the sinusoidal FOC torque estimate is:

```
Kt = 1.5 × pole_pairs × flux = 1.5 × 20 × 0.012336
   = 0.37008 N·m/A
```

Current-to-acceleration fits on the unloaded F410 bench gave approximately
9.4–10 mechanical RPS/s/A, corresponding to **J_rotor ≈ 0.006 kg·m²** at
this Kt. The speed-step fit gave 0.00625 kg·m²; independent 0.7/1.4-A torque
pulses gave about 0.0059–0.0061 kg·m². These are system-identification
estimates, not a mechanical weighing of the rotor. Current calibration,
the electrical Kt estimate, friction and observer delay affect them.

Using J_rotor = 0.00625:

```
J_total = 1.29657 kg·m²
a = Kt / (2π J_total) = 0.0454276 RPS/s/A
```

For an ideal current actuator and negligible viscous drag, the speed-loop
characteristic polynomial is `s² + a Kp s + a Ki`. Therefore:

```
omega_n = sqrt(a Ki)
zeta = a Kp / (2 sqrt(a Ki))
```

The original 2/2 tuning gives **zeta ≈ 0.15** with this road inertia. The experimental
3/0.15 tuning gives **zeta ≈ 0.83**. Positive viscous drag adds damping.
This calculation alone is insufficient for the unloaded wheel: its
acceleration gain is roughly 200 times larger, and Hall/PLL, sampled
feedback and current-loop delays determine the permissible Kp.

Kp was therefore limited by unloaded stability rather than raised to obtain
a fast loaded response. Kp=3.5 and 4 worked on the measured bench, but
produced sustained oscillation in the combined lighter-rotor/higher-torque
model corner. Kp=3 retained that margin. Ki=0.15 prioritized loaded damping
over integral convergence; the subsequent load-response feedback rejected that trade-off.

At 20 A the ideal 80-kg acceleration ceiling is approximately **0.91 RPS/s
or 0.72 m/s²**, before rolling and motor losses. The command ramp setting
is not a promise that the motor can supply the corresponding acceleration.
The smaller integral gain also makes low-speed unloaded startup slower:
on a 0.3-RPS start from rest, the last five seconds of a 60-second hold
averaged 0.285 RPS. This deliberately conservative tuning trades response
speed for stability over the large inertia range.

## Earlier 3/0.15 physical checks, 2026-11-11

NUCLEO-F410RB + IHM08M1, MOTOR_1, about 40.25 V, FOC, nominal 500-Hz
current-loop tuning. TM, PLL and Hall diagnostics were recorded at 100 Hz.
The unloaded wheel was physically tested; the 80-kg case is a model result.

The long runtime-gain qualification held 0.3 RPS for 60 seconds, then
1, 3, 3.2, 3, 6 and 8 RPS, followed by zero. Selected final-window results:

| Target, RPS | Measured mean, RPS | Standard deviation, RPS |
|---:|---:|---:|
| 0.3 | 0.28492 | 0.02429 |
| 1 | 0.99602 | 0.00817 |
| 3 | 2.98360 | 0.00592 |
| 3.2 | 3.19050 | 0.00582 |
| 3, after downstep | 2.99451 | 0.00610 |
| 6 | 6.00297 | 0.00670 |
| 8 | 7.98252 | 0.00470 |
| 0 | 0 | 0 |

The small 3→3.2 and 3.2→3 steps peaked at 3.28 and 2.91 RPS respectively,
then decayed to the noise floor. There were no Hall faults or growing/
sustained large-amplitude speed oscillations. At the final stop, current
command was zero and measured current stayed within ±0.01 A.

Both float and full-fixed F410 images were subsequently flashed with verification
and exercised using their **boot defaults**, without an `SPID` override. Both
reported `+SPID:3.000,0.150` and a maximum current of `20.00` in `+MAXVALS`.
The fixed image passed forward 1→3→0 RPS; the final float image passed reverse
1→3→0 RPS. Reverse final-window standard deviations were 0.0090 and 0.0048 RPS.
Both ended with zero current command; that experiment left its float image parked at
`DMODE=2`, `SPD=0`, current/duty zero.

Raw captures, candidate comparisons, identification inputs and build logs
are retained locally in `/tmp/opencode/motor1-speed-pi-20261111/`.

## Reproducible host checks

`tests/test_speed_loop/run.sh` exercises the actual PI and Hall PLL kernels
with torque-driven mechanics, float and full-fixed arithmetic, and the
8/16/20-kHz fast-loop rates. Its README lists plant assumptions, nominal
and uncertainty cases, transient bounds and sustained-ringing checks.
It reads the shared MOTOR_1 constants, including the 20-A output ceiling.

With the experimental **3/0.15**, all **54** mechanical cases passed. In the nominal 80-kg scenario, the 6-RPS
startup overshoot was 0.591 RPS (9.85%); increasing rolling resistance from
0.008 to 0.020 caused a 0.735-RPS speed dip with a 0.006-RPS rebound. The old
2/2 tuning failed 24 loaded cases, including an 18.1% nominal startup overshoot.

`tests/test_command_processor/run.sh` checks real shared-main startup
parameters for both motors, float/fixed and manual input on/off.
`tests/test_pi_interface/run.sh` checks representability of the current
MOTOR_1 settings as well as float/fixed arithmetic differences.

During that experiment, those suites and `tests/test_control/run.sh float|fixed|numeric-fixed` passed.
Six Release MOTOR_1 builds also passed: STM32F410 float/fixed, STM32G431
float/fixed, ES32F3553GLTE fixed and ES32F3553LTE fixed. G431 was built with
an explicit `-DMOTOR_1` for this check; its normal JK42 selection is independent.
