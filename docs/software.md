# Software and control algorithms

This document describes the implemented OpenECU control pipeline. It is the
shared reference for algorithm behavior, tuning and numerical conventions.
Motor parameters live in `libecu/include/motor_config.hpp`; board-specific
sampling, PWM limits and loop rates enter through `BoardInfo`.

## Contents

- [Control pipeline and units](#control-pipeline-and-units)
- [Speed-loop tuning](#speed-loop-tuning)
- [Hall feedback and PLL](#hall-feedback-and-pll)
- [Six-step commutation](#six-step-commutation)
- [Field-oriented control](#field-oriented-control)
- [Current-feedback filtering](#current-feedback-filtering)
- [Hybrid Iq control through Uq](#hybrid-iq-control-through-uq)
- [Polygon voltage limiting](#polygon-voltage-limiting)
- [Real-time execution](#real-time-execution)
- [Verification](#verification)

## Control pipeline and units

The controller separates the mechanical command from electrical actuation:

```text
Velocity mode, MOTOR_1 with FOC:
Hall edges -> six-sector period / PLL -> mechanical feedback
ramped speed - mechanical feedback -> shape error -> speed PI with current limits -> Iq* (A)
phase ADC + rotor angle from PLL -> measured Id/Iq
Id* = 0, Iq* and measured Id/Iq -> d/q current PIs -> Vd/Vq -> voltage limit -> SVPWM
```

The current loop and PLL run at **20 kHz** on STM32 (`Ts = 50 us`). The
outer speed PI runs at **1 kHz** by default. Speed measurement and its
`alpha=0.5` filter, the setpoint ramp, traction ceiling and foreground
telemetry stay at **100 Hz**. On a coincident tick the new limits and feedback
precede a single PI step; between measurements the PI integrates the held
error and observes the latest saturation. `-DSPEED_PI_HZ=100` selects the
original PI cadence. The [matched rate experiment](../tests/test_speed_pi_rate/README.md)
compares 100 and 500 Hz on the host with a 16-kHz MOTOR_1 current loop.

The mechanical mode chooses the current source: in velocity mode the speed
PI supplies the target; torque and open-loop modes take a signed ampere
command. The electrical algorithm chooses how to meet it: six-step regulates
the conducting shunt, while FOC regulates d/q currents. Hybrid Iq is an
optional q-axis strategy within FOC.

In open-loop operation, a PWM-clocked angle generator supplies the commanded
field angle and speed. Hall tracking continues to describe the physical rotor.
In feedback operation, the PLL supplies electrical angle and speed to FOC;
the outer loop uses the mechanical feedback below.

`U` and `V` both denote voltage in this document; source fields use `v_d/v_q`.
Currents are amperes, the regulator's voltage outputs are volts, and PWM duty
is a fraction.
Mechanical speed commands use revolutions per second (RPS). The PLL uses
electrical **60-degree steps**:

```text
one electrical revolution = 6 steps
theta_e = angle_steps * pi/3 + motor_angle_offset
omega_e = speed_steps_per_second * pi/3
mechanical_RPS = speed_steps_per_second / (6 * pole_pairs)
```

`num_poles` is twice the number of pole pairs. MOTOR_1 has 40 poles, so one
mechanical revolution contains 120 Hall steps.

Sources: [shared application](../libecu/src/main.cpp),
[controller](../libecu/src/bldc_controller.cpp),
[sampling clocks](../libecu/include/control_timing.hpp).

## Speed-loop tuning

**Mechanical feedback, 100 Hz.** The MOTOR_1 Hall IRQ timestamps real transitions.
The speed service checks six consecutive sector intervals against the sectors accepted
by the current interrupt. They span one electrical turn: signed mechanical RPS is
`direction / (pole_pairs * elapsed_seconds)`. Use the PLL through 0.5 RPS, blend
linearly up to 1 RPS, then use the period alone (thresholds use its magnitude).
A reversal, missing/invalid edge, IRQ-only bounce, stale window or implied speed over
twice the motor maximum falls back to the PLL until there is a valid new turn.
Duplicate IRQs do not refresh a timestamp.

`in.velocity_feedback_rps` filters this estimate for the speed PI and both current
ceilings. `in.current_speed_rps` independently filters the original PLL speed for
telemetry and guards. Both are signed relative to the selected direction and use
`alpha = 0.5` per 10 ms. JK42 uses the PLL for both. See the
[period estimator](../libecu/include/hall_period_speed.hpp) and [signal ownership](CONTROL_IO.md).

**Setpoint and error, 100 Hz.** MOTOR_1 ramps the requested speed at 4 RPS/s
(0.04 RPS per tick). From the ramped target and measured speed, prepare one PI input:

```text
e    = ramped_target - velocity_feedback_rps
m   += (e - m)/80
e_PI = e - (7/8)*clamp(e - m, -0.25, +0.25)     # mechanical RPS
```

Small, rapid deviations from the moving mean get about one eighth of their usual
gain. A sustained error passes in full as the mean catches up; there is no deadband.
For a large transient the correction is at most 0.21875 RPS. Both P and I use
`e_PI`; the traction ceiling uses the **unshaped** lag `e`.

**Speed PI, 1 kHz on STM32 by default.** Each PI tick reuses the latest error
and ceilings; it does not manufacture a new speed measurement:

```text
I = clamp(I + Ki*Ts*(e_PI + previous_e_PI)/2, I_min, I_max)  # trapezoidal
requested = Kp*e_PI + I
Iq* = clamp(requested, braking_floor, traction_ceiling)       # A in FOC
I = clamp(I + Kb*Ts*(Iq* - requested), I_min, I_max)         # back-calculation
```

If the live inner-loop saturation has the same sign as the error, skip the
trapezoidal increment; the integral bounds and back-calculation still apply.
Parking discards the prepared demand. Rearming seeds the ramp from the new signed
feedback, and a zero request at standstill clears the residual PI output.

`BldcController::setSpeedPi(kp, ki)` retunes the speed PI without resetting it.
A live Kp change adjusts the integral to preserve the last output at the last
sampled error; a Ki-only edit keeps
the integral and error history. Integration continues on the next tick.
The setter rejects unrepresentable gains. While velocity control is active,
it also rejects a changed tuning if either loop is saturated, the new
integral would exceed its limits, or the operating point changed during
the rebase. On failure it returns `false` and keeps the previous gains.

### MOTOR_1 defaults

| Setting | Value |
|---|---:|
| Current-output speed Kp | 11 A / mechanical RPS |
| Current-output speed Ki | 3 A / (mechanical RPS * s) |
| Current output limits | -6 to +20 A on STM32F410 and STM32G431 with MOTOR_1 (motor ceiling +45 A, limited by the board) |
| Speed-PI integral limits | -4 A to a rolling upper bound: 12 A through 0.125 RPS, then +16 A per RPS up to the current limit |
| Command ramp / measured acceleration setting | 4 / 3 rev/s² |
| Positive traction ceiling | 3 A/RPS base gain; 6 A/RPS extra on excess speed loss; at most +1.5 A per 10 ms |
| Speed-PI back-calculation gain | 2 / s |
| Measured-speed LPF alpha | 0.5 per 10-ms measurement |
| Target-speed LPF | Disabled; acceleration limiting is separate |

With `MOTOR_1_LOADED` undefined, the [calibration](../libecu/include/motor_config.hpp)
uses **11/3** for both the free wheel and a rolling load. It does not estimate
inertia or switch gains at runtime. Explicit `=0` and `=1` select only the earlier
Kp/Ki pairs **1.25/5** and **3/24**, respectively.

### How much positive current is available?

Every 10 ms the [traction limiter](../libecu/include/traction_acceleration.hpp)
compares the change in measured speed `dv` with `3 RPS/s * 0.01 s = 0.03 RPS`.
It starts from the **last limited PI output**, not accumulated unused headroom:

```text
dA = 3 * (0.03 - dv)
if e > 0.25 and dv < -0.03: dA += 6 * (-dv - 0.03)
traction_ceiling = clamp(max(last_Iq_request, 0) + min(dA, 1.5), 0, board_motor_limit)
```

At constant speed this allows +0.09 A per tick (**9 A/s**) if the PI needs it.
A sudden load can unlock extra current; after load release the ceiling can drop
immediately. This bounds positive **demand**, not physical acceleration or braking.
The rolling ceiling in the table limits only the integral. Neither ceiling changes
the raw phase-current trip. The [dynamics fixture](../tests/test_speed_dynamics/README.md)
exercises the startup, load application and release.

### Inertia and earlier gains

Load inertia explains the tuning trade-off. An ideal continuous model for a
surface-magnet motor, before current/estimator/filter/sampling delays, gives:

```text
Kt = 1.5 * pole_pairs * flux
J_load = mass * wheel_radius^2
a = Kt / (2*pi*J_total)                  # RPS/s per ampere
b = viscous_torque_per_RPS / (2*pi*J_total)
speed-loop characteristic: s^2 + (a*Kp + b)*s + a*Ki
omega_n = sqrt(a*Ki)
zeta = (a*Kp + b) / (2*omega_n)
```

For the MOTOR_1 model, `Kt = 0.37008 N*m/A`. A 10-inch tire and 80-kg
effective translating mass contribute `J_load = 1.29032 kg*m^2`, compared
with an identified unloaded-wheel inertia of approximately `0.00625 kg*m^2`.
Consequently, unloaded stability and loaded response impose very different
requirements.

An earlier `Kp/Ki = 3/0.15` experiment improved modeled loaded damping but
responded too slowly to real load changes; it was replaced by `3/2`.
On 2026-11-17, the then-default became **3/20**, following the user's
successful free-wheel test on STM32F410. This is a speed-PI change; the
electrical current-loop gains and the hybrid Iq correction gain are separate.

### SPID 3/20 stability assessment

This historical 100-Hz characterization uses the shared speed PI and Hall PLL,
float/full-fixed arithmetic, and 8/16/20-kHz current clocks. Each case runs
two 120-s phases; settled metrics use the last 20 s of each phase.

The ideal nominal-wheel damping ratio falls from 3.309 to 1.046; at 80 kg
it falls from 0.230 to 0.073. Including a 250-Hz current lag, the PLL's
integral-speed estimate, 0.5 feedback LPF and 100-Hz trapezoidal PI in a
sampled small-signal model gives the following at a 16-kHz fast clock:

| Plant | Oscillatory-mode damping, 3/2 to 3/20 | 3/20 mode frequency | Largest pole magnitude, 3/20 |
|---|---:|---:|---:|
| Nominal free wheel | 0.265 to 0.209 | 4.945 Hz | 0.93582 |
| Light/high-torque free wheel | 0.069 to 0.013 | 6.188 Hz | 0.99492 |
| Nominal 80-kg road load | 0.225 to 0.056 | 0.152 Hz | 0.99947 |

Pole magnitudes below one indicate local decay in this smooth-angle,
unsaturated approximation. It omits Hall quantization and PLL lock/reseed
events; their nonlinear result below is decisive for the sensitivity corner.
The loaded mode rings with a period near 6.6 s and an amplitude decay time
constant near 18.8 s. Faster load rejection therefore comes with weaker damping.

| Model measurement | 3/2 | 3/20 |
|---|---:|---:|
| Nominal free-wheel startup overshoot, 0 to 6 RPS | 0.031-0.033 RPS | 0.296-0.301 RPS |
| Nominal free-wheel undershoot, 6 to 3 RPS | 0.130-0.132 RPS | 0.306-0.308 RPS |
| Nominal 80-kg startup overshoot, 0 to 6 RPS | 0.986 RPS | 0.391 RPS |
| Nominal 80-kg rolling-load speed dip | 0.357-0.358 RPS | 0.141-0.142 RPS |
| Nominal 80-kg recovery rebound | 0.173 RPS | 0.118-0.119 RPS |

The nominal free wheel converges at every tested rate/profile, with only
0.0023-0.0054 RPS settled peak-to-peak variation. All 60-100-kg road cases
also converge, but the load recovery remains underdamped. Current/integral
clipping influences the large startup transient, so its smaller peak does
not establish improved small-signal damping.

The combined low-inertia/high-torque corner (`Jwheel*0.75`, `Kt*1.2`,
1.6 times nominal acceleration per ampere) loses robustness: 11 of its
12 rate/profile/current-bandwidth cases exceed settled-motion limits,
with up to **1.643 RPS peak-to-peak** sustained motion. At a 500-Hz current
bandwidth, the outcome depends on rate and numeric profile; a single pass
does not establish a robust margin. This is a sensitivity case, not a
measurement of the user's F410 wheel.

The unchanged characterization limits reject **53 of 54** cases: 42 exceed
only transient/rebound budgets, and 11 also fail settled-motion checks.
With `3/2`, 24 cases exceeded transient budgets, with no settled-motion
failures. Thus the successful nominal free-wheel result does not imply
stability over the full uncertainty range. The model approximates torque
response by a bounded 125-500-Hz first-order lag; it does not qualify
voltage saturation, six-step ripple, or the full hybrid Uq/Iq dynamics.

Sources: [PI implementation](../libecu/include/pi_controller.hpp),
[motor configuration](../libecu/include/motor_config.hpp),
[speed-loop model and comparison limits](../tests/test_speed_loop/README.md).

## Hall feedback and PLL

### Hall acquisition

Three Hall inputs identify one of six electrical sectors. The Hall interrupt
sets a pending flag; the next PWM frame reads one decoded
sector and shares it between the Hall monitor and PLL. Multiple interrupts
can coalesce. This mechanism does not guarantee a settling-time debounce;
electrical input qualification belongs to the board.

Invalid sectors never enter the PLL. They are reread on subsequent ticks even
without another edge. The separate Hall monitor detects persistent invalid
codes and excessive direction changes using decaying event counts. Its faults
latch until cleared and park the bridge when Hall feedback controls the drive.
PLL synchronization and Hall-health fault status are separate signals.

### Continuous angle from discrete sectors

The PLL compares its estimated angle with the latest Hall sector exactly once
per current tick. In `NEUTRAL` it tracks a coasting wheel while PWM
and acquisition run. Without PWM it holds its last estimate; pending Hall
edges, SysTick and telemetry do not create a rotor tick.

Ignoring finite-number rounding, the tracking update is:

```text
error = clamp(wrap_shortest(hall_sector - angle), -0.5, +0.5)
speed_integral = clamp(speed_integral + Ki*Ts*error, -speed_limit, +speed_limit)
phase_velocity = speed_integral + Kp*error
angle = wrap6(angle + Ts*phase_velocity)
```

The shortest difference is formed over a six-step electrical revolution;
the subsequent clamp limits detector error to half a step. The proportional
term corrects phase promptly, while the integral estimates sustained speed.
MOTOR_1 uses `Kp = 50`, `Ki = 1250`, regardless of the outer speed PI.
Its ideal linear natural frequency is about 5.63 Hz; Hall quantization,
detector clipping and reseeding modify the actual response.

**Reported PLL speed is the integral term.** Angle advancement uses the full
`Kp*error + integral` result. The mechanical fallback and any optional
back-EMF model use the integral speed, not the phase-correction ripple.
Diagnostics expose both quantities as `pll_integral` and `angle_per_second`.

### Synchronization and reseeding

The lock detector filters absolute phase error with
`alpha = clamp(abs(previous_phase_velocity)/(6*f_sample), 0.0001, 0.05)`.
Away from those bounds, its averaging scale is approximately one electrical
revolution.

- A synchronized PLL loses lock and reseeds angle when filtered error reaches
  0.4 step.
- While unsynchronized, an error clipped to +/-0.5 step also triggers reseeding.
  Otherwise a previous phase-velocity magnitude above 5 steps/s establishes lock.
- An internal reseed assigns the current Hall sector to angle and copies the
  newly computed full PI velocity into the speed integrator.
- An explicit reset clears speed, integral and lock history and seeds angle
  from the latest Hall sector.

Loss of synchronization does not automatically select raw-Hall commutation.
For six-step, `use_pll` explicitly selects rounded interpolated angle versus
the decoded Hall sector; the lock flag does not make that selection.

Continuous gains are converted to per-sample coefficients using the actual
fast-loop frequency. Discretization, fixed-point resolution and the lock
filter's per-tick bounds still affect behavior when the rate changes.

Sources: [PLL core](../libecu/include/motor_pll_core.hpp),
[Hall monitor](../libecu/include/hall_monitor_core.hpp).

## Six-step commutation

Six-step control advances the stator field through six discrete switching
configurations per electrical revolution. Two phases conduct and the third
floats:

| Step | U | V | W |
|---|---|---|---|
| 0 | UP | DOWN | OFF |
| 1 | UP | OFF | DOWN |
| 2 | OFF | UP | DOWN |
| 3 | DOWN | UP | OFF |
| 4 | DOWN | OFF | UP |
| 5 | OFF | DOWN | UP |

`UP` is the driven high-side/PWM phase, `DOWN` the return phase, and `OFF`
disconnects the phase. The board implements these abstract switching states.
PLL interpolation determines when the next table row is selected; adjacent
rows are not blended into continuously varying phase duties.

The selected row is the rounded PLL step angle plus a motor-specific torque
alignment, wrapped modulo six. Raw-Hall commutation uses the decoded sector
instead. Normal alignment is +1 step for forward and -2 for reverse;
inverse-table configuration exchanges them. Direction is represented by
commutation alignment, while scalar duty remains nonnegative.

### Current regulation and blanking

One scalar PI compares the ampere request with the current
of the **previously active DOWN phase**. Low-side ADC polarity is positive
for current leaving the motor toward the bridge, so this reading is used
directly. It is a phase-current controller, not a rotating-frame Iq controller.

Current selection precedes this tick's commutation because the sample belongs
to the preceding switching period. After a commutation, the configured
blanking window holds the previous duty and saturation flag and leaves both
PI terms untouched. The default six-cycle window corresponds to 300 us at
20 kHz or 375 us at 16 kHz.

The PI outputs duty, clamped to `[0, max_duty_cycle]`. All three duty fields
carry this scalar value; the switching states determine the active path.
Saturation passed to the speed loop includes the final duty clamp.

For a model of two phase windings in series:

```text
current / duty = Vbus / (2*R + 2*L*s)
Kp = 2*L*omega_c / Vbus
Ki = 2*R*omega_c / Vbus
```

MOTOR_1 uses a separate 500-Hz six-step tuning input. These gains depend on
nominal bus voltage. Six-step telemetry sets `Id/Iq/Ud` to zero and reports
`Uq = duty*Vbus` as a scalar convention, not as a Park-transformed voltage.

Sources: [six-step implementation](../libecu/src/six_step_algorithm.cpp),
[current-loop tuning](../libecu/include/current_loop_tuning.hpp).

## Field-oriented control

FOC describes currents and voltages in a coordinate system rotating with the
rotor. The d-axis is aligned with rotor flux; the q-axis is perpendicular to
it. For the surface-magnet motor model, torque is approximately
`1.5*pole_pairs*flux*Iq`. The normal target is `Id = 0`, leaving current for
torque production.

Unlike six-step, FOC keeps all three bridge phases in PWM-capable `UP` state
and changes their individual duties. There are no six-step commutation events
or post-commutation blanking windows.

### One current-loop update

1. **Acquire and reconstruct currents.** Negate the low-side ADC currents to
   obtain the FOC sign convention. Discard the phase with the highest previous
   duty by default, and reconstruct it using `Iu + Iv + Iw = 0`. A board that
   qualifies a different pair can override the discarded phase for one frame.
   The board must validate the sampling apertures of both retained phases.
2. **Convert to stationary coordinates.** With zero phase-current sum,
   Clarke gives `Ialpha = Iu`, `Ibeta = (Iv-Iw)/sqrt(3)`.
3. **Rotate into field coordinates.** Use the PLL angle (or the generated
   angle in open loop) plus the motor's electrical offset:

   ```text
   Id =  Ialpha*cos(theta) + Ibeta*sin(theta)
   Iq = -Ialpha*sin(theta) + Ibeta*cos(theta)
   ```

4. **Regulate current.** Filter the feedback as configured and calculate
   requested `Ud/Uq`. The ordinary strategy uses independent d/q PIs; the
   hybrid strategy replaces only the q-axis controller.
5. **Limit voltage.** Apply the d-priority polygon limiter and update regulator
   integrators using the voltage selected by that limiter.
6. **Generate PWM.** Inverse Park returns `Ualpha/Ubeta`; space-vector PWM
   converts these to three phase duties using measured bus voltage.

MOTOR_1's Hall-to-d-axis offset is -60 electrical degrees; the inverse
JK42BLS01 configuration uses +120 degrees. Correct Hall decoding alone does
not establish this motor-specific alignment.

### Ordinary d/q PI strategy

**MOTOR_1 closes both axes on measured current** (`current_loop.pi_only=true`).
At 20 kHz on STM32 the d PI uses `Id* = 0`; the q PI uses the signed current
request `Iq*` from the outer PI. Feedback filtering is [board-specific](#current-feedback-filtering).
Neither axis adds model-led voltage or decoupling.

```text
ed = wd * (Id* - filtered_Id)     Vd_requested = Kp*ed + integral_d
eq = wq * (Iq* - filtered_Iq)     Vq_requested = Kp*eq + integral_q
```

Both axes use **Kp = 0.549775 V/A, Ki = 516.792 V/(A·s)**. The weights use
the magnitude of electrical PLL speed; for this 120-sector/rev motor:

| Mechanical speed | d-axis error weight | q-axis error weight |
|---|---:|---:|
| Up to 0.5 RPS (60 sectors/s) | 1/16 | 1/16 |
| 0.5–0.625 RPS | Linear rise to 1 | Linear rise to 1 |
| 0.625–3 RPS | 1 | 1 |
| 3–4 RPS (360–480 sectors/s) | 1 | Linear fall to 1/16 |
| At or above 4 RPS | 1 | 1/16 |

At 4 RPS (240 RPM) the effective q gains are about **0.0344 V/A** and
**32.3 V/(A·s)**. The controller corrects a measured current error more slowly;
it does **not** divide `Iq*` by 16 or switch to hybrid voltage control. JK42BLS01
uses unit weights on both axes.

After voltage limiting, each axis integral receives `Ki*Ts*error`, followed, when enabled,
by `Kb*(limited-requested)`. Here FOC `Kb` is a **dimensionless per-sample
fraction**, normally 1; it differs from the time-scaled `Kb` of the generic
speed/six-step PI. Integrals are also bounded by the available voltage scale.

Optional decoupling adds `-omega_e*L*Iq` to Ud and
`omega_e*(L*Id + flux)` to Uq. Shared startup defaults disable it.
FOC PI gains come from `Kp = L*omega_c`, `Ki = R*omega_c`: outputs are volts,
so these gains do not contain a bus-voltage divisor.

### Space-vector PWM

Inverse Park gives the stationary vector, from which:

```text
Ua = Ualpha
Ub = -Ualpha/2 + sqrt(3)*Ubeta/2
Uc = -Ualpha/2 - sqrt(3)*Ubeta/2
```

Centered modulation subtracts `(max(Ua,Ub,Uc)+min(Ua,Ub,Uc))/2`, divides by
Vbus and adds 0.5 duty. Low-side-clamped modulation instead subtracts the
minimum phase voltage and uses zero as the duty baseline. This changes common
mode while preserving line-to-line voltages; one low side stays on, extending
the current-sampling window. The final duty clamp is a rounding backstop.

Sources: [FOC core](../libecu/include/foc_control_core.hpp),
[FOC adapter](../libecu/src/foc_algorithm.cpp),
[transforms and modulation](../libecu/include/profile_math.hpp).

## Current-feedback filtering

Filtering occurs after phase reconstruction and Clarke/Park, before ordinary
current PI errors. Independent first-order filters use:

```text
y += alpha * (input - y)
```

Only first order and bypass are implemented. The first valid sample initializes
the ordinary filter's state. Its coefficient is designed outside the ISR for
the actual sample rate and requested digital -3-dB cutoff:

```text
w = 2*pi*f_cutoff/f_sample
k = 1 - cos(w)
alpha = sqrt(k*k + 2*k) - k
```

The cutoff belongs to the board's analog front end and is published through
`BoardInfo::current_feedback_filter_hz`; zero selects bypass.

| Board | Feedback cutoff |
|---|---:|
| STM32F410 / IHM08M1 | 2 kHz |
| STM32G431 | Bypass |

At 20 kHz sampling, the 2-kHz filter has `alpha ~= 0.455887`. The MOTOR_1
calibration stores `Kp = 0.549775 V/A`, `Ki = 516.792 V/(A*s)`. This pair
corresponds to a 125-Hz model input with R=0.658 ohm and L=0.7 mH;
125 Hz is not a measured closed-loop bandwidth.

The F410 comparison selected the 2-kHz filter with these quarter-strength
gains instead of the earlier unfiltered 2.199115/2067.168 pair. It reduced
high-frequency Iq error by roughly 11-21% over the measured 1-8-RPS points
and was subjectively quieter, while increasing lower-frequency current ripple.
The median downward-step t90 increased from 0.50 to 1.50 ms in that test.
A smoother filtered trace therefore does not establish lower total raw ripple.

Raw dq values remain available in scope data. The separate reported-current
LPF (`alpha = 0.005` per fast tick) is for slow telemetry, not current feedback.
`LIBECU_RAW_CURRENT_TELEMETRY=1` publishes the current frame's value in that
telemetry field without changing the dq feedback filters.
Raw-current protection and board acquisition checks precede these filters.

The optional zero-current noise-floor feature is disabled by default. When
enabled, it suppresses feedback only at zero d/q demand and near standstill,
with hysteresis. Small nonzero demands and zero-current regulation while
rotating retain feedback.

In hybrid mode, Id keeps its ordinary filter. Iq goes to the dedicated
error-averaging path below; the otherwise unused ordinary q filter is bypassed
unless the optional idle-noise detector needs it.

Source: [feedback filter](../libecu/include/current_feedback_filter.hpp).

## Hybrid Iq control through Uq

The hybrid strategy combines **fast model-based voltage** with **slow measured
current correction**. It targets noisy current feedback, where a fast
proportional response can turn switching spikes into voltage disturbances.

It remains current control: the outer speed PI or `AT+CUR` supplies an ampere demand.
`HybridIqParams::enabled` selects this strategy in a core that supports it;
the scooter calibration uses the ordinary q PI.

A board opts in through `BoardInfo::hybrid_iq_control`. Shared main supplies
phase resistance from `BLDC_R_PHASE_OHM`, inductance from `BLDC_L_PHASE_H`
and flux linkage from `BLDC_KE_V_S_PER_RAD_E`. Direct users configure
`FocParams::hybrid_iq` and call `FocAlgorithm::setParams()`.

Setup validates coefficients outside the ISR; invalid or unrepresentable
parameters leave the previous configuration intact. Enabled hybrid requires
positive phase resistance and flux linkage, nonnegative inductance, and a
positive averaging time and slew rate. Integral gain and correction magnitude
may be zero. `HybridIqParams` defaults to zero resistance: supply a motor model
before enabling it.

### Fast model path

For signed q-current demand:

```text
Uq_requested = R_est*Iq_target + flux_est*omega_e + correction
```

The resistive term responds immediately to a new current request; the
back-EMF term supplies the voltage needed for the present rotor speed.
For MOTOR_1, increasing Iq demand by 1 A adds 0.658 V immediately at unchanged
speed and correction. Actual current then follows the motor's electrical
dynamics.

The underlying q-axis model is
`Uq = R*Iq + L*dIq/dt + omega_e*(L*Id + flux)`. With the usual zero Id target,
the fast path supplies its steady resistive and back-EMF terms.
There is no `L*dIq_target/dt` feed-forward: differentiating the outer
loop's stepped request would create voltage kicks.

Back-EMF feed-forward operates with hybrid Iq, including when
`use_decoupling` is false. If optional cross-coupling is enabled, it uses
**reference currents** rather than raw ADC currents, so it does not bypass
the slow feedback path. Shared startup defaults disable that coupling.

### Slow error correction

Every fast tick filters the current error:

```text
e_avg += alpha * (Iq_target - Iq_raw - e_avg)
alpha = Ts / (averaging_time + Ts)
```

After the current tick's voltage has been limited, the next correction is:

```text
delta = clamp(Ki_trim*Ts*e_avg, -slew*Ts, +slew*Ts)
correction_next = clamp(correction + delta, -correction_limit, +correction_limit)
```

| Parameter | Current default |
|---|---:|
| Error-averaging time constant | 20 ms, approximately 8-Hz cutoff |
| Trim integral gain | 2 V/(A*s) |
| Correction magnitude | +/-4 V |
| Correction slew | 10 V/s |
| MOTOR_1 R estimate | 0.658 ohm |
| MOTOR_1 flux estimate | 0.012336 V*s/electrical radian |

At 20 kHz, the error-filter coefficient is `1/401` and correction changes
by at most 0.5 mV per tick. At 16 kHz the corresponding values are `1/321`
and 0.625 mV. For a sustained averaged 1-A error, the unconstrained rate
is 2 V/s at either frequency.

Filtering the **error** is equivalent to filtering reference and measurement
with the same pole. Filtering only measured Iq would make a correctly followed
reference step appear to leave a persistent deficit. Error history starts at
zero, so the first sample cannot initialize a large long-lived bias.

There is no proportional term driven by instantaneous measured-current error.
The approximate trim crossover `Ki/(2*pi*R) ~= 0.48 Hz` describes model
adaptation, not the response time of the fast demand path. Neither filtering
nor integration is moved to a lower sample rate.

### Saturation, reset and model limits

The correction update observes the **final d-priority vector limiter**. If
its delta would push Uq further into clipping, the correction is held and
`e_avg` is cleared. Integration that moves out of saturation is allowed.
Instantaneous clipping is not back-calculated into this slow trim, because
doing so could inject Id noise through the shared voltage limit.

Id retains its ordinary PI with a zero target. Parking/reset and changes to
the gains, filter or electrical angle offset clear hybrid adaptation. Switching
between ordinary PI and hybrid also clears the old q PI integral.

The MOTOR_1 R estimate includes inverter effects and is an upper-bound estimate
from a no-load fit. Model errors can affect the first electrical transient;
slow adaptation corrects sustained error. Averaging does not remove sensor
gain/offset bias, and approximate average-current tracking is not an
instantaneous current limit. Raw-current and hardware protection are separate.

Sources: [hybrid controller](../libecu/include/hybrid_iq_control.hpp),
[FOC integration](../libecu/include/foc_control_core.hpp).

## Polygon voltage limiting

### Geometry and d priority

FOC limits the requested voltage with a **circumscribed regular 32-gon in
rotating dq coordinates**, rather than an exact circle. This replaces the
usual square-root q-headroom calculation with constant-coefficient lines.

Let `Lambda` be the polygon's inradius. First preserve d priority:

```text
Ud = clamp(Ud_requested, -Lambda, +Lambda)
x = abs(Ud)
q_limit = min(k=0..7) [sec(k*pi/16)*Lambda - tan(k*pi/16)*x]
Uq = clamp(Uq_requested, -q_limit, +q_limit)
```

These eight tangent lines plus the vertical d face describe one quadrant;
reflection produces the full boundary. Constants are precomputed.
Both voltage signs are supported, and the requested vector is not uniformly
rescaled: d gets its allocation first, then q receives the remaining headroom.

The ideal polygon contains the circle of radius `Lambda`. Its maximum radial
extension is `sec(pi/32)-1 ~= 0.483857%`. This is not a relative bound on q
headroom: at `abs(Ud)=Lambda`, the circle has zero q headroom but the polygon
still permits `Lambda*tan(pi/32) ~= 0.0984914*Lambda`.

This dq polygon is distinct from the inverter's stationary alpha/beta hexagon.
It neither implements overmodulation nor transitions FOC to six-step.
Constant attainable Ud/Uq still produce sinusoidal line-to-line voltages.

### Fitting the available PWM range

The corners must remain inside the physically available duty span at every
rotor angle. The configured modulation request is therefore bounded by:

```text
span = max_duty_cycle                    # low-side-clamped PWM
span = 2*max_duty_cycle - 1               # centered PWM
m = min(max_modulation, span*cos(pi/32))
Lambda = max(0, m*Vbus/sqrt(3))
```

With a 95% duty maximum, the modulation inradius ceiling is about 94.5425%
in low-side-clamped mode or 89.5666% in centered mode. A smaller requested
ceiling, such as 20%, stays unchanged. The cosine reserve compensates for
polygon corner extension; the final duty clamp handles rounding.

Bus-derived limits and modulation normalization are cached, but refreshed
before any changed bus sample is used, including a voltage collapse.
With `LIBECU_PWM_BUS_LOOKAHEAD=1`, a single lower-priority writer can call
`prepareFocBus(voltage)` on the controller when a calibrated auxiliary
sample arrives. The current loop can use that result only on an exact match
of bus voltage and modulation; otherwise it computes the values itself.

### Arithmetic and anti-windup

- The limiter performs no runtime trigonometry, square root or division.
- Requests are compared with bounds before narrowing their numeric range.
- An interior-diamond shortcut accepts common unsaturated requests. Otherwise
  a binary search selects a face, with adjacent-face checks preserving the
  quantized envelope near vertices.
- Fixed-point slopes/intercepts use directed Q28 rounding and wide products;
  q headroom rounds inward relative to the ideal polygon. Its deficit is less
  than `1 + 2*Lambda.raw()/2^28` Q18 counts, below 1.04 counts at 20 V.
  Float uses single precision.
- Saturation means an output was actually clamped. A point outside the old
  circle but inside the polygon does not falsely trigger anti-windup.

Ordinary d/q PIs back-calculate using the limiter's selected voltage. Hybrid
q correction uses conditional integration instead. The outer speed PI receives
the clipping sign so it can stop integrating an unavailable current request.

Source: [polygon limiter](../libecu/include/polygon_voltage_limit.hpp).

## Real-time execution

The shared control path supports float and full fixed-point numeric profiles.
Continuous tuning is converted to per-sample gains outside the fast loop.
The fixed path retains wide accumulators and bounded, explicitly rounded
operations; sine/cosine use a table with interpolation.

The board calls `BldcController::pwmInterruptHandler()` once per current
period. The controller advances Hall/PLL tracking, selects rotor feedback
or the generated field and reads the native current target from `MotorIo::in`.
It reads shunt currents and cached bus voltage into `out.electrical`, then
FOC or six-step writes the voltages, duties and saturation there directly.
Six-step also commits the phase switching states on a commutation. In
`NEUTRAL`, acquisition and physical rotor tracking can continue with the
bridge parked.

`io()` is live. Lower-priority readers copy the fields needed for one report
under a short critical section; scope capture reads the completed tick in
the current interrupt. FOC's three phase duties update every current tick,
but its scalar modulation for telemetry is derived at 100 Hz when enabled.

The optional interrupt-token interface lets each `CriticalSection` restore
its entry mask. Nested token guards and legacy C pairs require reverse-order
release; ports without the option retain their existing nesting implementation.

In-place updates, bounded LPF arithmetic and signed sine-table
interpolation reduce work in the fast path without lowering the control rate.
Target execution time still depends on the compiler, numeric profile and
board's acquisition/interrupt implementation.

Sources: [controller I/O and reader rules](CONTROL_IO.md),
[critical-section interface](../libecu/include/critical_section.hpp),
[frame ownership and optional IRQ hooks](CURRENT_FRAME.md).

## Verification

Run host checks from `open-ecu/`:

```sh
bash tests/test_hall_monitor/run.sh
bash tests/test_motor_pll/run.sh
bash tests/test_control/run.sh float
bash tests/test_control/run.sh numeric-fixed
bash tests/test_control/run.sh scooter-fixed
bash tests/test_motor_calibration/run.sh
bash tests/test_speed_pi_rate/run.sh
bash tests/test_foc/run.sh float
bash tests/test_foc/run.sh fixed
bash tests/test_current_filter/run.sh
bash tests/test_hybrid_iq/run.sh
bash tests/test_voltage_limit/run.sh
bash tests/test_profile_math/run.sh
bash tests/test_command_processor/run.sh
bash tests/test_mtte/run.sh
```

The suites cover Hall/PLL behavior, algorithm handover, delayed motor
models, feedback filters, both torque signs, hybrid model error and saturation
recovery, independent polygon geometry and reconstructed PWM voltages.
The hybrid motor model uses an ideal rotor angle; it does not validate PLL
noise or mechanical vibration. The profile-math suite also audits the
Cortex-M0 fixed path for floating-point helpers.

The hybrid runner builds float and fixed profiles sequentially; `BUILD_DIR`
selects its output directory. Its motor model uses reconstructed PWM voltages,
one-frame actuation delay and rotor motion at 8/16/20 kHz. Shared-main tests
verify board opt-in and motor-resistance wiring in both numeric profiles.

The [rolling-wheel torque observer](../libecu/include/mtte_observer.hpp)
uses synchronized Hall speed and measured q-axis current to bound positive
PI output. `test_mtte` checks its signs, timing and behavior under simulated
traction changes. The observer is not enabled by either STM32 port.

The [speed-loop characterization](../tests/test_speed_loop/README.md) has its
own retained comparison budgets: the historical 3/20 tuning exceeded them in
53/54 cases, including 11 settled-motion failures in the light/high-torque
corner. See [the stability assessment](#spid-320-stability-assessment) for
the distinction between transient-budget failures and sustained motion.
Keep model results, binary-specific hardware measurements and subjective
sound observations distinct when interpreting control changes.
