# MOTOR_1 mechanical speed-loop characterization

Current MOTOR_1 tuning is **3/2**: the user restored Ki=2 after observing poor
real load-step response with the experimental Ki=0.15. The transient budgets
below characterize that earlier damping-oriented design; they are retained
as comparative diagnostics, not relaxed to make a different tuning pass.
Use `./tests/test_speed_loop/run.sh 3 0.15` to reproduce its 54-case result.
With the current 3/2 settings, all 30 unloaded cases pass; 24 loaded cases
fail the earlier startup/rebound budgets and the command exits nonzero.
Their final-convergence and sustained-oscillation checks still pass.

```sh
./tests/test_speed_loop/run.sh          # shared motor_config.hpp, -DMOTOR_1
./tests/test_speed_loop/run.sh 3 0.1    # exploratory override; does not edit firmware tuning
./tests/test_speed_loop/run.sh 2 2      # old tuning comparison
```

No hardware, Python, emulator or cross compiler. Builds one host C++17 binary
with undefined-behaviour checks. All six combinations of `FloatProfile` /
`FixedProfile` and 8/16/20 kHz current/PLL clocks are exercised. Fixed includes
the real fixed PLL, speed PI, feedback filter and ramp formats, not just a fixed
PI behind floating-point feedback. Exit status is nonzero on any failed limit.

## Actual controller code and simulated plant

The production `PiFloatInterface` / `PiController` and `MotorPLLCore` execute
unchanged. The PI uses the shared MOTOR_1 Kp/Ki, current limits and acceleration;
the PLL uses its shared gains and pole count. This harness mirrors the few lines
of native feedback LPF and target ramp from `BldcController::update()` and
`applyAccelerationLimit()`. Their settings come from shared `main.cpp`: 100 Hz,
feedback alpha 0.5, target alpha 0, integral [-4,12] A and back-calculation gain 2.
The PLL's **integrator** is the speed observation, as in the actual controller.

Mechanical speed and angle result from torque, never from a prescribed speed
trajectory:

```
Kt = 1.5 * 20 * 0.012336 = 0.37008 Nm/A
Jwheel = 0.00625 kg*m²               (identified from 100 Hz free-wheel steps)
Jroad = mass * 0.127² + Jwheel       (80 kg nominal, 10 inch wheel)
d(speed)/dt = (Kt*Iq - bearing_drag - rolling_resistance) / (2*pi*J)
bearing_drag = sign(speed)*Kt_nom*(0.656 + 0.0484*abs(speed))
rolling_resistance = sign(speed)*Crr*mass*9.80665*0.127
```

The loaded inertia includes the wheel: the nominal 80 kg contribution alone is
1.29032 kg*m². Bearing drag and inertia are estimates, not exact machine facts.
Road rolling resistance is an explicit scenario assumption. At rest, opposing
friction balances torque up to the static threshold; a crossing stops at zero.
Compensated float accumulation prevents a long high-rate plant simulation from
artificially sticking at a speed LSB. Hall sectors come from the integrated
electrical angle, sampled on the current clock.

Torque current follows an exact first-order response with bandwidth 125, 250 or
500 Hz and the shared current bounds (currently [-6,20] A). The current request is held for each 100 Hz tick;
its exact within-tick current average drives the mechanical equation. The PI
still owns its real trapezoidal integration, saturation and anti-windup. There
is no independent voltage saturation in this model, so external saturation is
zero. It does not simulate voltage headroom, current-sensor faults, FOC angle
torque loss, commutation ripple, tyre slip or the controller's mode transitions.
Passing is evidence of mechanical-loop damping in this model, not a hardware
qualification or a simulation of the entire ECU.

## Experiments and fixed acceptance limits

Nine deterministic plant cases span free-wheel inertia 0.75–1.5 times the
estimate, Kt 0.8–1.2 times nominal, bearing drag 0.75–1.25 times nominal, road
mass 60–100 kg, and current-response bandwidth 125–500 Hz. The fast/light and
slow/heavy cases test delay margin and mechanical damping respectively. These
are uncertainty corners, not a statistical confidence interval or a full
Cartesian sweep. In particular, the combined J×0.75/Kt×1.2 free-wheel corner
has 1.6 times the measured acceleration gain, equivalent to J=0.00390625 kg*m²
at nominal Kt. It is a deliberately severe sensitivity check, not a measured
property of the F410: the identified J itself depends on the assumed Kt.

Most cases start from rest with a 6 RPS command and the real 6 RPS/s command ramp.
After 120 s, free-wheel cases receive 3 RPS; road cases retain 6 RPS while Crr
steps from 0.008 to 0.020. Each phase lasts 120 s. This deliberately allows the
slow integral bias correction to settle; it does not hide startup overshoot,
which is measured over the whole first phase. The additional nominal free-wheel
low-speed case commands 0.3 RPS from rest, then 1 RPS. It reports the first 12 s
mean and the speed at 12 s to expose slow friction-bias correction, and retains
the same final-convergence and oscillation checks with actual quantized Hall feedback.

Limits are the same for all gains, rates and numerical profiles:

* Nominal 80 kg and higher-speed free-wheel startup overshoot ≤10% of target
  (0.60 RPS). The 60/100 kg uncertainty cases allow ≤15% (0.90 RPS) finite
  overshoot. Their sustained-oscillation, load-rebound and convergence limits
  are identical: load uncertainty may change the transient, not permit ringing.
* Higher-speed free-wheel down-step undershoot ≤10% of step (0.30 RPS).
* Low-speed finite transient over/undershoot ≤0.20 RPS. This absolute budget is
  about 33 ms of travel at the 6 RPS/s ramp, comparable to PLL/feedback-LPF lag.
  Stable low-speed operation still must meet every settled-speed, feedback and
  current-oscillation limit below; a finite decaying excursion is not sustained
  instability.
* Road-load dip ≤20% (1.20 RPS); recovery rebound ≤25% of the dip + 0.02 RPS.
  This checks transient damping as well as eventual stability.
* Last 20 s of **each** phase: mechanical mean error ≤0.05 RPS, worst error
  ≤0.08 RPS, speed peak-to-peak ≤0.08 RPS, current peak-to-peak ≤1.0 A.
* Final PLL/LPF mean error ≤0.05 RPS and disagreement with actual mechanical
  speed ≤0.08 RPS. A stale or incorrect estimator cannot pass by itself.
* Finite states and configured current/integral bounds throughout.

The old 2/2 gains are a useful negative control for loaded-plant overshoot and
load-step ringing. CLI overrides use exactly the same limits as the standard
configured run.

## Observed tuning trade-offs (20 A limit)

The test was exercised with the following exploratory gains. Numbers below are
mechanical RPS, not the PLL's reported speed.

| Kp / Ki | Nominal 80 kg startup overshoot | Nominal road-load dip | Other findings |
|---|---:|---:|---|
| 2 / 2 | 1.084 | 0.391 | All road cases fail damping; heavy tail 0.123 RPS peak-to-peak. |
| 3 / 2 (current) | 0.986 | 0.358 | Free cases pass; road cases settle but exceed earlier transient/rebound budgets. |
| 3 / 0.1 | 0.292 | 0.784 | Passes uncertainty and settled low-speed checks. |
| 3 / 0.15 | 0.591 | 0.735 | Free-wheel stable; 100 kg overshoot 0.662–0.788 is within its 15% budget. |
| 3 / 0.2 | 0.832 | 0.692 | Free-wheel stable; nominal/high-inertia road startup checks fail. |
| 3.5 / 0.15 | 0.418 | 0.665 | Nominal stable; light/high-Kt corner 1.0–1.34 RPS peak-to-peak. |
| 4 / 0.2 | 0.307 | 0.581 | Nominal stable; light/high-Kt corner 1.3–3.9 RPS peak-to-peak. |

The low-speed 0.3→1 RPS experiment finds 0.11–0.14 RPS finite transient overshoot
for 3/0.1, 3/0.15, 3/0.2 and 3.5/0.15. This decays; it is not sustained
instability. Low-speed **settled** motion is bounded: ≤0.028 RPS peak-to-peak.
At 3/0.1, initial 0.3 RPS launch averages only 0.118 RPS over the first 12 s;
3/0.15 and 3/0.2 improve that to 0.134 and 0.148 RPS. The simple friction model
does not reproduce all real breakaway behaviour; use these as sensitivity and
regression results alongside measured motion, not a precise launch prediction.

Acceptance was clarified against the requested stability goal during model
review: an initial blanket 10% transient limit also rejected damped 100 kg
overshoot and the 0.3→1 RPS Hall/PLL transient. The explicit nominal/uncertainty
and absolute low-speed budgets above replace that blanket rule. The raw
excursions remain reported; sustained oscillation, recovery ringing and final
convergence gates were not relaxed. In particular, old 2/2 still fails nominal
road damping and 3.5/0.15 still fails sustained free-wheel oscillation in the
combined uncertainty corner.
