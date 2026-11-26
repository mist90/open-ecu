# MOTOR_1 historical mechanical speed-loop characterization

This characterizes one variable: the speed PI gain pair. The default **3/0.15**
reproduces the 54-case damping reference on the original 20 A, 100/5000 PLL
fixture. For the active scooter calibration, setpoint ramp and rolling integral limit, run
the [100/500 Hz controller comparison](../test_speed_pi_rate/README.md).

The earlier **3/20** tuning was run unloaded on STM32F410. Against this fixture's
unchanged budgets, 53 of 54 cases fail at least one limit (135 checks): 42 fail
only transient/rebound budgets; 11 also fail settled-motion checks, all in the
combined light/high-Kt free-wheel corner. Nominal free-wheel and all road cases
converge. The table below preserves that comparison.

```sh
./tests/test_speed_loop/run.sh          # 3/0.15 reference, all 54 cases pass
./tests/test_speed_loop/run.sh 3 20     # historical 3/20, same plant and limits
./tests/test_speed_loop/run.sh 3 2      # earlier 3/2
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
unchanged. The MOTOR_1 pole count and back-EMF constant set the mechanical
conversion. For reproducible gain comparisons, the fixture sets the Hall PLL
to 100/5000, the PI to 100 Hz, the command ramp to 6 rev/s², feedback alpha
to 0.5, target alpha to 0, the static integral to [-4,12] A and
back-calculation gain to 2.
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
500 Hz and the fixture's [-6,20] A current bounds. The current request is held for each 100 Hz tick;
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
load-step ringing. CLI overrides use exactly the same limits as the reference run.

## Observed tuning trade-offs (20 A limit)

The test was exercised with the following exploratory gains. Numbers below are
mechanical RPS, not the PLL's reported speed.

| Kp / Ki | Nominal 80 kg startup overshoot | Nominal road-load dip | Other findings |
|---|---:|---:|---|
| 2 / 2 | 1.084 | 0.391 | All road cases fail damping; heavy tail 0.123 RPS peak-to-peak. |
| 3 / 2 (previous) | 0.986 | 0.358 | Free cases pass; road cases settle but exceed earlier transient/rebound budgets. |
| 3 / 20 | 0.391 | 0.141-0.142 | Nominal free/road cases settle; 11/12 light/high-Kt cases fail settled limits, up to 1.643 RPS peak-to-peak. |
| 3 / 0.1 | 0.292 | 0.784 | Passes uncertainty and settled low-speed checks. |
| 3 / 0.15 | 0.591 | 0.735 | Free-wheel stable; 100 kg overshoot 0.662–0.788 is within its 15% budget. |
| 3 / 0.2 | 0.832 | 0.692 | Free-wheel stable; nominal/high-inertia road startup checks fail. |
| 3.5 / 0.15 | 0.418 | 0.665 | Nominal stable; light/high-Kt corner 1.0–1.34 RPS peak-to-peak. |
| 4 / 0.2 | 0.307 | 0.581 | Nominal stable; light/high-Kt corner 1.3–3.9 RPS peak-to-peak. |

The 3/2 and 3/20 results were rerun on 2026-11-17 with identical model and
acceptance code. At 3/2, 24 cases fail 42 transient checks; all settled-motion
checks pass. At 3/20, nominal free-wheel startup overshoot increases from
0.031-0.033 to 0.296-0.301 RPS, and 6-to-3-RPS undershoot from 0.130-0.132
to 0.306-0.308 RPS. Its settled variation remains only 0.0023-0.0054 RPS
peak-to-peak. The low-speed 0.3-to-1-RPS overshoot increases to 0.333-0.342
RPS but decays to at most 0.017 RPS peak-to-peak.

At 80 kg, the load dip improves but recovery rebound remains 0.118-0.119
RPS, exceeding the unchanged damping budget. All 60-100-kg cases pass
settled-motion checks. In the severe light/high-Kt corner, all six 125-Hz
current-bandwidth cases sustain 1.1375-1.6430 RPS peak-to-peak motion. The
500-Hz cases depend on rate/profile: five of six fail settled-motion checks,
whereas full-fixed at 20 kHz passes. This isolated pass does not demonstrate
robustness across arithmetic and timing changes.

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
