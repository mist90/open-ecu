# MOTOR_1 speed and load transients on the host

From `open-ecu/`, run the production speed PI, Hall/PLL and FOC against a simulated motor.
The test needs a native C++17 compiler with UBSan and Python 3; no ARM toolchain is required.

```sh
tests/test_speed_dynamics/run.sh                                      # STM32F410, float
DYNAMICS_PORT=STM32G431 NUMERIC_PROFILE=fixed tests/test_speed_dynamics/run.sh /tmp/g431-trace.csv > /tmp/g431-metrics.csv
```

`DYNAMICS_PORT` selects the STM32 port's `ecu_config.hpp`; `NUMERIC_PROFILE` selects `float` or
`fixed`. Both ports default to a 20 kHz current loop, 1 kHz speed PI and 100 Hz service. Set
`SPEED_PI_HZ=100` for the slower PI comparison. The default run exercises the production
MOTOR_1 gain schedule and exits nonzero on a failed response budget. An optional path keeps
the 2 ms trace, including `effective_kp` and `effective_ki` for the latest speed measurement.
Trailing `Kp Ki` arguments select constant gains for an exploratory run that reports metrics.

## What is simulated

Both builds use the same virtual bridge so that changing profile does not also change the plant.
It uses the [public F410/IHM08M1 electrical scales](../../STM32F410/Core/Inc/board_config.hpp):
31.3 V bus, centre-aligned PWM timer, shunt and gain, bus divider and 2 kHz current-feedback
filter. A fixture cap of 90% duty/modulation leaves a 10% low-side window. The three quantized
PWM compares set the phase voltages; the model returns 12-bit phase-current and bus codes to
libecu's ADC conversion. It checks the PWM window, −6 to +20 A command rails, a separate 25 A
phase-current budget, valid Hall transitions and that parking clears torque.

The two-axis RL model gets pole count, phase resistance, inductance and back-EMF constant from the
[MOTOR_1 calibration](../../libecu/include/motor_config.hpp). Its own rotor angle determines
back-EMF and Hall edges; integrated phase currents determine torque. No ideal speed, angle or
requested current is fed back to the controller. The free wheel uses the
[speed-loop fixture's](../test_speed_loop/README.md) inertia and bearing drag. The 80 kg case
adds `mass × radius²` to the inertia and `0.008 × mass × g × radius` to rolling torque
(`radius = 0.760 m / 2π`). Ideal Hall placement, no slip and one ideal current sample per phase
per carrier are model assumptions. The test does not qualify either board's switching behaviour,
ADC aperture, protection timing or real wheel response.

## Scenarios and acceptance

Each mass starts at +0.13 Hall sectors from centre. The fixture runs 20 s launches to 4 RPS,
with and without a brake pulse at 10–12 s (3 A torque equivalent, about 1.11 N·m). It separately
preconditions for 140 s before a 3→4 or 5→4 RPS step, or the same brake pulse at constant 4 RPS.
The light wheel also runs ten 10 s plateaus: 0.5, 1, 2, …, 9 RPS.

The trace separates model wheel speed, selected PI feedback, monitor/PLL speed and requested,
FOC-measured and torque-producing `Iq`. The [response budgets](metrics.py) use **model wheel
speed**. `t90` is the first crossing of 90% of the step; `tsettle` is the first entry into a
continuous 3 s window within ±2%. Tail metrics cover the last 3 s.

| Case | Checks on default gains |
| --- | --- |
| Launch without brake | `0.7 < t90 < 2 s` (light) or `4 < t90 < 7 s` (80 kg); settles; tail peak-to-peak <5 RPM |
| 3→4 and 5→4 RPS | `t90` <1 / <5 s and `tsettle` <3 / <14 s (light / 80 kg); tail error <4 RPM and overshoot <30 RPM |
| Free-wheel sweep | Tail peak-to-peak <6 RPM and tail error <4 RPM on **each** plateau |
| Brake at steady 4 RPS, light wheel | Minimum >2.5 RPS during braking, maximum <5.1 RPS in the first 2 s after release; tail peak-to-peak <5 RPM |

The pulse summaries also report recovery time from brake release to the first 3 s in-band
window. A missing crossing is left empty in the CSV.
The runner prints every scenario summary and reports each missed budget on stderr
before returning a nonzero status.

With the scheduled gains, the simulated free wheel fails the 0.5 RPS plateau. On the
F410 fixture, the final 3 s span **77.34 RPM** peak-to-peak in float and **32.78 RPM**
in fixed point; maximum errors are **45.29** and **17.46 RPM**, respectively.
With the earlier acceleration ceiling, the same float fixture spanned 0.37 RPM
at constant 11/3 and 64.71 RPM with the schedule. All other plateaus and
scenarios meet the budgets above. The 0.5 RPS check remains enabled.
