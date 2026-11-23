# MOTOR_1 speed PI: 100 Hz vs 500 Hz

```sh
# From open-ecu; C++17 and UBSan, no cross-toolchain or board.
./tests/test_speed_pi_rate/run.sh
# Optional: keep the two full 2-ms traces plus the summary.
out=$(mktemp -d "${TMPDIR:-/tmp}/scooter-ab.XXXXXX")
./tests/test_speed_pi_rate/run.sh "$out" > "$out/summary.csv"
```

The script compiles the same fixture and libecu sources twice. Only the
`SpeedPiHz` template argument changes. Both arms use the same 500-Hz SysTick
divider, a 16-kHz carrier/PLL, and the production **fixed-point**
`BldcController`. `t=0` is a coincident service/PI tick: command 155 rpm from
rest and hold for 40 s. `update()` handles slow work and the coincident PI,
`updateSpeedPi()` the four intermediate ticks in the 500-Hz arm. Every current
request acts on the plant from that boundary. No catch-up or wall-clock timing.

The actual controller owns the Hall PLL (50/1250), 100-Hz measured-speed LPF
(α=0.5), 2 rev/s² target ramp, rolling integral ceiling, 1 rev/s²
measured-speed traction feedback, current rails (−6/+45 A) and PI with
back-calculation (`Kb=2`). It reads the same filtered speed and holds the same
ramp/ceiling within each 10-ms frame. The three Kp/Ki pairs are fixed fixture
inputs: the scooter's unloaded 1.25/5, loaded 3/24, and a 6/24 sensitivity
check. Choosing a different boot PI does not change these arms. A matching
3/24 run sets only the traction rate to zero; the setpoint ramp and rolling
integral limit still apply.
The runner checks the observed slow/fast tick counts, intermediate held
feedback and current publication, current rails, Hall tracking and that the
traction setting affects the 3/24 launch in both plants. It doesn't require
one cadence to outperform the other.

## Observed on the host

**With the traction ceiling active, neither rate changed the launch** in these
fixtures. Comparing the 2-ms traces shows that the current requests were
identical up to the first 90% crossing for all three gain pairs. On the free
wheel both rates reached 90% in **2.71 s** (10–90% in **2.15 s**); the peak
200-ms acceleration to that crossing was **1.336 rev/s²** in the plant and
**1.462 rev/s²** from the filtered Hall feedback. On the 80-kg equivalent
load the figures were **9.05 s**, **5.34 s**, and **0.530 / 0.805 rev/s²**.
For 3/24, the first current requests to differ were at 3.380 s and 9.870 s,
respectively.

Over the full 40 s, later oscillations change the answer. Arrows compare
**100 → 500 Hz**, same load and gains; `wheel / feedback` are peak positive
200-ms differences in rev/s². Overshoot and tail peak-to-peak are mechanical
wheel rpm, sampled every 10 ms. Target: 155 rpm. Measured here with g++ 11.4.

| Load | Kp/Ki | Wheel / feedback peak | Overshoot, rpm | Last 5 s peak-to-peak, rpm |
|---|---|---:|---:|---:|
| Free | 1.25/5 | 1.336 / 1.462 → 1.336 / 1.462 | 17.69 → 17.89 | 0.04 → 0.04 |
| Free | 3/24 | 2.385 / 2.037 → 2.523 / 2.262 | 24.38 → 24.58 | 30.32 → 32.95 |
| Free | 6/24 | 2.806 / 2.387 → 2.887 / 2.457 | 17.99 → 18.13 | 40.37 → 35.89 |
| 80 kg | 1.25/5 | 0.552 / 0.805 → 0.552 / 0.805 | 61.16 → 61.19 | 35.41 → 35.25 |
| 80 kg | 3/24 | 0.567 / 0.805 → 0.567 / 0.805 | 32.18 → 32.24 | 15.33 → 15.38 |
| 80 kg | 6/24 | 0.561 / 0.805 → 0.562 / 0.805 | 28.35 → 28.40 | 2.61 → 2.62 |

With only the traction ceiling **off**, 3/24 free-wheel peak launch
acceleration was **6.666 → 5.566 rev/s²** (feedback **5.633 → 4.956**);
first 90% was 0.83 s in both, but the final 5-s oscillation **grew** from
71.15 to 94.43 rpm peak-to-peak. On the heavier plant launch acceleration
was **1.933 → 1.928 rev/s²**, first 90% at 2.36 s in both. There is no
consistent reduction of the acceleration peak or oscillation at 500 Hz.

## Plant and measurements

Wheel speed is integrated from torque, not prescribed. The mechanical model
borrows the [MOTOR_1 free-wheel fixture](../test_speed_loop/README.md) for
`Jwheel=0.00625 kg·m²`, `Kt=1.5 × pole_pairs × Ke ≈ 0.37008 N·m/A`, and drag
equivalent to `0.656 + 0.0484 × |rev/s|` A. The heavier case adds `80 × r²`
to inertia plus rolling resistance `0.008 × 80 × g × r` N·m,
`r=0.760/(2π)` m; it assumes rigid no-slip rolling. Dry friction opposes
motion, including breakaway. Torque uses the interval-average current of a
250-Hz first-order actuator at 16 kHz. The controller gets only Hall sector
changes from integrated angle, 120 sectors/rev; its own carrier preparation
consumes the edges and ticks its PLL. The current plant replaces PWM/FOC
completion, so external current-loop voltage saturation is zero.

The summary samples **both arms at 100 Hz**, not once per PI call:

* `launch_wheel200` / `launch_feedback200`: peak positive
  `[n(t)−n(t−0.20)]/0.20` from rest through the first **physical** 90%
  crossing. `wheel` is mechanical model truth; `feedback` is the production
  filtered Hall/PLL speed. Unprefixed `wheel200` and `feedback200` search the
  entire 40 s, including oscillation on the plateau. `wheel10` and `feedback10`
  instead use a 10-ms difference across the entire run. All are rev/s².
* `hall2x200`: peak difference of two adjacent 200-ms mean speeds obtained
  by counting Hall edges, divided by 0.20 s (400 ms total support). All
  windows slide by 10 ms. The count-based resolution is
  `1/(120 × 0.20²) ≈ 0.208 rev/s²` per edge in the second difference.
* `rise10_90` and `t90` are first-crossing times from model speed (10-ms
  resolution). Overshoot is max positive excess above 155 rpm over 40 s;
  `tail_mean` and `tail_pp` give mean and peak-to-peak wheel rpm on [35,40) s.
  A first crossing is not settling.
* Each optional trace row is the state at `t` just **after** clock dispatch,
  before advancing mechanics: wall time, model wheel speed, 100-Hz held
  feedback, unwrapped signed Hall count, just-applied current request and
  actual current from the preceding interval. All runs start at the same
  0.13-step electrical angle, with zero speed/current and PI/PLL history.

At 155 rpm there are only 0.62 Hall edges per 2 ms on average; at 30 rpm
there are 0.12. More PI calls neither create Hall edges nor refresh the
100-Hz feedback filter. Q16 speed has a resolution of 1/65536 rev/s:
the nominal `0.01 rev/s` traction increment is 655 counts, or about
`0.0099945 rev/s` per 10-ms service sample. A one-tick feedback derivative
can peak well above the physical derivative; a long window can hide a
transient. The 1 rev/s²
setting controls a sampled **positive torque ceiling**, not a hard physical
acceleration limit. There is no independently calibrated loaded-roller inertia/torque,
PWM/ADC, tyre slip, bench vibration, interrupt latency, or deadline measurement
in this model. The two synthetic loads are matched rate comparisons, not
predictions for a rider or measurements on silicon.
