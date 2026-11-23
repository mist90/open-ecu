# Scooter speed PI cadence: 100 vs 500 Hz

`./tests/test_speed_pi_rate/run.sh [trace-directory]` builds the same real fixed-point
controller twice. The host fixture's SysTick is always 500 Hz; only the speed PI rate differs.
Temperature, speed feedback, shaped error, target ramp and traction ceiling update once
per 10 ms. At 500 Hz the intermediate PI calls immediately publish current using those held
observations and the live inner-loop saturation sign.

Each case requests 155 rpm from rest and holds for 40 s on a free wheel or an 80 kg
equivalent load. The default calibration and the fixed reference PI pairs 1.25/5, 3/24
and 6/24 are exercised. A separate 3/24 arm disables only the traction ceiling. This
fixture isolates speed control: a 250 Hz first-order current actuator and integrated
mechanics generate real Hall events; no mechanical speed or ideal angle is injected
into `BldcController`. Raw Hall timing and the downstream PLL run independently.

The summary samples model wheel speed and the **selected PI feedback**, not monitor
speed, on the same 100 Hz grid. It reports 10/200-ms speed differences, acceleration
from consecutive 200-ms Hall count windows, launch `t90` (first crossing, **not**
settling), peak overshoot and final 5-s peak-to-peak. The optional two traces use a
common 2-ms grid. The test checks cadence, no repeated slow work, current rails,
live limiting and tracking. To exercise actual generated voltages, filtered current
feedback, sudden load/release and small speed steps, use the
[FOC/PMSM dynamics fixture](../test_speed_dynamics/README.md).
