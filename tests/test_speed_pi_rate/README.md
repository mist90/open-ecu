# Scooter speed PI cadence: 100 vs 500 Hz

`./tests/test_speed_pi_rate/run.sh [trace-directory]` builds the same real fixed-point
controller twice. The host fixture's SysTick is always 500 Hz; only the speed PI rate differs.
Temperature, speed feedback, shaped error, target ramp and rolling integral limit update once
per 10 ms. At 500 Hz the intermediate PI calls immediately publish current using those held
observations and the live inner-loop saturation sign.

Each case requests 155 rpm from rest and holds for 40 s on a free wheel or an 80 kg
equivalent load. Both use the production speed schedule: 27.5/12 through 0.125 RPS,
interpolated to 11/3 at 1 RPS using absolute measured feedback. The 4 RPS/s ramp
applies to the speed reference in all four rate/load combinations.
A 250 Hz first-order current actuator and integrated mechanics generate Hall events;
no mechanical speed or ideal angle is injected
into `BldcController`. Raw Hall timing and the downstream PLL run independently.

The summary samples model wheel speed and the **selected PI feedback**, not monitor
speed, on the same 100 Hz grid. It reports 10/200-ms speed differences, acceleration
from consecutive 200-ms Hall count windows, launch `t90` (first crossing, **not**
settling), peak overshoot and final 5-s peak-to-peak. The optional two traces use a
common 2-ms grid; their `kp,ki` fields are the effective gains, whereas the summary
identifies the initial gain pair. The test checks the number of PI ticks and temperature
reads, the immediate publication of intermediate current requests, and both current rails.
At each PI rate the loaded wheel must accelerate more slowly and reach 90% later
than the free wheel. Both must stay within 2% of 155 rpm over the final 5 s, with
mean wheel and feedback speeds within 0.05 RPS. To exercise actual generated voltages,
filtered current feedback, sudden load/release and small speed steps, use the
[FOC/PMSM dynamics fixture](../test_speed_dynamics/README.md).
