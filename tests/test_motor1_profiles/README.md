# MOTOR_1 fixed-point bench profiles

These opt-in `libecu` profiles describe the motor, not a board or its current
protection. They use a 16 kHz current loop and a 100 Hz speed loop. They do
not change the default settings for a port that does not select them.

- `LIBECU_MOTOR1_SMOOTH_BENCH=1` schedules the d/q PI error weights and
  low-speed Hall angle and current corrections for an unloaded wheel.
- Adding `LIBECU_MOTOR1_SPAN_BENCH=1` uses held corrections prepared by
  `BldcController::updateBenchTrims1k()` on a 1 kHz tick. The optional
  DC-bus reciprocal cache falls back to the exact calculation on a miss.
- `LIBECU_MOTOR1_T60_LOAD=1` selects the loaded-roller PI and PLL tuning
  instead. The speed PI caps its integral using **measured forward speed**:
  12 A up to 7.5 RPM, then 20 A at 37.5 RPM (45 A at 131.25 RPM with
  `LIBECU_MOTOR1_T60_45A=1`). The negative current limit stays at −6 A.
  A command limit is distinct from a phase-current trip.
- On T60, `LIBECU_T60_UNLOADED_PI=1` selects outer speed-loop **Kp = 1.25,
  Ki = 5**. This pair was tested only on a free wheel. Loaded operation needs
  different gains, tuned and verified under the intended load. Without this
  override, the T60 speed PI defaults to Kp = 3, Ki = 24.

Run `bash tests/test_motor1_profiles/run.sh` from the project root to check
the current corrections, their handoff, the wheel-order observer and exact
reciprocal-cache hits and misses. `bash tests/test_pi_interface/run.sh` also
checks the per-update integral ceiling and saturation in both current ranges.
