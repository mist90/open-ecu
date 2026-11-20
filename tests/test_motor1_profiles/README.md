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
  12 A at rest, 20 A at 37.5 RPM. `LIBECU_MOTOR1_T60_45A=1` together with
  the load profile extends the positive command limit to 45 A; the negative
  limit stays at −6 A. A command limit is distinct from a phase-current trip.

Run `bash tests/test_motor1_profiles/run.sh` from the project root to check
the current corrections, their handoff, the wheel-order observer and exact
reciprocal-cache hits and misses. `bash tests/test_pi_interface/run.sh` also
checks the per-update integral ceiling and saturation in both current ranges.
