# Motor calibration in the FOC and speed loops

Run `bash tests/test_motor_calibration/run.sh` from `open-ecu`. The test drives
the real `FocAlgorithm::updateNative` at 16 kHz with known phase currents,
independently signed Hall speeds, torque requests and both drive directions.
It checks the d/q proportional voltage and the voltage retained by each
integrator when the error vanishes across a change in gain. The `MOTOR_1`
fixture also checks that requesting d/q decoupling fails and leaves it off.
The JK42 fixture enables decoupling and checks the sign and magnitude of the
back-EMF voltage at zero current in both directions.

| Absolute Hall speed (steps/s) | MOTOR_1 d-axis weight | MOTOR_1 q-axis weight |
|---|---|---|
| 0–60 | 1/16 | 1/16 |
| 60–75 | linear to 1 | linear to 1 |
| 75–360 | 1 | 1 |
| 360–480 | 1 | linear to 1/16 |
| ≥480 | 1 | 1/16 |

The test runs float and fixed point for MOTOR_1 and JK42. JK42 and the
standalone generic FOC have unity weights. The fixed-point probe also checks
raw currents against a separately rounded oracle at single-LSB offsets,
including negative ties and both limits of signed speed and current.

Related regressions:

* `tests/test_pi_interface/run.sh` checks the positive speed-PI integral cap:
  12 A through 0.125 mechanical rev/s (including reverse motion in the
  commanded-direction frame), then +16 A per rev/s up to the board/motor
  current rail. The JK42 fixture keeps its configured integral limit.
* `tests/test_command_processor/run.sh` boots the shared main and verifies
  the 20 A default board rail, a 45 A board, lower board limits, both motor
  limits and the independent −6 A braking floor.
* `tests/test_control/run.sh float|fixed|numeric-fixed` verifies MOTOR_1's
  27.5/12 → 11/3 schedule on STM32, including each endpoint of the integral
  step when the gains change. The [100/500 Hz comparison](../test_speed_pi_rate/README.md)
  drives the speed controller with the same 100 Hz measured feedback in both runs.
