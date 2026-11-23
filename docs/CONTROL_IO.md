# Controller I/O: commands in, sampled results out

`BldcController::io()` returns a read-only reference to one native
[`MotorIo`](../libecu/include/motor_io.hpp). A control tick reads commands and
writes results in place: there is no second status object to publish or
per-period frame to copy. PI integrators, filter history and phase accumulators
persist between calls.

| Writer | Fields it updates |
|---|---|
| Command interface | `in.control_mode`, `in.drive_mode`, `in.algorithm`, speed/current demand and the prepared open-loop field rate. |
| SysTick | `in.current_speed_rps`, `in.temperature_c` and (in velocity mode) `in.target_current`. |
| Current interrupt | `out.measured_position`, `out.hall`, `out.pll`, `out.field` and `out.electrical`: shunts, bus voltage, d/q results, phase duties and saturation. |

Setters validate and convert commands before publishing related values in
one short critical section. Mode transitions may also seed or reset state.
On STM32, the speed PI runs at 1 kHz by default; filtered mechanical feedback,
temperature, the setpoint ramp and traction limits update at 100 Hz.

## Which signal drives the motor?

In velocity mode (`AT+MODE=1`), `AT+SPD` is a mechanical speed request and the
speed PI sets the current demand in amperes. In torque mode (`MODE=2`),
`AT+CUR` supplies that demand. Open loop (`MODE=0`) also regulates `CUR`, but
`SPD` sets the rate of a generated field. The Hall-derived angle in
`out.pll.angle_steps` and electrical speed in `out.pll.speed_steps_s` still
describe the **rotor**. `in.current_speed_rps` is the filtered mechanical
speed in revolutions per second, signed relative to the drive direction.
`NEUTRAL` clears the drive demands while acquisition can continue tracking it.

Six-step regulates the conducting shunt; FOC regulates d/q current. `v_d`,
`v_q` and phase duties are outputs. Reads and writes of the former `AT+EMODE`
and `AT+DUT` commands return `ERROR`. Changing speed-PI gains preserves its
output when a live retune is possible; changing current-PI gains resets that
regulator's history.

## Reading a result

- Foreground readers copy only the fields they need under `CriticalSection`,
  then format or transmit after releasing it. The reference from `io()` alone
  does not freeze a sample.
- `AT+CUR?` reads the target. `AT+STATUS?` includes measured Id/Iq; its
  second field is the literal `1` for wire compatibility. `reportedDuty()` is
  six-step applied duty or, when enabled, FOC modulation computed on the slow
  tick. The individual FOC phase duties are updated on **every** current tick.
- Both STM32 ports call `pwmInterruptHandler()` once per full PWM period.
  The board passes `controller.io()` to `captureCurrentSample()` only after
  a successful tick and only if `wantsCurrentSample()` is true. AT records
  the sample there and transmits it later in the foreground.

See [the two STM32 interrupt paths](CURRENT_FRAME.md), the
[FOC handoff](FOC_HANDOFF.md) and the [AT wire format](AT_COMMANDS.md).
