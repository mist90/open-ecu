# FOC handoff: commands, feedback and timing

Select FOC with `AT+ALGO=1`. The electrical command is torque current in
amperes; FOC computes the d/q voltage and three PWM duties from the target,
measured shunts and DC bus. Use `AT+MODE` to choose the source of the target:

| Mechanical mode | Current source | Field angle |
|---|---|---|
| `MODE=0` (open loop) | `AT+CUR` | Generated at the mechanical RPS set by `AT+SPD`; Hall still reports the rotor. |
| `MODE=1` (velocity) | Speed PI driven by `AT+SPD` | Hall/PLL |
| `MODE=2` (torque) | `AT+CUR` | Hall/PLL |

`AT+DMODE` selects forward, reverse or neutral. Change algorithms at standstill
or in neutral; the transition resets regulator history. With the bridge enabled,
zero current is still a regulation target. To disconnect the bridge, select
neutral (`DMODE=2`).

`AT+CUR?` returns the current target. Use `+STATUS` or `+OSC` for measured Id
and Iq. `+STATUS` field 2 is always `1`, retained for wire compatibility. The
reported FOC duty is modulation depth, not a phase command. For command
framing, field order and gain units see the [AT reference](AT_COMMANDS.md).
For angle alignment, STM32G431's [JK42BLS01 defaults](JK42BLS01_FOC.md)
use +120 electrical degrees; STM32F410's MOTOR_1 uses −60. See the
[FOC algorithm](software.md#field-oriented-control) for the angle convention.

## ISR timing

At 20 kHz the STM32 current loop has 50 µs per carrier period. The G431
[build notes](../STM32G431/build.sh) refer to the former §§1 and 8.4: **before
FOC's transforms and regulators**, the ISR took 30.2 µs mean and 38.3 µs
maximum at `-O0`, versus 14.4 µs at `-Os`. These are not timings for the
current kernel. Use the optimized build and measure the complete current
interrupt when qualifying a change. The [STM32 interrupt paths](CURRENT_FRAME.md)
identify where the current frame starts on each board.

## STM32G431 dead time

The [G431 board port](../STM32G431/Core/Src/board.cpp) requests **350 ns**.
Its note on the dead-time sweep (formerly §15) records lower DC input current
at this setting than at 235 ns on the measured inverter. The timer quantizes
dead time; the boot message reports both requested and programmed values.
Recheck the setting if the switches, gate driver or carrier frequency change.
