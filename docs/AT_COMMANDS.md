# AT Command Reference

Open ECU communicates with a host computer over UART using a simple AT command protocol. All commands require a CRC-16 checksum.

## Quick Start

### Connecting

Open a serial terminal at **2,000,000 baud, 8 data bits, no parity, 1 stop bit** (2M 8N1).

```
Device:   /dev/ttyACM0  (Linux USB CDC)
Baud:     2000000
Data:     8 bits
Parity:   None
Stop:     1 bit
Flow:     None
```

The UART is routed through **USART2** on the STM32G431:

| Signal | Pin | Direction |
|--------|-----|-----------|
| TX | PB3 | MCU output |
| RX | PB4 | MCU input |

### Sending Your First Command

1. Connect your serial adapter
2. Send a command with CRC: `AT+VER?*CRC\r\n`
3. Calculate CRC of `AT+VER?` using CRC-16/CCITT-FALSE
4. You should receive `+VER:1.0.0\r\n`

Python example:

```python
import serial

CRC_POLY = 0x1021

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ CRC_POLY
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

def send(ser: serial.Serial, cmd: str):
    crc = crc16(cmd.encode("ascii"))
    frame = f"{cmd}*{crc:04X}\r\n"
    ser.write(frame.encode("ascii"))
    return ser.readline().decode("ascii").strip()

ser = serial.Serial("/dev/ttyACM0", 2000000, timeout=1)
print(send(ser, "AT+VER?"))   # +VER:1.0.0
print(send(ser, "AT+SPD=50.0"))  # OK
```

A PyQt6-based monitor application is available at `utility/monitor.py` with real-time telemetry plotting and a current waveform tab.

## CRC-16 Checksum

Every command must carry a CRC-16 checksum appended after a `*` character. The CRC covers the entire command string from `A` up to (but not including) the `*`.

### Algorithm

| Parameter | Value |
|-----------|-------|
| Name | CRC-16/CCITT-FALSE |
| Polynomial | 0x1021 |
| Initial value | 0xFFFF |
| Input reflection | No |
| Output reflection | No |
| Final XOR | 0x0000 |

The implementation uses a 256-entry lookup table for speed. The full source is in `libecu/src/crc16.cpp`.

### Wire Format

```
AT+CMD=value*A1B2\r\n
             ^^^^  4 hexadecimal digits of the CRC
```

The CRC hex digits are uppercase. Four digits, zero-padded. For example a CRC of `0x00F1` becomes `*00F1`.

### Frame Breakdown

```
  A   T   +   S   P   D   =   5   0   .   5   *   A   1   B   2   \r  \n
  |<--- command text (CRC input) --->|   |<- CRC ->|   |<- line end ->|
```

### Parser States

The firmware processes characters through a five-state machine:

| State | Meaning |
|-------|---------|
| Idle | Waiting for first non-newline character |
| Receiving | Accumulating command text |
| CrcParsing | Looking for `*` in accumulated buffer |
| CrcAccumulating | Reading 4 hex CRC digits |
| Execute | Command ready to dispatch |

Lines beginning with `\r` or `\n` are silently ignored. If no `*` is found before `\r\n`, the response is `ERROR`.

### Python Reference Implementation

```python
CRC_POLY = 0x1021

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ CRC_POLY
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

def format_at_command(cmd: str) -> str:
    crc = crc16_ccitt(cmd.encode("ascii"))
    return f"{cmd}*{crc:04X}\r\n"
```

## Command Reference

### Speed Control (AT+SPD)

Set target motor speed or read the current measured speed.

| | |
|---|---|
| **Set** | `AT+SPD=<val>*<CRC>\r\n` |
| **Query** | `AT+SPD?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+SPD:123.45\r\n` |
| **Range** | 0.0 to 200.0 RPS |
| **Unit** | Revolutions per second |

**Examples:**

```
> AT+SPD=50.5*A1B2\r\n     (CRC of "AT+SPD=50.5")
< OK\r\n

> AT+SPD?*B3C4\r\n         (CRC of "AT+SPD?")
< +SPD:48.32\r\n
```

### Current Control (AT+CUR)

Set target motor current or read the current target value.

| | |
|---|---|
| **Set** | `AT+CUR=<val>*<CRC>\r\n` |
| **Query** | `AT+CUR?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+CUR:2.50\r\n` |
| **Range** | -6.0 to 6.0 A |

Negative current values reverse the current direction, enabling regenerative braking when the motor is spinning forward.

**Examples:**

```
> AT+CUR=3.5*D1E2\r\n
< OK\r\n

> AT+CUR?*F3A4\r\n
< +CUR:3.50\r\n
```

### Duty Cycle (AT+DUT)

Set duty cycle in open-loop mode or read current duty cycle.

| | |
|---|---|
| **Set** | `AT+DUT=<val>*<CRC>\r\n` |
| **Query** | `AT+DUT?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+DUT:0.30\r\n` |
| **Range** | 0.0 to 1.0 |

### Control Mode (AT+MODE)

Set or read the mechanical control mode.

| | |
|---|---|
| **Set** | `AT+MODE=<val>*<CRC>\r\n` |
| **Query** | `AT+MODE?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+MODE:1\r\n` |

| Value | Mode | Description |
|-------|------|-------------|
| 0 | OPEN_LOOP | Open loop control, timing-based, no sensors |
| 1 | VELOCITY | Closed loop velocity control with PID and Hall sensors |
| 2 | TORQUE | Closed loop torque control, fixed duty or current with Hall sensors |

**Examples:**

```
> AT+MODE=1*ABCD\r\n
< OK\r\n

> AT+MODE?*EF01\r\n
< +MODE:1\r\n
```

### Electric Mode (AT+EMODE)

Set or read the electrical control strategy.

| | |
|---|---|
| **Set** | `AT+EMODE=<val>*<CRC>\r\n` |
| **Query** | `AT+EMODE?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+EMODE:1\r\n` |

| Value | Mode | Description |
|-------|------|-------------|
| 0 | VOLTAGE | Direct voltage or duty cycle control |
| 1 | CURRENT | Current control with PI inner loop at 40kHz (PWM frequency) |

### Drive Algorithm (AT+ALGO)

Select the electrical algorithm that drives the inverter. Orthogonal to
`AT+EMODE`: the two combine into four working modes.

| | |
|---|---|
| **Set** | `AT+ALGO=<val>*<CRC>\r\n` |
| **Query** | `AT+ALGO?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+ALGO:0\r\n` |

| Value | Algorithm | Description |
|-------|-----------|-------------|
| 0 | SIX_STEP | Trapezoidal six-step commutation |
| 1 | FOC | Field-oriented control with space-vector modulation (default) |

Combined with `AT+EMODE`:

| ALGO | EMODE | What runs |
|------|-------|-----------|
| 0 | 0 | Trapezoidal, duty commanded directly by `AT+DUT` |
| 0 | 1 | Trapezoidal, inner current PI on the DOWN-phase shunt (`AT+CPID`) |
| 1 | 0 | SVPWM, q-axis voltage commanded by `AT+DUT`, no current loop |
| 1 | 1 | SVPWM with d/q current PI (`AT+FPID`) — full field-oriented control |

`ALGO=1, EMODE=0` exists for bring-up: it exercises the modulator and the
angle without a current loop in the way, which is the only practical way to
find a wrong `AT+FANG` offset.

> **Switch at standstill or in NEUTRAL.** Changing algorithm is a step change
> in the applied voltage — the outgoing algorithm's integrators are cleared and
> the incoming one reconfigures the bridge.

### FOC Current PID Parameters (AT+FPID)

Set or read the FOC d/q current regulator gains. Both axes share one set of
gains, which is correct for a motor with Ld == Lq.

| | |
|---|---|
| **Set** | `AT+FPID=<kp>,<ki>*<CRC>\r\n` |
| **Query** | `AT+FPID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+FPID:2.1991,2067.2000\r\nOK\r\n` |

> **These are not `AT+CPID` numbers.** The FOC regulators output **volts**
> (kp in V/A, ki in V/A/s) and see one phase; the six-step regulator outputs
> **duty** and sees two phases in series. For MOTOR_1 at 500 Hz the six-step
> gains are ≈0.141 / 132 and the FOC gains are ≈2.20 / 2067. Setting one from
> the other will not end well.

Derivation: placing the PI zero on the plant pole (`Ti = L/R`) gives
`kp = L·ω_c` and `ki = R·ω_c`. See `tuneFocCurrentPi()` in
`libecu/include/current_loop_tuning.hpp`.

### FOC Angle Offset (AT+FANG)

Set or read the electrical angle offset between the PLL's Hall-derived step
angle and the rotor d axis, in **electrical degrees**.

| | |
|---|---|
| **Set** | `AT+FANG=<degrees>*<CRC>\r\n` |
| **Query** | `AT+FANG?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+FANG:-60.00\r\nOK\r\n` |

Range -360.0 to 360.0.

The default of -60° is **derived, not measured**: row *k* of the six-step
commutation table places the stator field at (k·60 − 30)°, and the PLL applies
step round(angle + 1) in FORWARD, so the field sits at (angle·60 + 30)°. At the
six-step torque optimum the field leads the rotor by 90°, giving
θ_d = (angle − 1)·60°.

Expect to trim it on the bench. If it is wrong it is wrong by a whole multiple
of 60°, and the symptom is little or no torque. An inverse commutation table
needs its own value.

### Drive Mode (AT+DMODE)

Set or read the motor rotation direction.

| | |
|---|---|
| **Set** | `AT+DMODE=<val>*<CRC>\r\n` |
| **Query** | `AT+DMODE?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+DMODE:0\r\n` |

| Value | Mode | Description |
|-------|------|-------------|
| 0 | FORWARD | Forward rotation |
| 1 | REVERSE | Reverse rotation |
| 2 | NEUTRAL | Coast, no drive |

### Speed PID Parameters (AT+SPID)

Set or read the speed (outer-loop) PID controller gains. Changes take effect immediately and reset the integrator.

| | |
|---|---|
| **Set** | `AT+SPID=<kp>,<ki>[,<kd>]*<CRC>\r\n` |
| **Query** | `AT+SPID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+SPID:0.010,0.100,0.000\r\nOK\r\n` |

The `kd` parameter is optional. If omitted, it defaults to 0.0. Output limits and sample time are preserved.

**Examples:**

```
> AT+SPID=0.05,1.0*XXXX\r\n
< OK\r\n

> AT+SPID?*XXXX\r\n
< +SPID:0.050,1.000,0.000\r\n
< OK\r\n
```

### Current PID Parameters (AT+CPID)

Set or read the current (inner-loop) PID controller gains. Changes take effect immediately and reset the integrator.

| | |
|---|---|
| **Set** | `AT+CPID=<kp>,<ki>[,<kd>]*<CRC>\r\n` |
| **Query** | `AT+CPID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+CPID:0.010,0.100,0.000\r\nOK\r\n` |

**Examples:**

```
> AT+CPID=0.1,2.0,0.01*XXXX\r\n
< OK\r\n

> AT+CPID?*XXXX\r\n
< +CPID:0.100,2.000,0.010\r\n
< OK\r\n
```

### Firmware Version (AT+VER)

Read the firmware version string.

| | |
|---|---|
| **Query** | `AT+VER?*<CRC>\r\n` |
| **Response** | `+VER:1.0.0\r\n` |

### Motor Status (AT+STATUS)

Read a snapshot of the motor's current state.

| | |
|---|---|
| **Query** | `AT+STATUS?*<CRC>\r\n` |
| **Response** | `+STATUS:<ctrl_mode>,<elec_mode>,<speed>,<target_cur>,<duty>,<bus_volt>,<algo>,<id>,<iq>,<angle_e>\r\n` |

Response fields in order:

| Field | Type | Description |
|-------|------|-------------|
| ctrl_mode | int | Control mode (0=OPEN_LOOP, 1=VELOCITY, 2=TORQUE) |
| elec_mode | int | Electric mode (0=VOLTAGE, 1=CURRENT) |
| speed | float | Current motor speed in RPS |
| target_cur | float | Target current in Amperes |
| duty | float | Current duty cycle (0.0 to 1.0) |
| bus_volt | float | Bus voltage in Volts |
| algo | int | Drive algorithm (0=SIX_STEP, 1=FOC) |
| id | float | Measured d-axis current in Amperes (0 in six-step) |
| iq | float | Measured q-axis current in Amperes (0 in six-step) |
| angle_e | float | Electrical angle used by the algorithm, degrees |

The last four fields were added with FOC; the first six are unchanged, so a
consumer that only knows them still parses.

**Example:**

```
> AT+STATUS?*DEAD\r\n
< +STATUS:1,1,23.45,1.50,0.35,24.56,1,-0.02,1.48,213.7\r\n
```

### Maximum Values (AT+MAXVALS)

Read the firmware's configured safety limits. Useful for host-side UI to set slider ranges.
The values are taken from the live `MotorControlParams` of the `BldcController`, so they
reflect the actual build configuration (see `motor_params` in `main.cpp`).

| | |
|---|---|
| **Query** | `AT+MAXVALS?*<CRC>\r\n` |
| **Response** | `+MAXVALS:<max_speed>,<min_current>,<max_current>,<max_voltage>,<max_duty>,<max_temp>\r\n` |

Response fields in order (all printed with 2 decimals):

| Field | Type | Source |
|-------|------|--------|
| max_speed | float | `max_speed_rps` -- maximum speed in RPS |
| min_current | float | `min_current` -- minimum (negative) current in Amperes |
| max_current | float | `max_current` -- maximum current in Amperes |
| max_voltage | float | `max_voltage` -- maximum bus voltage in Volts |
| max_duty | float | `max_duty_cycle` -- maximum duty cycle (0.0-1.0) |
| max_temp | float | `max_temperature_c` -- thermal cut-out in degrees Celsius. Above it the drive goes to NEUTRAL, the same way over-voltage does |

**Example:**

```
> AT+MAXVALS?*XXXX\r\n
< +MAXVALS:200.00,-6.00,6.00,36.00,0.95,100.00\r\n
```

## Telemetry (AT+TM)

Enable or disable continuous telemetry streaming. When enabled, the controller sends motor state data at 100Hz (every 10ms), synchronized with the speed control loop (SysTick). Both +TM and +PLL telemetry (see [AT+PLL](#pll-telemetry-at+pll)) are sent every tick (100Hz).

| | |
|---|---|
| **Command** | `AT+TM=<0|1>*<CRC>\r\n` |
| **Response** | `OK\r\n` |

| Value | Effect |
|-------|--------|
| 0 | Disable +TM telemetry |
| 1 | Enable +TM telemetry (default: enabled) |

### Telemetry Format

Each line is a newline-terminated tuple (uses `\n` only, not `\r\n`):

```
+TM:<meas_pos>;<tgt_pos>;<tgt_speed>;<cur_speed>;<duty>;<tgt_cur>;<meas_cur>;<bus_volt>;<pll_angle>;<temp>
```

| Field | Type | Description |
|-------|------|-------------|
| meas_pos | uint8 | Measured rotor position (Hall sensor, 0-5) |
| tgt_pos | uint8 | Target rotor position (commutation step, 0-5) |
| tgt_speed | float | Target speed in RPS |
| cur_speed | float | Current measured speed in RPS |
| duty | float | Current duty cycle |
| tgt_cur | float | Target current in Amperes |
| meas_cur | float | Measured current in Amperes, low-pass filtered (`measured_current_lpf_alpha`, 10 ms by default). The raw per-PWM-cycle sample would alias the chopping ripple at the 100 Hz telemetry rate; use `AT+OSC` for the unfiltered waveform |
| bus_volt | float | Bus voltage in Volts |
| pll_angle | float | PLL rotor angle in steps (0.0-6.0, one electrical period) |
| temp | float | NTC temperature in degrees Celsius (PB14). `-273.0` means the sensor read open or the conversion failed -- not a cold motor. Above `max_temperature_c` (see [AT+MAXVALS](#maximum-values-at+maxvals)) the drive trips to NEUTRAL |

`temp` was appended after the other nine fields, so a parser that splits on `;` and
reads the first nine still works. Prefer `len(fields) >= 9` over `== 9`.

**Example:**

```
+TM:3;5;23.45;22.10;0.35;1.50;1.45;24.56;3.2;41.7
+TM:3;5;23.45;22.15;0.35;1.50;1.46;24.55;3.4;41.7
+TM:4;5;23.45;22.20;0.35;1.50;1.47;24.56;3.6;41.8
```

The telemetry output is invoked from the 100Hz speed control loop (SysTick). Both +TM and +PLL are emitted every tick (100Hz). Each call to `sendTelemetry()` or `sendPllTelemetry()` produces one line.

## Oscilloscope (AT+OSC)

Start or stop the oscilloscope feature, which captures high-speed samples from the PWM ISR at 40kHz (PWM frequency).

| | |
|---|---|
| **Command** | `AT+OSC=<0|1>*<CRC>\r\n` |
| **Query** | `AT+OSC?*<CRC>\r\n` |
| **Response** | `OK\r\n` |
| **Query response** | `+OSC:1\r\n` |

| Value | Effect |
|-------|--------|
| 0 | Stop oscilloscope, stop streaming |
| 1 | Start oscilloscope, begin capturing |

### Buffer Architecture

The oscilloscope uses a **single buffer with two-phase swapping** for safe capture from the PWM ISR:

| Parameter | Value |
|-----------|-------|
| Buffer size | 1024 samples |
| Capture rate | 40kHz (from PWM ISR) |
| Burst duration | 1024 / 40000 = 25.6ms per burst |
| Swap trigger | Buffer full (write index reaches 1024) |

When the write phase fills the buffer (write index reaches 1024), the phase swaps under a critical section (interrupts disabled): the accumulated buffer becomes the output buffer, and the output buffer is reset for the next capture cycle.

### Captured Data

Each sample captures seven values, packed into 12 bytes:

| Field | Type | Description |
|-------|------|-------------|
| target_current_ma | int16 | Target current setpoint, milliamps |
| measured_current_ma | int16 | Raw measured current, milliamps |
| i_d_ma | int16 | d-axis current, milliamps (0 in six-step) |
| i_q_ma | int16 | q-axis current, milliamps (0 in six-step) |
| angle_e | uint16 | Electrical angle, full scale = one electrical revolution |
| duty_cycle | uint8 | Current duty cycle × 100 (0-100) |
| position | uint8 | Rotor position from Hall sensors |

The currents are milliamps rather than floats because the buffer is 1024
samples deep and the MCU has 32 KB of RAM in total. int16 milliamps spans
±32.7 A, which covers the ±34 A the shunt/PGA chain can measure at all, so
the packing loses nothing the hardware could have reported. Carrying the
three FOC signals as floats would have taken the buffer from 12 KB to 24 KB;
this way it stays at 12 KB.

### Output Format

Samples stream out as `\r\n`-terminated lines, one per call to `processOscOutput()`:

```
+OSC:<sample_index>,<meas_cur_mA>,<tgt_cur_mA>,<duty_x100>,<position>,<id_mA>,<iq_mA>,<angle_deg_e>\r\n
```

| Field | Type | Description |
|-------|------|-------------|
| sample_index | int | Sequential sample counter starting at 0 |
| meas_cur_mA | int16 | `measured_current * 1000`, signed integer |
| tgt_cur_mA | int16 | `target_current * 1000`, signed integer |
| duty_x100 | uint8 | `duty_cycle * 100` (0-100), unsigned integer |
| position | uint8 | Rotor position (Hall sensor, 0-5) |
| id_mA | int16 | d-axis current × 1000 (0 in six-step) |
| iq_mA | int16 | q-axis current × 1000 (0 in six-step) |
| angle_deg_e | int | Electrical angle in degrees, 0-359 |

The last three fields were added with FOC. The first five are unchanged, so a
consumer that splits on commas and takes what it knows still works —
`utility/capture_osc.py` accepts either width.

The end of a burst is signaled by an empty data line:

```
+OSC:\r\n
```

After the end-of-burst marker, oscilloscope continues capturing the next burst (the phase swaps back to Accumulating).

**Example burst:**

```
+OSC:0,1500,2000,35,3
+OSC:1,1485,2000,35,3
+OSC:2,1520,2000,35,3
+OSC:3,1490,2000,35,4
...
+OSC:1023,1510,2000,35,5
+OSC:
```

To convert the scaled values back to physical units:

```python
measured_current_A = meas_cur_x1000 / 1000.0
target_current_A   = tgt_cur_x1000 / 1000.0
duty_cycle         = duty_x100 / 100.0
```

## PLL Telemetry (AT+PLL)

Enable or disable continuous PLL (Phase-Locked Loop) telemetry streaming. When enabled, the controller emits a `+PLL:` line at 100Hz (every control tick) containing the internal state of the MotorPLL observer.

| | |
|---|---|
| **Command** | `AT+PLL=<0|1>*<CRC>\r\n` |
| **Query** | `AT+PLL?*<CRC>\r\n` |
| **Response** | `OK\r\n` |
| **Query response** | `+PLL:1\r\n` |

| Value | Effect |
|-------|--------|
| 0 | Disable +PLL telemetry |
| 1 | Enable +PLL telemetry (default: disabled) |

### PLL Telemetry Format

Each line is a newline-terminated tuple (uses `\n` only, not `\r\n`):

```
+PLL:<angle_per_second>;<pll_integral>;<angle>;<hall_state_raw>;<is_sync>
```

| Field | Type | Description |
|-------|------|-------------|
| angle_per_second | float | PLL-estimated rotor speed in steps/sec (6 steps = 1 electrical revolution) |
| pll_integral | float | PI integrator term in steps/sec (clamped to ±max_electrical_speed) |
| angle | float | PLL-estimated rotor angle in steps [0..6) (6 steps = 1 electrical revolution) |
| hall_state_raw | int | Actual Hall sensor reading [0..5] (raw GPIO state before any filtering) |
| is_sync | int | PLL synchronized with Hall sensor flag (1 = tracking, 0 = snapping angle to Hall) |

**Example:**

```
+PLL:291.158;218.631;3.142;4;1
+PLL:295.430;220.104;0.571;5;1
+PLL:288.712;219.502;5.028;2;0
```

**Note:** The `angle` field mirrors `+TM:pll_angle` (same value, sampled at the same tick). Position fields `meas_pos` and `tgt_pos` remain `+TM:`-only. To compute the PLL tracking error, use `+TM:` fields: `error = measured_position - pll_angle` (wrapped to [-3, +3] per electrical period). The slip threshold is 3.0 steps.

## Hall Health Telemetry (AT+HSTATUS)

Enable or disable continuous Hall sensor health telemetry. When enabled, the controller emits a `+HSTATUS:` line at 100Hz containing the state of the `HallMonitor`.

Unbuffered, like `+PLL`. Every field is a leaky accumulator or a running total rather than an instantaneous sample, so a 100Hz poll sees the same picture the monitor does - a raw Hall reading at this rate would be an arbitrary point between commutations.

| | |
|---|---|
| **Command** | `AT+HSTATUS=<0|1>*<CRC>\r\n` |
| **Query** | `AT+HSTATUS?*<CRC>\r\n` |
| **Response** | `OK\r\n` |
| **Query response** | `+HSTATUS:1\r\n` |

### Format

```
+HSTATUS:<fault>;<invalid_score>;<erratic_score>;<edge_accum>;<invalid_events>;<edges>;<last_pos>;<standing_us>
```

| Field | Type | Description |
|-------|------|-------------|
| fault | int | 0 = none, 1 = INVALID_CODE, 2 = ERRATIC_SEQUENCE. Latched until `AT+HCLEAR` |
| invalid_score | float | Leaky accumulator of illegal 000/111 codes. Counting is **disabled** by default (see below) |
| erratic_score | float | Non-advancing edges as a **fraction** of all edges (0..1). Faults at 0.35 |
| edge_accum | float | Leaky edge count; divide by `decay_time_s` (0.5) for edges/sec |
| invalid_events | uint32 | Illegal-code readings since reset |
| edges | uint32 | Hall transitions since reset |
| last_pos | uint8 | Last valid decoded position (0-5) |
| standing_us | float | Microseconds the latest reading has been an illegal code. **This is the detector**; faults at `invalid_persist_time_s` (300 us) |

**Example:**

```
+HSTATUS:0;0.00;0.00;301.50;0;18422;3;0
+HSTATUS:0;0.00;0.00;298.12;41;18512;5;50
+HSTATUS:1;0.00;0.00;44.80;37;18544;2;312
```

### Reading it

The two scores are the useful numbers, because each is directly comparable to its threshold - they are the margin you have against a false trip.

- **`standing_us` is the fault detector.** Illegal codes turn out to be common on intact wiring - Hall bounce, strongly speed dependent - but they are always *transient*, because a bouncing line keeps raising interrupts and the next reading is valid. A stuck line has nothing left to raise an edge, so its illegal code stands for a whole commutation step. Measured on MOTOR_1 at 31 V:

  | speed | illegal/s | max standing | step period |
  |---|---|---|---|
  | 6 RPS | 0 | 0 us | 1389 us |
  | 8 RPS | 44 | 0 us | 1042 us |
  | 9 RPS | 3532 | **50 us** | 926 us |

  A 20x gap, so the 300 us threshold sits six times above the benign maximum and three times below the fault signature.

- **`invalid_score` is counting, and counting does not work here.** At 9 RPS the benign rate settles it near 1766, while a genuinely stuck line would settle it near 90 - the benign rate is twenty times the fault signature, so no threshold separates them. `invalid_threshold` therefore defaults to 0 (disabled). Enable it only on hardware where illegal codes are genuinely rare.
- **`erratic_score` is the position-corruption margin, expressed as a fraction.** It is deliberately not a count: a count scales with the edge rate, so any absolute threshold is really a speed limit. A captured false positive showed this - at 6 RPS under 2 A of load, healthy running produced a surplus of 14-21 non-advancing edges against ~345 total, and the old absolute threshold of 20 tripped on it. As a fraction that is 0.06, against a threshold of 0.35. Unloaded running measures near 0.00; genuine chatter approaches 1.0 because the edges cancel completely.
- **`edge_accum / 0.5` is the edge rate**, which should equal the electrical speed in steps/sec. A large mismatch against `+TM:cur_speed` means edges are being lost or manufactured.

A latched fault forces `DriveMode::NEUTRAL`. It does *not* re-assert every cycle - it latches once, so a persistent condition reads as a fault rather than as commands being silently ignored.

## Clear Hall Fault (AT+HCLEAR)

Release a latched Hall fault and zero the monitor's accumulators. Use after dealing with the wiring; if the fault condition is still present it will latch again within milliseconds.

| | |
|---|---|
| **Command** | `AT+HCLEAR*<CRC>\r\n` |
| **Response** | `OK\r\n` |

The drive stays in `NEUTRAL` after clearing - re-enable it with `AT+DMODE`.

## PLL Gain Tuning (AT+PLLID)

Set or read the PLL PI gains.

| | |
|---|---|
| **Set** | `AT+PLLID=<kp>,<ki>*<CRC>\r\n` |
| **Query** | `AT+PLLID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+PLLID:100.000,5000.000\r\nOK\r\n` |

**Default values:** `kp=100.0`, `ki=5000.0` (giving ω_n=70.7 rad/s, ζ=0.707).

The `kd` parameter is not used (PLL is PI-only, no derivative term).

**Examples:**

```
> AT+PLLID=100,5000*XXXX\r\n
< OK\r\n

> AT+PLLID?*XXXX\r\n
< +PLLID:100.000,5000.000\r\n
< OK\r\n
```

**Tuning notes:**
- `ki` determines the PLL bandwidth: ω_n = √ki. Higher ki = faster tracking but potential instability if too high.
- `kp` determines damping: ζ = kp / (2·√ki). Target ζ ≈ 0.7–1.0 for well-damped response.
- The steady-state tracking error during acceleration is: `e_ss = α / ki` (in steps), where α is the rotor acceleration in steps/s².
- The commutation reversal threshold is 1.5 steps (90° field offset). If the PLL tracking error exceeds 1.5 steps, the effective stator field lead becomes negative, causing torque reversal and motor runaway.
- With ki=5000 and max acceleration 100 RPS/s (4800 steps/s²): e_ss = 0.96 steps — safely below the 1.5-step threshold.

## Error Responses

All error conditions produce the same response:

```
ERROR\r\n
```

| Condition | Cause |
|-----------|-------|
| Invalid CRC | Computed CRC does not match received CRC |
| Out of range | Parameter value exceeds valid range for the command |
| Unknown command | Command string is not recognized (no `AT+` prefix, or unknown name) |
| Controller unavailable | `BldcController` pointer is null when a command needs it |
| Buffer overflow | Command exceeds 64 characters (MAX_COMMAND_LENGTH) |
| Invalid hex in CRC | Non-hex characters found in the 4-digit CRC field |
| Missing CRC | No `*` found before line ending |

## Command Summary Table

| Command | Set/Query | Range | Response |
|---------|-----------|-------|----------|
| `AT+SPD=<val>` | Set | 0.0 .. 200.0 RPS | `OK` |
| `AT+SPD?` | Query | -- | `+SPD:123.45` |
| `AT+CUR=<val>` | Set | -6.0 .. 6.0 A | `OK` |
| `AT+CUR?` | Query | -- | `+CUR:2.50` |
| `AT+DUT=<val>` | Set | 0.0 .. 1.0 | `OK` |
| `AT+DUT?` | Query | -- | `+DUT:0.30` |
| `AT+MODE=<val>` | Set | 0, 1, 2 | `OK` |
| `AT+MODE?` | Query | -- | `+MODE:1` |
| `AT+EMODE=<val>` | Set | 0, 1 | `OK` |
| `AT+EMODE?` | Query | -- | `+EMODE:1` |
| `AT+DMODE=<val>` | Set | 0, 1, 2 | `OK` |
| `AT+DMODE?` | Query | -- | `+DMODE:0` |
| `AT+SPID=<kp>,<ki>[,<kd>]` | Set | any float | `OK` |
| `AT+SPID?` | Query | -- | `+SPID:0.050,1.000,0.000` |
| `AT+CPID=<kp>,<ki>[,<kd>]` | Set | any float | `OK` |
| `AT+CPID?` | Query | -- | `+CPID:0.050,1.000,0.000` |
| `AT+ALGO=<val>` | Set | 0, 1 | `OK` |
| `AT+ALGO?` | Query | -- | `+ALGO:0` |
| `AT+FPID=<kp>,<ki>` | Set | any float | `OK` |
| `AT+FPID?` | Query | -- | `+FPID:2.1991,2067.2000` |
| `AT+FANG=<deg>` | Set | -360.0 .. 360.0 | `OK` |
| `AT+FANG?` | Query | -- | `+FANG:-60.00` |
| `AT+PLLID=<kp>,<ki>` | Set | any float | `OK` |
| `AT+PLLID?` | Query | -- | `+PLLID:100.000,5000.000` |
| `AT+VER?` | Query | -- | `+VER:1.0.0` |
| `AT+STATUS?` | Query | -- | `+STATUS:1,1,23.45,...` |
| `AT+MAXVALS?` | Query | -- | `+MAXVALS:200.00,-6.00,6.00,36.00,0.95` |
| `AT+TM=<0|1>` | Set | 0, 1 | `OK` |
| `AT+PLL=<0|1>` | Set/Query | 0, 1 | `OK` / `+PLL:1` |
| `AT+HSTATUS=<0|1>` | Set/Query | 0, 1 | `OK` / `+HSTATUS:1` |
| `AT+HCLEAR` | Action | -- | `OK` |
| `AT+OSC=<0|1>` | Set/Query | 0, 1 | `OK` / `+OSC:1` |
