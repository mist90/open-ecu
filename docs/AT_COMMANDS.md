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

Both STM32 ports use **USART2**:

| Board | MCU TX | MCU RX |
|-------|--------|--------|
| STM32G431 | PB3 | PB4 |
| STM32F410 | PA2 | PA3 |

On the Nucleo the [ST-LINK USB VCP](NUCLEO_F410RB_IHM08M1.md#uart-over-usb-and-swd)
carries this link.

### Sending Your First Command

1. Open the board's serial port.
2. Send `AT+VER?*0DA3\r\n` (`0DA3` is the CRC-16/CCITT-FALSE of `AT+VER?`).
3. Look for `+VER:<version>\r\n`; periodic telemetry can be interleaved.

Python example:

```python
import serial
import time

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

def query(ser: serial.Serial, cmd: str, prefix: str):
    crc = crc16(cmd.encode("ascii"))
    frame = f"{cmd}*{crc:04X}\r\n"
    ser.write(frame.encode("ascii"))
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii").strip()
        if line.startswith(prefix) or line == "ERROR":
            return line
    raise TimeoutError(cmd)

with serial.Serial("/dev/ttyACM0", 2000000, timeout=0.1) as ser:
    print(query(ser, "AT+VER?", "+VER:"))
    print(query(ser, "AT+STATUS?", "+STATUS:"))
```

A PyQt6-based monitor application is available at `utility/monitor.py` with real-time telemetry plotting and a current waveform tab.

Connecting or reconnecting the monitor only sends CRC-framed queries; it does
not change the mode or demand. It restores the speed setpoint from the first
`+TM:tgt_speed`, not the measured speed in `+STATUS`. In open-field mode,
`AT+SPD` sets field speed and `AT+CUR` sets current. The former voltage-mode
selection (`AT+EMODE`) and direct duty (`AT+DUT`) commands return `ERROR` on
both reads and writes. Applied duty and bus voltage remain available in telemetry.

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
AT+SPD=50.5*1ED8\r\n
            ^^^^  4 hexadecimal digits of the CRC
```

The CRC hex digits are uppercase. Four digits, zero-padded. For example a CRC of `0x00F1` becomes `*00F1`.

### Frame Breakdown

```
  A   T   +   S   P   D   =   5   0   .   5   *   1   E   D   8   \r  \n
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

Set a nonnegative mechanical-RPS request. In VELOCITY it is the rotor speed
target; in OPEN_LOOP it is the rotating field speed. `DMODE` sets direction.
`AT+SPD?` always returns **measured rotor speed**, not the request.

| | |
|---|---|
| **Set** | `AT+SPD=<val>*<CRC>\r\n` |
| **Query** | `AT+SPD?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+SPD:123.45\r\n` |
| **Set range** | 0.0 to 200.0 RPS on the wire; the controller clamps to its configured `max_speed_rps`. |
| **Unit** | Revolutions per second |

In VELOCITY mode, `SPD=0` continues regulating while the rotor slows down. Once
both the PLL speed and filtered speed feedback settle within 0.1 Hall steps/s
(0.000833 RPS for a 40-pole motor), the speed PI integral and target-speed filters
are cleared and the current demand is exactly zero. This releases residual
torque after stopping, including any unfinished setpoint ramp. Nonzero speed
commands retain normal PI control, even at standstill.

**Examples:**

```
> AT+SPD=50.5*1ED8\r\n     (CRC of "AT+SPD=50.5")
< OK\r\n

> AT+SPD?*B0A0\r\n         (CRC of "AT+SPD?")
< +SPD:48.32\r\n
```

### Current Control (AT+CUR)

Set the torque-current demand in TORQUE or OPEN_LOOP, or read the active current
target. In VELOCITY the next active speed PI step overwrites a manual current
setting. In NEUTRAL the drive clears the target and ignores new current and
speed demands, even though well-formed writes in range receive `OK`.

| | |
|---|---|
| **Set** | `AT+CUR=<val>*<CRC>\r\n` |
| **Query** | `AT+CUR?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+CUR:2.50\r\n` |
| **Range** | -6.0 to 6.0 A |

The AT setter accepts −6 to +6 A; the controller also clamps to its configured
motor/board limits. The velocity PI can use the configured limits reported by
`AT+MAXVALS?`, independently of the AT setter range. Current is signed relative
to `AT+DMODE`. FOC can request braking torque with negative current; the
six-step PI can reduce duty to zero but cannot apply negative duty.

**Examples:**

```
> AT+CUR=3.5*6CA6\r\n
< OK\r\n

> AT+CUR?*E922\r\n
< +CUR:3.50\r\n
```

### Acceleration Limit (AT+ACC)

Set or read the slew rate limit applied to the **speed setpoint**, in RPS per
second.

| | |
|---|---|
| **Set** | `AT+ACC=<val>*<CRC>\r\n` |
| **Query** | `AT+ACC?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+ACC:4.00\r\n` |
| **Range** | 0.0 to 1000.0 RPS/s |
| **Startup setting** | `BLDC_MAX_ACCELERATION` from `libecu/include/motor_config.hpp`; the selected protocol may change it. Query `AT+ACC?` for the live value. |

This limits how fast the *command* is allowed to move, not the torque: it is
the second stage of the target-speed filter (LPF → slew rate limiter, see
`applyAccelerationLimit()`), so it only applies in `MODE=1` (VELOCITY). In
OPEN_LOOP, `SPD` sets the rotating field's speed directly; in TORQUE, the
speed loop does not run.

`0` disables the setpoint ramp, so `AT+SPD` can step the reference. The speed PI
can then request current up to the drive or braking rail on the first sample.
Measured wheel acceleration has no separate setting.

Changing the rate does not move the ramp that is already in progress; the new
rate applies from the next control tick.

> **The ramp is not visible on `+TM:tgt_speed`.** That field reports the raw
> setpoint, which steps the moment `AT+SPD` lands; the limiter output is
> internal. Watch `cur_speed` for wheel response. In an earlier MOTOR_1 bench test stepping to 5 RPS:
> `ACC=1` gives 0.99 RPS/s, `ACC=4` gives 4.7 RPS/s, `ACC=0` gives 25.6 RPS/s.

Note that `AT+SPD` is ignored entirely while `DMODE=2` (NEUTRAL), so a ramp
cannot be observed there.

**Keep it below what the PLL can follow.** The steady-state PLL tracking error
during a ramp is `e_ss = α / ki` steps (α in steps/s², 1 RPS/s = 120 steps/s² on
a 40-pole motor), and past 1.5 steps the commutation lead goes negative and the
motor runs away. See the tuning notes under [AT+PLLID](#pll-gain-tuning-atpllid).

**Examples:**

```
> AT+ACC=2.5*021B\r\n
< OK\r\n

> AT+ACC?*C5CB\r\n
< +ACC:2.50\r\n
```

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
| 0 | OPEN_LOOP | `SPD` sets the rotating field's mechanical RPS; `CUR` sets its current. Hall telemetry still measures the rotor. |
| 1 | VELOCITY | The speed PI compares `SPD` with measured rotor speed and supplies the current demand. |
| 2 | TORQUE | `CUR` sets current directly; the field follows rotor feedback. |

**Examples:**

```
> AT+MODE=1*1A11\r\n
< OK\r\n

> AT+MODE?*234E\r\n
< +MODE:1\r\n
```

### Drive Algorithm (AT+ALGO)

Select how the inverter regulates the current demand. Both algorithms run at
the board's current-loop rate. Voltage and phase duties are regulator outputs;
the host does not set them.

| | |
|---|---|
| **Set** | `AT+ALGO=<val>*<CRC>\r\n` |
| **Query** | `AT+ALGO?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+ALGO:0\r\n` |

| Value | Algorithm | Description |
|-------|-----------|-------------|
| 0 | SIX_STEP | The current PI (`AT+CPID`) uses the active DOWN-phase shunt and sets duty for the commutated phases. |
| 1 | FOC | The d/q current PIs (`AT+FPID`) convert the errors in `Id` and `Iq` to voltage requests for space-vector PWM. |

> **Switch at standstill or in NEUTRAL.** Changing algorithm is a step change
> in the applied voltage — the outgoing algorithm's integrators are cleared and
> the incoming one reconfigures the bridge.

### FOC Current PI Parameters (AT+FPID)

Set or read the FOC d/q current regulator gains. Both axes share one set of
gains, which is correct for a motor with Ld == Lq.

| | |
|---|---|
| **Set** | `AT+FPID=<kp>,<ki>*<CRC>\r\n` |
| **Query** | `AT+FPID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+FPID:0.5498,516.7920\r\nOK\r\n` (MOTOR_1 example) |

> **These are not `AT+CPID` numbers.** The FOC regulators output **volts**
> (kp in V/A, ki in V/(A·s)) and see one phase; the six-step regulator outputs
> **duty** and sees two phases in series. For MOTOR_1 at 31.3 V the six-step
> gains are ≈0.141 / 132 (500-Hz design input). The [motor calibration](../libecu/include/motor_config.hpp)
> sets FOC to 0.549775 / 516.792; the two pairs are not interchangeable.

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
| **Query response** | `+FANG:-60.00\r\n` |

Range -360.0 to 360.0.

The MOTOR_1 offset of -60° is derived from the normal table: row *k* of the six-step
commutation table places the stator field at (k·60 − 30)°, and the PLL applies
step round(angle + 1) in FORWARD, so the field sits at (angle·60 + 30)°. At the
six-step torque optimum the field leads the rotor by 90°, giving
θ_d = (angle − 1)·60°.

An inverse commutation table needs its own alignment: the
[JK42BLS01 profile](JK42BLS01_FOC.md) uses +120°. A misaligned field can
produce little torque or accelerate the rotor the wrong way.

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

### Speed PI Parameters (AT+SPID)

Set or read the speed (outer-loop) PI gains. Its output is current in amperes
for either electrical algorithm.
On `MOTOR_1`, the query returns the gains selected from the latest measured
speed. An accepted write, even of that same pair, selects constant gains for
the rest of the session. See [MOTOR_1 defaults](software.md#motor_1-defaults).
Repeating a setting or changing only `ki` retains the integral and the previous
error. With velocity control active, a `kp` change compensates the integral to
preserve the last command at the same error. Normal integration and response to
new speed errors continue on the next speed tick.

| | |
|---|---|
| **Set** | `AT+SPID=<kp>,<ki>*<CRC>\r\n` |
| **Query** | `AT+SPID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+SPID:0.010,0.100\r\nOK\r\n` |

Exactly two finite coefficients are required. A third coefficient, including zero,
returns `ERROR`. A changed setting also returns `ERROR` without applying it if
the active loop is saturated, the compensation would exceed an integral limit
(including the configured speed-dependent ceiling), or the operating point changes during
preparation. The caller can retry later. Output limits are preserved. Gains are
continuous-time values; the PI caches the discrete coefficients at configuration
time for the board's `SPEED_PI_HZ` (1 kHz by default on STM32). Measured speed
and the setpoint ramp are updated separately at 100 Hz.

**Examples:**

```
> AT+SPID=0.05,1.0*EB29\r\n
< OK\r\n

> AT+SPID?*5695\r\n
< +SPID:0.050,1.000\r\n
< OK\r\n
```

### Current PI Parameters (AT+CPID)

Set or read the six-step current (inner-loop) PI controller gains. Changes take
effect immediately and reset the integrator. Exactly two finite coefficients are
required; a third coefficient returns `ERROR`. The sampling frequency is the
compile-time current-loop rate (20 kHz on STM32G431).

| | |
|---|---|
| **Set** | `AT+CPID=<kp>,<ki>*<CRC>\r\n` |
| **Query** | `AT+CPID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+CPID:0.010,0.100\r\nOK\r\n` |

**Examples:**

```
> AT+CPID=0.1,2.0*F47B\r\n
< OK\r\n

> AT+CPID?*52CF\r\n
< +CPID:0.100,2.000\r\n
< OK\r\n
```

### Firmware Version (AT+VER)

Read the version string selected by the board and numeric profile in
[G431](../STM32G431/Core/Inc/ecu_config.hpp) or [F410](../STM32F410/Core/Inc/ecu_config.hpp).

| | |
|---|---|
| **Query** | `AT+VER?*<CRC>\r\n` |
| **Response** | `+VER:1.2.0-control-float\r\n` (G431 float example) |

### Motor Status (AT+STATUS)

Read a snapshot of the motor's current state.

| | |
|---|---|
| **Query** | `AT+STATUS?*<CRC>\r\n` |
| **Response** | `+STATUS:<ctrl_mode>,1,<speed>,<target_cur>,<duty>,<bus_volt>,<algo>,<id>,<iq>,<angle_e>\r\n` |

Response fields in order:

| Field | Type | Description |
|-------|------|-------------|
| ctrl_mode | int | Control mode (0=OPEN_LOOP, 1=VELOCITY, 2=TORQUE) |
| 1 | int | Legacy field 2: always the literal `1` for wire compatibility. |
| speed | float | Filtered mechanical rotor speed in RPS, signed relative to the drive direction; not the speed command. |
| target_cur | float | Target current in Amperes |
| duty | float | Six-step applied duty, or FOC modulation depth when enabled (see below). |
| bus_volt | float | Bus voltage in Volts |
| algo | int | Drive algorithm (0=SIX_STEP, 1=FOC) |
| id | float | Measured d-axis current in Amperes (0 in six-step) |
| iq | float | Measured q-axis current in Amperes (0 in six-step) |
| angle_e | float | Electrical angle used by the algorithm, degrees |

`+STATUS`, `+TM` and `+OSC` report the same scalar duty: the applied duty for
six-step, or the modulation depth derived from `Ud`, `Uq` and the DC bus on
the 100-Hz tick for FOC. It reads zero in NEUTRAL or if the controller is
not running. With `LIBECU_FOC_DUTY_TELEMETRY=0`, only the FOC scalar is suppressed; FOC still
updates all three phase duties each current tick. Field positions have not
changed.

**Example:**

```
> AT+STATUS?*A8D4\r\n
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
> AT+MAXVALS?*2C81\r\n
< +MAXVALS:200.00,-6.00,6.00,36.00,0.95,100.00\r\n
```

## Telemetry (AT+TM)

Enable or disable continuous telemetry streaming. The foreground sends motor
state on the 100-Hz service flag. [PLL telemetry](#pll-telemetry-atpll) uses
the same cadence.

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
| duty | float | Six-step duty or reported FOC modulation depth |
| tgt_cur | float | Target current in Amperes |
| meas_cur | float | Measured current in amperes. Builds with `LIBECU_RAW_CURRENT_TELEMETRY` report the latest sample; other builds apply the configured low-pass filter. Use `AT+OSC` for consecutive unfiltered current samples. |
| bus_volt | float | Bus voltage in Volts |
| pll_angle | float | PLL rotor angle in steps (0.0-6.0, one electrical period) |
| temp | float | NTC temperature in degrees Celsius (G431 PB14, F410 PC2). `-273.0` means an invalid reading. Above `max_temperature_c` (see [AT+MAXVALS](#maximum-values-atmaxvals)) the drive trips to NEUTRAL. |

`temp` was appended after the other nine fields, so a parser that splits on `;` and
reads the first nine still works. Prefer `len(fields) >= 9` over `== 9`.

**Example:**

```
+TM:3;5;23.45;22.10;0.35;1.50;1.45;24.56;3.2;41.7
+TM:3;5;23.45;22.15;0.35;1.50;1.46;24.55;3.4;41.7
+TM:4;5;23.45;22.20;0.35;1.50;1.47;24.56;3.6;41.8
```

SysTick signals the 100-Hz service tick; the foreground emits +TM and +PLL. Each call to `sendTelemetry()` or `sendPllTelemetry()` produces one line.

## Oscilloscope (AT+OSC)

Start or stop oscilloscope capture from the current-loop interrupt.

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

The oscilloscope alternates between capture and transmission using one buffer:

| Parameter | Value |
|-----------|-------|
| Buffer size | `LIBECU_OSC_BUFFER_SIZE` (default 1024 samples) |
| Capture rate | One sample per completed current tick while accumulating (20 kHz on both STM32 ports) |
| Burst duration | 1024 / 20000 = 51.2 ms at 20 kHz |
| End of capture | Buffer full |

Foreground code transmits the completed burst without overwriting it. After
the final sample and end marker, capture resumes; samples are contiguous
within a burst, with gaps between bursts.

### Captured Data

Each sample captures seven values, packed into 12 bytes:

| Field | Type | Description |
|-------|------|-------------|
| target_current_ma | int16 | Target current setpoint, milliamps |
| measured_current_ma | int16 | Raw measured current, milliamps |
| i_d_ma | int16 | d-axis current, milliamps (0 in six-step) |
| i_q_ma | int16 | q-axis current, milliamps (0 in six-step) |
| angle_e | uint16 | Electrical angle, full scale = one electrical revolution |
| duty_cycle | uint8 | Reported duty/modulation × 100 (0–100); the FOC scalar updates on the slow tick. |
| position | uint8 | Rotor position from Hall sensors |

The default 1024-sample buffer occupies 12 KiB. Each current is encoded in
milliamps and clipped to the signed 16-bit range (−32.768 to +32.767 A).
This packing limit applies to the capture, not to the current regulator.

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
+OSC:0,1500,2000,35,3,-12,1500,212
+OSC:1,1485,2000,35,3,-15,1485,212
+OSC:2,1520,2000,35,3,-9,1520,213
+OSC:3,1490,2000,35,3,-10,1490,213
...
+OSC:1023,1510,2000,35,5,-8,1510,345
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
| hall_state_raw | int | Last valid decoded Hall sector [0..5] supplied to the PLL (not the three-bit GPIO pattern). |
| is_sync | int | PLL synchronized with Hall sensor flag (1 = tracking, 0 = snapping angle to Hall) |

**Example:**

```
+PLL:291.158;218.631;3.142;4;1
+PLL:295.430;220.104;0.571;5;1
+PLL:288.712;219.502;5.028;2;0
```

`angle` and `+TM:pll_angle` describe the same physical rotor, but the two
reports take separate snapshots; a current tick can occur between them.
Within one `+TM` line, `meas_pos - pll_angle` wrapped to [−3, +3] gives
sector-to-estimate error in Hall steps. See [PLL lock detection](software.md#synchronization-and-reseeding)
for the filtered error threshold.

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
| edge_accum | float | Leaky edge count; divide by the decay time (0.5 s / `decay_time_us = 500000`) for edges/sec |
| invalid_events | uint32 | Illegal-code readings since reset |
| edges | uint32 | Hall transitions since reset |
| last_pos | uint8 | Last valid decoded position (0-5) |
| standing_us | integer | Microseconds derived from elapsed PWM ticks. **This is the detector**; faults at `invalid_persist_time_us` (300 us), rounded up to a whole sample |

**Example:**

```
+HSTATUS:0;0.00;0.00;301.50;0;18422;3;0
+HSTATUS:0;0.00;0.00;298.12;41;18512;5;50
+HSTATUS:1;0.00;0.00;44.80;37;18544;2;300
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
- **`edge_accum / 0.5` estimates edges per second.** At steady speed compare
  it with `abs(+TM:cur_speed) × 6 × pole_pairs`; `cur_speed` alone is in
  mechanical RPS. A large discrepancy can indicate missing or spurious edges.

A latched Hall fault parks the bridge in Hall-controlled modes. Re-enabling
those modes while the fault remains latched causes the next current tick to
park the bridge again. The monitor also observes Hall in open loop, where the
generated field does not use that feedback.

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

**Motor defaults:** JK42BLS01 uses `kp=100`, `ki=5000` (ω_n=70.7 rad/s,
ζ=0.707); MOTOR_1 uses `kp=50`, `ki=1250`.

Exactly two finite coefficients are required, as for the other PI controllers.

**Examples:**

```
> AT+PLLID=100,5000*728F\r\n
< OK\r\n

> AT+PLLID?*03A9\r\n
< +PLLID:100.000,5000.000\r\n
< OK\r\n
```

**Tuning notes:**
- `ki` determines the PLL bandwidth: ω_n = √ki. Higher ki = faster tracking but potential instability if too high.
- `kp` determines damping: ζ = kp / (2·√ki). Target ζ ≈ 0.7–1.0 for well-damped response.
- The steady-state tracking error during acceleration is: `e_ss = α / ki` (in steps), where α is the rotor acceleration in steps/s².
- The commutation reversal threshold is 1.5 steps (90° field offset). If the PLL tracking error exceeds 1.5 steps, the effective stator field lead becomes negative, causing torque reversal and motor runaway.
- For example, 40 mechanical RPS/s on a 40-pole motor is 4800 electrical steps/s²; with `ki=5000`, the steady-state estimate is 0.96 steps.

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
| `AT+ACC=<val>` | Set | 0.0 .. 1000.0 RPS/s | `OK` |
| `AT+ACC?` | Query | -- | `+ACC:4.00` |
| `AT+MODE=<val>` | Set | 0, 1, 2 | `OK` |
| `AT+MODE?` | Query | -- | `+MODE:1` |
| `AT+DMODE=<val>` | Set | 0, 1, 2 | `OK` |
| `AT+DMODE?` | Query | -- | `+DMODE:0` |
| `AT+SPID=<kp>,<ki>` | Set | two finite floats | `OK` |
| `AT+SPID?` | Query | -- | `+SPID:0.050,1.000` |
| `AT+CPID=<kp>,<ki>` | Set | two finite floats | `OK` |
| `AT+CPID?` | Query | -- | `+CPID:0.050,1.000` |
| `AT+ALGO=<val>` | Set | 0, 1 | `OK` |
| `AT+ALGO?` | Query | -- | `+ALGO:0` |
| `AT+FPID=<kp>,<ki>` | Set | two finite gains representable in the active profile | `OK` |
| `AT+FPID?` | Query | -- | `+FPID:0.5498,516.7920` |
| `AT+FANG=<deg>` | Set | -360.0 .. 360.0 | `OK` |
| `AT+FANG?` | Query | -- | `+FANG:-60.00` |
| `AT+PLLID=<kp>,<ki>` | Set | two finite gains representable in the active profile | `OK` |
| `AT+PLLID?` | Query | -- | `+PLLID:100.000,5000.000` |
| `AT+VER?` | Query | -- | `+VER:1.2.0-control-float` |
| `AT+STATUS?` | Query | -- | `+STATUS:1,1,23.45,...` |
| `AT+MAXVALS?` | Query | -- | `+MAXVALS:200.00,-6.00,6.00,36.00,0.95,100.00` |
| `AT+TM=<0|1>` | Set | 0, 1 | `OK` |
| `AT+PLL=<0|1>` | Set/Query | 0, 1 | `OK` / `+PLL:1` |
| `AT+HSTATUS=<0|1>` | Set/Query | 0, 1 | `OK` / `+HSTATUS:1` |
| `AT+HCLEAR` | Action | -- | `OK` |
| `AT+OSC=<0|1>` | Set/Query | 0, 1 | `OK` / `+OSC:1` |
