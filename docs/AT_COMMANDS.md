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
4. You should receive `+VER:1.0.0\r\nOK\r\n`

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

The implementation uses a 256-entry lookup table for speed. The full source is in `libecu/src/platform/crc16.cpp`.

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
| **Query response** | `+SPD:123.45\r\nOK\r\n` |
| **Range** | 0.0 to 200.0 RPS |
| **Unit** | Revolutions per second |

**Examples:**

```
> AT+SPD=50.5*A1B2\r\n     (CRC of "AT+SPD=50.5")
< OK\r\n

> AT+SPD?*B3C4\r\n         (CRC of "AT+SPD?")
< +SPD:48.32\r\n
< OK\r\n
```

### Current Control (AT+CUR)

Set target motor current or read the current target value.

| | |
|---|---|
| **Set** | `AT+CUR=<val>*<CRC>\r\n` |
| **Query** | `AT+CUR?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+CUR:2.50\r\nOK\r\n` |
| **Range** | -6.0 to 6.0 A |

Negative current values reverse the current direction, enabling regenerative braking when the motor is spinning forward.

**Examples:**

```
> AT+CUR=3.5*D1E2\r\n
< OK\r\n

> AT+CUR?*F3A4\r\n
< +CUR:3.50\r\n
< OK\r\n
```

### Duty Cycle (AT+DUT)

Set duty cycle in open-loop mode or read current duty cycle.

| | |
|---|---|
| **Set** | `AT+DUT=<val>*<CRC>\r\n` |
| **Query** | `AT+DUT?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+DUT:0.30\r\nOK\r\n` |
| **Range** | 0.0 to 1.0 |

### Control Mode (AT+MODE)

Set or read the mechanical control mode.

| | |
|---|---|
| **Set** | `AT+MODE=<val>*<CRC>\r\n` |
| **Query** | `AT+MODE?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+MODE:1\r\nOK\r\n` |

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
< OK\r\n
```

### Electric Mode (AT+EMODE)

Set or read the electrical control strategy.

| | |
|---|---|
| **Set** | `AT+EMODE=<val>*<CRC>\r\n` |
| **Query** | `AT+EMODE?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+EMODE:1\r\nOK\r\n` |

| Value | Mode | Description |
|-------|------|-------------|
| 0 | VOLTAGE | Direct voltage or duty cycle control |
| 1 | CURRENT | Current control with PI inner loop at 40kHz (PWM frequency) |

### Drive Mode (AT+DMODE)

Set or read the motor rotation direction.

| | |
|---|---|
| **Set** | `AT+DMODE=<val>*<CRC>\r\n` |
| **Query** | `AT+DMODE?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+DMODE:0\r\nOK\r\n` |

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
| **Response** | `+VER:1.0.0\r\nOK\r\n` |

### Motor Status (AT+STATUS)

Read a snapshot of the motor's current state.

| | |
|---|---|
| **Query** | `AT+STATUS?*<CRC>\r\n` |
| **Response** | `+STATUS:<ctrl_mode>,<elec_mode>,<speed>,<target_cur>,<duty>,<bus_volt>\r\nOK\r\n` |

Response fields in order:

| Field | Type | Description |
|-------|------|-------------|
| ctrl_mode | int | Control mode (0=OPEN_LOOP, 1=VELOCITY, 2=TORQUE) |
| elec_mode | int | Electric mode (0=VOLTAGE, 1=CURRENT) |
| speed | float | Current motor speed in RPS |
| target_cur | float | Target current in Amperes |
| duty | float | Current duty cycle (0.0 to 1.0) |
| bus_volt | float | Bus voltage in Volts |

**Example:**

```
> AT+STATUS?*DEAD\r\n
< +STATUS:1,1,23.45,1.50,0.35,24.56\r\n
< OK\r\n
```

### Maximum Values (AT+MAXVALS)

Read the firmware's configured safety limits. Useful for host-side UI to set slider ranges.

| | |
|---|---|
| **Query** | `AT+MAXVALS?*<CRC>\r\n` |
| **Response** | `+MAXVALS:<max_speed>,<min_current>,<max_current>,<max_voltage>,<max_duty>\r\nOK\r\n` |

Response fields in order:

| Field | Type | Description |
|-------|------|-------------|
| max_speed | float | Maximum speed in RPS (200.0) |
| min_current | float | Minimum current in Amperes (-6.0) |
| max_current | float | Maximum current in Amperes (6.0) |
| max_voltage | float | Maximum bus voltage in Volts (36.0) |
| max_duty | float | Maximum duty cycle (0.95) |

**Example:**

```
> AT+MAXVALS?*XXXX\r\n
< +MAXVALS:200.0,-6.0,6.0,36.0,0.95
< OK
```

## Telemetry (AT+TM)

Enable or disable continuous telemetry streaming. When enabled, the controller sends motor state data at 100Hz (every 10ms), synchronized with the speed control loop (SysTick). +TM is sent every other tick (50Hz); +PLL telemetry (see [AT+PLL](#pll-telemetry-at+pll)) is sent every tick (100Hz).

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
+TM:<meas_pos>;<tgt_pos>;<tgt_speed>;<cur_speed>;<duty>;<tgt_cur>;<meas_cur>;<bus_volt>;<pll_angle>
```

| Field | Type | Description |
|-------|------|-------------|
| meas_pos | uint8 | Measured rotor position (Hall sensor, 0-5) |
| tgt_pos | uint8 | Target rotor position (commutation step, 0-5) |
| tgt_speed | float | Target speed in RPS |
| cur_speed | float | Current measured speed in RPS |
| duty | float | Current duty cycle |
| tgt_cur | float | Target current in Amperes |
| meas_cur | float | Measured current in Amperes |
| bus_volt | float | Bus voltage in Volts |
| pll_angle | float | PLL rotor angle in steps (0.0-6.0, one electrical period) |

**Example:**

```
+TM:3;5;23.45;22.10;0.35;1.50;1.45;24.56;3.2
+TM:3;5;23.45;22.15;0.35;1.50;1.46;24.55;3.4
+TM:4;5;23.45;22.20;0.35;1.50;1.47;24.56;3.6
```

The telemetry output is invoked from the 100Hz speed control loop (SysTick). +TM is emitted every other tick (50Hz); +PLL is emitted every tick (100Hz). Each call to `sendTelemetry()` or `sendPllTelemetry()` produces one line.

## Oscilloscope (AT+OSC)

Start or stop the oscilloscope feature, which captures high-speed samples from the PWM ISR at 40kHz (PWM frequency).

| | |
|---|---|
| **Command** | `AT+OSC=<0|1>*<CRC>\r\n` |
| **Query** | `AT+OSC?*<CRC>\r\n` |
| **Response** | `OK\r\n` |
| **Query response** | `+OSC:1\r\nOK\r\n` |

| Value | Effect |
|-------|--------|
| 0 | Stop oscilloscope, stop streaming |
| 1 | Start oscilloscope, begin capturing |

### Buffer Architecture

The oscilloscope uses a **single buffer with two-phase swapping** for safe capture from the PWM ISR:

| Parameter | Value |
|-----------|-------|
| Buffer size | 512 samples |
| Capture rate | 40kHz (from PWM ISR) |
| Burst duration | 512 / 40000 = 12.8ms per burst |
| Swap trigger | Buffer full (write index reaches 512) |

When the write phase fills the buffer (write index reaches 512), the phase swaps under a critical section (interrupts disabled): the accumulated buffer becomes the output buffer, and the output buffer is reset for the next capture cycle.

### Captured Data

Each sample captures four values:

| Field | Type | Description |
|-------|------|-------------|
| duty_cycle | float | Current duty cycle |
| target_current | float | Target current setpoint |
| measured_current | float | Raw measured current from shunt |
| position | uint8 | Rotor position from Hall sensors |

### Output Format

Samples stream out as `\r\n`-terminated lines, one per call to `processOscOutput()`:

```
+OSC:<sample_index>,<meas_cur_x1000>,<tgt_cur_x1000>,<duty_x1000>,<position>\r\n
```

| Field | Type | Description |
|-------|------|-------------|
| sample_index | int | Sequential sample counter starting at 0 |
| meas_cur_x1000 | int32 | `measured_current * 1000`, signed integer |
| tgt_cur_x1000 | int32 | `target_current * 1000`, signed integer |
| duty_x1000 | int32 | `duty_cycle * 1000`, signed integer |
| position | uint8 | Rotor position (Hall sensor, 0-5) |

The end of a burst is signaled by an empty data line:

```
+OSC:\r\n
```

After the end-of-burst marker, oscilloscope continues capturing the next burst (the phase swaps back to Accumulating).

**Example burst:**

```
+OSC:0,1500,2000,350,3
+OSC:1,1485,2000,350,3
+OSC:2,1520,2000,350,3
+OSC:3,1490,2000,350,4
...
+OSC:511,1510,2000,350,5
+OSC:
```

To convert the scaled values back to physical units:

```python
measured_current_A = meas_cur_x1000 / 1000.0
target_current_A   = tgt_cur_x1000 / 1000.0
duty_cycle         = duty_x1000 / 1000.0
```

## PLL Telemetry (AT+PLL)

Enable or disable continuous PLL (Phase-Locked Loop) telemetry streaming. When enabled, the controller emits a `+PLL:` line at 100Hz (every control tick) containing the internal state of the MotorPLL observer.

| | |
|---|---|
| **Command** | `AT+PLL=<0|1>*<CRC>\r\n` |
| **Query** | `AT+PLL?*<CRC>\r\n` |
| **Response** | `OK\r\n` |
| **Query response** | `+PLL:1\r\nOK\r\n` |

| Value | Effect |
|-------|--------|
| 0 | Disable +PLL telemetry |
| 1 | Enable +PLL telemetry (default: enabled) |

### PLL Telemetry Format

Each line is a newline-terminated tuple (uses `\n` only, not `\r\n`):

```
+PLL:<angle_per_second>;<pll_integral>;<time_since_last_hall>;<kp>;<ki>
```

| Field | Type | Description |
|-------|------|-------------|
| angle_per_second | float | PLL-estimated rotor speed in steps/sec (6 steps = 1 electrical revolution) |
| pll_integral | float | PI integrator term in steps/sec (clamped to ±max_electrical_speed) |
| time_since_last_hall | float | Seconds since last Hall sensor edge (resets to 0 on each edge, 5s timeout) |
| kp | float | Current effective proportional gain (after adaptive scaling) |
| ki | float | Current effective integral gain (after adaptive scaling) |

**Example:**

```
+PLL:291.158;218.631;0.0018;55.61;612.15
+PLL:295.430;220.104;0.0009;55.63;612.59
+PLL:288.712;219.502;0.0021;55.59;611.82
```

**Note:** Position and angle fields available in `+TM:` telemetry (`meas_pos`, `tgt_pos`, `pll_angle`) are not duplicated here. To compute the PLL tracking error, use `+TM:` fields: `error = measured_position - pll_angle` (wrapped to [-3, +3] per electrical period). The slip threshold is 3.0 steps.

## PLL Gain Tuning (AT+PLLID)

Set or read the PLL base PI gains. These are the base values used by the adaptive gain schedule. The effective gains at runtime are: `kp = base_kp + 20 * speed_factor`, `ki = base_ki + 400 * speed_factor`, where `speed_factor = |angle_per_second| / max_electrical_speed` (clamped to [0, 1]).

| | |
|---|---|
| **Set** | `AT+PLLID=<kp>,<ki>*<CRC>\r\n` |
| **Query** | `AT+PLLID?*<CRC>\r\n` |
| **Set response** | `OK\r\n` |
| **Query response** | `+PLLID:200.000,10000.000\r\nOK\r\n` |

**Default values:** `kp=200.0`, `ki=10000.0` (giving ω_n=100 rad/s, ζ=1.0).

The `kd` parameter is not used (PLL is PI-only, no derivative term).

**Examples:**

```
> AT+PLLID=200,10000*XXXX\r\n
< OK\r\n

> AT+PLLID?*XXXX\r\n
< +PLLID:200.000,10000.000\r\n
< OK\r\n
```

**Tuning notes:**
- `ki` determines the PLL bandwidth: ω_n = √ki. Higher ki = faster tracking but potential instability if too high.
- `kp` determines damping: ζ = kp / (2·√ki). Target ζ ≈ 1.0 for critically damped response.
- The steady-state tracking error during acceleration is: `e_ss = α / ki` (in steps per 10 electrical periods), where α is the rotor acceleration in steps/s².
- The commutation reversal threshold is 1.5 steps (90° field offset). If the PLL tracking error exceeds 1.5 steps, the effective stator field lead becomes negative, causing torque reversal and motor runaway.
- With ki=10000 and max acceleration 100 RPS/s (4800 steps/s²): e_ss = 0.48 steps — safely below the 1.5-step threshold.

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
| `AT+PLLID=<kp>,<ki>` | Set | any float | `OK` |
| `AT+PLLID?` | Query | -- | `+PLLID:200.000,10000.000` |
| `AT+VER?` | Query | -- | `+VER:1.0.0` |
| `AT+STATUS?` | Query | -- | `+STATUS:1,1,23.45,...` |
| `AT+MAXVALS?` | Query | -- | `+MAXVALS:200.0,-6.0,6.0,36.0,0.95` |
| `AT+TM=<0|1>` | Set | 0, 1 | `OK` |
| `AT+PLL=<0|1>` | Set/Query | 0, 1 | `OK` / `+PLL:1` |
| `AT+OSC=<0|1>` | Set/Query | 0, 1 | `OK` / `+OSC:1` |
