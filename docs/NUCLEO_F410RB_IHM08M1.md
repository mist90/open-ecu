# NUCLEO-F410RB + X-NUCLEO-IHM08M1

The `STM32F410` port targets this exact board pair: a NUCLEO-F410RB with an
X-NUCLEO-IHM08M1 power shield stacked through **both 38-pin ST morpho
connectors CN7 and CN10**, plus **three extra Hall-to-TIM5 jumper wires**
described below. The Arduino headers alone are not enough. The motor is
selected by the `MOTOR_1` define of this port's CMake: 40 poles, the existing
motor parameters and FOC at startup.

Place the Nucleo below the shield, with both boards component-side up.
The shield's wide power section and motor terminals point away from the
ST-LINK USB connector; the Nucleo's B1 and B2 buttons remain accessible.

The clock source is the mandatory quartz-derived 8 MHz HSE from ST-LINK.
Hall acquisition uses TIM5 with a 5.12 µs hardware input filter.

## 1. Reference documents

* **UM1724 Rev 17** — STM32 Nucleo-64 boards user manual: power (§7.5),
  UART (§7.10), solder bridges (Table 10), Figure 25 and Tables 20/33 which
  are specific to the **F410RB**.
  <https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf>
* **MB1136 C-04** board schematic (Nucleo-64), sheets 2–5: power, MCU,
  ST-LINK and connectors.
  <https://www.st.com/resource/en/schematic_pack/mb1136-default-c04_schematic.pdf>
* **UM1996 Rev 4** — IHM08M1 getting started: assembly (§2.2), jumper
  Table 1, current sensing (§4.1.2), BOM R19 and electrical Table 7.
  <https://www.st.com/resource/en/user_manual/um1996-getting-started-with-xnucleoihm08m1-lowvoltage-bldc-motor-driver-expansion-board-based-on-stl220n6f7-for-stm32-nucleo-stmicroelectronics.pdf>
* **X-NUCLEO-IHM08M1 schematic, version 3**, sheets 1–3: power stage, analog
  conditioning, pin routing.
  <https://www.st.com/resource/en/schematic_pack/x-nucleo-ihm08m1_schematic.pdf>
* **STM32F410x8/B DS11144 Rev 8** datasheet, Tables 9/10: pins and alternate
  functions. <https://www.st.com/resource/en/datasheet/stm32f410rb.pdf>
* **RM0401 Rev 4** reference manual: ADC1, TIM1, TIM5, DMA; in particular
  Table 60 (complementary output control with MOE/CCxE/CCxNE/OSSI/OSSR).
  <https://www.st.com/resource/en/reference_manual/rm0401-stm32f410-advanced-armbased-32bit-mcus-stmicroelectronics.pdf>
* **L6398** gate driver datasheet, Tables 1/2 and dead-time data: **HIN is
  active HIGH, LIN is active LOW**, built-in dead time typically 320 ns.
  <https://www.st.com/resource/en/datasheet/l6398.pdf>

Notation: `CN10.23` below means pin 23 of connector CN10, not an MCU package
pin. Connector pin numbers and Rxx/Jxx designators follow the schematics.

## 2. Jumpers and power supply

### NUCLEO-F410RB: USB-powered logic configuration

| Item | Position | Purpose |
|---|---|---|
| CN2, both jumpers | fitted | ST-LINK programs the on-board F410 |
| JP1 | open | default USB power management |
| JP5 | **1–2, U5V** | Nucleo and shield logic powered from USB |
| JP6 (IDD) | closed | MCU supply |
| BOOT0 / CN7.5–7 | no jumper | boot from Flash; BOOT0 pulled down |
| SB13, SB14 | **ON** | USART2 routed to the ST-LINK virtual COM port |
| SB61, SB62, SB63 | **OFF** | PA3 not tied to AVDD; UART isolated from D0/D1 |
| SB51, SB56 | **ON** | PC0/PC1 reach CN7.38/CN7.36 for the W/V currents |
| SB46, SB52 | **OFF** | PB9/PB8 I²C not substituted on those analog inputs |
| SB17 | ON | user button B1 connected to PC13 |
| SB16, SB50 | **ON** | quartz-derived 8 MHz MCO from ST-LINK to PH0/HSE of the F410 |
| SB54 / SB55 | ON / OFF | factory MCO routing; R35/R37 and X3 are not fitted |

SBxx are solder bridges, normally factory-set as listed. SB15 connects PB3
to the **input** side of the ST-LINK SWO — it is not a second output on any
Hall line; this port leaves PB3 in high-impedance analog mode and never
enables SWO/JTAG trace. SWD on PA13/PA14 keeps working. SB21/LD2 on PA5 is
acceptable: BEMF3 is not measured here and PA5 is never driven as an LED.

Clock tree: **ST-LINK crystal 8 MHz → MCO → SB16/SB50 → PH0 with HSE in
bypass → PLL /8 × 200 / 2 = 100 MHz**. The crystal reference defines the
speed and PWM timebases. If HSE is absent, initialization fails with the
bridge disabled; there is no operating fallback to HSI. Clock security
(CSS) raises an NMI on HSE loss, which also parks the bridge.

### Mandatory extra Hall wiring

The sensors stay connected to the stock J3 header of the IHM08M1. Downstream
of the shield's RC filters, pull-ups (JP3) and clamping diodes, three jumper
wires route the same signals to TIM5:

| Signal | From | To |
|---|---|---|
| H1 | **CN7.17 — PA15** | **CN10.16 — PB12 / AF2 TIM5_CH1** |
| H2 | **CN10.31 — PB3** | **CN7.1 — PC10 / AF2 TIM5_CH2** |
| H3 | **CN10.25 — PB10** | **CN7.2 — PC11 / AF2 TIM5_CH3** |

The reason for these wires: the Hall lines run in the same cable as the
motor phases and pick up strong switching interference. On its native pins
PA15/PB3/PB10 the F410 has no timer with a common Hall-sensor interface,
and a plain EXTI input has no input filter at all. TIM5 provides the XOR of
three inputs with a digital filter (see the Hall section below). The
original PA15/PB3/PB10 pads are left Hi-Z, the shield's R79/R81/R84 stay
fitted, and keeping J3 in the path preserves the shield's whole analog
front end. Wire the jumpers with USB and J1 power removed.

### X-NUCLEO-IHM08M1: three-shunt bipolar sensing

The settings are the same for FOC and for `AT+ALGO=0` in this firmware:

| Item | Position / action |
|---|---|
| JP1 | **close** — V-channel amplifier bias for bipolar current |
| JP2 | **close** — equal gain on all three channels |
| JP3 | **close** — Hall pull-ups to 3.3 V |
| J5 | **3Sh**, SENS1 tied to Vshunt_1 |
| J6 | **3Sh**, SENS3 tied to Vshunt_3 |
| J9 | **open** — motor power is not fed onto the Nucleo VIN input |
| C3, C5, C7 | **remove**, as UM1996 requires for FOC |
| R170 | fitted, stock 0 Ω: the shield's 5 V output feeds the Nucleo E5V |
| J7 | left free — debug header |

**J5/J6 on a production board may be solder bridges** rather than removable
jumpers: move the connection from `1Sh` to `3Sh`, do not bridge both sides.
Reconfigure with USB and J1 power removed. Do not verify the setting with a
continuity-to-ground test: through the 10 mΩ shunts both sides are nearly
shorted.

The stock build must have R47/R48/R50 (currents), R51 (Vbus), R54
(temperature), R56/R58/R64/R67/R70/R72 (PWM), R78 (BKIN) and R79/R81/R84
(Hall) fitted. Do not fit R76: it would tie the PA4 potentiometer line to
CURRENT REF.

### Power paths

* **Nucleo USB Mini-B CN1 → U5V → T2 → JP5.1–2 → +5V → U4 3.3V → JP6 → MCU**.
  +3.3V powers the shield's analog section through CN7.16/CN6.4.
* **Nucleo +5V → CN7.18 → shield `5V` net → J3.4**, the Hall sensor supply.
* **J1.1 = V+/VIN+, J1.2 = V−/GND** — the separate motor power input. The
  board is rated 10–48 V; this port allows a DC link up to **42 V**, with
  overvoltage shutdown above that limit. The nominal link for PI tuning is 31.3 V.
* **J1 → U4 ST1S14 → +15V → L6398**, then **U19 LD1117-5 → `+5V` → R170 →
  CN7.6 E5V**. On the shield schematic `5V` and `+5V` are different nets!
* With JP5=U5V the E5V shield output is not tied to the USB +5V, so R170
  stays fitted. **Keep J9 open with MOTOR_1**: closed, it would put ~31 V on
  a Nucleo VIN input rated for 7–12 V.
* The grounds of both boards join through the morpho connectors. Motor
  current returns through J1.2.

To run without USB logic power, set JP5 to **2–3, E5V** and power J1 — the
stock UM1996 configuration. USB is then still needed for the VCP/flashing.

## 3. Signal paths

### PWM and motor

All six timer outputs are **AF1 TIM1**, 20 kHz, center-aligned.

| Phase | MCU / function | Morpho → shield resistor → driver → MOSFET |
|---|---|---|
| U high | PA8 / CH1 | CN10.23 → R56 → U22.HIN (2) → HVG → Q11 |
| U low | PA7 / CH1N | CN10.15 → R58 → U22.LIN (1) → LVG → Q12 |
| V high | PA9 / CH2 | CN10.21 → R64 → U20.HIN (2) → HVG → Q7 |
| V low | PB0 / CH2N | CN7.34 → R67 → U20.LIN (1) → LVG → Q8 |
| W high | PA10 / CH3 | CN10.33 → R70 → U21.HIN (2) → HVG → Q9 |
| W low | PB1 / CH3N | CN10.24 → R72 → U21.LIN (1) → LVG → Q10 |

**On the schematic the J16 pin order is reversed relative to the phase
names:**

* U = OUT1 = midpoint of Q11/Q12 = **J16.3**.
* V = OUT2 = midpoint of Q7/Q8 = **J16.2**.
* W = OUT3 = midpoint of Q9/Q10 = **J16.1**.

Move the U/V/W motor wires and Hall1/2/3 in the same logical order as on
the G431; the board schematics cannot tell you the wire colors of a given
motor cable.

CHxN polarity is LOW; OISx=0, OISxN=1. For the L6398:

| State | HIN | LIN | Result |
|---|---|---|---|
| OFF / NEUTRAL | 0 | 1 | both switches off |
| high side active | 1 | 1 | high-side switch on |
| DOWN | 0 | 0 | low-side switch on |
| dead time | 0 | 1 | both off |

For OFF the timer keeps CCxE=1, clears CCxNE, forces OCxREF inactive and
sets OSSR=1: both line levels are then driven explicitly. MOE=0 with OSSI=1
gives the same safe levels. Copying the G431 polarity, or disabling both
CCxE/CCxNE, is not valid for this driver. The programmed TIM1 dead time is
1 µs (100 ticks of 10 ns).

### Phase currents

| Phase | Low-side and analog signal path | MCU |
|---|---|---|
| U | Q12 → SENS1 → J5=3Sh → R43 10 mΩ → GND; Vshunt_1 → R2/R1 → U10A → R3 → Curr_fdbk1 → R47 → CN7.28 | PA0 / ADC1_IN0, injected rank 1 |
| V | Q8 → SENS2/Vshunt_2 → R44 10 mΩ → GND; R7/R6 → U10B → R8 → Curr_fdbk2 → R48 → CN7.36 | PC1 / ADC1_IN11, rank 2 |
| W | Q10 → SENS3 → J6=3Sh → R45 10 mΩ → GND; Vshunt_3 → R13/R12 → U10C → R14 → Curr_fdbk3 → R50 → CN7.38 | PC0 / ADC1_IN10, rank 3 |

With JP1/JP2 closed:

```
G = (1 + 4700/1000) × 6800/(6800+680) = 57/11 ≈ 5.181818
Vzero = 3.3 × 680/(6800+680) × 5.7 ≈ 1.710 V ≈ 2122 counts
I = (raw - zero) × 3.3 / (4095 × 0.010 × G)
```

These formulas use single-conversion counts. With the default software x2,
the HAL publishes sums of two (`0..8190`), including `2*raw` for a single-scan
fallback. Calibration, physical current/Vbus conversion and software
protection account for that factor automatically. NTC and pot remain `0..4095`.

At startup the firmware averages the real zero of every phase with the
bridge off. A zero far from mid-scale is rejected, which catches an
unpowered amplifier, a wrong JP1 position or a broken analog connection. On
failure the bridge stays disabled until reset, while the AT console keeps
working for diagnostics. Calibrate with the motor stationary. Positive
current is the direction out of the phase through the low-side switch into
the shunt; FOC converts that sign into its own phase-current convention.

### Bus voltage, temperature and manual inputs

| Signal | Path | MCU / use |
|---|---|---|
| Vbus | J1.1 → R17 169 kΩ → VBUS_SENS/VBUS; R18 9.31 kΩ to GND, C14 4.7 nF → R51 → CN7.30 | PA1 / ADC1_IN1, injected rank 4 |
| Temperature | 3.3V → R19 NTC 10 kΩ → temperature feedback → R20 4.7 kΩ → GND; C10 10 nF → R54 → CN7.35 | PC2 / ADC1_IN12, regular rank 1 |
| Potentiometer | R42 wiper / SPEED → R181 → CN7.32 | PA4 / ADC1_IN4, regular rank 2 |
| Button B1 | VDD → R30 4.7 kΩ → SB17/PC13; B1 shorts to GND | PC13, active LOW |

Vbus = raw × 3.3 / 4095 × (169000+9310)/9310, ratio ≈ 19.153. The BOM NTC
is a TDK NTCG103JF103F: R25 = 10 kΩ, B25/85 = 3435 K; the pin voltage rises
with temperature, about 1.055 V at 25 °C. R42 is missing from the published
schematic sheet but present on the board and in the BOM; the SPEED net and
R181 are shown explicitly. It is a known schematic omission, not an
alternative pinout.

By default the AT console controls the drive. The `--throttle` build option
enables the potentiometer as the speed demand and B1 as a brake (zero speed
command with the bridge still enabled). That option can spin the motor at
power-up if the pot is open — use the stock AT build for first bring-up.

### Hall

| J3 | Shield path | Morpho / MCU |
|---|---|---|
| 1: A+/H1 | R30 1.8 kΩ → A+/H1 → R79 | CN7.17 → jumper → CN10.16 → PB12 / TIM5_CH1, bit 0 |
| 2: B+/H2 | R31 1.8 kΩ → B+/H2 → R81 | CN10.31 → jumper → CN7.1 → PC10 / TIM5_CH2, bit 1 |
| 3: Z+/H3 | R32 1.8 kΩ → Z+/H3 → R84 | CN10.25 → jumper → CN7.2 → PC11 / TIM5_CH3, bit 2 |
| 4 | 5V | Hall supply from the Nucleo +5V |
| 5 | GND | common return |

JP3 enables the R27/R28/R29 10 kΩ pull-ups to 3.3 V; D2/D3/D4 clamp the
positive level. Most sensors here are open-collector — check the pinout of
your particular cable before connecting, there is no standard color order.
The sector decode is shared with the G431: 001→0, 011→1, 010→2, 110→3,
100→4, 101→5; 000/111 are invalid.

Because the F410 has no timer Hall interface on PA15/PB3/PB10, the three
jumpers above feed TIM5. It runs with the XOR of the three inputs enabled
(`TI1S=1`), captures both filtered edges through TI1F_ED/TRC on channel 1
and raises the TIM5 interrupt; the controller then re-reads the sector
level in the PWM ISR, as on the G431.

Input filter: **CKD=DIV2, IC1F=0xF** — eight samples at fDTS/32 =
1.5625 MHz, a nominal qualification window of **5.12 µs**. The setting is
deliberately conservative: a shorter window admits PWM-correlated pulses
picked up on the shared motor cable, while genuine Hall edges on this motor
are orders of magnitude slower. The filter clock division does not affect
the timer counter (see below).

### Hardware overcurrent protection

```
Vshunt_1/2/3 → R171/R174/R176 + RC → U24/U25/U26 LMV331
3.3V → R179 33k / R180 3.3k → Vref ≈ 0.30 V
open-collector outputs → BKIN → R78 → CN10.13 → PA6 / AF1 TIM1_BKIN
```

Trip level is about **30 A** of positive current. BKIN is active LOW and
clears MOE asynchronously, without any ISR. AOE is off; the break handler
latches the fault until reset. In addition the port disables the bridge on
|Iphase| > 25 A, Vbus < 10 V while enabled, ADC/DMA failure or stall, and
current-ISR overrun. The shared controller adds the board's 42 V overvoltage
limit, the MOTOR_1 overtemperature limit and Hall health. These thresholds do not
replace the hardware BKIN.

### Nets tied together on the shield

These matter even with a fixed pin map:

* R58/R86 tie **PA7 and PB15** together on the U-LIN net. On the F410 PB15
  is **TIM1_CH3N**, not CH1N: it is left Hi-Z.
* R73/R74/R78 put **PA11, PB14 and PA6** on BKIN. PA11 is TIM1_CH4, but the
  ADC trigger uses CH4 only inside TIM1 — it must not reach the PA11 pin.
  PA11/PB14 are left Hi-Z.
* R63/R62/R65 tie **PA5, PB13, PC5** to BEMF3. All left Hi-Z.
* BEMF2 via R60 lands on CN10.18, which is **NC on the F410RB**, not PB11.
  The other branch, R61, goes to PC4. Sensorless operation is not used in
  this port.
* PC3/BEMF1, PC9/GPIO_BEMF, PB4/CURRENT REF and PA12/CPOUT are never driven.
  CPOUT is a separate comparator with its own threshold; the hardware BKIN
  uses U24–U26 and is independent of CURRENT REF.

### UART over USB and SWD

* **F410 PA2 / USART2_TX AF7 → USART_TX → SB13 → ST-LINK PA3 / RX → USB VCP**.
* **USB VCP → ST-LINK PA2 / TX → SB14 → USART_RX → F410 PA3 / USART2_RX AF7**.
* The connector is the **CN1 USB Mini-B on the ST-LINK half** of the Nucleo.
  No external USB-UART adapter is needed. The STM32F410 itself has no USB
  peripheral — the ST-LINK/V2-1 provides the virtual COM port.
* **2,000,000 baud, 8N1, no flow control**, matching the G431. USART2 runs
  from 50 MHz APB1 with an exact baud divider of 25. Linux usually creates
  `/dev/ttyACM0`; with several boards attached, pick the port under
  `/dev/serial/by-id/`.
* RX uses DMA1 Stream5 Channel4 into a 256-byte ring. Transmits happen from
  the main loop, never from the current-loop ISR. The project's utilities
  default to the same 2 Mbaud link.
* SWD: ST-LINK → CN2 → PA13 SWDIO / PA14 SWCLK; NRST through SB12.

## 4. Peripherals and timing

| Resource | Configuration |
|---|---|
| HCLK / APB2 | 100 MHz / 100 MHz |
| APB1 / TIM5CLK | 50 MHz / 100 MHz (doubled because the APB prescaler is 2) |
| TIM1 | PSC=0, ARR=2500, RCR=0, center-aligned → 20 kHz |
| TIM1 CH4 | CCR4=2400, PWM1, TRGO=OC4REF, falling edge |
| ADC1 | APB2/4 = 25 MHz, 12 bit, U/V/W/Vbus injected, 15-cycle sampling |
| DMA2 Stream0 Channel0 | NTC/pot, circular, two halfwords; regular sequence started once per 10 ms |
| TIM5 | PSC=99, ARR=0xFFFFFFFF, 1 MHz counter; Hall XOR/IC1 with CKD=DIV2, IC1F=0xF |
| SysTick | 1 kHz HAL tick, divided by 10 for the 100 Hz speed loop |
| IRQ priorities | BKIN=0; ADC/ADC DMA=1; Hall TIM5=2; SysTick=3 |

The TIM5 slave reset is disabled (`SMS=0`): Hall captures do not reset CNT,
so `time_us()` stays a monotonic 32-bit microsecond counter. CKD divides
only the digital-filter clock, not the counter rate. CH2/CH3 of TIM5 are
inputs to the XOR only and are never enabled as outputs.

The ADC is triggered **once per full PWM period**, by the falling OC4REF
while counting up, 1 µs before the CNT peak. The three current conversions
take about 3×(15+12)/25 MHz = **3.24 µs**, all four injected channels
**4.32 µs**, plus a small trigger synchronization. Software x2 is enabled by
default: the first JEOC snapshots U/V/W/Vbus and starts another injected
scan when the low-side window permits. The second JEOC publishes their sums
and runs the controller once. There is still one control frame per 50 µs.

Pairing and trigger switching are entirely inside `Stm32Adc`. Both previous
and current duties must be below 60%, the timer period must be unchanged,
and TIM1 must be running down with PSC=0 and more than 450 ticks left before
the largest compare. Otherwise one scan publishes `2*raw`, keeping identical
scaling. The first frame and high-to-low duty transitions also use this
fallback. Overcurrent checks inspect each unaveraged scan before the control
update.

Unlike the G431, the currents are sampled sequentially, with no internal
PGA and no hardware oversampling. Each current's sample-and-hold aperture
is 15/25 MHz = 0.60 µs; successive apertures start 1.08 µs apart. Relative
to the counter peak, U/V/W sampling starts at −1.00/+0.08/+1.16 µs, plus
up to 0.13 µs trigger latency (DS11144: three ADC clocks and one APB2 clock).

FOC uses **low-side-clamped space-vector modulation (DPWMMIN)**: subtracting
the lowest phase reference from all three references preserves the
line-to-line voltages and keeps one low-side switch continuously on. The
zero vector uses all low sides, with no all-high zero-vector interval.
`max_modulation=0.95` matches the duty range **0.00–0.95**. The highest-duty
phase is reconstructed from the other two currents: its 2.5 µs low-side
window at maximum duty is too short for the complete sequential scan.
For a circular voltage vector at 95% modulation, the two lower duties never
exceed `sqrt(3)/2 × 0.95 ≈ 0.82273`. Each retained shunt therefore has at
least 8.86 µs of low-side window. Even the earliest aperture starts after
the 1 µs dead time and the 2 µs settling budget (UM1996 T-noise + T-rise);
the latest aperture ends before its low-side switch turns off.

The voltage-vector limit is applied before modulation, so the current
regulators' anti-windup uses the same available voltage range as the PWM
outputs. These settings are supplied by the board through `BoardInfo`.

The NTC and potentiometer use a separate DMA-driven regular sequence with
144-cycle sampling. The injected group preempts it. No ADC wait happens in
an ISR — the interface returns cached regular values. The HAL tick stays at
1 ms, so UART timeouts and `HAL_Delay` are not stretched.

`AT+SPD` specifies mechanical RPS for VELOCITY or the OPEN_LOOP field;
TORQUE uses the `AT+CUR` current demand. The open-loop field phase is a
DDS accumulator advanced on every
PWM tick (`libecu/include/open_loop_phase.hpp`); the six-step sector is its
rounded value with the usual commutation offset. Physical Hall tracking
and the speed/position telemetry keep reporting the real rotor in open
loop; the virtual field never overwrites them.

`f410_diagnostics` is reachable over SWD:

* `samples`: number of complete control frames, not individual injected scans;
* `max_current_loop_cycles`: maximum first-JEOC-to-controller/OSC completion
  interval, including the second-scan gap when software x2 is used;
* `overruns`: those intervals exceeding the 5000-cycle period, or a new JEOC
  pending on exit;
* `faults`: bitmask — 1=BKIN, 2=|I|>25 A, 4=Vbus<10 V, 8=ADC/DMA/stale,
  16=ISR overrun, 32=out-of-range NTC while driving, 64=current-input
  calibration failed;
* `hall_captures`: hardware TIM5 captures after the digital filter;
* `hall_overcaptures`: lost-capture (CC1OF) events. This is distinct from
  the decoded Hall edge count published in HSTATUS.

On any hardware fault the console prints `Board fault mask ...`; the bridge
stays latched off until reset. The 5000-cycle budget is the full 50 µs PWM
period.

## 5. Code, build and first bring-up

* `STM32F410/Core/Src/board.cpp` — board factory, clocks, UART/DMA, ISR
  routing.
* `libecu/hal/stm32f4/` — the fixed PWM/Hall/ADC drivers of this port.
* `STM32F410/Core/Inc/board_config.hpp` — electrical constants of the stack.
* `STM32F410/Core/Inc/ecu_config.hpp` — loop rates and numeric profile.
* `STM32F410/Drivers.tar.gz` — pinned ST HAL/CMSIS sources; no network is
  needed at build time. The vector table is specifically STM32F410Rx.
* The shared `libecu/src/main.cpp` takes duty/modulation limits from
  BoardInfo. The ADC conversion core accepts fractional gains; the integer
  16 of the G431 keeps working.
* All objects are statically allocated with a 4 KiB reserved stack. The
  F410 console uses heapless nanoprintf/fast_float instead of newlib's
  allocating float printf/strtof; `_sbrk` always fails.

From the repository root:

```sh
./build.sh --platform STM32F410
./flash.sh --platform STM32F410 --verify

# Optional build with the manual throttle/brake inputs:
./build.sh --platform STM32F410 --throttle

# Debug uses -Og rather than -O0; check real-time behavior on hardware:
./build.sh --platform STM32F410 --debug
./flash.sh --platform STM32F410 --debug --verify

# Optional single-scan build (software x2 is ON by default):
cmake -S STM32F410 -B STM32F410/build-adc-x1 -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ADC_SOFTWARE_X2=OFF
cmake --build STM32F410/build-adc-x1 -j
```

The HAL's compile-time switch is `STM32F4_ADC_SOFTWARE_X2=1`; CMake sets it
from `ENABLE_ADC_SOFTWARE_X2`. Setting it to `0` removes the pairing state and
restores single-scan acquisition and calibration units. Apply the define
consistently to all translation units that include `stm32_adc.hpp`.

Release uses **`-O2 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`** with the CPU at
100 MHz and float arithmetic by default. Artifacts land in
`STM32F410/build-release/open-ecu.{elf,hex,bin,map}`; Debug in
`STM32F410/build/`. Fixed-point profile: `--numeric-profile fixed`.
Flashing uses OpenOCD `interface/stlink.cfg` + `target/stm32f4x.cfg`, or
`--method stlink`; verification is always on. The ST-LINK USB connection
carries both flashing and the AT console.

First bring-up is done **over USB only, without J1 power**. After flashing,
check the banner, `AT+VER?`, `AT+ALGO?`, `AT+DMODE?`, `AT+SPD?`, telemetry
and zero currents — the ADC zeros should sit near 1.71 V. Then rotate the
motor by hand and check the Hall readings. Verifying PWM/BKIN/currents
needs J1 power and an oscilloscope; output polarity, current scale and the
FOC angle offset should be confirmed on the bench before loaded operation.
The current-sense calibration works from USB, since the amplifier is
powered from the Nucleo 3.3 V. The commands above are shown without
transport framing: the UART expects `AT+...*CRC16\r\n`, encoded by
`frame()` in `utility/ecu.py`.

Leave the drive parked after every bench session:

```
AT+SPD=0
AT+DMODE=2
```
