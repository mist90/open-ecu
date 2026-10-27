# Porting Open ECU to another MCU and inverter

This guide describes the hardware contract of `libecu` and the issues found
during STM32G431 / STM32F410 bring-up and the October 2026 current-noise work.
The Wind5 reconstruction is a useful hardware reference, but is a separate
controller: its q-axis voltage policy is not equivalent to Open ECU's Iq PI.

## 1. Port boundary

Keep the application in `libecu/src/main.cpp` and implement a board factory in
`<platform>/Core/Src/board.cpp`:

- Implement `Board::initialize()`, `info()`, `pwm()`, `hall()`, `adc()`,
  `createConsole()`, `startCurrentLoop()`, `startControlLoop()`,
  `takeControlTick()` and `fail()`.
- Supply `createBoard()`, a monotonic `time_us()`, and nesting-safe critical
  sections which restore the previous interrupt-mask state.
- Provide `ecu_config.hpp`: `ActiveTiming`, `ActiveNumericProfile`,
  `ActivePiProfile` and a distinguishable firmware version.
- Implement the PWM, Hall and ADC interfaces under `libecu/hal/<platform>/`.

`BoardInfo` contains hardware facts: PWM/control rates, nominal and maximum
bus voltage, requested and actual dead time, duty/modulation limits and the
low-side-clamped PWM choice. Motor poles, electrical model and loop tuning
belong in `motor_config.hpp` / the control configuration.

See [board.hpp](../libecu/include/board.hpp) and the
[F410 board port](../STM32F410/Core/Src/board.cpp).
The current application requires an ADC interface even for its PWM-rate
voltage-mode path; returning `nullptr` does not provide a working motor loop.

## 2. MCU peripheral requirements

| Resource | Required capability / acceptance criterion |
|---|---|
| Motor PWM | Three synchronized center-aligned channels. Implement all `UP`, `DOWN` and `OFF` states, including per-leg high impedance for six-step operation. Existing power stages use three complementary output pairs. |
| Output polarity | Independent main/complementary polarity and explicit inactive/idle levels compatible with the gate driver. |
| Dead time | Hardware non-overlap generation, or an equivalent guarantee in the gate driver. Convert nanoseconds using the actual dead-time clock and report the programmed result. |
| Fast shutdown | A hardware path from overcurrent detection to disabled gates, independent of ISR latency; present boards use timer break. |
| PWM update | Buffered compare registers and a coherent three-phase commit at a known boundary. State/commutation updates must also be coherent. |
| ADC trigger | A hardware PWM-relative trigger at a programmable point in the valid current-sensing window. An internal compare channel is sufficient; it need not be routed to a pin. |
| Current ADC | At least two valid phase measurements per control frame; three measurements are preferable when their windows permit full Clarke. Quantify resolution, input settling and channel-to-channel timing. |
| Auxiliary ADC | Bus voltage, temperature and optional throttle, with explicit freshness and scaling contracts. |
| ADC completion | Interrupt/DMA completion that identifies a complete frame. A timer event starting conversion does not mean that data are ready. |
| DMA | Useful for background ADC and UART RX; prevents regular traffic from consuming the fast-loop interrupt budget. |
| Hall inputs | Three digital inputs with appropriate voltage levels and noise filtering. A timer Hall/XOR interface is preferable to unfiltered GPIO interrupts on noisy motor wiring. |
| Timebase | Monotonic timestamps independent of Hall edges; a periodic slow-control tick, currently 100 Hz. Preserve the HAL's timeout timebase separately. |
| Debug | A console and a way to measure real execution time: cycle counter, spare GPIO or a free-running timer. |
| Clock | Accurate, known timer/ADC/UART clocks and enough processing margin for the actual worst-case path. |

An FPU is useful but not required by `libecu`: the fixed profile uses integer
control kernels, including transforms and limiting. Cortex-M4 DSP instructions
accelerate some operations; software fallbacks have different costs. MCU MHz
alone is not a sufficient performance criterion.

The present targets have 128 KiB Flash and 32 KiB RAM. A 16-KiB target needs a
separate RAM budget, particularly for the 1024-sample console oscilloscope and
the stack. The filter/ADC diagnostic F410 image used about 29.7 KiB including
its additional trace buffer and reserved stack; this is not the minimum size
of a production port. Check the linker map and stack-usage files.

## 3. PWM, gate polarity and startup

For a center-aligned counter, one carrier period is `2*ARR` counter clocks.
At 100 MHz, PSC=0 and ARR=2500 give 20 kHz. Check whether the peripheral's clock
differs from PCLK because of APB prescaling. The dead-time/filter clock divider
is a separate concern from the counter prescaler.

The current ports and reconstructed Wind5 all use center-aligned PWM1.
The modulation policy is separate:

- G431 uses centered min/max SVPWM: zero voltage gives equal duties near 50%.
- F410 uses DPWMMIN: one low side is clamped, and zero voltage gives 0/0/0%.
- Wind5 calculates centered compares; its stopped-drive policy can disable
  the bridge entirely.

### Driver-specific electrical levels

The IHM08M1's **L6398 HIN is active high and LIN is active low**. Its F410 port
inverts CHxN and uses HIN=0, LIN=1 for both-off. Copying G431 complementary
polarity would be incorrect. Verify `OFF`, forced-low-side, PWM, MOE-clear,
reset and fault-handler levels against the actual schematic.

Start with gates disabled, configure the timer/state preloads, and only then
allow the pins to drive the power stage. Fault handlers must actively command
off, including failures occurring before normal timer initialization.

### Dead-time findings on IHM08M1

At approximately 40.8 V, stationary tests passed with programmed dead times
of 1000, 900 and 800 ns. At 700 ns, equal 50% PWM caused **BKIN**. An earlier
31-V experiment also tripped at 500 ns. The retained setting is **1 us**.
These tests establish a trip boundary for those conditions, not a universally
valid minimum across load and temperature. BKIN identifies the protection
input event; it does not by itself identify the shape of the current spike.

### First ADC frame after enabling outputs

The diagnostic 1.2-A threshold exposed an invalid first frame immediately
after software enable/COM, including with filtering disabled. Its phase
measurements did not satisfy the expected current sum. Enable can occur
during an ADC aperture or before the analog front end has settled.

The diagnostic F410 port holds the zero vector for two full PWM periods
before starting feedback. Hardware break and the original fast overcurrent
check remain active. A new port should explicitly define when its first
frame is valid and seed digital filters from a valid frame.

## 4. ADC contract and timing

### Current hardware configurations

| Property | STM32G431 | STM32F410 |
|---|---|---|
| Current conversion | Two ADCs, simultaneous injected mode | One ADC, sequential injected scan |
| ADC clock | 42.5 MHz | 25 MHz |
| Sampling time | 6.5 ADC cycles | 15 ADC cycles |
| Injected ranks | ADC1: U, Vbus; ADC2: V, W | U, V, W, Vbus |
| Oversampling | Hardware x2, no right shift | Window-qualified software x2, enabled by default |
| Trigger | TIM1 OC4REF/TRGO2, rising near the peak | TIM1 OC4REF/TRGO, falling 1 us before the peak |
| Fast-loop entry | Timer update callback in current code | JEOC after the complete one- or two-scan frame |

On F410, each conversion takes `(15+12)/25 MHz = 1.08 us`.
The complete four-rank scan takes about **4.32 us**, plus trigger
synchronization. Relative to the counter peak, current apertures start near
**-1.00, +0.08 and +1.16 us** and last **0.60 us**. Account for the datasheet's
trigger latency as well.

On G431 the two ranks with x2 oversampling take approximately **1.79 us**.
U/V and W are still sampled at different times. The current timer-update
callback has no explicit JEOS readiness check; its execution delay must not
be treated as a portable guarantee that all JDR registers are fresh. Prefer
an explicit acquisition-complete event or a documented coherent-frame handoff.

### Low-side measurements have validity windows

Sampling must follow low-side turn-on, dead time, switching noise and
amplifier settling, and finish before the next relevant edge. IHM08M1's
UM1996 specifies `T-noise=1 us` and `T-rise=1 us`; the port additionally budgets
its 1-us timer dead time.

At 95% DPWMMIN modulation and 20 kHz, each of the two retained lower-duty
phases has at least about **8.86 us** of low-side conduction. The highest-duty
phase may have only 2.5 us and must be reconstructed. Validate every retained
aperture over angle and modulation, not just the total ADC sequence length.

Use full Clarke when all three measurements are valid; reconstruct a phase
when required by the actual window. The research firmware uses a conservative
all-three threshold of 60% duty. A production policy should derive validity
from each board's timing and the PWM frame being sampled. An unconditional
largest-duty rejection near zero adds measurement noise and can switch the
feedback discontinuously when the ADC current sum contains error.

### Scaling and calibration

Define whether a sample is one conversion, an average, or a sum of N
conversions. Set `CurrentSensorCalibration::injected_sample_sum` accordingly.

- G431 retains a sum of two conversions; its injected current and bus scaling
  include the factor of two.
- F410 also uses sums of two with software x2 enabled; a single-scan fallback
  publishes `2*raw`. Disabling x2 restores single-conversion units.
- Regular temperature/throttle channels remain single-conversion values.
- Zero offsets, plausibility checks and raw-count protection thresholds must
  use the same units as the samples to which they are applied.
- Calibrate each current channel with a stationary motor and a known
  zero-current bridge state. On IHM08M1 the nominal bias is around **1.71 V**,
  not exactly Vref/2; use measured offsets.
- Keep current polarity explicit. Here positive shunt current is out of the
  phase into the low-side shunt; FOC negates it before its transforms.

### F410 software x2 in the HAL

RM0401 Rev 4, chapter 11, provides no hardware oversampling accumulator.
Injected sequences are limited to **four** conversions; regular sequences
allow sixteen and support DMA. Repeated channels are explicitly supported
in a sequence (§11.3.3). Writing a sequence register during conversion resets
the current conversion, so sequence changes need a defined idle boundary.

`libecu/hal/stm32f4/stm32_adc.hpp` defines `STM32F4_ADC_SOFTWARE_X2` as `1`
unless overridden. CMake exposes the default-ON `ENABLE_ADC_SOFTWARE_X2`
option. For example, build the original single-scan mode with:

```sh
cmake -S STM32F410 -B STM32F410/build-adc-x1 -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ADC_SOFTWARE_X2=OFF
cmake --build STM32F410/build-adc-x1 -j
```

The HAL preserves U/V/W/Vbus and performs **two complete injected scans
within one PWM period**:

1. The normal PWM trigger starts scan A.
2. Its JEOC handler stores the four raw values and, if the window permits,
   starts scan B using JSWSTART. It returns immediately; there is no ADC wait
   loop in the ISR.
3. Scan B's JEOC handler sums matching channels, restores the external trigger
   configuration and executes one FOC update.
4. If the window is insufficient, execute one scan and publish `2*raw` to
   retain the same sum-of-two scaling contract.

The extra scan requires both previous and current maximum duties below 60%,
an unchanged timer period, PSC=0, a running down-counter, and more than 450
timer ticks remaining before the largest compare. At 100 MHz this reserves
4.5 us from the decision point for software start, synchronization and the
last current aperture. The first frame, a high-to-low duty transition or a
delayed first IRQ can thus fall back to one scan. This threshold is specific
to the F410/IHM08M1 timing.

Pairing state, trigger switching, window qualification and the first-JEOC
timestamp live in `Stm32Adc`. `captureInjected()` returns false after starting
scan B and true only after publishing a complete frame. The board routes one
control update per frame and gets calibration/protection units from
`Stm32Adc::INJECTED_SAMPLE_SUM`; the portable controller has no x2 branches.
The unaveraged current of **each** scan is checked for overcurrent, including
scan A before frame completion, so averaging cannot conceal a current peak.
Regular NTC/pot conversions keep their original single-sample units.

At standstill the mode produced one control frame per 50 us and two real scans
for every requested x2 frame. It reduced raw Iq standard deviation from about
**68 to 48 mA**, close to the expected sqrt(2) gain for uncorrelated noise.
Summing preserves half-LSB information; averaging two adjacent PWM frames
would instead add a full-frame history dependency and is a different design.

The additional scan moves the effective measurement time and consumes deadline
margin. In the prototype, first-order filtering plus x2 acquisition took about
**35.9 us from first JEOC entry to loop completion**, versus 29.9 us with one
scan. The x2 number includes the interval between the two IRQs; it is not pure
CPU execution time. Measured CCR writes with first-order filtering occurred
by approximately **38.6 us after the timer peak**, before the next peak at
50 us. Verify the Hall-update and other worst-case branches separately.

#### Earlier prototype timing: float versus fixed

The subsequent comparison included the zero-current noise floor described
below. Both F410 builds used 100 MHz, Release `-O2`, 20-kHz PWM, first-order
2-kHz filtering, PI gains 0.549775/516.792, software x2, and active OSC/telemetry.
Each capture contained **100000 control frames**:

| Numeric profile | Operating point | Mean window | Maximum window |
|---|---|---:|---:|
| fixed | Zero-current standstill | 36.67 us | 39.69 us |
| fixed | 0.5-RPS speed command | 36.30 us | 42.37 us |
| float | Zero-current standstill | 30.37 us | 33.01 us |
| float | 0.5-RPS speed command | 29.57 us | 35.01 us |

These are first-JEOC-entry-to-controller-completion intervals, including the
second ADC conversion gap, not pure CPU busy time. The first scan takes about
4.32 us before this interval, plus synchronization/IRQ latency; account for
that when budgeting the complete trigger-to-output path. No overruns or board
faults occurred. All captured acquisition frames used two scans. At standstill
the noise-floor state held all three CCRs at zero throughout both measurements.

The fixed image was restored after the comparison:
`1.2.0-f410-idle-adc2-fixed`, BIN SHA-256
`b5106248147270aaecb477b47da5bbaf93a6c0fe8dfe5f6ea3b373bbc222d386`.
Raw timing data and the measurement script are under
`/tmp/opencode/ecu-pwm-acoustics/timing-float-fixed-x2-idle/` and
`/tmp/opencode/ecu-pwm-acoustics/profile_float_fixed_x2.py`.

Those measurements describe the diagnostic tuning and full-Clarke policy.
The main F410 HAL now includes software x2; the main application's optional
feedback filters and noise floor retain their own independent defaults.
The prototype source is retained under
`/tmp/opencode/ecu-pwm-acoustics/diagnostic-src/` for comparison.

#### Main HAL validation

The integrated Release firmware was built and run in all four combinations
of float/fixed and software x1/x2. Each stationary, zero-current-demand capture
contained 100000 control frames, with active OSC/telemetry and runtime PI gains
0.549775/516.792. The main application's feedback filter and noise floor were
disabled:

| Numeric profile | Acquisition | Mean window | Maximum window |
|---|---|---:|---:|
| fixed | x1 | 26.65 us | 29.15 us |
| fixed | x2 | 33.58 us | 37.98 us |
| float | x1 | 21.53 us | 23.73 us |
| float | x2 | 28.45 us | 31.46 us |

All four maintained 20 kHz with zero overruns and board faults. Measured Vbus
was about 40.8 V in every build; calibrated current zeros changed from roughly
2122/2129/2140 counts to 4244/4256/4282 with x2. SWD observation confirmed that
the HAL suppresses the external trigger during the second software scan.
These windows start at the first JEOC; add the first 4.32-us scan and interrupt
latency when budgeting the complete trigger-to-control path.

A subsequent fixed/x2 0.5-RPS start was stopped by the test script's 1.5-A
current guard: measured current tracked the rising reference, while Hall
speed stayed zero. The integrated firmware's rotation timing was therefore
not validated in this run. The earlier prototype rotation results above use
different control settings.

After the automated run, the user confirmed successful motor rotation with
the installed main firmware on 2026-10-27. The timing table above remains the
stationary capture.

The final installed image is the main float/x2 build, with its startup-default
PI gains and the bridge parked: `1.2.0-f410-float`, BIN SHA-256
`c431869a8e57318b533eaa3bc690e874c03e9102f7272892ee2b30f6dd2eaadd`.
Builds and captures are under
`/tmp/opencode/ecu-pwm-acoustics/main-hal-x2-builds/`,
`main-hal-x2-hardware-authorized/` and `main-hal-x2-hardware-idle/`.

## 5. Analog front end

On IHM08M1, close the required FOC bias/gain jumpers and remove **C3/C5/C7** as
specified in UM1996. Their 15-nF capacitance with the input network gives an
approximately 9.3-us time constant and about 43 us to settle to 1%, incompatible
with short low-side windows. After removal the TSV994-to-ADC path is wideband.

An analog filter must settle inside the valid window and suit the amplifier's
load/stability requirements. A digital filter cannot repair invalid samples
or undo aliasing. VESC hardware using phase shunts and switchable analog
filters is not an electrically interchangeable model for this low-side board.

For Wind5, verify the exact HW revision and current-sensing transfer function.
The recovered VDS/RDS(on)-based acquisition must not inherit the STM32 boards'
shunt/gain constants. A quiet voltage-controlled reference firmware does not
validate that acquisition path for a fast Iq PI.

## 6. Timing, control tuning and digital filters

Distinguish the carrier rate, ADC conversion rate, ADC IRQ rate, control-frame
rate and slow-loop rate. RCR=0 produces updates at both timer extrema, even
when feedback executes once per full carrier period. Software x2 acquisition
doubles ADC completions, not FOC updates.

Measure the complete path:

```text
PWM event -> analog aperture -> ADC completion -> IRQ latency -> FOC
          -> coherent compare write -> selected preload-transfer boundary
```

An ISR shorter than 50 us can still miss its intended compare-transfer
boundary. Include critical sections and worst-case Hall/PLL processing;
do not infer deadlines from optimization flags. Debug builds can be unsuitable
for an energized 20-kHz loop.

### Current-model mismatch found during filter testing

The original MOTOR_1 tuning used L=0.7 mH, R=0.658 ohm, Kp=2.1991 and
Ki=2067.168. A second-order 2-kHz filter with those gains developed a growing
roughly **1.5-kHz oscillation** after a small current step. MCU filter output
matched offline filtering of the recorded raw currents within quantization;
this was not a detected coefficient/implementation error.

A rough fit to the transient suggests an effective inductance around
0.17–0.19 mH. This is not a calibrated motor model: dead time, saturation,
sensor scaling and operating point all matter. In particular, the configured
"500-Hz bandwidth" was not proof of a real 500-Hz loop.

Both filter orders subsequently passed the stationary/pulse comparison at
**Kp=0.549775, Ki=516.792**. Small steps near zero remain dominated by the
inverter's dead zone: a zero-current integrator can retain voltage inside that
dead zone, making the next step history-dependent. For comparable captures,
the integrators were reset before each pulse. Do not identify winding R/L
from this region using an ideal PWM-to-voltage assumption.

### Portable filter implementation

`current_feedback_filter.hpp` implements first-order and second-order
Butterworth filters for both numeric profiles. Coefficients are prepared
outside the ISR and states are reset on FOC entry/reset or an angle-offset
change. Filter Id/Iq after selecting valid measurements and applying Park.
Raw currents remain available for telemetry/protection.

Opt in through `FocParams`:

```cpp
motor_params.foc.current_filter_order = libecu::CurrentFilterOrder::FIRST_ORDER;
motor_params.foc.current_filter_cutoff_hz = 2000.0f;
motor_params.foc.current_noise_floor_a = 0.10f;
// Alternative for controlled comparison:
// libecu::CurrentFilterOrder::BUTTERWORTH_SECOND_ORDER
```

The portable default is `OFF`; enabling a filter and selecting PI gains are
explicit tuning decisions. The existing telemetry filter (`alpha=0.005`,
about 16 Hz at 20 kHz) is much too slow for the nominal fast feedback loop.

### Zero-current noise floor

`current_noise_floor_a` is an optional per-axis dq noise threshold; zero
disables this feature. At zero d/q current demand and near standstill
(at most 0.1 electrical 60-degree steps/s), feedback inside the threshold is
treated as zero. Both current integrators are cleared, producing a zero voltage
vector rather than retaining old duty inside the inverter dead zone.

The release threshold is 1.5 times the entry threshold: the tested setting
enters below 0.10 A on both filtered axes and resumes regulation if either
exceeds 0.15 A. Hysteresis prevents repeated transitions near the noise floor.
The raw ADC and raw dq telemetry remain available for protection/diagnosis.

Nonzero current references, including small references below the noise floor,
bypass this idle suppression. Zero-current control while rotating also stays
active, because it must generate voltage to reject back-EMF. Direct voltage
mode retains its command. Filtering or discarding all small phase-current
samples unconditionally would lose these properties.

At equal 2-kHz cutoffs and the reduced PI gains, first-order filtering gave
about 28/37 mA filtered Id/Iq standard deviation; second order gave 24/31 mA.
Second order required about 1.3 us more execution window and introduced more
phase lag. With x2 acquisition the difference narrowed: approximately
26/27 mA for first order and 29/26 mA for second order in that series.

The experimental recommendation is first order at 2 kHz, with the validated
lower PI gains and optional window-qualified x2 acquisition. The selected
configuration also passed a slow-rotation check: 0.514 RPS at a 0.5-RPS command,
about 0.634 A, no Hall faults/overruns, and a maximum processing window of
41.57 us. These results do not replace loaded or high-speed validation.

## 7. Hall, console and fault behavior

- The F410 port uses TIM5 Hall XOR capture with a 5.12-us input filter. Its
  Hall lines share a noisy motor cable. Preserve the monotonic timer when
  configuring Hall capture; resetting it on edges breaks `time_us()`.
- Keep Hall units consistent: 40 motor poles mean 20 pole pairs and 120 Hall
  edges per mechanical revolution, not 240.
- The bench uses 2-Mbaud UART with DMA RX and framed AT commands with CRC16.
  Keep parsing/formatting out of the fast ISR. Use `utility/ecu.py` for framing.
- OSC captures contain continuous samples within each buffer, with gaps
  between transmitted buffers. Analyze their spectra separately.
- Record raw phase peaks, Id and Iq as well as filtered telemetry. A filtered
  Iq limit can miss a large d-axis oscillation.
- Distinguish break, software overcurrent, ADC failure, loop overrun and Hall
  faults. On F410, a latched board fault requires reset to re-arm.
- The experimental 1.2-A guard is enabled only by the automated experiment
  script. It previously blocked a manual `monitor.py` start and must not be
  confused with hardware BKIN or the normal 25-A software threshold.
- `CUR=0` and `SPD=0` leave feedback active when the drive is enabled. Use
  `DMODE=2` for a parked high-impedance bridge.

## 8. Bring-up and verification order

1. Check the board schematic, MCU identity, pin aliases, driver polarities,
   memory map, clocks and startup/fault gate levels.
2. Bring up the console, timebase and acquisition with the bridge disabled.
   Verify raw channels, offsets, current-sum scaling and plausible Vbus/NTC.
3. Verify timer frequency, dead time, the ADC trigger and compare-transfer
   boundary. Confirm all retained sample apertures over angle/modulation.
4. Run host tests, then measure the real optimized ISR/whole-frame budget.
5. Exercise static zero-voltage and zero-current modes before motion. Capture
   raw and filtered currents together. Check startup validity separately.
6. Use small bounded current steps, then rotation and load to validate tuning,
   signs, saturation recovery, sampling transitions and protection behavior.
7. End a bench run with `SPD=0`, `CUR=0`, `DMODE=2`; report any latched fault
   and the exact installed image/settings.

Relevant host suites:

- `tests/test_current_filter/run.sh`: measured gain/phase, DC initialization,
  reset, delayed-loop step/reversal/saturation recovery, and noise-floor
  hysteresis/zero-vector behavior without masking real demands, float and fixed.
- `tests/test_foc/run.sh float|fixed`: transforms, PWM, current signs and FOC.
- `tests/test_f410_sampling/run.sh`: retained-shunt aperture margins.
- `tests/test_f410_adc_x2/run.sh`: actual HAL against anonymous fake MMIO;
  paired frames, window transitions, half-LSB scaling, per-scan protection and
  timestamp wrap, in float/fixed and enabled/disabled builds.
- `tests/test_adc_scaling/run.sh float|fixed`: injected sums and regular-channel
  isolation; also use `tests/test_numeric_adc/run.sh` when changing formats.
- Hall/open-loop/control suites described in the README for those interfaces.

Board-specific wiring and setup are documented in
[NUCLEO_F410RB_IHM08M1.md](NUCLEO_F410RB_IHM08M1.md).
The experiment source, raw captures and analysis scripts are retained under
`/tmp/opencode/ecu-pwm-acoustics/`.
