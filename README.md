# Open ECU - Multi-Platform BLDC Motor Controller

Open-source BLDC motor control firmware with platform-independent architecture. Initially developed for the b-g431b-esc1 board (STM32G431CBU), but designed to support multiple hardware platforms.

The firmware implements FOC and 6-step trapezoidal commutation with Hall sensor feedback, closed-loop PI speed control, and comprehensive safety monitoring.

## Supported Platforms

- **STM32G431** (b-g431b-esc1 board) - Primary target
- **STM32F410** (NUCLEO-F410RB + X-NUCLEO-IHM08M1) — MOTOR_1, ST-LINK USB VCP.
  One 12-bit U/V/W injected ADC scan per 20-kHz control frame, with
  single-conversion calibration units. Software double-scan acquisition has been removed.
  [Wiring, signal paths, jumpers and bring-up](docs/NUCLEO_F410RB_IHM08M1.md).
- Additional platforms can be added following the platform abstraction pattern

### STM32G431 Hardware Pinout

**PWM Outputs (TIM1):**
- PA8 - TIM1CH1 (Phase U HIN)
- PC13 - TIM1CH1N (Phase U LIN)
- PA9 - TIM1CH2 (Phase V HIN)
- PA12 - TIM1CH2N (Phase V LIN)
- PA10 - TIM1CH3 (Phase W HIN)
- PB15 - TIM1CH3N (Phase W LIN)

**Hall Sensors (GPIO):**
- PB6 - H1 (A+)
- PB7 - H2 (B+)
- PB8 - H3 (Z+)

**Current Sensing (OPAMPs + ADC):**
- PA1 - OPAMP1+ (Phase U shunt, 0.03Ω)
- PA7 - OPAMP2+ (Phase V shunt, 0.03Ω)
- PB0 - OPAMP3+ (Phase W shunt, 0.03Ω)

All shunts are low-side current sensors between ground and low-side MOSFETs.

## Firmware Architecture

The firmware uses a layered architecture separating platform-independent control algorithms from hardware-specific implementations:

### libecu Library (Platform-Independent Core)

Located in `libecu/src/` and `libecu/include/`:

- **SixStepAlgorithm**: 6-step trapezoidal commutation with Hall sensor feedback and an inner current loop
- **FocAlgorithm**: Field-oriented control with space-vector modulation (SVPWM)
- **MotorAlgorithm**: Common interface selected at runtime with `AT+ALGO`; motor-specific startup algorithm
- **PiController**: Fixed-frequency PI with trapezoidal integration and anti-windup
- **SafetyMonitor**: Real-time fault detection (overcurrent, overtemperature, undervoltage)
- **BldcController**: High-level motor controller integrating all subsystems

**Hardware Interfaces (abstract):**
- **PwmInterface**: 3-phase PWM abstraction
- **HallInterface**: Hall sensor reading interface

### HAL Layer (Platform-Specific)

Located in `libecu/hal/<platform>/`:

**STM32G4 Implementation** (`libecu/hal/stm32g4/`):
- **Stm32Pwm**: TIM1-based 3-phase PWM with complementary outputs, deadtime, center-aligned mode
- **Stm32HallSensor**: GPIO-based Hall sensor reading (PB6/PB7/PB8)

### Application Layer

Located in `<platform>/Core/`:

The ECU application itself is platform-independent and lives in
`libecu/src/main.cpp`. It reaches hardware only through the abstract factory
`libecu::Board` (`libecu/include/board.hpp`), so a new target is a new board
port rather than a new main.

`CommandProcessor` handles console I/O and control-loop notifications. Both
STM32 ports use `AtCommandProcessor`. A port can supply another protocol
without carrying the AT parser or its oscilloscope buffer; see the
[command processor and transport contract](docs/PORTING_GUIDE.md#command-protocol-and-transport).

**STM32G431** (`STM32G431/Core/Src/board.cpp`):
- Implements `libecu::Board` for this hardware
- Initializes STM32 HAL and peripherals, and creates the PWM/Hall/ADC drivers
- Routes the Hall and PWM interrupts, and arms the speed and current loops

### Key Features

See [Software and control algorithms](docs/software.md) for the control pipeline,
FOC, six-step, PLL, filtering, hybrid Iq and voltage-limiter implementation.

- **FOC** with Hall sensor position feedback
- **Experimental hybrid Iq**: opt-in model-led voltage with slow averaged-current
  trim; [algorithm and tuning](docs/software.md#hybrid-iq-control-through-uq)
- **Reusable current frames**: in-place acquisition, control and PWM application;
  [frame ownership and optional IRQ-state hooks](docs/CURRENT_FRAME.md)
- **Root-free voltage limiting** with a d-priority circumscribed 32-gon and
  consistent anti-windup: [geometry and duty margins](docs/software.md#polygon-voltage-limiting)
- **6-step trapezoidal control** with Hall sensor position feedback
- **Closed-loop speed control** using PI algorithm
- **Safety monitoring** with overcurrent, overtemperature, and undervoltage protection
- **Modular architecture** allowing easy porting to other MCUs
- **PWM-clocked open-loop field**: `AT+SPD` uses mechanical RPS in both directions;
  Hall speed/position telemetry remains physical feedback in open-loop mode
- **Real-time performance** optimized for embedded systems

### Control Loop

The firmware implements a dual-rate control architecture:

**100-Hz speed control loop:**
1. Reads the PLL's speed estimate and applies feedback filtering
2. Limits the target acceleration
3. Runs the speed PI (setpoint → current or duty request)
4. Updates slow status and protection checks

**Board-clocked current loop (normally 20 kHz on STM32):**
1. Observes pending Hall changes and advances the PLL
2. Consumes PWM-synchronized current samples
3. Runs six-step or FOC and applies the resulting PWM duties

The loop rates are configured through the board and shared sampling clocks;
see the [control pipeline](docs/software.md#control-pipeline-and-units).

### Safety Features
- Overcurrent protection (120% of rated current)
- Overtemperature protection (95% of max temperature)
- Undervoltage protection (minimum bus voltage monitoring)
- Emergency stop functionality
- Fault counters and diagnostic information

## Build Instructions

### Prerequisites

Install the required toolchain:
```bash
sudo apt install gcc-arm-none-eabi cmake stlink-tools openocd
```

### Building Firmware

**Option 1: Multi-platform build script (Recommended)**
```bash
# Build for default platform (STM32G431) - Release (-Os), the default
./build.sh

# Build for specific platform
./build.sh --platform STM32G431

# NUCLEO-F410RB + X-NUCLEO-IHM08M1, AT console over USB at 2000000 baud
./build.sh --platform STM32F410

# STM32G431 Debug build (-O0); use Release to measure current-loop timing
./build.sh --debug

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose

# Disable PWM ISR debug capture
./build.sh --nodebug-pwm
```

**Option 2: Platform-specific build**
```bash
cd STM32G431
./build.sh
```

**Option 3: STM32CubeIDE**
1. Open `STM32G431` directory in STM32CubeIDE
2. Build Debug or Release configuration
3. Use built-in debugger for testing

Build outputs:
- Debug: `STM32G431/build/open-ecu.{elf,hex,bin}`
- Release: `STM32G431/build-release/open-ecu.{elf,hex,bin}`

### Flashing Firmware

**Multi-platform flash script:**
```bash
# Flash default platform (STM32G431) release build
./flash.sh

# Flash with specific method
./flash.sh --method openocd   # or stlink, dfu

# Flash debug build
./flash.sh --debug

# Flash with verification
./flash.sh --verify

# Show the Nucleo flash options using the equivalent short entry point
./flash -p STM32F410 --help
```

`flash` and `flash.sh` use the same dispatcher. Both the build and flash
wrappers locate the selected platform relative to the script, so you can
invoke them by path from another directory.

**Platform-specific flash:**
```bash
cd STM32G431
./flash.sh --method stlink --verify
```

### Testing

Select control/ADC/PWM arithmetic with
`./build.sh --numeric-profile float|fixed` or CMake `-DNUMERIC_PROFILE=fixed`.
On STM32G431, `PI_NUMERIC_PROFILE=fixed` is a PI-only override for the float control profile.

The fixed control kernels use integer arithmetic, including transforms and voltage limiting.
On ARM cores with DSP instructions, saturating 32-bit operations use `QADD`/`QSUB`;
host and Cortex-M0 builds use the portable implementation. Numeric formats and rounding
are shared by both paths. The F410 bridge-disabled timing image also checks the native
integer operations before starting its synthetic control loop.

Motor parameters live in `libecu/include/motor_config.hpp`. An undefined
`MOTOR_1` selects JK42BLS01; defining it selects the scooter motor.
The STM32F410 target defines `MOTOR_1` in CMake.

Additional host regressions:
- `tests/test_motor_alignment/run.sh` — both motors, both directions, PLL and discrete commutation
- `tests/test_hall_handoff/run.sh` — physical Hall resynchronization after open-loop operation and parking
- `tests/test_open_loop/run.sh float|fixed` — PWM-clocked field speed, direction and independent Hall feedback
- `tests/test_f410_console/run.sh` — heapless console formatting and parsing
- `tests/test_f410_sampling/run.sh` — retained-shunt ADC timing across the F410 modulation range
- `tests/test_f410_adc/run.sh` — actual F410 ADC HAL, single-scan acquisition/scaling/protection, float/fixed
- `tests/test_adc_scaling/run.sh` — injected ADC sums, zero calibration and regular-channel isolation
- `tests/test_numeric_pll/run.sh` — paired float/fixed traces at 10/20/40 kHz
- `tests/test_profile_math/run.sh` — LUT/transforms, voltage limits and Cortex-M0 helper audit
- `tests/test_numeric_adc/run.sh` — full-range current, bus, NTC and PWM comparisons
- `tests/test_foc/run.sh float|fixed` — closed-loop motor model and anti-windup recovery
- `tests/test_hybrid_iq/run.sh` — float/fixed hybrid current control, delayed motor model, noise and recovery
- `tests/test_current_filter/run.sh` — dq filters, delayed-loop stability and zero-current noise suppression
- `tests/test_control/run.sh float|numeric-fixed` — commands, native I/O, reusable current frames, protection, ramp and OSC
- `tests/test_control/run.sh t60-fixed|t60-feedback` — T60 speed PI, live retuning and optional acceleration feedback
- `tests/test_mtte/run.sh` — rolling-wheel torque observer and traction model, using synthetic data

Host tests live under `tests/`; each directory has a standalone `run.sh`.

Manual hardware testing required:
- Test on b-g431b-esc1 board with BLDC motor
- Monitor via USART2 at 2,000,000 baud on PB3 (TX) and PB4 (RX); see the
  [AT command reference](docs/AT_COMMANDS.md#connecting).
- `make test-compile` in `libecu/` validates library builds only (not actual tests)

## Project Structure

```
open-ecu/
├── libecu/                    # Platform-independent motor control library
│   ├── include/
│   │   ├── pi_controller.hpp  # Fixed-frequency, numerical-type-parametric PI
│   │   ├── fixed_point.hpp    # Explicit binary fixed-point arithmetic
│   │   └── *_interface.hpp   # Hardware abstraction interfaces (flat directory)
│   ├── src/                   # Core algorithm implementations
│   │   ├── bldc_controller.cpp
│   │   ├── six_step_algorithm.cpp
│   │   ├── foc_algorithm.cpp
│   │   └── motor_pll.cpp
│   ├── hal/                   # Platform-specific HAL implementations
│   │   └── stm32g4/           # STM32G4 drivers
│   │       ├── stm32_pwm.cpp
│   │       └── stm32_hall_sensor.cpp
│
├── STM32G431/                 # STM32G431 platform-specific files
│   ├── Core/                  # Application code
│   │   ├── Inc/               # Headers, including compile-time ecu_config.hpp
│   │   ├── Src/               # board.cpp and peripheral initialization
│   │   └── Startup/           # Startup assembly
│   ├── Drivers/               # STM32 HAL and CMSIS
│   ├── CMakeLists.txt         # Platform build configuration
│   ├── build.sh               # Platform build script
│   ├── flash.sh               # Platform flash script
│   └── STM32G431CBUX_FLASH.ld # Linker script
│
├── cmake/                     # Shared CMake toolchain files
│   ├── arm-none-eabi-gcc.cmake
│   └── stm32g4-config.cmake
│
├── build.sh                   # Multi-platform build wrapper
├── flash                      # Short entry point for flash.sh
└── flash.sh                   # Multi-platform flash wrapper
```

## Adding New Platforms

See [the porting guide](docs/PORTING_GUIDE.md) for MCU peripheral requirements,
PWM/ADC timing contracts, acquisition scaling and hardware bring-up findings.

To port libecu to a different microcontroller:

1. **Create platform directory**: `mkdir NewPlatform/`

2. **Implement HAL interfaces** in `libecu/hal/<your_mcu>/`:
   - Implement `PwmInterface` for your PWM peripheral
   - Implement `HallInterface` for Hall sensor inputs

3. **Add platform files**:
   - Copy `STM32G431/CMakeLists.txt` as template
   - Update toolchain, includes, and source paths
   - Add your platform's `Core/` and `Drivers/` directories
   - Create `build.sh` and `flash.sh` scripts

4. **Build**: `./build.sh --platform NewPlatform`

The libecu control algorithms require NO changes - only HAL implementations are platform-specific.

## License

See LICENSE file for details.
