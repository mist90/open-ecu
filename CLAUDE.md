# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BLDC ECU firmware for the b-g431b-esc1 board (STM32G431CBU). Implements 6-step trapezoidal commutation with Hall sensor feedback, closed-loop PID speed control, and comprehensive safety monitoring.

**Target Hardware:** STM32G431CBU (Cortex-M4, 128KB Flash, 32KB RAM)

## Project Structure

The project is organized to support multiple hardware platforms:

```
open-ecu/
├── libecu/              # Platform-independent motor control library
│   ├── include/         # Public headers (algorithms, interfaces)
│   ├── src/             # Core algorithm implementations
│   ├── hal/             # Platform-specific HAL implementations
│   │   └── stm32g4/     # STM32G4-specific drivers
│   └── tests/           # Unit tests (host platform)
├── STM32G431/           # STM32G431 platform-specific files
│   ├── Core/            # Application code (main.cpp, HAL config)
│   ├── Drivers/         # STM32 HAL/CMSIS drivers
│   ├── CMakeLists.txt   # Platform build configuration
│   ├── build.sh         # Platform build script
│   ├── flash.sh         # Platform flash script
│   └── STM32G431CBUX_FLASH.ld  # Linker script
├── cmake/               # Shared CMake toolchain files
│   ├── arm-none-eabi-gcc.cmake
│   └── stm32g4-config.cmake
├── build.sh             # Multi-platform build wrapper
└── flash.sh             # Multi-platform flash wrapper
```

## Build System

### Multi-Platform Build (Recommended)
```bash
# Build for default platform (STM32G431)
./build.sh

# Build for specific platform
./build.sh --platform STM32G431

# Release build (optimized for size)
./build.sh --release

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose

# Enable PWM ISR debug capture
./build.sh --debug-pwm
```

Build outputs: `STM32G431/build/open-ecu.elf`, `STM32G431/build/open-ecu.hex`, `STM32G431/build/open-ecu.bin`

### Platform-Specific Build
```bash
# Build directly from platform directory
cd STM32G431
./build.sh --release
```

### STM32CubeIDE
Open the STM32G431 project in STM32CubeIDE for Eclipse-based development.

## Flash Commands

### Multi-Platform Flash
```bash
# Flash default platform (STM32G431) debug build
./flash.sh

# Flash specific platform
./flash.sh --platform STM32G431

# Flash release build
./flash.sh --release

# Flash using ST-Link (default is OpenOCD)
./flash.sh --method stlink

# Flash with verification
./flash.sh --verify
```

### Platform-Specific Flash
```bash
# Flash directly from platform directory
cd STM32G431
./flash.sh --method openocd --verify
```

## Testing

Unit tests for libecu library (platform-independent, runs on host):

```bash
cd libecu/tests
make test    # or 'make run'
make clean
```

Tests use mock hardware interfaces to validate control algorithms without target hardware.

## Architecture

### Modular Design Principles

The firmware is split into **platform-independent** and **platform-specific** layers:

#### libecu Library (Platform-Independent Core)
Located in `libecu/include/` and `libecu/src/`:

- **CommutationController** (`algorithms/commutation_controller.hpp`): 6-step commutation with Hall sensor position feedback. Supports both closed-loop (sensor-driven) and open-loop (timing-based) modes.
- **PidController** (`algorithms/pid_controller.hpp`): Digital PID controller with anti-windup, derivative filtering, and configurable setpoint ramping.
- **SafetyMonitor** (`safety/safety_monitor.hpp`): Real-time fault detection for overcurrent, overtemperature, and undervoltage with configurable thresholds and fault counters.
- **BldcController** (`bldc_controller.hpp`): High-level motor controller integrating commutation, speed control, and safety monitoring.

Hardware Interfaces (abstract):
- **PwmInterface** (`interfaces/pwm_interface.hpp`): 3-phase PWM abstraction
- **HallInterface** (`interfaces/hall_interface.hpp`): Hall sensor reading interface

#### HAL Layer (STM32G4-Specific)
Located in `libecu/hal/stm32g4/`:

- **Stm32Pwm** (`stm32_pwm.hpp/.cpp`): TIM1-based 3-phase PWM with complementary outputs, deadtime insertion, and center-aligned mode
- **Stm32HallSensor** (`stm32_hall_sensor.hpp/.cpp`): GPIO-based Hall sensor reading (PB6/PB7/PB8)

These implement the platform-independent interfaces for STM32G4.

#### Application Layer
`STM32G431/Core/Src/main.cpp`: Instantiates HAL implementations, creates libecu controllers, and runs the control loop via SysTick interrupt (5kHz).

### Control Flow

1. **5kHz Control Loop** (SysTick interrupt):
   - Read Hall sensor states
   - Update commutation sequence
   - Calculate motor speed from position history
   - Run PID controller (setpoint → duty cycle)
   - Monitor safety parameters
   - Update PWM duty cycles

2. **Commutation**: Maps Hall sensor states (0-5) to 6-step switching sequence for U/V/W phases

3. **Speed Calculation**: Adaptive time window algorithm (10ms-5000ms) tracks position changes to compute RPM

4. **Safety**: Monitors current/temperature/voltage and triggers emergency stop on faults

### Key Design Patterns

- **Dependency Injection**: Controllers accept interface references in constructors, enabling testability
- **Hardware Abstraction**: Platform-independent algorithms in libecu, platform-specific in hal/
- **Open-Loop Fallback**: CommutationController supports timing-based commutation if sensors fail
- **USER CODE sections**: STM32CubeMX generated files preserve custom code between `/* USER CODE BEGIN */` and `/* USER CODE END */` markers

## Code Conventions

### C++ Standards
- **Language Standard**: C++17 (CMAKE_CXX_STANDARD 17)
- **Exceptions/RTTI**: Disabled (`-fno-exceptions -fno-rtti`) for embedded constraints
- **Style**: Doxygen comments (`/** @brief */`), snake_case for members, PascalCase for types

### STM32 HAL Conventions
- **Naming**: Handles use Hungarian notation (`htim1`, `huart2`)
- **Error Handling**: Call `Error_Handler()` on HAL function failures
- **Struct Init**: Use `= {0}` for zero-initialization

### Memory Constraints
- **Flash**: 128KB @ 0x08000000
- **RAM**: 32KB @ 0x20000000
- **Compiler Flags**: `-ffunction-sections -fdata-sections` + `--gc-sections` for dead code elimination
- **Optimization**: Debug: `-O0 -g3`, Release: `-Os -DNDEBUG`

## Hardware Pinout

### PWM Outputs (TIM1)
- PA8: TIM1_CH1 (Phase U HIN)
- PC13: TIM1_CH1N (Phase U LIN)
- PA9: TIM1_CH2 (Phase V HIN)
- PA12: TIM1_CH2N (Phase V LIN)
- PA10: TIM1_CH3 (Phase W HIN)
- PB15: TIM1_CH3N (Phase W LIN)

### Hall Sensors (GPIO)
- PB6: H1 (A+)
- PB7: H2 (B+)
- PB8: H3 (Z+)

### Current Sensing (OPAMPs + ADC)
- PA1: OPAMP1+ (Phase U shunt, 0.03Ω)
- PA7: OPAMP2+ (Phase V shunt, 0.03Ω)
- PB0: OPAMP3+ (Phase W shunt, 0.03Ω)

All shunts are low-side current sensors between ground and low-side MOSFETs.

## Porting to Other MCUs

To port libecu to a different microcontroller/platform:

1. **Create platform directory**: Create a new directory (e.g., `STM32F4xx/`, `ESP32/`, etc.) at project root

2. **Implement HAL interfaces** in `libecu/hal/<your_mcu>/`:
   - Implement `PwmInterface` for your timer/PWM peripheral
   - Implement `HallInterface` for GPIO or capture inputs

3. **Add platform files**:
   - Create `<platform>/Core/` directory with your application code
   - Add `<platform>/Drivers/` or equivalent HAL/SDK files
   - Create `<platform>/CMakeLists.txt` based on STM32G431 example:
     - Set toolchain file path: `../cmake/<your-toolchain>.cmake`
     - Add platform-specific include directories (local paths)
     - Add platform-specific source files (local paths)
     - Reference libecu with: `../libecu/include`, `../libecu/src`, `../libecu/hal/<your_mcu>`
     - Configure MCU-specific compile and linker flags
   - Create `<platform>/build.sh` script that:
     - Changes to build directory
     - Runs `cmake ..` (points to platform CMakeLists.txt)
     - Builds with `make`
   - Create `<platform>/flash.sh` script for programming
   - Add linker script for your target

4. **Create toolchain file** (if needed):
   - Add `cmake/<your-toolchain>.cmake` with compiler paths and flags
   - Add `cmake/<your-platform>-config.cmake` for platform-specific configuration (optional)

5. **Test with unit tests**: Run `libecu/tests/` on host to verify algorithm correctness before hardware testing

The libecu algorithms (CommutationController, PidController, SafetyMonitor) require NO changes. Only HAL implementations and application layer need platform-specific code.

## Troubleshooting Build Issues

### Toolchain Not Found
```
sudo apt install gcc-arm-none-eabi cmake stlink-tools openocd
```

### Linker Errors (undefined `__libc_init_array`)
Ensure `--specs=nano.specs --specs=nosys.specs` are used (already configured in CMakeLists.txt).

### Flash Issues (ST-Link not detected)
- Check USB connection and permissions
- Install udev rules for ST-Link
- Try different flash method: `./flash.sh --method openocd`

## Development Workflow

1. **Modify libecu algorithms**: Edit platform-independent code in `libecu/src/` and `libecu/include/`
2. **Run unit tests**: `cd libecu/tests && make test` to validate changes on host
3. **Build firmware**: `./build.sh` to compile for STM32G4
4. **Flash device**: `./flash.sh` to program the board
5. **Debug**: Use OpenOCD + GDB or STM32CubeIDE debugger

## Safety Considerations

- **Overcurrent threshold**: 120% of rated current (configured in SafetyMonitor)
- **Overtemperature threshold**: 95% of max temperature
- **Undervoltage protection**: Minimum bus voltage monitoring
- **Emergency stop**: `CommutationController::emergencyStop()` immediately disables all phases
- **Fault counters**: SafetyMonitor tracks persistent faults before triggering shutdown
