# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BLDC ECU firmware for the b-g431b-esc1 board (STM32G431CBU). Implements 6-step trapezoidal commutation with Hall sensor feedback, closed-loop PID speed control, and comprehensive safety monitoring.

**Target Hardware:** STM32G431CBU (Cortex-M4, 128KB Flash, 32KB RAM)

## Build System

This project supports two build methods:

### CMake Build (Recommended for CLI)
```bash
# Debug build
./build.sh

# Release build (optimized for size)
./build.sh --release

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose
```

Build outputs: `build/open-ecu.elf`, `build/open-ecu.hex`, `build/open-ecu.bin`

### STM32CubeIDE
Standard Eclipse-based build for Debug/Release configurations.

## Flash Commands

```bash
# Flash debug build (uses OpenOCD by default)
./flash.sh

# Flash release build
./flash.sh --release

# Flash using ST-Link
./flash.sh --method stlink

# Flash with verification
./flash.sh --verify
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
`Core/Src/main.cpp`: Instantiates HAL implementations, creates libecu controllers, and runs the control loop via SysTick interrupt (5kHz).

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

To port libecu to a different microcontroller:

1. **Implement HAL interfaces** in `libecu/hal/<your_mcu>/`:
   - Implement `PwmInterface` for your timer/PWM peripheral
   - Implement `HallInterface` for GPIO or capture inputs
2. **Update build system**: Add new HAL sources to CMakeLists.txt or your build tool
3. **Adapt main loop**: Initialize HAL implementations and libecu controllers in main()
4. **Test with unit tests**: Run `libecu/tests/` on host to verify algorithm correctness before hardware testing

The libecu algorithms (CommutationController, PidController, SafetyMonitor) require NO changes.

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
