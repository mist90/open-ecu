# AGENTS.md - Open ECU Multi-Platform BLDC Motor Controller

## Quick Reference
- **Language**: Mixed C/C++ (C++17 for libecu, C for STM32 HAL)
- **Target platforms**: STM32G431CBU (b-g431b-esc1) and STM32F410RB (Nucleo + IHM08M1)
- **Build System**: CMake 3.16+ with ARM GCC toolchain
- **Control Loop**: STM32 speed PI at 1 kHz, feedback/ramp/telemetry at 100 Hz, current loop at 20 kHz
- **Architecture**: Layered (libecu core → HAL → Application)
- **Debug UART**: USART2 at 2,000,000 baud; G431 PB3/PB4 (TX/RX), F410 PA2/PA3 via ST-LINK VCP
- **IDE support**: CMake generates `STM32G431/build/compile_commands.json`

## MCU Constraints (CRITICAL)
- **Flash (128KB)**: G431 Release uses `-Os` and filters HAL modules with `cmake/stm32g4-config.cmake`; F410 Release uses `-O2`.
- **RAM (32KB)**: use static allocation, no heap. Check the linked image and stack for the selected profile; the default AT scope buffer alone uses 12 KiB.
- **FPU**: `fpv4-sp-d16` with hard float ABI; use `float` only, never `double`
- **Linker scripts**: `STM32G431/STM32G431CBUX_FLASH.ld` and `STM32F410/STM32F410RBTX_FLASH.ld`
- **Stack monitoring**: `-fstack-usage` enabled — check `.su` files in build output

## Build Commands

### Multi-Platform Build (Recommended)
```bash
# Build for default platform (STM32G431) - Release (-Os), the default
./build.sh

# Build for specific platform
./build.sh --platform STM32G431

# G431 Debug build (-O0); use Release when measuring current-loop timing.
./build.sh --debug

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose

# Disable PWM ISR debug capture (it is ON by default)
./build.sh --nodebug-pwm
```

### Platform-Specific Build (STM32G431)
```bash
cd STM32G431

# Release build (default)
./build.sh

# Debug build
./build.sh --debug

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose

# Disable PWM ISR debug capture (it is ON by default)
./build.sh --nodebug-pwm
```

Build outputs:
- Debug: `STM32G431/build/open-ecu.{elf,hex,bin}`
- Release: `STM32G431/build-release/open-ecu.{elf,hex,bin}`

See [FOC handoff: ISR timing](docs/FOC_HANDOFF.md#isr-timing) for the older
pre-FOC baseline and the 50 µs carrier budget.

### libecu Library Build (Host Platform)
```bash
cd libecu

# Build static library
make

# Clean
make clean

# Test compilation
make test-compile

# Show library info
make info
```

### Flashing Firmware
```bash
# Flash default platform (STM32G431) release build
./flash.sh

# Flash with specific method (default: openocd; also: stlink, dfu)
./flash.sh --method openocd

# Flash debug build
./flash.sh --debug

# Flash with verification
./flash.sh --verify

# Platform-specific flash
cd STM32G431
./flash.sh --method stlink --verify
```

### Testing
- **C++ host tests** live in `tests/<name>/`, each with a `run.sh` that compiles
  the libecu sources natively with g++ and runs them:
  - `tests/test_foc/run.sh` — FOC maths and closed-loop behaviour against a
    simulated PMSM (`float` or `fixed`)
  - `tests/test_hall_monitor/run.sh`
  - `tests/test_motor_pll/run.sh` (optional `--plot` uses NumPy/Matplotlib)
  - `tests/test_command_processor/run.sh` — real shared main with a non-AT
    processor, lifecycle/default silence, float/fixed and manual input on/off
  - `tests/test_motor_calibration/run.sh` — signed d/q error weights and FOC
    response for the scooter and JK42 motors
  - `tests/test_control/run.sh float|fixed|numeric-fixed|scooter-fixed` — controller and AT
    integration, including rejection of removed commands, the `AT+STATUS?` wire
    fields and capture hooks. `scooter-fixed` is a 16-kHz MOTOR_1 host fixture;
    `LIBECU_OSC_BUFFER_SIZE=256` exercises a smaller AT capture buffer
- **Python simulations** in `tests/` (`sim_current.py`, `sim_3phase_current.py`) model current dynamics; they are not firmware tests
- Hardware testing on the b-g431b-esc1 + BLDC rig via `utility/` scripts
  (`pll_sweep.py`, `fang_sweep.py`, `capture_osc.py`). **Always leave the drive
  in `AT+DMODE=2` with `AT+SPD=0`.**
- `make test-compile` in `libecu/` validates library builds only

## Project Structure
```
open-ecu/
├── libecu/                    # Platform-independent motor control library
│   ├── include/               # Public headers, all in one flat directory:
│   │                          #   board.hpp        - abstract board factory
│   │                          #   *_interface.hpp  - PWM/Hall/ADC abstraction
│   │                          #   motor_config.hpp - per-motor configuration
│   │                          #   motor_io.hpp     - controller inputs and sampled results
│   │                          #   algorithms (six-step, FOC, PI, PLL, ...)
│   │                          #   crc16/critical_section utilities
│   ├── src/                   # Core implementations + main.cpp (the ECU itself)
│   ├── hal/                  # stm32g4/ and stm32f4/ drivers
│   └── Makefile              # Host platform build (excludes main.cpp)
├── STM32G431/                # STM32G431 platform
│   ├── Core/                  # Board port
│   │   ├── Inc/              # Headers (main.h)
│   │   ├── Src/              # Sources (board.cpp, stm32g4xx_it.c)
│   │   └── Startup/          # Startup assembly
│   ├── Drivers/              # STM32 HAL and CMSIS
│   ├── CMakeLists.txt        # Platform build config
│   ├── build.sh              # Platform build script
│   ├── flash.sh              # Platform flash script
│   └── STM32G431CBUX_FLASH.ld # Linker script
├── cmake/                    # Shared CMake toolchains
├── STM32F410/                # Nucleo + IHM08M1 board port
├── tests/                    # Host regressions and Python simulations
├── build.sh                  # Multi-platform build wrapper
└── flash.sh                  # Multi-platform flash wrapper
```

## Code Style Guidelines

### Language Standards
- **C++17** for libecu library (embedded subset)
- **C17** for STM32 HAL code
- **No exceptions**: `-fno-exceptions` (embedded constraint)
- **No RTTI**: `-fno-rtti` (size optimization)
- **No dynamic allocation**: Use static allocation for predictable behavior

### Header Files
- **Include guards**: `#ifndef LIBECU_<MODULE>_HPP` for C++, `#ifndef __<MODULE>_H` for C
- **C++ headers**: Use `.hpp` extension
- **C headers**: Use `.h` extension
- **Extern C**: Wrap C++ headers with `extern "C"` guards when needed

Example (C++ libecu header):
```cpp
#ifndef LIBECU_PI_CONTROLLER_HPP
#define LIBECU_PI_CONTROLLER_HPP

namespace libecu {
// ...
} // namespace libecu

#endif // LIBECU_PI_CONTROLLER_HPP
```

Example (STM32 HAL header):
```c
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// ...

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
```

### Naming Conventions

#### libecu Library (C++)
- **Classes**: `PascalCase` (e.g., `PiController`, `BldcController`)
- **Functions/Methods**: `camelCase` (e.g., `update()`, `setParameters()`)
- **Member variables**: `snake_case_` with trailing underscore (e.g., `error_`, `integral_`)
- **Local variables**: `snake_case` (e.g., `duty_cycle`, `hall_state`)
- **Constants**: `UPPER_CASE` or `kConstant` (e.g., `MAX_SPEED`, `kDefaultFrequency`)
- **Enums**: `PascalCase` enum class (e.g., `enum class PwmState`)
- **Namespaces**: `lowercase` (e.g., `namespace libecu`)

#### STM32 HAL Code (C)
- **Functions**: `Prefix_PascalCase()` (e.g., `MX_TIM1_Init()`, `HAL_TIM_Base_Init()`)
- **Variables**: Hungarian notation for handles (e.g., `htim1`, `huart2`, `hadc1`)
- **Constants**: `UPPER_CASE` (e.g., `GPIO_PIN_6`, `TIM_CHANNEL_1`)
- **Macros**: `UPPER_CASE` (e.g., `DEBUG_PWM_ISR`)

### Formatting
- **Indentation**: 2 spaces (STM32CubeMX style) or 4 spaces (libecu)
- **Braces**: Kernighan & Ritchie style (opening brace on same line for functions)
- **Line length**: Aim for 100 characters, hard limit 120
- **Alignment**: Align related assignments and comments where it improves readability

### Comments and Documentation
- **Doxygen style** for all public APIs
- **File headers**: Include `@file` and `@brief` tags
- **Function docs**: `@brief`, `@param`, `@return` tags
- **Inline comments**: Use `//` for C++, `/* */` for C
- **TODOs**: `// TODO: description` format

Example:
```cpp
/**
 * @file pi_controller.hpp
 * @brief Fixed-frequency PI controller with anti-windup
 */

/**
 * @brief Advance the PI controller by one sample
 * @param setpoint Desired value
 * @param feedback Current value
 * @param external_saturation Inner loop's saturation sign
 * @return Control output
 */
Output update(Input setpoint, Input feedback, int external_saturation = 0) noexcept;
```

### Includes
- **Order**: STM32 HAL → libecu → C++ std → C std
- **Relative paths** from `libecu/src/` to public headers: `#include "../include/board.hpp"`
- **System includes** with angle brackets: `#include <cstdint>`

Example (`libecu/src/main.cpp`):
```cpp
#include "../include/board.hpp"
#include "../include/bldc_controller.hpp"
#include <algorithm>
```

### Types and Variables
- **Fixed-width integers**: Use `uint8_t`, `uint16_t`, `uint32_t`, `int32_t` (from `<cstdint>`)
- **Floating point**: Use `float` for embedded (hardware FPU support)
- **Initialization**: Always initialize variables
  - C++: Member initializer lists preferred
  - C: `= {0}` for struct initialization
- **Const correctness**: Use `const` for read-only parameters and methods

### Error Handling
- **libecu**: Use return values (bool for success/failure, nullptr for errors)
- **STM32 HAL**: Check `HAL_StatusTypeDef` return values
- **Critical failures**: Call `Error_Handler()` (STM32 platform)
- **No exceptions**: Embedded environment doesn't support exceptions

Example:
```cpp
// libecu style
bool PwmInterface::initialize(uint32_t frequency, uint16_t dead_time_ns);

// STM32 HAL style
if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
}
```

### STM32CubeMX USER CODE Sections
- **Always** place custom code between markers:
  ```c
  /* USER CODE BEGIN <section> */
  // Your code here
  /* USER CODE END <section> */
  ```
- Prevents code deletion when regenerating with STM32CubeMX
- Common sections: `Includes`, `Private defines`, `2` (before main loop), `3` (inside main loop)

### Real-Time Constraints
- **Speed control**: SysTick at 1 kHz; every tenth tick updates feedback and limits before one PI step.
- **Current control**: One 20-kHz tick per full 50 µs carrier period on both STM32 boards.
  F410 enters on injected ADC completion; G431 uses the down-counting TIM1 update.
  Measure the whole path against the required PWM preload-transfer boundary.
- **Avoid blocking**: No polling loops in ISRs
- **Minimize allocations**: Use static allocation, avoid heap
- **Critical sections**: Use `disable_interrupts()`/`enable_interrupts()` sparingly
- **Stack usage**: Monitor with `-fstack-usage` flag (enabled in CMake)

### Platform Independence (libecu)
- **No hardware dependencies** in `libecu/src/` and `libecu/include/`
- **Abstract interfaces** in `libecu/include/` (`pwm_interface.hpp`, `hall_interface.hpp`, `adc_interface.hpp`, `board.hpp`)
- **HAL implementations** in `libecu/hal/<platform>/`
- **Platform-specific code** only in HAL layer

### Adding a board
`libecu/src/main.cpp` is the ECU and is the same source on every target. A port
supplies one translation unit (`<platform>/Core/Src/board.cpp` on STM32G431)
that:
1. derives from `libecu::Board` and builds the concrete `PwmInterface`,
   `HallInterface`, `AdcInterface` and `CommandProcessor`,
2. implements `libecu::createBoard()` returning that instance,
3. implements `time_us()`, `disable_interrupts()` and `enable_interrupts()`,
4. fills `BoardInfo` with the facts the ECU cannot know: PWM and control loop
   rates, nominal bus voltage, positive drive-current limit and dead time.

Motor properties (poles, limits, electrical model, loop tuning) stay in
`libecu/include/motor_config.hpp`; board properties (shunt, divider, NTC,
dead time, loop rates, DC link) stay in the port. Nothing above `board.hpp`
includes a vendor HAL, and nothing below it knows which motor is attached.

### Selecting a command protocol
`Board::createConsole()` returns a board-owned `CommandProcessor&` combining
the selected protocol with the UART's `read()` / length-delimited `write()`.
`AtCommandProcessor` is the default for existing STM32 ports; their UART
bridges are templates over the processor. Framing and telemetry live in the
processor, not in the shared main or the transport.

The foreground calls `onStarted()` once, `onControlTick()` on slow ticks and
`process()` / `processOutput()` on every iteration. Default optional hooks
are silent. On a successful `pwmInterruptHandler()` the board calls
`captureCurrentSample(controller.io())` only when `wantsCurrentSample()` is
true; neither ISR hook may transmit. See [controller I/O](docs/CONTROL_IO.md)
and the [STM32 current paths](docs/CURRENT_FRAME.md).
AT owns its capture storage (`LIBECU_OSC_BUFFER_SIZE`, default 1024 samples,
12 bytes each). A different protocol inherits no AT parser or buffer.
Define the buffer size consistently across a target's translation units.

Keep diagnostic `printf` routing in the board port separate from any binary
command UART. A protocol that owns drive inputs must build without
`THROTTLE_BRAKE_CONTROL`; its foreground tick then owns input arbitration.

## Key Conventions
- **HAL drivers**: All STM32 peripherals configured via HAL (not bare metal)
- **Complementary PWM**: 3-phase inverter with dead-time (prevent shoot-through)
- **Electrical algorithms**: FOC with SVPWM (default) and 6-step trapezoidal, switchable at runtime with `AT+ALGO`
- **Safety first**: Always implement overcurrent, overtemperature, undervoltage protection
- **Modular design**: libecu core can be ported to other MCUs by implementing HAL interfaces
