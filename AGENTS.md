# AGENTS.md - Open ECU Multi-Platform BLDC Motor Controller

## Quick Reference
- **Language**: Mixed C/C++ (C++17 for libecu, C for STM32 HAL)
- **Target Platform**: STM32G431CBU (Cortex-M4, FPU)
- **Build System**: CMake 3.16+ with ARM GCC toolchain
- **Control Loop**: 1kHz speed control + 40kHz current loop (Hall sensors: async)
- **Architecture**: Layered (libecu core → HAL → Application)

## Build Commands

### Multi-Platform Build (Recommended)
```bash
# Build for default platform (STM32G431) - Debug
./build.sh

# Build for specific platform
./build.sh --platform STM32G431

# Release build (optimized for size with -Os)
./build.sh --release

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose

# Enable PWM ISR debug capture (~8KB RAM)
./build.sh --debug-pwm
```

### Platform-Specific Build (STM32G431)
```bash
cd STM32G431

# Debug build (default)
./build.sh

# Release build
./build.sh --release

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose

# Enable PWM ISR debug capture
./build.sh --debug-pwm
```

Build outputs:
- Debug: `STM32G431/build/open-ecu.{elf,hex,bin}`
- Release: `STM32G431/build-release/open-ecu.{elf,hex,bin}`

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
# Flash default platform (STM32G431) debug build
./flash.sh

# Flash with specific method (default: openocd; also: stlink, dfu)
./flash.sh --method openocd

# Flash release build
./flash.sh --release

# Flash with verification
./flash.sh --verify

# Platform-specific flash
cd STM32G431
./flash.sh --method stlink --verify
```

### Testing
- **No automated unit tests** - the `libecu/tests/` directory does not exist
- Manual hardware testing required on b-g431b-esc1 board with BLDC motor
- Monitor via UART (115200 baud) on PA2 (TX), PA3 (RX)
- `make test-compile` in `libecu/` validates library builds only (not actual tests)

## Project Structure
```
open-ecu/
├── libecu/                    # Platform-independent motor control library
│   ├── include/               # Public headers
│   │   ├── interfaces/        # Hardware abstraction (PwmInterface, HallInterface)
│   │   ├── algorithms/        # Control algorithms (PID, commutation)
│   │   └── safety/            # Safety monitoring
│   ├── src/                   # Core implementations
│   ├── hal/stm32g4/          # STM32G4-specific HAL implementations
│   └── Makefile              # Host platform build
├── STM32G431/                # STM32G431 platform
│   ├── Core/                  # Application code
│   │   ├── Inc/              # Headers (main.h)
│   │   └── Src/              # Sources (main.cpp, stm32g4xx_it.c)
│   ├── Drivers/              # STM32 HAL and CMSIS
│   ├── CMakeLists.txt        # Platform build config
│   ├── build.sh              # Platform build script
│   └── flash.sh              # Platform flash script
├── cmake/                    # Shared CMake toolchains
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
#ifndef LIBECU_PID_CONTROLLER_HPP
#define LIBECU_PID_CONTROLLER_HPP

namespace libecu {
// ...
} // namespace libecu

#endif // LIBECU_PID_CONTROLLER_HPP
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
- **Classes**: `PascalCase` (e.g., `PidController`, `BldcController`)
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
 * @file pid_controller.hpp
 * @brief PID controller for motor speed control with anti-windup
 */

/**
 * @brief Update PID controller
 * @param setpoint Desired value
 * @param feedback Current value
 * @param dt Time step in seconds
 * @return Control output
 */
float update(float setpoint, float feedback, float dt);
```

### Includes
- **Order**: STM32 HAL → libecu → C++ std → C std
- **Relative paths** for libecu internal includes: `#include "../include/algorithms/pid_controller.hpp"`
- **System includes** with angle brackets: `#include <cstdint>`

Example (main.cpp):
```cpp
#include "../Inc/main.h"                        // Platform headers first
#include "../../libecu/include/libecu.hpp"      // libecu headers
#include <stdint.h>                             // C standard library
#include <stdio.h>
```

### Types and Variables
- **Fixed-width integers**: Use `uint8_t`, `uint16_t`, `uint32_t`, `int32_t` (from `<cstdint>`)
- **Floating point**: Use `float` for embedded (hardware FPU support)
- **Initialization**: Always initialize variables
  - C++: Member initializer lists preferred
  - C: `= {0}` for struct initialization
- **Const correctness**: Use `const` for read-only parameters and methods

Example (C++):
```cpp
PidController::PidController(const PidParameters& params)
    : params_(params)
    , error_(0.0f)
    , integral_(0.0f)
    , first_run_(true)
{
}
```

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
- **1kHz speed control loop**: Keep periodic timer ISR under 1ms
- **40kHz current loop**: Keep PWM ISR under 25μs
- **Avoid blocking**: No polling loops in ISRs
- **Minimize allocations**: Use static allocation, avoid heap
- **Critical sections**: Use `disable_interrupts()`/`enable_interrupts()` sparingly
- **Stack usage**: Monitor with `-fstack-usage` flag (enabled in CMake)

### Platform Independence (libecu)
- **No hardware dependencies** in `libecu/src/` and `libecu/include/`
- **Abstract interfaces** in `libecu/include/interfaces/`
- **HAL implementations** in `libecu/hal/<platform>/`
- **Platform-specific code** only in HAL layer

## Key Conventions
- **HAL drivers**: All STM32 peripherals configured via HAL (not bare metal)
- **Complementary PWM**: 3-phase inverter with dead-time (prevent shoot-through)
- **6-step commutation**: Hall sensor-based trapezoidal control
- **Safety first**: Always implement overcurrent, overtemperature, undervoltage protection
- **Modular design**: libecu core can be ported to other MCUs by implementing HAL interfaces