# AGENTS.md - STM32 BLDC ECU Project

## Build Commands
- **Build Debug**: Use STM32CubeIDE build system (Eclipse-based)
- **Build Release**: Use STM32CubeIDE build system with Release configuration  
- **Clean**: `rm -rf` build artifacts from Debug/Release folders
- **No automated testing framework found** - manual testing required

## Project Structure
- `Core/` - Application code (Inc/ and Src/ directories)
- `Drivers/` - STM32 HAL drivers and CMSIS libraries
- `libecu/` - Platform-independent BLDC control library
- `STM32G431CBUX_FLASH.ld` - Linker script for STM32G431CBU

## Code Style Guidelines
- **Language**: Mixed C/C++ (main.cpp is C++, HAL code is C)
- **Headers**: Use include guards (`#ifndef __MAIN_H`)
- **Naming**: 
  - Functions: `MX_TIM1_Init()` (prefix notation)
  - Variables: `htim1`, `huart2` (Hungarian notation for handles)
  - Constants: `GPIO_PIN_6`, `TIM_CHANNEL_1` (UPPER_CASE)
- **Formatting**: STM32CubeMX generated style with 2-space indentation
- **Comments**: Doxygen-style (`/** @brief */`) for function documentation
- **Error Handling**: Use `Error_Handler()` for HAL function failures
- **Memory**: Initialize structs with `= {0}`
- **USER CODE sections**: Place custom code between `/* USER CODE BEGIN */` and `/* USER CODE END */` markers

## Specialized Development Subagents

### BLDC-Expert Agent
**Responsibilities:**
- Design and implement 6-step commutation algorithms
- Hall sensor signal processing and position detection
- Motor control state machines and timing
- Electrical angle calculation and phase sequencing
- Torque and speed control strategies

**Key Skills:**
- BLDC motor theory and control principles
- Embedded motor control algorithms
- Real-time system design
- Power electronics understanding

### C++ Embedded Programmer Agent  
**Responsibilities:**
- Implement libecu core library in C++
- Design platform-independent interfaces and abstractions
- Optimize code for real-time performance
- Implement data structures and algorithms
- Memory management and resource optimization

**Key Skills:**
- Modern C++ for embedded systems
- Template programming and RAII patterns
- Real-time programming constraints
- Cross-platform library design

### HAL Adapter Agent
**Responsibilities:**
- Implement STM32G4-specific hardware drivers
- Integrate with STM32 HAL libraries
- PWM configuration and control (TIM1)
- GPIO and ADC interface implementation
- Hardware abstraction layer design

**Key Skills:**
- STM32 HAL library expertise
- Hardware peripheral configuration
- Interrupt handling and DMA
- MCU-specific optimization

### Safety Expert Agent
**Responsibilities:**
- Implement safety monitoring systems
- Fault detection and recovery mechanisms
- Emergency stop and protection logic
- Overcurrent, overtemperature, undervoltage protection
- Safety-critical software design

**Key Skills:**
- Functional safety standards (ISO 26262)
- Fault-tolerant system design
- Safety monitoring algorithms
- Emergency response systems

## Key Conventions
- HAL library usage for peripheral configuration
- Embedded C/C++ patterns for STM32G4 microcontroller
- Platform-independent libecu library for portability
- Real-time constraints: 100Hz control loop
- Safety-first design approach