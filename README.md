# Open ECU - Multi-Platform BLDC Motor Controller

Open-source BLDC motor control firmware with platform-independent architecture. Initially developed for the b-g431b-esc1 board (STM32G431CBU), but designed to support multiple hardware platforms.

The firmware implements 6-step trapezoidal commutation with Hall sensor feedback, closed-loop PID speed control, and comprehensive safety monitoring.

## Supported Platforms

- **STM32G431** (b-g431b-esc1 board) - Primary target
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

- **CommutationController**: 6-step trapezoidal commutation with Hall sensor feedback
- **PidController**: Digital PID controller with anti-windup and derivative filtering
- **SafetyMonitor**: Real-time fault detection (overcurrent, overtemperature, undervoltage)
- **BldcController**: High-level motor controller integrating all subsystems
- **CurrentController**: Phase current regulation

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

**STM32G431** (`STM32G431/Core/Src/main.cpp`):
- Initializes STM32 HAL and peripherals
- Creates libecu controllers with HAL implementations
- Runs 5kHz control loop via SysTick interrupt

### Key Features
- **6-step trapezoidal control** with Hall sensor position feedback
- **Closed-loop speed control** using PID algorithm
- **Safety monitoring** with overcurrent, overtemperature, and undervoltage protection
- **Modular architecture** allowing easy porting to other MCUs
- **Real-time performance** optimized for embedded systems

### Control Loop

The firmware runs a 5kHz control loop (SysTick interrupt) that:
1. Reads Hall sensor states
2. Updates commutation sequence based on rotor position
3. Calculates motor speed from position history (adaptive time window)
4. Executes PID speed controller (setpoint → duty cycle)
5. Monitors safety parameters in real-time
6. Updates PWM duty cycles for all three phases

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

**Option 2: Platform-specific build**
```bash
cd STM32G431
./build.sh --release
```

**Option 3: STM32CubeIDE**
1. Open `STM32G431` directory in STM32CubeIDE
2. Build Debug or Release configuration
3. Use built-in debugger for testing

Build outputs: `STM32G431/build/open-ecu.{elf,hex,bin}`

### Flashing Firmware

**Multi-platform flash script:**
```bash
# Flash default platform (STM32G431) debug build
./flash.sh

# Flash with specific method
./flash.sh --method openocd   # or stlink, dfu

# Flash release build
./flash.sh --release

# Flash with verification
./flash.sh --verify
```

**Platform-specific flash:**
```bash
cd STM32G431
./flash.sh --method stlink --verify
```

### Testing

Unit tests for libecu (platform-independent, runs on host):
```bash
cd libecu/tests
make test
make clean
```

## Project Structure

```
open-ecu/
├── libecu/                    # Platform-independent motor control library
│   ├── include/
│   │   ├── interfaces/        # Hardware abstraction interfaces
│   │   ├── algorithms/        # Control algorithms (commutation, PID)
│   │   └── safety/            # Safety monitoring
│   ├── src/                   # Core algorithm implementations
│   │   ├── bldc_controller.cpp
│   │   ├── commutation_controller.cpp
│   │   ├── pid_controller.cpp
│   │   ├── current_controller.cpp
│   │   └── safety_monitor.cpp
│   ├── hal/                   # Platform-specific HAL implementations
│   │   └── stm32g4/           # STM32G4 drivers
│   │       ├── stm32_pwm.cpp
│   │       └── stm32_hall_sensor.cpp
│   └── tests/                 # Unit tests (host platform)
│
├── STM32G431/                 # STM32G431 platform-specific files
│   ├── Core/                  # Application code
│   │   ├── Inc/               # Headers
│   │   ├── Src/               # main.cpp and initialization
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
└── flash.sh                   # Multi-platform flash wrapper
```

## Adding New Platforms

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
