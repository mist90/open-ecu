# CMake Build System for STM32 BLDC ECU

This document describes how to build the STM32 BLDC ECU project using CMake with the ARM GCC toolchain.

## Prerequisites

### Required Tools
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install cmake gcc-arm-none-eabi stlink-tools openocd dfu-util

# Verify installation
arm-none-eabi-gcc --version
cmake --version
st-flash --version
```

### Optional Tools
```bash
# For advanced debugging
sudo apt install gdb-multiarch

# For code analysis
sudo apt install cppcheck clang-format
```

## Project Structure

```
open-ecu2/
├── CMakeLists.txt              # Main CMake configuration
├── build.sh                    # Build script
├── flash.sh                    # Flash script
├── cmake/
│   ├── arm-none-eabi-gcc.cmake # ARM toolchain configuration
│   └── stm32g4-config.cmake   # STM32G4 specific settings
├── Core/                       # Application source code
│   ├── Inc/                    # Headers
│   ├── Src/                    # Sources
│   └── Startup/                # Startup assembly file
├── Drivers/                    # STM32 HAL and CMSIS drivers
│   ├── STM32G4xx_HAL_Driver/  # HAL library
│   └── CMSIS/                  # CMSIS library
├── libecu/                     # Motor control library
│   ├── include/                # Headers
│   └── src/                    # Sources
└── STM32G431CBUX_FLASH.ld     # Linker script
```

## Build Commands

### Quick Build (Debug)
```bash
./build.sh
```

### Build Options
```bash
# Release build
./build.sh --release

# Clean build
./build.sh --clean

# Verbose output
./build.sh --verbose

# Combined options
./build.sh --release --clean --verbose
```

### Manual CMake Build
```bash
# Create build directory
mkdir build
cd build

# Configure (Debug)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Configure (Release)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Build
make -j$(nproc)

# Build with verbose output
make VERBOSE=1
```

## Flash Commands

### Quick Flash (Debug)
```bash
./flash.sh
```

### Flash Options
```bash
# Flash release build
./flash.sh --release

# Flash with OpenOCD
./flash.sh --method openocd

# Flash with DFU
./flash.sh --method dfu

# Flash with verification
./flash.sh --verify

# Combined options
./flash.sh --release --method stlink --verify
```

### Manual Flash Commands
```bash
# ST-Link
st-flash write build/open-ecu2.bin 0x8000000

# OpenOCD
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
        -c "program build/open-ecu2.elf verify reset exit"

# DFU (device in DFU mode)
dfu-util -a 0 -s 0x08000000:leave -D build/open-ecu2.bin
```

## CMake Configuration

### Key Features
- **Cross-compilation** for ARM Cortex-M4
- **Optimized HAL** inclusion (only essential modules)
- **Multiple build types** (Debug/Release)
- **Automatic binary generation** (.hex, .bin)
- **Size reporting** after build
- **Compile commands** export for IDE integration

### Build Types
- **Debug**: `-g3 -O0` with debug symbols
- **Release**: `-Os -DNDEBUG` optimized for size

### Compiler Flags
```cmake
# Common flags for STM32G4
-mcpu=cortex-m4
-mthumb
-mfpu=fpv4-sp-d16
-mfloat-abi=hard
--specs=nano.specs
-ffunction-sections
-fdata-sections

# C++ specific
-fno-exceptions
-fno-rtti
-fno-use-cxa-atexit

# Safety and analysis
-Wall
-fstack-usage
-fcyclomatic-complexity
```

### Memory Configuration
- **Flash**: 128KB starting at 0x08000000
- **RAM**: 32KB starting at 0x20000000
- **Linker script**: `STM32G431CBUX_FLASH.ld`

## HAL Module Selection

The build system only includes essential HAL modules to reduce code size:

### Essential Modules (Included)
- `stm32g4xx_hal.c` - Core HAL
- `stm32g4xx_hal_cortex.c` - Cortex-M specific
- `stm32g4xx_hal_gpio.c` - GPIO control
- `stm32g4xx_hal_tim.c` - Timer/PWM
- `stm32g4xx_hal_tim_ex.c` - Extended timer functions
- `stm32g4xx_hal_opamp.c` - Operational amplifiers
- `stm32g4xx_hal_adc.c` - ADC for current sensing
- `stm32g4xx_hal_uart.c` - UART communication
- `stm32g4xx_hal_dma.c` - DMA support
- `stm32g4xx_hal_rcc.c` - Clock configuration
- `stm32g4xx_hal_pwr.c` - Power management
- `stm32g4xx_hal_flash.c` - Flash programming

### Optional Modules (Excluded)
To include additional HAL modules, edit `cmake/stm32g4-config.cmake` and add the module to `HAL_ESSENTIAL_MODULES`.

## Troubleshooting

### Common Build Issues

#### 1. Toolchain Not Found
```
Error: arm-none-eabi-gcc not found!
```
**Solution**: Install ARM GCC toolchain
```bash
sudo apt install gcc-arm-none-eabi
```

#### 2. CMake Version Too Old
```
CMake 3.16 or higher is required
```
**Solution**: Update CMake
```bash
sudo apt install cmake
# Or install from CMake website for latest version
```

#### 3. Missing HAL Sources
```
No HAL sources found
```
**Solution**: Verify Drivers directory structure matches the project layout

#### 4. Linker Errors
```
undefined reference to '__libc_init_array'
```
**Solution**: Ensure `--specs=nano.specs --specs=nosys.specs` are used

### Flash Issues

#### 1. ST-Link Not Detected
```
Error: Could not find ST-Link
```
**Solution**: 
- Check USB connection
- Install st-link udev rules
- Run with sudo (not recommended)

#### 2. Device Not Found
```
Error: Target not found
```
**Solution**:
- Verify device is in normal mode (not DFU)
- Check connections and power
- Try different flash method

#### 3. Flash Verification Failed
```
Verification failed
```
**Solution**:
- Check for flash protection
- Ensure correct memory address
- Try erasing flash first

## Performance Optimization

### Size Optimization
```bash
# Use Release build
./build.sh --release

# Check size breakdown
arm-none-eabi-nm --size-sort build/open-ecu2.elf
arm-none-eabi-objdump -h build/open-ecu2.elf
```

### Debug Optimization
```bash
# Use Debug build with symbols
./build.sh

# Generate assembly listing
arm-none-eabi-objdump -S build/open-ecu2.elf > listing.txt
```

## IDE Integration

### VS Code
```bash
# Generate compile_commands.json
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
ln -s build/compile_commands.json .
```

### CLion
- Open CMakeLists.txt as project
- Configure toolchain in Settings → Build → Toolchains
- Set CMake options in Settings → Build → CMake

### Eclipse CDT
- Import as existing CMake project
- Configure ARM toolchain in project properties

## Advanced Usage

### Custom Defines
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -DCUSTOM_DEFINE=1 ..
```

### Memory Analysis
```bash
# Generate map file
make VERBOSE=1 2>&1 | grep "\.map"

# Analyze with parser
python scripts/analyze_memory.py build/open-ecu2.map
```

### Static Analysis
```bash
# Run cppcheck
cppcheck --enable=all --inconclusive Core/Src/ libecu/src/

# Format code
find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```