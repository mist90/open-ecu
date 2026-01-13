cmake_minimum_required(VERSION 3.16)

# Set toolchain before project declaration
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/arm-none-eabi-gcc.cmake)

# Project configuration
project(open-ecu C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# Build configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Debug options
option(ENABLE_DEBUG_PWM_ISR "Enable PWM ISR debug data capture (uses ~8KB RAM)" OFF)

# MCU and target configuration
set(MCU_FAMILY STM32G4xx)
set(MCU_MODEL STM32G431xx)
set(CPU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Compile definitions
add_compile_definitions(
    STM32G4
    STM32G431xx
    USE_HAL_DRIVER
    $<$<CONFIG:Debug>:DEBUG>
)

# Optional debug features
if(ENABLE_DEBUG_PWM_ISR)
    add_compile_definitions(DEBUG_PWM_ISR)
    message(STATUS "PWM ISR debug capture enabled (uses ~8KB RAM)")
endif()

# Include directories
include_directories(
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    ../libecu/include
    ../libecu/include/interfaces
    ../libecu/include/algorithms
    ../libecu/include/hal/stm32g4
)

# Convert CPU_PARAMETERS list to string
string(REPLACE ";" " " CPU_PARAMS_STR "${CPU_PARAMETERS}")

# Compiler flags
set(CMAKE_C_FLAGS "${CPU_PARAMS_STR} -ffunction-sections -fdata-sections -Wall -fstack-usage")
set(CMAKE_CXX_FLAGS "${CPU_PARAMS_STR} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage")

# Debug/Release specific flags
set(CMAKE_C_FLAGS_DEBUG "-g3 -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -DNDEBUG")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CPU_PARAMS_STR} --specs=nano.specs --specs=nosys.specs -u _printf_float -u _scanf_float -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")


# Set linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431CBUX_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")

# Core source files
file(GLOB_RECURSE CORE_SOURCES
    "Core/Src/*.c"
    "Core/Src/*.cpp"
)

# Include STM32G4 specific configuration
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/stm32g4-config.cmake)

# STM32 HAL Driver sources
file(GLOB HAL_SOURCES
    "Drivers/STM32G4xx_HAL_Driver/Src/*.c"
)

# Filter HAL sources to include only essential modules
filter_hal_sources(HAL_SOURCES)

# CMSIS sources (headers only, system files are in Core/Src)
set(CMSIS_SOURCES "")

# Startup file
file(GLOB STARTUP_FILE
    "Core/Startup/startup_stm32g431cbux.s"
)

# libecu library sources (using platform-specific hal directory)
file(GLOB LIBECU_CORE_SOURCES
    "../libecu/src/bldc_controller.cpp"
    "../libecu/src/commutation_controller.cpp"
    "../libecu/src/pid_controller.cpp"
    "../libecu/src/current_controller.cpp"
    "../libecu/src/safety_monitor.cpp"
)

file(GLOB LIBECU_HAL_SOURCES
    "../libecu/hal/stm32g4/*.cpp"
)

set(LIBECU_SOURCES ${LIBECU_CORE_SOURCES} ${LIBECU_HAL_SOURCES})

# Additional system sources (already included in CORE_SOURCES)
set(SYSTEM_SOURCES "")

# Debug: Print found sources
message(STATUS "Core sources: ${CORE_SOURCES}")
message(STATUS "HAL sources: ${HAL_SOURCES}")
message(STATUS "CMSIS sources: ${CMSIS_SOURCES}")
message(STATUS "Startup file: ${STARTUP_FILE}")
message(STATUS "libecu sources: ${LIBECU_SOURCES}")

# Create executable
add_executable(${PROJECT_NAME}.elf
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${CMSIS_SOURCES}
    ${STARTUP_FILE}
    ${SYSTEM_SOURCES}
    ${LIBECU_SOURCES}
)

# Create hex and bin files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)

# Print size information
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-size $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Size information:"
)

# Create compile_commands.json for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Print build information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "MCU Family: ${MCU_FAMILY}")
message(STATUS "MCU Model: ${MCU_MODEL}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
