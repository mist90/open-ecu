cmake_minimum_required(VERSION 3.16)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/arm-none-eabi-gcc.cmake)
project(open-ecu C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(ENABLE_THROTTLE_BRAKE_CONTROL "Use IHM08M1 potentiometer and Nucleo B1 brake" OFF)
option(ENABLE_PWM_TIMING_BENCH "Synthetic sensors with MOE permanently OFF for ISR timing" OFF)
set(NUMERIC_PROFILE "float" CACHE STRING "Control arithmetic: float or fixed")
set_property(CACHE NUMERIC_PROFILE PROPERTY STRINGS float fixed)
if(NOT NUMERIC_PROFILE MATCHES "^(float|fixed)$")
    message(FATAL_ERROR "NUMERIC_PROFILE must be float or fixed")
endif()

set(DRIVERS ${CMAKE_CURRENT_SOURCE_DIR}/Drivers)
if(NOT EXISTS ${DRIVERS})
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf Drivers.tar.gz
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Cannot extract Drivers.tar.gz; recreate with python3 fetch_drivers.py")
    endif()
endif()
set(HAL ${DRIVERS}/STM32F4xx_HAL_Driver)
set(DEVICE ${DRIVERS}/CMSIS/Device/ST/STM32F4xx)
set(ECU ${CMAKE_CURRENT_SOURCE_DIR}/../libecu)
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

set(HAL_SOURCES)
foreach(module hal hal_cortex hal_rcc hal_rcc_ex hal_flash hal_flash_ex hal_pwr hal_pwr_ex
               hal_gpio hal_dma hal_tim hal_tim_ex hal_adc hal_adc_ex hal_uart)
    list(APPEND HAL_SOURCES ${HAL}/Src/stm32f4xx_${module}.c)
endforeach()
set(ECU_SOURCES)
foreach(module main bldc_controller six_step_algorithm foc_algorithm motor_pll hall_monitor
               at_command_processor crc16)
    list(APPEND ECU_SOURCES ${ECU}/src/${module}.cpp)
endforeach()

add_executable(open-ecu.elf
    Core/Src/board.cpp Core/Src/stm32f4xx_it.c Core/Src/syscalls.c Core/Src/console_libc.cpp
    ${DEVICE}/Source/Templates/system_stm32f4xx.c
    ${DEVICE}/Source/Templates/gcc/startup_stm32f410rx.s
    ${HAL_SOURCES} ${ECU_SOURCES}
    ${ECU}/hal/stm32f4/stm32_pwm.cpp
    ${ECU}/hal/stm32f4/stm32_adc.cpp
    ${ECU}/hal/stm32f4/stm32_hall_sensor.cpp)
target_include_directories(open-ecu.elf PRIVATE
    Core/Inc ${HAL}/Inc ${HAL}/Inc/Legacy ${DEVICE}/Include
    ${DRIVERS}/CMSIS/Include ${ECU}/include ${ECU}/hal/stm32f4
    ThirdParty/nanoprintf ThirdParty/fast_float/include)
target_compile_definitions(open-ecu.elf PRIVATE STM32F410Rx USE_HAL_DRIVER MOTOR_1
    $<$<BOOL:${ENABLE_THROTTLE_BRAKE_CONTROL}>:THROTTLE_BRAKE_CONTROL>
    $<$<BOOL:${ENABLE_PWM_TIMING_BENCH}>:PWM_TIMING_BENCH>
    $<$<STREQUAL:${NUMERIC_PROFILE},fixed>:ECU_FIXED_NUMERIC>)
target_compile_options(open-ecu.elf PRIVATE ${CPU_FLAGS}
    -ffunction-sections -fdata-sections -fstack-usage -Wall -Wextra
    -Wno-unused-parameter -Wno-missing-field-initializers
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-use-cxa-atexit;-fno-threadsafe-statics>)
set(CMAKE_C_FLAGS_RELEASE "-O2 -g3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g3 -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F410RBTX_FLASH.ld)
target_link_options(open-ecu.elf PRIVATE ${CPU_FLAGS}
    --specs=nano.specs --specs=nosys.specs
    -Wl,--wrap=printf,--wrap=snprintf,--wrap=strtof,--wrap=puts,--wrap=putchar
    -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/open-ecu.map)
set_property(TARGET open-ecu.elf APPEND PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})
target_link_libraries(open-ecu.elf PRIVATE m)
add_custom_command(TARGET open-ecu.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:open-ecu.elf> open-ecu.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:open-ecu.elf> open-ecu.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:open-ecu.elf>)
