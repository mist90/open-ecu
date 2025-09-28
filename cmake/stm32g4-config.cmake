# STM32G4 Series Configuration
# MCU-specific settings for STM32G431CBU

# Flash and RAM memory configuration
set(FLASH_SIZE 128K)
set(RAM_SIZE 32K)

# CPU specific parameters
set(CPU_TYPE cortex-m4)
set(FPU_TYPE fpv4-sp-d16)

# HAL configuration files to exclude from build
set(HAL_EXCLUDE_SOURCES
    "stm32g4xx_hal_msp_template.c"
    "stm32g4xx_hal_timebase_rtc_alarm_template.c"
    "stm32g4xx_hal_timebase_rtc_wakeup_template.c"
    "stm32g4xx_hal_timebase_tim_template.c"
)

# Optional HAL modules (can be excluded to reduce code size)
set(HAL_OPTIONAL_MODULES
    "stm32g4xx_hal_adc_ex.c"
    "stm32g4xx_hal_can.c"
    "stm32g4xx_hal_comp.c"
    "stm32g4xx_hal_cordic.c"
    "stm32g4xx_hal_crc.c"
    "stm32g4xx_hal_crc_ex.c"
    "stm32g4xx_hal_cryp.c"
    "stm32g4xx_hal_dac.c"
    "stm32g4xx_hal_dac_ex.c"
    "stm32g4xx_hal_fmac.c"
    "stm32g4xx_hal_i2c.c"
    "stm32g4xx_hal_i2c_ex.c"
    "stm32g4xx_hal_i2s.c"
    "stm32g4xx_hal_irda.c"
    "stm32g4xx_hal_iwdg.c"
    "stm32g4xx_hal_lptim.c"
    "stm32g4xx_hal_ltdc.c"
    "stm32g4xx_hal_nand.c"
    "stm32g4xx_hal_nor.c"
    "stm32g4xx_hal_qspi.c"
    "stm32g4xx_hal_rng.c"
    "stm32g4xx_hal_rtc.c"
    "stm32g4xx_hal_rtc_ex.c"
    "stm32g4xx_hal_sai.c"
    "stm32g4xx_hal_sai_ex.c"
    "stm32g4xx_hal_smartcard.c"
    "stm32g4xx_hal_smartcard_ex.c"
    "stm32g4xx_hal_smbus.c"
    "stm32g4xx_hal_spi.c"
    "stm32g4xx_hal_spi_ex.c"
    "stm32g4xx_hal_sram.c"
    "stm32g4xx_hal_usart.c"
    "stm32g4xx_hal_usart_ex.c"
    "stm32g4xx_hal_wwdg.c"
)

# Essential HAL modules for BLDC motor control
set(HAL_ESSENTIAL_MODULES
    "stm32g4xx_hal.c"
    "stm32g4xx_hal_cortex.c"
    "stm32g4xx_hal_dma.c"
    "stm32g4xx_hal_dma_ex.c"
    "stm32g4xx_hal_exti.c"
    "stm32g4xx_hal_flash.c"
    "stm32g4xx_hal_flash_ex.c"
    "stm32g4xx_hal_gpio.c"
    "stm32g4xx_hal_opamp.c"
    "stm32g4xx_hal_opamp_ex.c"
    "stm32g4xx_hal_pwr.c"
    "stm32g4xx_hal_pwr_ex.c"
    "stm32g4xx_hal_rcc.c"
    "stm32g4xx_hal_rcc_ex.c"
    "stm32g4xx_hal_tim.c"
    "stm32g4xx_hal_tim_ex.c"
    "stm32g4xx_hal_uart.c"
    "stm32g4xx_hal_uart_ex.c"
    "stm32g4xx_hal_adc.c"
    "stm32g4xx_hal_adc_ex.c"
)

# Function to filter HAL sources
function(filter_hal_sources HAL_SOURCES_VAR)
    set(FILTERED_SOURCES)
    foreach(SOURCE ${${HAL_SOURCES_VAR}})
        get_filename_component(SOURCE_NAME ${SOURCE} NAME)
        
        # Skip template files
        if(SOURCE_NAME IN_LIST HAL_EXCLUDE_SOURCES)
            continue()
        endif()
        
        # Only include essential modules for smaller binary
        if(SOURCE_NAME IN_LIST HAL_ESSENTIAL_MODULES)
            list(APPEND FILTERED_SOURCES ${SOURCE})
        endif()
    endforeach()
    set(${HAL_SOURCES_VAR} ${FILTERED_SOURCES} PARENT_SCOPE)
endfunction()