/** @file main.h
 * @brief STM32F410 port interrupt and console entry points
 */
#ifndef OPEN_ECU_STM32F410_MAIN_H
#define OPEN_ECU_STM32F410_MAIN_H
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
/* USER CODE BEGIN Prototypes */
void Error_Handler(void);
void board_adc_interrupt(void);
void board_break_interrupt(void);
void board_hall_timer_interrupt(void);
void board_systick(void);
int board_console_write(const char* data, int length);
int board_console_read(void);
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif
