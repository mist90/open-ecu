/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define A__Pin GPIO_PIN_6
#define A__GPIO_Port GPIOB
#define B__Pin GPIO_PIN_7
#define B__GPIO_Port GPIOB
#define Z__Pin GPIO_PIN_8
#define Z__GPIO_Port GPIOB

#define BUTTON_Pin GPIO_PIN_10
#define BUTTON_GPIO_Port GPIOC


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
