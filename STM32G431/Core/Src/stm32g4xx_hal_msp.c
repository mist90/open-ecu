/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

#include "main.h"

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/

    /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
    */
    HAL_PWREx_DisableUCPDDeadBattery();
}

/**
  * @brief TIM_OC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_oc: TIM_OC handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
    if(htim_oc->Instance==TIM2) {
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

    }
}

/**
  * @brief TIM_OC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_oc: TIM_OC handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* htim_oc)
{
    if(htim_oc->Instance==TIM2) {
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if(huart->Instance==USART2) {
        /*
        * Initializes the peripherals clocks
        */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PB3     ------> USART2_TX
        PB4     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance==USART2) {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration
        PB3     ------> USART2_TX
        PB4     ------> USART2_RX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4);
    }
}

/**
  * @brief TIM Hall Sensor MSP Initialization
  * @param htim_hall: TIM Hall Sensor handle pointer
  * @retval None
  */
void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htim_hall)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim_hall->Instance==TIM4) {
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        /* TIM4 GPIO Configuration: PB6(CH1), PB7(CH2), PB8(CH3) -> AF2_TIM4 */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* TIM4 interrupt - priority 1 (Hall sensor) */
        HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}

/**
  * @brief TIM Hall Sensor MSP De-Initialization
  * @param htim_hall: TIM Hall Sensor handle pointer
  * @retval None
  */
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef* htim_hall)
{
    if(htim_hall->Instance==TIM4) {
        /* Disable TIM4 clock */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /* Deinit TIM4 GPIO: PB6, PB7, PB8 */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8);

        /* Disable TIM4 interrupt */
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
    }
}
