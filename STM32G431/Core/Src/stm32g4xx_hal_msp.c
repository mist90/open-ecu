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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

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

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

/**
  * @brief ADC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if(hadc->Instance==ADC1) {
    /*
     * Initializes the peripherals clocks
     */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if(HAL_RCC_ADC12_CLK_ENABLED==1) {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }
    } else if(hadc->Instance==ADC2) {
    /*
     * Initializes the peripherals clocks
     */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if(HAL_RCC_ADC12_CLK_ENABLED==1) {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }
    }

}

/**
  * @brief ADC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance==ADC1) {
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if(HAL_RCC_ADC12_CLK_ENABLED==0) {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }

        /* ADC1 DMA DeInit */
        HAL_DMA_DeInit(hadc->DMA_Handle);
    } else if(hadc->Instance==ADC2) {
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if(HAL_RCC_ADC12_CLK_ENABLED==0) {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }
    }

}

/**
  * @brief OPAMP MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hopamp: OPAMP handle pointer
  * @retval None
  */
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* hopamp)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hopamp->Instance==OPAMP1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**OPAMP1 GPIO Configuration
        PA1     ------> OPAMP1_VINP
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    } else if(hopamp->Instance==OPAMP2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**OPAMP2 GPIO Configuration
        PA7     ------> OPAMP2_VINP
        */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    } else if(hopamp->Instance==OPAMP3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**OPAMP3 GPIO Configuration
        PB0     ------> OPAMP3_VINP
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

}

/**
  * @brief OPAMP MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hopamp: OPAMP handle pointer
  * @retval None
  */
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* hopamp)
{
    if(hopamp->Instance==OPAMP1) {
        /**OPAMP1 GPIO Configuration
        PA1     ------> OPAMP1_VINP
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
    } else if(hopamp->Instance==OPAMP2) {
        /**OPAMP2 GPIO Configuration
        PA7     ------> OPAMP2_VINP
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
    } else if(hopamp->Instance==OPAMP3) {
        /**OPAMP3 GPIO Configuration
        PB0     ------> OPAMP3_VINP
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
    }

}

/**
  * @brief TIM_PWM MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_pwm: TIM_PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
    if(htim_pwm->Instance==TIM1) {
        /* Peripheral clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

    }
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim->Instance==TIM1) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM1 GPIO Configuration
        PC13     ------> TIM1_CH1N
        PB15     ------> TIM1_CH3N
        PA8     ------> TIM1_CH1
        PA9     ------> TIM1_CH2
        PA10     ------> TIM1_CH3
        PA12     ------> TIM1_CH2N
        */
        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

}
/**
  * @brief TIM_PWM MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_pwm: TIM_PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
    if(htim_pwm->Instance==TIM1) {
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();
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
