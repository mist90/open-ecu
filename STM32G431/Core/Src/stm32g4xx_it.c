/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */

#include "main.h"
#include "stm32g4xx_it.h"

// Forward declarations - actual functions are in main.cpp
extern void motor_controller_hall_interrupt_handler(void);
extern void motor_controller_pwm_interrupt_handler(void);

void HAL_SYSTICK_Callback(void);

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_adc1;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_Callback();
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  *        Handles Hall sensor GPIO interrupts (PB6, PB7, PB8)
  */
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(A__Pin);
    HAL_GPIO_EXTI_IRQHandler(B__Pin);
    HAL_GPIO_EXTI_IRQHandler(Z__Pin);
}

/**
  * @brief GPIO EXTI callback for Hall sensor state changes
  * @param GPIO_Pin: Pin that triggered the interrupt
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Check if interrupt is from Hall sensor pins */
    if ((GPIO_Pin == A__Pin) || (GPIO_Pin == B__Pin) || (GPIO_Pin == Z__Pin)) {
        // Call the motor controller's Hall sensor interrupt handler
        motor_controller_hall_interrupt_handler();
    }
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  *        Used for high-frequency current control loop (20kHz)
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief TIM1 period elapsed callback (called at PWM frequency, 20kHz)
  * @param htim: Timer handle
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        // In center-aligned mode, update event fires twice per PWM period:
        // 1. At CNT=ARR (overflow, counting down) - ADC conversion is complete
        // 2. At CNT=0 (underflow, counting up) - ADC conversion not yet triggered
        // Process only when counting DOWN (after overflow) to read fresh ADC values
        // Check DIR bit in CR1: 0=upcounting, 1=downcounting
        if (htim->Instance->CR1 & TIM_CR1_DIR) {
            // Call motor controller's PWM interrupt handler for current control loop
            motor_controller_pwm_interrupt_handler();
        }
    }
}
