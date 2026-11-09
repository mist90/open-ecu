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
/* USER CODE BEGIN BenchDeclarations */
#ifdef BENCH_PROFILING
extern uint32_t bench_pwm_irq_begin(void);
extern void bench_pwm_irq_end(uint32_t started);
#endif
/* USER CODE END BenchDeclarations */

void HAL_SYSTICK_Callback(void);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN AdcDeclarations */
extern void board_adc_error_interrupt(void);
/* USER CODE END AdcDeclarations */

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
  * @brief This function handles TIM4 interrupt.
  */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

/**
  * @brief TIM Input Capture callback
  * @param htim: Timer handle
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        motor_controller_hall_interrupt_handler();
    }
}

/**
  * @brief ADC2 current-only completion; ADC1's auxiliary tail keeps running.
  */
void DMA1_Channel2_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel2_IRQn */
#ifdef BENCH_PROFILING
    const uint32_t started = bench_pwm_irq_begin();
#endif
    motor_controller_pwm_interrupt_handler();
#ifdef BENCH_PROFILING
    bench_pwm_irq_end(started);
#endif
    /* USER CODE END DMA1_Channel2_IRQn */
}

/* USER CODE BEGIN ADC1_2_IRQn */
void DMA1_Channel1_IRQHandler(void) { board_adc_error_interrupt(); }
void ADC1_2_IRQHandler(void) { board_adc_error_interrupt(); }
/* USER CODE END ADC1_2_IRQn */
