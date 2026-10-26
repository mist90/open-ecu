/** @file stm32f4xx_it.c
 * @brief STM32F410 interrupt vector entry points
 */
#include "main.h"
/* USER CODE BEGIN IRQ */
void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) {
    HAL_IncTick();
    board_systick();
}
void ADC_IRQHandler(void) { board_adc_interrupt(); }
void TIM1_BRK_TIM9_IRQHandler(void) { board_break_interrupt(); }
void EXTI3_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3)) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        board_hall_interrupt();
    }
}
void EXTI15_10_IRQHandler(void) {
    const uint32_t pending = EXTI->PR & (GPIO_PIN_10 | GPIO_PIN_15);
    if (pending) {
        __HAL_GPIO_EXTI_CLEAR_IT(pending);
        board_hall_interrupt();
    }
}
/* USER CODE END IRQ */
