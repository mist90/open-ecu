/** @file stm32_hall_sensor.cpp
 * @brief F410 has no TIM2: the shield's three Hall lines use both-edge EXTI
 */
#include "stm32f4xx_hal.h"
#include "stm32_hall_sensor.hpp"
#include "pwm_timing_bench.hpp"
namespace libecu {
bool Stm32HallSensor::initialize() {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_NOPULL; // JP3: external 10k pull-ups to 3V3
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_10);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    return true;
}
uint8_t Stm32HallSensor::getPosition() {
#ifdef PWM_TIMING_BENCH
    return static_cast<uint8_t>(f410_bench_inputs.hall_position);
#else
    static constexpr uint8_t positions[8] = {0xFF, 0, 2, 1, 4, 5, 3, 0xFF};
    const uint32_t a = GPIOA->IDR;
    const uint32_t b = GPIOB->IDR;
    const uint32_t code = ((a >> 15) & 1) | (((b >> 3) & 1) << 1) | (((b >> 10) & 1) << 2);
    return positions[code];
#endif
}
} // namespace libecu
