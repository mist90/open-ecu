/**
 * @file stm32_hall_sensor.cpp
 * @brief STM32G4 TIM4 Hall Sensor Interface implementation
 */

#include "stm32_hall_sensor.hpp"
#include "../../Core/Inc/main.h"
#include <cstdint>

// Global TIM4 handle (shared with IRQ handler via extern in stm32g4xx_it.c)
extern TIM_HandleTypeDef htim4;

namespace libecu {

// Hall state to motor position lookup table
const uint8_t Stm32TimHallSensor::POSITION_TABLE[8] = {
    0xFF,    // 000 - invalid
    0,       // 001
    2,       // 010
    1,       // 011
    4,       // 100
    5,       // 101
    3,       // 110
    0xFF     // 111 - invalid
};

Stm32TimHallSensor::Stm32TimHallSensor(const HallGpioConfig& config) noexcept
    : config_(config)
{
}

bool Stm32TimHallSensor::initialize() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

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

    htim4 = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_HallSensor_InitTypeDef hall_config = {0};
    hall_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    hall_config.IC1Prescaler = TIM_ICPSC_DIV1;
    hall_config.IC1Filter = 0x0F;  // FDIV32_N8: ~1.5μs hardware debounce at fDTS=170MHz
    hall_config.Commutation_Delay = 0;

    if (HAL_TIMEx_HallSensor_Init(&htim4, &hall_config) != HAL_OK) {
        return false;
    }

    /* TIM4 interrupt - priority 1 (Hall sensor) */
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    if (HAL_TIMEx_HallSensor_Start_IT(&htim4) != HAL_OK) {
        return false;
    }

    return true;
}

uint8_t Stm32TimHallSensor::getPosition() {
    uint8_t state = 0;

    // Pin mapping: A=PB6 (bit 0), B=PB7 (bit 1), C=PB8 (bit 2)
    if (GPIOB->IDR & A__Pin)       state |= (1 << 0);
    if (GPIOB->IDR & B__Pin)       state |= (1 << 1);
    if (GPIOB->IDR & Z__Pin)       state |= (1 << 2);

    return POSITION_TABLE[state & 0x07];
}

} // namespace libecu
