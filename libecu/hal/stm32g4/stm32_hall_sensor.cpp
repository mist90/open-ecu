/**
 * @file stm32_hall_sensor.cpp
 * @brief STM32G4 GPIO-based Hall sensor implementation
 */

#include "stm32_hall_sensor.hpp"
#include "../../Core/Inc/main.h"

namespace libecu {

// Hall state to motor position lookup table
const uint8_t Stm32HallSensor::POSITION_TABLE[8] = {
    0xFF,    // 000
    0, // 001
    2, // 010
    1, // 011
    4, // 100
    5, // 101
    3, // 110
    0xFF     // 111
};

Stm32HallSensor::Stm32HallSensor(const HallGpioConfig& config) noexcept
    : config_(config) {
}

bool Stm32HallSensor::initialize() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pins : A__Pin B__Pin Z__Pin */
    GPIO_InitStruct.Pin = config_.hall_a_pin | config_.hall_b_pin | config_.hall_c_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(static_cast<GPIO_TypeDef*>(config_.gpio_port), &GPIO_InitStruct);

    return true;
}

uint8_t Stm32HallSensor::getPosition() {
    uint8_t state;

    state = readGpioPin(config_.gpio_port, config_.hall_a_pin) & 0x1;
    state |= (readGpioPin(config_.gpio_port, config_.hall_b_pin) & 0x1) << 1;
    state |= (readGpioPin(config_.gpio_port, config_.hall_c_pin) & 0x1) << 2;

    return POSITION_TABLE[state & 0x07];
}

bool Stm32HallSensor::readGpioPin(void* port, uint16_t pin) noexcept {
    return HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef*>(port), pin) == GPIO_PIN_SET;
}

} // namespace libecu