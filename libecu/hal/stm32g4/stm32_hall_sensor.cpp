/**
 * @file stm32_hall_sensor.cpp
 * @brief STM32G4 GPIO-based Hall sensor implementation
 */

#include "stm32_hall_sensor.hpp"

#ifdef STM32G4
#include "../../Core/Inc/main.h"
#else
#define GPIO_PIN_6   0x40
#define GPIO_PIN_7   0x80  
#define GPIO_PIN_8   0x100
typedef struct {
    uint32_t pins;
} GPIO_TypeDef;
extern GPIO_TypeDef GPIOB_Mock;
#define GPIOB (&GPIOB_Mock)
#endif

namespace libecu {

// Hall state to motor position lookup table
const MotorPosition Stm32HallSensor::POSITION_TABLE[8] = {
    MotorPosition::INVALID,    // 000
    MotorPosition::POSITION_4, // 001
    MotorPosition::POSITION_6, // 010
    MotorPosition::POSITION_5, // 011
    MotorPosition::POSITION_2, // 100
    MotorPosition::POSITION_3, // 101
    MotorPosition::POSITION_1, // 110
    MotorPosition::INVALID     // 111
};

Stm32HallSensor::Stm32HallSensor(const HallGpioConfig& config)
    : config_(config), last_state_{false, false, false}, state_change_count_(0) {
}

bool Stm32HallSensor::initialize() {
    return true;
}

HallState Stm32HallSensor::readState() {
    HallState state;
    
    state.hall_a = readGpioPin(config_.gpio_port, config_.hall_a_pin);
    state.hall_b = readGpioPin(config_.gpio_port, config_.hall_b_pin);
    state.hall_c = readGpioPin(config_.gpio_port, config_.hall_c_pin);
    
    // Track state changes
    if (state.getValue() != last_state_.getValue()) {
        state_change_count_++;
        last_state_ = state;
    }
    
    return state;
}

MotorPosition Stm32HallSensor::getPosition(const HallState& state) {
    uint8_t hall_value = state.getValue();
    return POSITION_TABLE[hall_value & 0x07];
}

bool Stm32HallSensor::isValidState(const HallState& state) {
    uint8_t hall_value = state.getValue();
    return (hall_value >= 1 && hall_value <= 6);
}

bool Stm32HallSensor::readGpioPin(void* port, uint16_t pin) {
#ifdef STM32G4
    return HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef*>(port), pin) == GPIO_PIN_SET;
#else
    GPIO_TypeDef* gpio = static_cast<GPIO_TypeDef*>(port);
    return (gpio->pins & pin) != 0;
#endif
}

} // namespace libecu