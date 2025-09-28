#include "../include/hal/stm32g4/stm32_hall_sensor.hpp"

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

STM32HallSensor::STM32HallSensor() 
    : a_pin_(GPIO_PIN_6), b_pin_(GPIO_PIN_7), z_pin_(GPIO_PIN_8),
      a_port_(GPIOB), b_port_(GPIOB), z_port_(GPIOB),
      last_valid_state_{false, false, false}, invalid_count_(0) {
}

STM32HallSensor::~STM32HallSensor() {
}

bool STM32HallSensor::initialize() {
    return true;
}

HallState STM32HallSensor::readState() {
    HallState state;
    
    state.hall_a = readPin(a_port_, a_pin_);
    state.hall_b = readPin(b_port_, b_pin_);
    state.hall_c = readPin(z_port_, z_pin_);
    
    if (isValidState(state)) {
        last_valid_state_ = state;
        invalid_count_ = 0;
    } else {
        invalid_count_++;
        if (invalid_count_ < 5) {
            state = last_valid_state_;
        }
    }
    
    return state;
}

MotorPosition STM32HallSensor::getPosition(const HallState& state) {
    uint8_t hall_value = state.getValue();
    
    switch (hall_value) {
        case 1: return MotorPosition::POSITION_1;  // 001
        case 2: return MotorPosition::POSITION_2;  // 010
        case 3: return MotorPosition::POSITION_3;  // 011
        case 4: return MotorPosition::POSITION_4;  // 100
        case 5: return MotorPosition::POSITION_5;  // 101
        case 6: return MotorPosition::POSITION_6;  // 110
        default: return MotorPosition::INVALID;
    }
}

bool STM32HallSensor::isValidState(const HallState& state) {
    uint8_t hall_value = state.getValue();
    return (hall_value >= 1 && hall_value <= 6);
}

bool STM32HallSensor::readPin(void* port, uint32_t pin) const {
#ifdef STM32G4
    return HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef*>(port), pin) == GPIO_PIN_SET;
#else
    GPIO_TypeDef* gpio = static_cast<GPIO_TypeDef*>(port);
    return (gpio->pins & pin) != 0;
#endif
}

}