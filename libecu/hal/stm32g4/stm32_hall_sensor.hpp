/**
 * @file stm32_hall_sensor.hpp
 * @brief STM32G4 TIM4 Hall Sensor Interface implementation
 */

#ifndef LIBECU_STM32_HALL_SENSOR_HPP
#define LIBECU_STM32_HALL_SENSOR_HPP

#include "../../include/interfaces/hall_interface.hpp"
#include "stm32g4xx_hal.h"

namespace libecu {

/**
 * @brief GPIO pin configuration for Hall sensor
 */
struct HallGpioConfig {
    void* gpio_port;     ///< GPIO port (GPIOB for this project)
    uint16_t hall_a_pin; ///< Hall A pin (PB6)
    uint16_t hall_b_pin; ///< Hall B pin (PB7)
    uint16_t hall_c_pin; ///< Hall C pin (PB8)
};

/**
 * @brief STM32G4 Hall sensor implementation using TIM4
 */
class Stm32TimHallSensor : public HallInterface {
public:
    /**
     * @brief Constructor
     * @param config GPIO configuration
     */
    explicit Stm32TimHallSensor(const HallGpioConfig& config) noexcept;

    // HallInterface implementation
    bool initialize() override;
    uint8_t getPosition() override;

private:
    HallGpioConfig config_;

    // Hall state to motor position lookup table
    static const uint8_t POSITION_TABLE[8];
};

} // namespace libecu

#endif // LIBECU_STM32_HALL_SENSOR_HPP