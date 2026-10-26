/** @file stm32_hall_sensor.hpp
 * @brief IHM08M1 Hall connector on PA15/PB3/PB10, acquired with EXTI
 */
#ifndef LIBECU_STM32F4_HALL_SENSOR_HPP
#define LIBECU_STM32F4_HALL_SENSOR_HPP
#include "../../include/hall_interface.hpp"
namespace libecu {
class Stm32HallSensor final : public HallInterface {
public:
    bool initialize() override;
    uint8_t getPosition() override;
};
} // namespace libecu
#endif
