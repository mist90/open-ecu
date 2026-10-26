/** @file stm32_hall_sensor.hpp
 * @brief IHM08M1 Hall signals jumpered to PB12/PC10/PC11 and filtered by TIM5
 */
#ifndef LIBECU_STM32F4_HALL_SENSOR_HPP
#define LIBECU_STM32F4_HALL_SENSOR_HPP
#include "stm32f4xx_hal.h"
#include "../../include/hall_interface.hpp"
namespace libecu {
class Stm32HallSensor final : public HallInterface {
public:
    /** @brief Share the board's free-running TIM5 microsecond timebase. */
    explicit Stm32HallSensor(TIM_HandleTypeDef& timer) noexcept : timer_(timer) {}
    bool initialize() override;
    uint8_t getPosition() override;
private:
    TIM_HandleTypeDef& timer_;
};
} // namespace libecu
#endif
