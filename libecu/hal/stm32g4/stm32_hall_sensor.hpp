/**
 * @file stm32_hall_sensor.hpp
 * @brief STM32G4 GPIO-based Hall sensor implementation
 */

#ifndef LIBECU_STM32_HALL_SENSOR_HPP
#define LIBECU_STM32_HALL_SENSOR_HPP

#include "../../include/interfaces/hall_interface.hpp"

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
 * @brief STM32G4 Hall sensor implementation using GPIO
 */
class Stm32HallSensor : public HallInterface {
public:
    /**
     * @brief Constructor
     * @param config GPIO configuration
     */
    explicit Stm32HallSensor(const HallGpioConfig& config);

    // HallInterface implementation
    bool initialize() override;
    HallState readState() override;
    MotorPosition getPosition(const HallState& state) override;
    bool isValidState(const HallState& state) override;

    /**
     * @brief Get Hall state change count (for debugging)
     * @return Number of Hall state changes
     */
    uint32_t getStateChangeCount() const { return state_change_count_; }

private:
    HallGpioConfig config_;
    HallState last_state_;
    uint32_t state_change_count_;
    
    // Hall state to motor position lookup table
    static const MotorPosition POSITION_TABLE[8];
    
    /**
     * @brief Read GPIO pin state
     * @param port GPIO port
     * @param pin GPIO pin
     * @return Pin state (true/false)
     */
    bool readGpioPin(void* port, uint16_t pin);
};

} // namespace libecu

#endif // LIBECU_STM32_HALL_SENSOR_HPP