#ifndef STM32_HALL_SENSOR_HPP
#define STM32_HALL_SENSOR_HPP

#include "../../interfaces/hall_interface.hpp"
#include <cstdint>

namespace libecu {

class STM32HallSensor : public HallInterface {
public:
    STM32HallSensor();
    ~STM32HallSensor() override;
    
    bool initialize() override;
    HallState readState() override;
    MotorPosition getPosition(const HallState& state) override;
    bool isValidState(const HallState& state) override;

private:
    uint32_t a_pin_;
    uint32_t b_pin_;
    uint32_t z_pin_;
    void* a_port_;
    void* b_port_;
    void* z_port_;
    
    HallState last_valid_state_;
    uint32_t invalid_count_;
    
    bool readPin(void* port, uint32_t pin) const;
};

}

#endif