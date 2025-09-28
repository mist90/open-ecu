#ifndef STM32_PWM_HPP
#define STM32_PWM_HPP

#include "../../interfaces/pwm_interface.hpp"
#include <cstdint>

namespace libecu {

class STM32PWM : public PwmInterface {
public:
    STM32PWM();
    ~STM32PWM() override;
    
    bool initialize(uint32_t frequency) override;
    void setDutyCycle(PwmChannel channel, float duty_cycle) override;
    void setState(PwmChannel channel, PwmState state) override;
    void enable(bool enable) override;
    void emergencyStop() override;
    uint32_t getFrequency() const override;

private:
    void* timer_handle_;
    uint32_t pwm_frequency_;
    uint32_t timer_period_;
    uint32_t timer_prescaler_;
    uint32_t max_duty_cycle_;
    
    void calculateTimerSettings();
    uint32_t getTimerChannel(PwmChannel channel) const;
};

}

#endif
