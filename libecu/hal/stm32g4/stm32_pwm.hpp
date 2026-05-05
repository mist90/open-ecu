/**
 * @file stm32_pwm.hpp
 * @brief STM32G4 TIM1-based PWM implementation for 3-phase motor control
 */

#ifndef LIBECU_STM32_PWM_HPP
#define LIBECU_STM32_PWM_HPP

#include "../../include/interfaces/pwm_interface.hpp"

namespace libecu {

/**
 * @brief STM32G4 PWM implementation using TIM1
 */
class Stm32Pwm : public PwmInterface {
public:
    /**
     * @brief Constructor
     * @param htim Timer handle (TIM1) - pass as void* to avoid header dependencies
     */
    explicit Stm32Pwm(void* htim);

    // PwmInterface implementation
    bool initialize(uint32_t frequency, uint16_t dead_time_ns) override;
    void setChannelState(PwmChannel channel, PwmState state, float duty_cycle = 0.0f) override;
    void enable(bool enable) override;
    void emergencyStop() override;

private:
    void* htim_;
    uint32_t frequency_;
    uint32_t period_;
    uint16_t dead_time_ns_;
    bool enabled_;

    /**
     * @brief Convert PWM channel to TIM channel
     * @param channel PWM channel
     * @return TIM channel constant
     */
    uint32_t getTimChannel(PwmChannel channel);

    /**
     * @brief Calculate duty cycle value from percentage
     * @param duty_cycle Duty cycle (0.0 to 1.0)
     * @return Timer compare value
     */
    uint32_t calculateCompareValue(float duty_cycle);
};

} // namespace libecu

#endif // LIBECU_STM32_PWM_HPP