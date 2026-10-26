/** @file stm32_pwm.hpp
 * @brief Fixed TIM1 / active-low LIN driver for NUCLEO-F410RB + IHM08M1
 */
#ifndef LIBECU_STM32F4_PWM_HPP
#define LIBECU_STM32F4_PWM_HPP
#include "stm32f4xx_hal.h"
#include "../../include/pwm_interface.hpp"

namespace libecu {
class Stm32Pwm final : public PwmInterface {
public:
    bool initialize(uint32_t frequency, uint16_t dead_time_ns) override;
    void setChannelState(PwmChannel channel, PwmState state) override;
    void updateDutyCycle(float u, float v, float w) override;
    void updateNativeDutyCycle(NativeDuty u, NativeDuty v, NativeDuty w) override;
    void enable(bool enabled) override;
    void apply() override;
    /** @brief Start the ADC trigger timebase with the bridge still disabled. */
    bool startTimebase() noexcept;
    /** @brief Latch a hardware/software trip; reset is required to re-arm. */
    void trip() noexcept;
    bool faulted() const noexcept { return faulted_; }
    uint16_t getActualDeadTimeNs() const noexcept { return actual_dead_time_ns_; }
    /** @brief Drive HIN=0, LIN=1 before configuring TIM1 (also used on fatal errors). */
    static void parkPins() noexcept;
private:
    TIM_HandleTypeDef timer_{};
    uint32_t period_ = 0;
    uint16_t actual_dead_time_ns_ = 0;
    volatile bool faulted_ = false;
};
} // namespace libecu
#endif
