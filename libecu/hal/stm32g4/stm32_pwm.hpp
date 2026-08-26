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
    explicit Stm32Pwm(void* htim) noexcept;

    // PwmInterface implementation
    bool initialize(uint32_t frequency, uint16_t dead_time_ns) override;
    void setChannelState(PwmChannel channel, PwmState state) override;
    void updateDutyCycle(float duty_u, float duty_v, float duty_w) override;
    void enable(bool enable) override;
    void apply() override;

    /**
     * @brief Dead time actually programmed into DTG, in nanoseconds
     *
     * Not the requested value: DTG is quantised in tDTS units (5.882 ns at
     * 170 MHz) and coarsens to steps of 2, 8 or 16 tDTS in the upper ranges, so
     * the hardware can only ever approximate a request. Worth printing at boot
     * - the requested figure was wrong by 18 % for years without anyone
     * noticing, because nothing ever reported back what was applied.
     *
     * @return Programmed dead time (ns), or 0 before initialize()
     */
    uint16_t getActualDeadTimeNs() const noexcept { return actual_dead_time_ns_; }

private:
    void* htim_;
    uint32_t period_;
    uint16_t dead_time_ns_;
    uint16_t actual_dead_time_ns_;   ///< What DTG really encodes; see the getter
    bool enabled_;

    /**
     * @brief Convert PWM channel to TIM channel
     * @param channel PWM channel
     * @return TIM channel constant
     */
    uint32_t getTimChannel(PwmChannel channel) noexcept;

    /**
     * @brief Calculate duty cycle value from percentage
     * @param duty_cycle Duty cycle (0.0 to 1.0)
     * @return Timer compare value
     */
    uint32_t calculateCompareValue(float duty_cycle) noexcept;
};

} // namespace libecu

#endif // LIBECU_STM32_PWM_HPP
