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
    void setChannelState(PwmChannel channel, PwmState state, float duty_cycle = 0.0f) override;
    void updateDutyCycle(PwmChannel channel, float duty_cycle = 0.0f) override;
    void enable(bool enable) override;
    void apply() override;

private:
    void* htim_;
    uint32_t period_;
    uint16_t dead_time_ns_;
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

    /**
     * @brief Calculate ADC trigger compare value (CCR4) that never collapses to zero
     *
     * The ADC is triggered by TIM1 TRGO2 = OC4REF (PWM1 mode). In PWM1,
     * OC4REF is high while CNT < CCR4. If CCR4 = 0, OC4REF is permanently
     * low (RM0440 §29.3.13: "If the compare value is zero then tim_ocxref
     * is held at 0"), so the ADC trigger disappears, the ADC stops
     * converting, and telemetry freezes.
     *
     * This method floors CCR4 to period/8 when compare_value/2 would be
     * zero, guaranteeing an OC4REF rising edge every PWM period regardless
     * of duty cycle. At duty=0 (mode switch, PID reset) this breaks the
     * deadlock that would otherwise freeze telemetry and stall commutation.
     *
     * @param compare_value The PWM compare value (duty * period) for the active phase
     * @param period The timer period (ARR value)
     * @return CCR4 value that is always > 0
     */
    static constexpr uint32_t calculateAdcTriggerCompare(uint32_t compare_value, uint32_t period) noexcept {
        uint32_t ccr4 = compare_value / 2;
        return (ccr4 != 0) ? ccr4 : (period / 8);
    }
};

} // namespace libecu

#endif // LIBECU_STM32_PWM_HPP
