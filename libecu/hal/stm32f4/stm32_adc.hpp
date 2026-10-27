/** @file stm32_adc.hpp
 * @brief F410 single-ADC injected phase scan and asynchronous regular telemetry
 */
#ifndef LIBECU_STM32F4_ADC_HPP
#define LIBECU_STM32F4_ADC_HPP
#include "stm32f4xx_hal.h"
#include "../../include/adc_interface.hpp"

#ifndef STM32F4_ADC_SOFTWARE_X2
#define STM32F4_ADC_SOFTWARE_X2 1
#endif
#if STM32F4_ADC_SOFTWARE_X2 != 0 && STM32F4_ADC_SOFTWARE_X2 != 1
#error "STM32F4_ADC_SOFTWARE_X2 must be 0 or 1"
#endif

namespace libecu {
class Stm32Adc final : public AdcInterface {
public:
    /// @brief Units returned for injected current/Vbus data, including single-scan fallback.
    static constexpr uint16_t INJECTED_SAMPLE_SUM = STM32F4_ADC_SOFTWARE_X2 ? 2 : 1;
    /** @brief Configure ADC1 and arm TIM1_TRGO injected conversions. */
    bool initializeHardware() noexcept;
    /**
     * @brief Consume one JEOC; return true only when a complete PWM frame is published.
     * @param irq_started_cycles Timestamp taken at interrupt entry, for whole-frame timing.
     * @note A first scan may start a second conversion and return false without waiting.
     */
    bool captureInjected(uint32_t irq_started_cycles) noexcept;
    /** @brief Entry timestamp of the first JEOC belonging to the current control frame. */
    uint32_t frameStartedCycles() const noexcept { return frame_started_cycles_; }
    /** @brief Latest unaveraged conversion, in injected-sum units, for fast protection. */
    uint32_t getRawProtectionCurrent(PwmChannel phase) const noexcept;
    /** @brief Start NTC/pot regular DMA scan; completion never blocks an ISR. */
    void startRegular() noexcept;
    /** @brief Check the live biased channels before accepting zero calibration. */
    bool offsetsPlausible() const noexcept;
    uint32_t getRawAdcValue(PwmChannel phase) override;
    uint32_t getRawAdcValue() override;
    uint32_t getRawTemperatureAdcValue() override { return regular_[0]; }
    uint32_t readRawPotentiometer() const noexcept { return regular_[1]; }
    uint32_t samples() const noexcept { return samples_; }
    void dmaInterrupt() noexcept { HAL_DMA_IRQHandler(&dma_); }
private:
    ADC_HandleTypeDef adc_{};
    DMA_HandleTypeDef dma_{};
    volatile uint32_t injected_[4]{};
    uint32_t frame_started_cycles_ = 0;
#if STM32F4_ADC_SOFTWARE_X2
    uint32_t first_[4]{};
    uint32_t latest_[4]{};
    uint32_t previous_max_compare_ = 0;
    uint32_t previous_period_ = 0;
    bool second_pending_ = false;
#endif
    alignas(4) volatile uint16_t regular_[2]{};
    volatile uint32_t samples_ = 0;
};
} // namespace libecu
#endif
