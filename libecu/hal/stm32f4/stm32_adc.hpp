/** @file stm32_adc.hpp
 * @brief F410 single-ADC injected phase scan and asynchronous regular telemetry
 */
#ifndef LIBECU_STM32F4_ADC_HPP
#define LIBECU_STM32F4_ADC_HPP
#include "stm32f4xx_hal.h"
#include "../../include/adc_interface.hpp"

namespace libecu {
class Stm32Adc final : public AdcInterface {
public:
    /// @brief Shared calibration scale: one original12-bit conversion per phase.
    static constexpr uint16_t INJECTED_SAMPLE_SUM = 1;
    // Previous-scan publication at 1 kHz gives 1..2 ms acquisition age. Allow
    // 500 us scheduling jitter, not a fresh timestamp when the DMA is collected.
    static constexpr uint32_t REGULAR_PERIOD_US = 1000;
    static constexpr uint32_t REGULAR_MAX_AGE_US = 2 * REGULAR_PERIOD_US + 500;
    /** @brief Configure ADC1 and arm TIM1_TRGO injected conversions. */
    bool initializeHardware() noexcept;
    /** @brief Publish one complete three-phase12-bit scan per PWM-triggered JEOC. */
    void captureInjected() noexcept;
    /** @brief SysTick-only: collect a complete Vbus/NTC/pot DMA scan and start the next.
     *  @return false on a latched DMA/ADC error or a scan missing its next 1 ms tick.
     */
    bool serviceRegular(uint32_t now_us) noexcept;
    /** @brief A complete scan has been published and acquisition has not failed. */
    bool regularReady() const noexcept { return regular_ready_ && !regular_failed_; }
    /** @brief Bound age from scan launch, including time waiting for the next SysTick. */
    bool regularFresh(uint32_t now_us) const noexcept {
        return regularReady() && uint32_t(now_us - regularFrame().started_us) <= REGULAR_MAX_AGE_US;
    }
    /** @brief Check the live biased channels before accepting zero calibration. */
    bool offsetsPlausible() const noexcept;
    uint32_t getRawAdcValue(PwmChannel phase) override;
    uint32_t getRawAdcValue() override;
    uint32_t getRawTemperatureAdcValue() override { return regularFrame().temperature; }
    uint32_t readRawPotentiometer() const noexcept { return regularFrame().potentiometer; }
    uint32_t samples() const noexcept { return samples_; }
private:
    ADC_HandleTypeDef adc_{};
    DMA_HandleTypeDef dma_{};
    volatile uint32_t injected_[3]{};
    struct RegularFrame {
        uint32_t bus = 0;
        uint32_t temperature = 0;
        uint32_t potentiometer = 0;
        uint32_t started_us = 0;
    };
    const RegularFrame& regularFrame() const noexcept {
        return regular_frames_[__atomic_load_n(&regular_index_, __ATOMIC_ACQUIRE)];
    }
    // DMA never writes the published cache. SysTick fills the inactive cache
    // before one atomic index change; a preempting current IRQ never spins.
    alignas(4) volatile uint16_t regular_dma_[3]{};
    RegularFrame regular_frames_[2]{};
    uint32_t regular_index_ = 0;
    uint32_t regular_started_us_ = 0;
    bool regular_pending_ = false;
    volatile bool regular_ready_ = false;
    volatile bool regular_failed_ = false;
    volatile uint32_t samples_ = 0;
};
} // namespace libecu
#endif
