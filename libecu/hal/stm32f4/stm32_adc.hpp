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
    /** @brief Configure ADC1 and arm TIM1_TRGO injected conversions. */
    bool initializeHardware() noexcept;
    /** @brief Snapshot a completed U,V,W,Vbus sequence before calling the control loop. */
    void captureInjected() noexcept;
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
    alignas(4) volatile uint16_t regular_[2]{};
    volatile uint32_t samples_ = 0;
};
} // namespace libecu
#endif
