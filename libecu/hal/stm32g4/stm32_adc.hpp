/** @file stm32_adc.hpp
 * @brief G431 independent ADC current capture with one-carrier auxiliary pipeline
 */
#ifndef LIBECU_STM32_ADC_HPP
#define LIBECU_STM32_ADC_HPP
#include "../../include/adc_interface.hpp"

#ifndef STM32G4_ADC_X2
#define STM32G4_ADC_X2 1
#endif
#if STM32G4_ADC_X2 != 0 && STM32G4_ADC_X2 != 1
#error "STM32G4_ADC_X2 must be 0 or 1"
#endif

namespace libecu {
class Stm32Adc final : public AdcInterface {
public:
    // Legacy calibration name: software sums of individual DMA conversions.
    static constexpr uint16_t INJECTED_SAMPLE_SUM = STM32G4_ADC_X2 ? 2 : 1;
    static constexpr unsigned CURRENT_RANKS = 2 * INJECTED_SAMPLE_SUM;
    static constexpr unsigned ADC1_RANKS = INJECTED_SAMPLE_SUM + 3;
    static constexpr unsigned ADC1_DMA_LENGTH = 2 * ADC1_RANKS;
    static constexpr uint32_t FRAME_PERIOD_US = 1000000 / ActiveTiming::CURRENT_HZ;
    static constexpr uint32_t REGULAR_MAX_AGE_US = 2 * FRAME_PERIOD_US + 5;
    bool initializeHardware() noexcept;
    bool beginFrames(uint32_t now_us) noexcept;
    /** @brief ADC2 TC: current U/V/W plus the previous complete ADC1 auxiliary bank. */
    bool captureFrame(uint32_t now_us) noexcept;
    bool rearmFrame(uint32_t now_us) noexcept;
    void failFrame() noexcept;
    /** @brief Every physical current sample at calibrated sum scale, before averaging. */
    uint32_t getRawProtectionCurrent(PwmChannel phase, unsigned scan) const noexcept;
    /** @brief SysTick-only coherent NTC/throttle snapshot; never launches ADC. */
    int serviceRegular(uint32_t now_us) noexcept; // -1=failure, 0=startup pending, 1=snapshot
    bool regularReady() const noexcept { return auxiliary_ready_ && !failed_; }
    bool regularFresh(uint32_t now_us) const noexcept {
        return regularReady() && int32_t(now_us - frame_started_us_) <= int32_t(REGULAR_MAX_AGE_US) &&
               int32_t(now_us - latest_.started_us) <= int32_t(REGULAR_MAX_AGE_US);
    }
    uint32_t frameStartedUs() const noexcept { return frame_started_us_; }
    uint32_t auxiliaryStartedUs() const noexcept { return latest_.started_us; }
    uint32_t samples() const noexcept { return samples_; }
    uint32_t getRawAdcValue(PwmChannel channel) override;
    uint32_t getRawAdcValue() override;
    uint32_t getRawTemperatureAdcValue() override { return slow_.temperature; }
    uint32_t readRawPotentiometer() const noexcept { return slow_.potentiometer; }
    float readPotentiometer(float max_value);
private:
    struct Auxiliary { uint32_t bus = 0, temperature = 0, potentiometer = 0, started_us = 0; };
    bool nextTrigger(uint32_t now_us) noexcept;
    // DMA1 CH1 keeps running while CPU copies. One full bank per PWM source;
    // current U and previous complete auxiliaries have distinct bank lifetimes.
    alignas(4) volatile uint16_t adc1_dma_[ADC1_DMA_LENGTH]{};
    alignas(4) volatile uint16_t adc2_dma_[CURRENT_RANKS]{};
    uint16_t raw_[3 * INJECTED_SAMPLE_SUM]{};
    uint32_t phase_sum_[3]{};
    Auxiliary latest_{}, slow_{};
    uint32_t expected_trigger_us_ = 0;
    volatile uint32_t frame_started_us_ = 0;
    uint32_t slow_generation_ = 0;
    bool armed_ = false;
    bool have_frame_ = false;
    bool auxiliary_ready_ = false;
    volatile bool failed_ = false;
    volatile uint32_t samples_ = 0;
    void initADC1() noexcept;
    void initADC2() noexcept;
    void initOPAMP1() noexcept;
    void initOPAMP2() noexcept;
    void initOPAMP3() noexcept;
};
} // namespace libecu
#endif
