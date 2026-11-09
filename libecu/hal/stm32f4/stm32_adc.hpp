/** @file stm32_adc.hpp
 * @brief F410 regular current DMA plus pipelined injected auxiliary acquisition
 */
#ifndef LIBECU_STM32F4_ADC_HPP
#define LIBECU_STM32F4_ADC_HPP
#include "stm32f4xx_hal.h"
#include "../../include/adc_interface.hpp"

// Compatibility option: x2 now means two physical triples in ONE regular scan.
#ifndef STM32F4_ADC_SOFTWARE_X2
#define STM32F4_ADC_SOFTWARE_X2 1
#endif
#if STM32F4_ADC_SOFTWARE_X2 != 0 && STM32F4_ADC_SOFTWARE_X2 != 1
#error "STM32F4_ADC_SOFTWARE_X2 must be 0 or 1"
#endif

namespace libecu {
class Stm32Adc final : public AdcInterface {
public:
    static constexpr uint16_t INJECTED_SAMPLE_SUM = STM32F4_ADC_SOFTWARE_X2 ? 2 : 1;
    static constexpr unsigned PHASE_RANKS = 3 * INJECTED_SAMPLE_SUM;
    static constexpr unsigned FRAME_RANKS = PHASE_RANKS;
    // ADC1=TIM1/4, 15 sampling +12 conversion clocks per rank (RM0401 §11.5).
    static constexpr uint32_t CURRENT_SCAN_TICKS = PHASE_RANKS * (15 + 12) * 4;
    static constexpr uint32_t FRAME_PERIOD_US = 1000000 / ActiveTiming::CURRENT_HZ;
    static constexpr uint32_t REGULAR_MAX_AGE_US = 2 * FRAME_PERIOD_US + 5;
    static constexpr uint32_t AUXILIARY_BUDGET_US = 21; // 18.72 us + ADC/start synchronization
    /** @brief Configure result DMA and the TIM1_CC4 -> SWSTART DMA bridge, timer stopped. */
    bool initializeHardware() noexcept;
    /** @brief Called immediately before starting TIM1; establish the first source deadline. */
    bool beginFrames(uint32_t now_us) noexcept;
    /** @brief Current TC: copy currents + previous JEOC, launch next injected auxiliaries. */
    bool captureFrame(uint32_t now_us) noexcept;
    /** @brief After protection/control, rearm both one-shot DMAs before the next trigger. */
    bool rearmFrame(uint32_t now_us) noexcept;
    /** @brief Latch acquisition failure and stop trigger requests; never wait in an ISR. */
    void failFrame() noexcept;
    /** @brief Qualify every consumed aperture against both PWM duty generations.
     *  x1/x2 may exclude one invalid leg ONLY when reconstructed_phase is the exact
     *  next FOC reconstruction choice and also the unique quantized CCR maximum.
     *  Both samples of each retained x2 leg must fit; -1 requires all three legs.
     */
    bool currentWindowValid(int reconstructed_phase = -1) const noexcept;
    /** @brief Union of invalid U/V/W aperture bits across every physical triple. */
    unsigned currentInvalidPhases() const noexcept;
    /** @brief Invalid U/V/W aperture bits for one physical triple; invalid scan returns all bits. */
    unsigned currentInvalidPhases(unsigned scan) const noexcept;
    /** @brief Packed physical-scan masks (bits0..2 and3..5), from one PWM snapshot. */
    unsigned currentInvalidScans() const noexcept;
    /** @brief Select one discarded phase per x2 triple, after qualifying its other two apertures.
     *  @return false for x1 or an index outside U/V/W (0/1/2), preserving existing choices.
     *  @note Caller owns the current IRQ and checks original raw protection first.
     *        Choices last until the next accepted frame; native feedback alone is reconstructed.
     */
    bool setCurrentReconstruction(unsigned first_phase, unsigned second_phase) noexcept {
        if (INJECTED_SAMPLE_SUM != 2 || first_phase > 2 || second_phase > 2) return false;
        reconstructed_phase_[0] = static_cast<uint8_t>(first_phase);
        reconstructed_phase_[1] = static_cast<uint8_t>(second_phase);
        return true;
    }
    /** @brief Physical scan sample, scaled only to match the calibrated current sum units. */
    uint32_t getRawProtectionCurrent(PwmChannel phase, unsigned scan) const noexcept;
    /** @brief SysTick-only bounded snapshot. Does not start ADC or touch acquisition registers. */
    int serviceRegular(uint32_t now_us) noexcept; // -1=failure, 0=first auxiliary pending, 1=snapshot
    bool regularReady() const noexcept { return auxiliary_ready_ && !failed_; }
    bool regularFresh(uint32_t now_us) const noexcept {
        return regularReady() && int32_t(now_us - frame_started_us_) <= int32_t(REGULAR_MAX_AGE_US) &&
               int32_t(now_us - latest_.started_us) <= int32_t(REGULAR_MAX_AGE_US);
    }
    uint32_t frameStartedUs() const noexcept { return frame_started_us_; }
    uint32_t auxiliaryStartedUs() const noexcept { return latest_.started_us; }
    bool offsetsPlausible() const noexcept;
    uint32_t getRawAdcValue(PwmChannel phase) override;
    void readNativeCurrents(NativeCurrent& u, NativeCurrent& v, NativeCurrent& w) noexcept override;
    uint32_t getRawAdcValue() override;
    uint32_t getRawTemperatureAdcValue() override { return slow_.temperature; }
    uint32_t readRawPotentiometer() const noexcept { return slow_.potentiometer; }
    uint32_t samples() const noexcept { return samples_; }
private:
    struct Auxiliary { uint32_t bus = 0, temperature = 0, potentiometer = 0, started_us = 0; };
    bool nextTrigger(uint32_t now_us) noexcept;
    ADC_HandleTypeDef adc_{};
    DMA_HandleTypeDef dma_{}, trigger_dma_{};
    alignas(4) volatile uint16_t dma_raw_[FRAME_RANKS]{};
    uint16_t raw_[FRAME_RANKS]{};
    uint32_t phase_sum_[3]{};
    // This word is immutable while trigger DMA is armed. No CPU RMW of CR2 races it.
    alignas(4) uint32_t trigger_word_ = 0;
    uint32_t auxiliary_word_ = 0;
    uint32_t auxiliary_launch_us_ = 0;
    uint32_t auxiliary_generation_ = 0;
    uint32_t expected_trigger_us_ = 0;
    volatile uint32_t frame_started_us_ = 0;
    uint32_t armed_compare_[3]{};
    uint32_t previous_compare_[3]{};
    uint32_t pwm_generation_ = 0;
    uint32_t slow_generation_ = 0;
    Auxiliary latest_{}, slow_{};
    bool armed_ = false;
    bool have_frame_ = false;
    bool auxiliary_pending_ = false;
    bool auxiliary_ready_ = false;
    volatile bool failed_ = false;
    bool pwm_window_exact_ = false;
    uint8_t reconstructed_phase_[2]{3, 3}; // Unselected: use the original phase sums.
    volatile uint32_t samples_ = 0;
};
} // namespace libecu
#endif
