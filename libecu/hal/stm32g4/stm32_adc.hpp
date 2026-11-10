/**
 * @file stm32_adc.hpp
 * @brief STM32G4 ADC implementation for 3-phase current sensing
 */

#ifndef LIBECU_STM32_ADC_HPP
#define LIBECU_STM32_ADC_HPP

#include "../../include/adc_interface.hpp"

namespace libecu {

/**
 * @brief Dual injected phase currents and SysTick-owned regular auxiliary DMA
 *
 * Reads 3 OPAMP currents via the dual injected group; ADC1 regular DMA holds auxiliaries.
 * Supports PWM-synchronized sampling via TIM1 TRGO trigger.
 */
class Stm32Adc : public AdcInterface {
public:
    /// @brief Scale of the unshifted two-sample injected ADC sums.
    static constexpr uint16_t INJECTED_SAMPLE_SUM = 2;
    // A previous-scan result is 1..2 ms old at 1 kHz; 500 us covers scheduling
    // jitter. Timestamp the launch, never collection of an older DMA result.
    static constexpr uint32_t REGULAR_PERIOD_US = 1000;
    static constexpr uint32_t REGULAR_MAX_AGE_US = 2 * REGULAR_PERIOD_US + 500;

    /**
     * @brief Constructor
     */
    explicit Stm32Adc() noexcept;

    /**
     * @brief Initialize ADC and OPAMP hardware
     *
     * Initializes ADC1, ADC2 in dual simultaneous injected mode and all 3 OPAMPs (PGA mode).
     * Configures ADC to be triggered by TIM1_TRGO2 for PWM-synchronized current sampling.
     *
     * @return true if initialization successful, false otherwise
     */
    bool initializeHardware() noexcept;

    /** @brief SysTick-only: publish the previous complete DMA scan and launch the next.
     *  @return false on a latched error or a scan missing its next 1 ms tick.
     */
    bool serviceRegular(uint32_t now_us) noexcept;
    /** @brief A full regular scan is available and acquisition has not failed. */
    bool regularReady() const noexcept { return regular_ready_ && !regular_failed_; }
    /** @brief Acquisition age includes time spent awaiting the collecting SysTick. */
    bool regularFresh(uint32_t now_us) const noexcept {
        return regularReady() && uint32_t(now_us - regularFrame().started_us) <= REGULAR_MAX_AGE_US;
    }

    // AdcInterface implementation
    uint32_t getRawAdcValue(PwmChannel channel) override;
    uint32_t getRawAdcValue() override;
    uint32_t getRawTemperatureAdcValue() override;

    /**
     * @brief Read potentiometer and convert to output
     * @param max_value Maximum speed corresponding to 3.3V
     * @return output (0 to max_value)
     */
    float readPotentiometer(float max_value);
    /** @brief Cached 12-bit throttle from the same DMA scan as Vbus and NTC. */
    uint32_t readRawPotentiometer();

private:
    struct RegularFrame {
        uint32_t bus = 0;
        uint32_t temperature = 0;
        uint32_t potentiometer = 0;
        uint32_t started_us = 0;
    };
    const RegularFrame& regularFrame() const noexcept {
        return regular_frames_[__atomic_load_n(&regular_index_, __ATOMIC_ACQUIRE)];
    }
    // DMA cannot overwrite either published cache. Readers preempting SysTick
    // see its old or new complete frame; no seqlock spin in the current IRQ.
    alignas(4) volatile uint16_t regular_dma_[3]{};
    RegularFrame regular_frames_[2]{};
    uint32_t regular_index_ = 0;
    uint32_t regular_started_us_ = 0;
    bool regular_pending_ = false;
    volatile bool regular_ready_ = false;
    volatile bool regular_failed_ = false;

    /**
     * @brief Initialize ADC1 peripheral
     * Configures ADC1 with injected channel for Phase U (OPAMP1)
     * and a fixed regular sequence for Vbus, NTC and potentiometer
     */
    void initADC1() noexcept;

    /**
     * @brief Initialize ADC2 peripheral
     * Configures ADC2 with injected channels for Phase V (OPAMP2) and Phase W (OPAMP3)
     */
    void initADC2() noexcept;

    /**
     * @brief Initialize OPAMP1 peripheral (Phase U current amplifier)
     */
    void initOPAMP1() noexcept;

    /**
     * @brief Initialize OPAMP2 peripheral (Phase V current amplifier)
     */
    void initOPAMP2() noexcept;

    /**
     * @brief Initialize OPAMP3 peripheral (Phase W current amplifier)
     */
    void initOPAMP3() noexcept;
};

} // namespace libecu

#endif // LIBECU_STM32_ADC_HPP
