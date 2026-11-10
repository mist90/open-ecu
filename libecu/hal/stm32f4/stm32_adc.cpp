/** @file stm32_adc.cpp
 * @brief Fixed IHM08M1 channels: PA0,PC1,PC0,PA1; PC2 NTC and PA4 pot
 */
#include "stm32_adc.hpp"
#include "board_config.hpp"
#include "pwm_timing_bench.hpp"
#include "critical_section.hpp"
namespace libecu {
bool Stm32Adc::initializeHardware() noexcept {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &gpio);

    adc_.Instance = ADC1;
    adc_.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 100 / 4 = 25 MHz (<36 MHz)
    adc_.Init.Resolution = ADC_RESOLUTION_12B;
    adc_.Init.ScanConvMode = ENABLE;
    adc_.Init.ContinuousConvMode = DISABLE;
    adc_.Init.DiscontinuousConvMode = DISABLE;
    adc_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc_.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    adc_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_.Init.NbrOfConversion = 3;
    adc_.Init.DMAContinuousRequests = ENABLE;
    adc_.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&adc_) != HAL_OK) return false;

    ADC_InjectionConfTypeDef injected{};
    injected.InjectedNbrOfConversion = 3;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
    injected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
    const uint32_t channels[3] = {ADC_CHANNEL_0, ADC_CHANNEL_11, ADC_CHANNEL_10};
    for (unsigned rank = 0; rank < 3; ++rank) {
        injected.InjectedRank = rank + 1;
        injected.InjectedChannel = channels[rank];
        if (HAL_ADCEx_InjectedConfigChannel(&adc_, &injected) != HAL_OK) return false;
    }

    ADC_ChannelConfTypeDef regular{};
    regular.SamplingTime = ADC_SAMPLETIME_144CYCLES; // high-impedance dividers/pot
    regular.Rank = 1;
    regular.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&adc_, &regular) != HAL_OK) return false;
    regular.Rank = 2;
    regular.Channel = ADC_CHANNEL_12;
    if (HAL_ADC_ConfigChannel(&adc_, &regular) != HAL_OK) return false;
    regular.Rank = 3;
    regular.Channel = ADC_CHANNEL_4;
    if (HAL_ADC_ConfigChannel(&adc_, &regular) != HAL_OK) return false;

    dma_.Instance = DMA2_Stream0;
    dma_.Init.Channel = DMA_CHANNEL_0;
    dma_.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma_.Init.MemInc = DMA_MINC_ENABLE;
    dma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma_.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_.Init.Mode = DMA_CIRCULAR;
    dma_.Init.Priority = DMA_PRIORITY_HIGH;
    dma_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&dma_) != HAL_OK) return false;
    __HAL_LINKDMA(&adc_, DMA_Handle, dma_);
    // Arm DMA once. Only SysTick starts the software-triggered single scans;
    // circular DMA reloads its three slots but cannot initiate an ADC scan.
    if (HAL_DMA_Start(&dma_, uint32_t(uintptr_t(&ADC1->DR)),
                      uint32_t(uintptr_t(regular_dma_)), 3) != HAL_OK)
        return false;
    SET_BIT(ADC1->CR2, ADC_CR2_DMA);
    __HAL_ADC_ENABLE_IT(&adc_, ADC_IT_OVR);
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    return HAL_ADCEx_InjectedStart_IT(&adc_) == HAL_OK;
}

bool Stm32Adc::captureInjected(uint32_t irq_started_cycles) noexcept {
#if STM32F4_ADC_SOFTWARE_X2
    latest_[0] = ADC1->JDR1;
    latest_[1] = ADC1->JDR2;
    latest_[2] = ADC1->JDR3;
    if (second_pending_) {
        second_pending_ = false;
        for (unsigned i = 0; i < 3; ++i) injected_[i] = first_[i] + latest_[i];
        MODIFY_REG(ADC1->CR2, ADC_CR2_JEXTEN, ADC_EXTERNALTRIGINJECCONVEDGE_FALLING);
    } else {
        frame_started_cycles_ = irq_started_cycles;
        const uint32_t period = TIM1->ARR;
        uint32_t highest = TIM1->CCR1;
        if (TIM1->CCR2 > highest) highest = TIM1->CCR2;
        if (TIM1->CCR3 > highest) highest = TIM1->CCR3;
        const uint32_t ceiling = period * 3 / 5;
        constexpr uint32_t scan_budget_ticks =
            uint64_t(ihm08::TIMER_CLOCK_HZ) * 4500 / 1000000000;
        // Scan A starts before the peak, where the previous duty can still be
        // active. Require a wide window for both the old and current PWM frame.
        // Scan B's final current aperture closes within 3 us of software start;
        // reserve 4.5 us for start instructions, ADC sync and aperture closure.
        const bool room = previous_period_ == period && previous_max_compare_ < ceiling &&
                          highest < ceiling && TIM1->PSC == 0 &&
                          (TIM1->CR1 & (TIM_CR1_CEN | TIM_CR1_DIR)) == (TIM_CR1_CEN | TIM_CR1_DIR) &&
                          TIM1->CNT > highest + scan_budget_ticks;
        previous_max_compare_ = highest;
        previous_period_ = period;
        if (room) {
            for (unsigned i = 0; i < 3; ++i) first_[i] = latest_[i];
            second_pending_ = true;
            CLEAR_BIT(ADC1->CR2, ADC_CR2_JEXTEN);
            SET_BIT(ADC1->CR2, ADC_CR2_JSWSTART);
            return false;
        }
        // Fallback retains the same current sum scale, without discarding
        // half-LSB information in paired scans.
        for (unsigned i = 0; i < 3; ++i) injected_[i] = INJECTED_SAMPLE_SUM * latest_[i];
    }
#else
    frame_started_cycles_ = irq_started_cycles;
    injected_[0] = ADC1->JDR1;
    injected_[1] = ADC1->JDR2;
    injected_[2] = ADC1->JDR3;
#endif
    ++samples_;
    return true;
}
uint32_t Stm32Adc::getRawProtectionCurrent(PwmChannel phase) const noexcept {
    const unsigned index = static_cast<unsigned>(phase);
#ifdef PWM_TIMING_BENCH
    return index < 3 ? INJECTED_SAMPLE_SUM * f410_bench_inputs.phase_adc[index] : 0;
#elif STM32F4_ADC_SOFTWARE_X2
    return index < 3 ? INJECTED_SAMPLE_SUM * latest_[index] : 0;
#else
    return index < 3 ? injected_[index] : 0;
#endif
}
bool Stm32Adc::serviceRegular(uint32_t now_us) noexcept {
    constexpr uint32_t dma_errors = DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0;
    const uint32_t dma_flags = DMA2->LISR;
    if ((dma_flags & dma_errors) || (ADC1->SR & ADC_SR_OVR) ||
        !(DMA2_Stream0->CR & DMA_SxCR_EN)) regular_failed_ = true;
    if (regular_failed_) return false;

    if (regular_pending_) {
        if (!(dma_flags & DMA_LISR_TCIF0)) {
            // This is the next scheduled SysTick, even if IRQ jitter makes its
            // timestamp delta slightly less than 1 ms. A scan normally takes
            // tens of microseconds; missing this cycle is a fault, not a wait.
            regular_failed_ = true;
            return false;
        }
        if (DMA2_Stream0->NDTR != 3 ||
            uint32_t(now_us - regular_started_us_) > REGULAR_MAX_AGE_US) {
            regular_failed_ = true;
            return false;
        }
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const uint32_t next = regular_index_ ^ 1U;
        regular_frames_[next] = {regular_dma_[0], regular_dma_[1], regular_dma_[2], regular_started_us_};
        __atomic_store_n(&regular_index_, next, __ATOMIC_RELEASE);
        regular_ready_ = true;
    }

    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
    regular_started_us_ = now_us;
    regular_pending_ = true;
    // The injected IRQ also writes CR2 (JEXTEN/JSWSTART for x2). Protect this
    // single RMW, not the scan, against restoring a stale injected trigger.
    CriticalSection lock;
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
    return true;
}
uint32_t Stm32Adc::getRawAdcValue(PwmChannel phase) {
    const unsigned index = static_cast<unsigned>(phase);
#ifdef PWM_TIMING_BENCH
    return index < 3 ? INJECTED_SAMPLE_SUM * f410_bench_inputs.phase_adc[index] : 0;
#else
    return index < 3 ? injected_[index] : 0;
#endif
}
uint32_t Stm32Adc::getRawAdcValue() {
#ifdef PWM_TIMING_BENCH
    return INJECTED_SAMPLE_SUM * f410_bench_inputs.bus_adc;
#else
    return INJECTED_SAMPLE_SUM * regularFrame().bus;
#endif
}
bool Stm32Adc::offsetsPlausible() const noexcept {
#ifdef PWM_TIMING_BENCH
    return samples_ != 0; // Only synthetic offsets are calibrated in this build.
#else
    // Expected ~1.71 V / 2122 counts. Reject an unpowered op-amp or missing FOC bias.
    for (unsigned i = 0; i < 3; ++i) {
        if (injected_[i] < INJECTED_SAMPLE_SUM * 1600 || injected_[i] > INJECTED_SAMPLE_SUM * 2600) return false;
    }
    return true;
#endif
}
} // namespace libecu
