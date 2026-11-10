/** @file main.cpp
 * @brief Real STM32 auxiliary drivers against fake HAL setup and anonymous MMIO
 */
#include "main.h"
#include "stm32_adc.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <sys/mman.h>

using namespace libecu;
static int checks = 0, failures = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

static ADC_InitTypeDef adc_config[2]{};
static ADC_InjectionConfTypeDef injected_config[2][4]{};
static ADC_ChannelConfTypeDef regular_config[3]{};
static unsigned injected_count[2]{}, regular_count = 0, clock_configs = 0;
static DMA_InitTypeDef dma_config{};
static uint16_t* dma_buffer = nullptr;
static bool fail_dma_start = false;
static unsigned critical_depth = 0, critical_entries = 0;
static bool inject_before_start = false;
static Stm32Adc* adc = nullptr;
static bool observe_publication = false;
static uint32_t old_values[3]{}, new_values[3]{};
static unsigned before_publications = 0, after_publications = 0;
#if defined(STM32F410Rx)
using RegularConfigArgument = ADC_ChannelConfTypeDef;
using InjectedConfigArgument = ADC_InjectionConfTypeDef;
#else
using RegularConfigArgument = const ADC_ChannelConfTypeDef;
using InjectedConfigArgument = const ADC_InjectionConfTypeDef;
#endif

extern "C" void disable_interrupts() {
#if defined(STM32F410Rx)
    // The injected IRQ can run immediately before the short CR2 critical
    // section. Its new trigger state must survive the regular-start RMW.
    if (inject_before_start) {
        inject_before_start = false;
        CLEAR_BIT(ADC1->CR2, ADC_CR2_JEXTEN);
        SET_BIT(ADC1->CR2, ADC_CR2_JSWSTART);
    }
#endif
    ++critical_depth;
    ++critical_entries;
}
extern "C" void enable_interrupts() {
    check(critical_depth == 1, "regular start has one bounded critical section");
    --critical_depth;
}
uint32_t time_us() { return 0; }

static bool cachedValues(const uint32_t* expected) {
    return adc->getRawAdcValue() == expected[0] * Stm32Adc::INJECTED_SAMPLE_SUM &&
           adc->getRawTemperatureAdcValue() == expected[1] && adc->readRawPotentiometer() == expected[2];
}
extern "C" void before_regular_publication() {
    if (!observe_publication) return;
    ++before_publications;
    check(cachedValues(old_values), "preempting reader sees the previous complete cache while SysTick publishes");
}
extern "C" void after_regular_publication() {
    if (!observe_publication) return;
    ++after_publications;
    check(cachedValues(new_values), "one atomic publication exposes all three new channels together");
}

// HAL stubs configure the fake peripheral state but never manufacture an ADC
// completion. Test code below drives individual DMA ranks and status flags.
extern "C" {
void Error_Handler() { std::abort(); }
void HAL_GPIO_Init(GPIO_TypeDef*, GPIO_InitTypeDef*) {}
void HAL_NVIC_SetPriority(IRQn_Type, uint32_t, uint32_t) {}
void HAL_NVIC_EnableIRQ(IRQn_Type) {}
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* handle) {
    const unsigned index = handle->Instance == ADC1 ? 0 : 1;
    adc_config[index] = handle->Init;
#if defined(STM32F410Rx)
    handle->Instance->CR2 = ADC_CR2_DDS;
#endif
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*, RegularConfigArgument* config) {
    if (regular_count >= 3) return HAL_ERROR;
    regular_config[regular_count++] = *config;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* handle, InjectedConfigArgument* config) {
    const unsigned index = handle->Instance == ADC1 ? 0 : 1;
    if (injected_count[index] >= 4) return HAL_ERROR;
    injected_config[index][injected_count[index]++] = *config;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef* handle) {
    dma_config = handle->Init;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef* handle, uint32_t source, uint32_t destination, uint32_t length) {
    if (fail_dma_start) return HAL_ERROR;
    check(source == uintptr_t(&ADC1->DR) && length == 3, "DMA drains exactly one complete ADC1 DR sequence");
    dma_buffer = reinterpret_cast<uint16_t*>(uintptr_t(destination));
#if defined(STM32F410Rx)
    handle->Instance->CR = DMA_SxCR_EN | DMA_SxCR_CIRC;
    handle->Instance->NDTR = length;
#else
    handle->Instance->CCR = DMA_CCR_EN | DMA_CCR_CIRC;
    handle->Instance->CNDTR = length;
#endif
    return HAL_OK;
}
#if defined(STM32F410Rx)
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* handle) {
    SET_BIT(handle->Instance->CR2, ADC_CR2_ADON | ADC_EXTERNALTRIGINJECCONVEDGE_FALLING);
    return HAL_OK;
}
#else
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef*) { ++clock_configs; return HAL_OK; }
HAL_StatusTypeDef HAL_OPAMP_Init(OPAMP_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_OPAMP_Start(OPAMP_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef*, const ADC_MultiModeTypeDef* config) {
    check(config->Mode == ADC_DUALMODE_INJECSIMULT && config->DMAAccessMode == ADC_DMAACCESSMODE_DISABLED,
          "dual mode remains injected-only; regular DMA is independent");
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* handle) {
    SET_BIT(handle->Instance->CR, ADC_CR_ADEN | ADC_CR_JADSTART);
    return HAL_OK;
}
uint32_t HAL_ADCEx_InjectedGetValue(const ADC_HandleTypeDef* handle, uint32_t rank) {
    return rank == ADC_INJECTED_RANK_1 ? handle->Instance->JDR1 : handle->Instance->JDR2;
}
#endif
}

static bool scanRequested() {
#if defined(STM32F410Rx)
    return ADC1->CR2 & ADC_CR2_SWSTART;
#else
    return ADC1->CR & ADC_CR_ADSTART;
#endif
}
static void acknowledgeLaunch() {
    // Emulate write-one-to-clear DMA status and the hardware start handshake.
#if defined(STM32F410Rx)
    DMA2->LISR &= ~DMA2->LIFCR;
    DMA2->LIFCR = 0;
    ADC1->CR2 &= ~ADC_CR2_SWSTART;
#else
    DMA1->ISR &= ~DMA1->IFCR;
    DMA1->IFCR = 0;
    ADC1->ISR = 0;
#endif
}
static void dmaRank(unsigned rank, uint16_t value) {
    dma_buffer[rank] = value;
#if defined(STM32F410Rx)
    DMA2_Stream0->NDTR = rank == 2 ? 3 : 2 - rank;
    if (rank == 2) DMA2->LISR |= DMA_LISR_TCIF0;
#else
    DMA1_Channel1->CNDTR = rank == 2 ? 3 : 2 - rank;
    if (rank == 2) {
        DMA1->ISR |= DMA_ISR_TCIF1;
        CLEAR_BIT(ADC1->CR, ADC_CR_ADSTART);
    }
#endif
}
static void complete(uint16_t bus, uint16_t ntc, uint16_t pot) {
    dmaRank(0, bus); dmaRank(1, ntc); dmaRank(2, pot);
}
static void newDriver() {
    if (adc) adc->~Stm32Adc();
    adc = new (reinterpret_cast<void*>(0x20000000)) Stm32Adc;
    new (reinterpret_cast<void*>(ADC1_BASE)) ADC_TypeDef{};
#if defined(STM32F410Rx)
    new (reinterpret_cast<void*>(DMA2_BASE)) DMA_TypeDef{};
    new (reinterpret_cast<void*>(DMA2_Stream0_BASE)) DMA_Stream_TypeDef{};
#else
    new (reinterpret_cast<void*>(ADC2_BASE)) ADC_TypeDef{};
    new (reinterpret_cast<void*>(DMA1_BASE)) DMA_TypeDef{};
    new (reinterpret_cast<void*>(DMA1_Channel1_BASE)) DMA_Channel_TypeDef{};
#endif
    injected_count[0] = injected_count[1] = regular_count = clock_configs = 0;
    fail_dma_start = false;
    CurrentSensorCalibration calibration{0.01f, 5.0f, 3.3f, 12};
    calibration.injected_sample_sum = Stm32Adc::INJECTED_SAMPLE_SUM;
    check(adc->initialize(calibration, {169000.0f, 18000.0f}), "physical conversion calibration initializes");
    check(adc->initializeHardware(), "hardware setup arms DMA and injected converters");
}

static void configuration() {
    newDriver();
    check(adc_config[0].NbrOfConversion == 3 && regular_count == 3, "Vbus, NTC and throttle use a full regular scan");
    check(regular_config[0].Channel == ADC_CHANNEL_1, "Vbus occupies the first regular rank");
    check(dma_config.Mode == DMA_CIRCULAR && dma_config.MemInc == DMA_MINC_ENABLE &&
          dma_config.PeriphDataAlignment == DMA_PDATAALIGN_HALFWORD &&
          dma_config.MemDataAlignment == DMA_MDATAALIGN_HALFWORD, "DMA preserves three physical 12-bit samples");
    check(!scanRequested() && !adc->regularReady(), "initialization does not launch or invent a regular result");
#if defined(STM32F410Rx)
    check(regular_config[1].Channel == ADC_CHANNEL_12 && regular_config[2].Channel == ADC_CHANNEL_4,
          "F410 NTC/pot ranks follow Vbus");
    check(injected_count[0] == 3 && injected_config[0][0].InjectedNbrOfConversion == 3 &&
          injected_config[0][0].InjectedChannel == ADC_CHANNEL_0 &&
          injected_config[0][1].InjectedChannel == ADC_CHANNEL_11 &&
          injected_config[0][2].InjectedChannel == ADC_CHANNEL_10, "F410 injected sequence is current-only");
#else
    check(regular_config[1].Channel == ADC_CHANNEL_5 && regular_config[2].Channel == ADC_CHANNEL_11,
          "G431 NTC/pot ranks follow Vbus");
    check(clock_configs == 1, "ADC clock setup has one owner");
    check(injected_count[0] == 2 && injected_count[1] == 2 &&
          injected_config[0][0].InjectedNbrOfConversion == 2 &&
          injected_config[1][0].InjectedNbrOfConversion == 2, "dual injected sequences keep equal lengths");
    check(injected_config[0][0].InjectedChannel == ADC_CHANNEL_VOPAMP1 &&
          injected_config[0][1].InjectedChannel == ADC_CHANNEL_VOPAMP1 &&
          injected_config[1][0].InjectedChannel == ADC_CHANNEL_VOPAMP2 &&
          injected_config[1][1].InjectedChannel == ADC_CHANNEL_VOPAMP3_ADC2, "G431 currents remain U/U versus V/W");
    for (unsigned converter = 0; converter < 2; ++converter) {
        for (unsigned rank = 0; rank < 2; ++rank) {
            const auto& config = injected_config[converter][rank];
            check(config.InjecOversamplingMode == ENABLE && config.InjecOversampling.Ratio == ADC_OVERSAMPLING_RATIO_2 &&
                  config.InjecOversampling.RightBitShift == ADC_RIGHTBITSHIFT_NONE &&
                  config.InjectedSamplingTime == ADC_SAMPLETIME_6CYCLES_5,
                  "injected x2 sums and simultaneous sample timing are preserved");
        }
    }
    check(adc_config[0].OversamplingMode == DISABLE, "regular samples are not injected x2 sums");
#endif
}

static void acquisition() {
    newDriver();
    check(!adc->regularFresh(0), "zero-initialized auxiliary cache cannot arm a drive");
    check(adc->serviceRegular(0) && scanRequested(), "first SysTick launches without waiting for a result");
    acknowledgeLaunch();
    check(!adc->regularReady(), "launch alone does not make data ready");
    old_values[0] = old_values[1] = old_values[2] = 0;
    dmaRank(0, 2029);
    check(cachedValues(old_values), "partial DMA cannot expose a new bus with stale NTC/pot");
    dmaRank(1, 1309); dmaRank(2, 2048);
    check(cachedValues(old_values), "DMA completion does not publish outside SysTick");
    new_values[0] = 2029; new_values[1] = 1309; new_values[2] = 2048;
    observe_publication = true;
    inject_before_start = true;
    check(adc->serviceRegular(1000) && scanRequested(), "next SysTick publishes and launches exactly one new scan");
    observe_publication = false;
#if defined(STM32F410Rx)
    check((ADC1->CR2 & ADC_CR2_JSWSTART) && !(ADC1->CR2 & ADC_CR2_JEXTEN),
          "regular-start RMW preserves an injected IRQ's software-x2 trigger state");
#endif
    acknowledgeLaunch();
    check(adc->regularReady() && cachedValues(new_values), "complete frame is ready with all distinct channels");
    check(adc->regularFresh(2500) && !adc->regularFresh(2501), "freshness uses launch=0, not collection=1000");
    const float expected = 2029.0f * 3.3f / 4095.0f * 187000.0f / 18000.0f;
    check(std::fabs(adc->readBusVoltage() - expected) < .002f, "bus units remain correct for each current sum scale");
    check(std::fabs(adc->convertAdcToThrottle(adc->readRawPotentiometer()) - 2048.0f / 4095.0f) < .0001f,
          "throttle remains unscaled physical 12-bit data");
    check(std::fabs(adc->readTemperature() - 25.0f) < 1.0f, "cached NTC preserves temperature conversion");

    for (unsigned i = 0; i < 3; ++i) old_values[i] = new_values[i];
    dmaRank(0, 1999);
    check(cachedValues(old_values), "next scan can overwrite DMA without modifying the published cache");
    dmaRank(1, 1450); dmaRank(2, 2700);
    new_values[0] = 1999; new_values[1] = 1450; new_values[2] = 2700;
    observe_publication = true;
    check(adc->serviceRegular(2000), "successive frames alternate the publication buffers");
    observe_publication = false;
    acknowledgeLaunch();
    check(adc->regularFresh(3500) && !adc->regularFresh(3501), "next frame retains its own launch time");
    check(before_publications == 2 && after_publications == 2, "preemption exercised both cache publications");
}

static void failuresAndWrap() {
    newDriver();
    check(adc->serviceRegular(10), "timeout case starts asynchronously");
    acknowledgeLaunch();
    dmaRank(0, 2100);
    check(!adc->serviceRegular(1000) && !adc->regularReady(),
          "partial DMA at next scheduled tick fails closed even with a sub-1-ms timestamp delta");
    complete(2100, 1309, 2000);
    check(!adc->serviceRegular(2010) && !adc->regularFresh(2010), "late completion cannot clear the latched failure");

    newDriver();
    check(adc->serviceRegular(0xfffffff0U), "scan can start before timer wrap");
    acknowledgeLaunch(); complete(2028, 1309, 2000);
    check(adc->serviceRegular(984) && adc->regularFresh(2484) && !adc->regularFresh(2485),
          "launch timestamp and freshness arithmetic survive wrap");
    acknowledgeLaunch();
#if defined(STM32F410Rx)
    DMA2->LISR |= DMA_LISR_TEIF0;
#else
    DMA1->ISR |= DMA_ISR_TEIF1;
#endif
    check(!adc->serviceRegular(1984) && !adc->regularReady(), "DMA transfer error invalidates previously ready data");

    newDriver();
    adc->serviceRegular(0); acknowledgeLaunch(); complete(2028, 1309, 2000);
#if defined(STM32F410Rx)
    ADC1->SR |= ADC_SR_OVR;
#else
    ADC1->ISR |= ADC_ISR_OVR;
#endif
    check(!adc->serviceRegular(1000), "ADC overrun wins over DMA transfer complete");

    newDriver();
    adc->serviceRegular(0); acknowledgeLaunch(); complete(2028, 1309, 2000);
#if defined(STM32F410Rx)
    DMA2_Stream0->NDTR = 1;
#else
    DMA1_Channel1->CNDTR = 1;
#endif
    check(!adc->serviceRegular(1000), "incoherent DMA count cannot publish even with a complete flag");

#if !defined(STM32F410Rx)
    newDriver();
    adc->serviceRegular(0); acknowledgeLaunch(); complete(2028, 1309, 2000);
    SET_BIT(ADC1->CR, ADC_CR_ADSTART);
    check(!adc->serviceRegular(1000), "a still-running converter cannot publish or be restarted");
#endif

    newDriver();
    adc->serviceRegular(0); acknowledgeLaunch(); complete(2028, 1309, 2000);
    check(!adc->serviceRegular(2501) && !adc->regularReady(), "an overage completed scan cannot be relabeled fresh");

    newDriver();
#if defined(STM32F410Rx)
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
#else
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
#endif
    check(!adc->serviceRegular(0) && !scanRequested(), "disabled DMA blocks scan launch");
    injected_count[0] = injected_count[1] = regular_count = 0;
    fail_dma_start = true;
    check(!adc->initializeHardware(), "DMA setup failure prevents successful board initialization");
    check(critical_depth == 0, "all bounded register-update critical sections are released");
}

static void map(uintptr_t address, size_t length) {
    if (mmap(reinterpret_cast<void*>(address), length, PROT_READ | PROT_WRITE,
             MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0) == MAP_FAILED) {
        std::perror("fake peripheral mapping"); std::exit(EXIT_FAILURE);
    }
}
int main() {
    // These are anonymous process pages, never /dev/mem or a physical board.
    map(0x20000000, 0x10000);
    map(0x40010000, 0x20000);
#if !defined(STM32F410Rx)
    map(0x48000000, 0x10000);
    map(0x50000000, 0x10000);
#endif
    configuration(); acquisition(); failuresAndWrap();
    std::printf("STM32 regular DMA %s, sum=%u: %d checks, %d failures\n",
                CONTROL_PROFILE_NAME, Stm32Adc::INJECTED_SAMPLE_SUM, checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
