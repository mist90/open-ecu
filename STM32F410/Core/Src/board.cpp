/** @file board.cpp
 * @brief NUCLEO-F410RB + X-NUCLEO-IHM08M1 implementation of libecu::Board
 */
#include "main.h"
#include "board_config.hpp"
#include "stm32_pwm.hpp"
#include "stm32_adc.hpp"
#include "stm32_hall_sensor.hpp"
#include "board.hpp"
#include "bldc_controller.hpp"
#include "at_command_processor.hpp"
#include "critical_section.hpp"
#include "pwm_timing_bench.hpp"
#include <cstdio>

/* USER CODE BEGIN Private variables */
namespace {
libecu::Stm32Pwm pwm_driver;
libecu::Stm32Adc adc_driver;
TIM_HandleTypeDef microseconds_timer{};
libecu::Stm32HallSensor hall_sensor(microseconds_timer);
UART_HandleTypeDef console_uart{};
DMA_HandleTypeDef console_rx_dma{};
alignas(4) volatile uint8_t rx_buffer[256]{};
uint16_t rx_index = 0;
libecu::BldcController* controller = nullptr;
libecu::AtCommandProcessor* console = nullptr;
volatile bool control_enabled = false;
volatile bool control_tick = false;
bool uart_ready = false;
uint32_t interrupt_depth = 0;
uint32_t saved_primask = 0;
int32_t zero_counts[3]{};
constexpr int32_t CURRENT_TRIP_COUNTS =
    static_cast<int32_t>(25.0f * ihm08::SHUNT_OHMS * ihm08::CURRENT_GAIN * 4095.0f / ihm08::ADC_REFERENCE);
enum Fault : uint32_t {
    BREAK = 1, OVERCURRENT = 2, UNDERVOLTAGE = 4, ADC_FAILURE = 8,
    LOOP_OVERRUN = 16, NTC_FAILURE = 32, ADC_CALIBRATION = 64
};

bool configureClocks() {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    // Mandatory crystal-derived 8 MHz MCO from ST-LINK, through SB16/SB50 to PH0.
    // HSI measured +1.55% on this board and is not a control-clock fallback.
    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_BYPASS;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 200;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 4;
    osc.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
    RCC_ClkInitTypeDef clocks{};
    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_3) != HAL_OK) return false;
    HAL_RCC_EnableCSS(); // Clock loss reaches the bridge-parking NMI handler.
    return true;
}

bool configureTimebase() {
    __HAL_RCC_TIM5_CLK_ENABLE();
    microseconds_timer.Instance = TIM5;
    microseconds_timer.Init.Prescaler = ihm08::TIMER_CLOCK_HZ / 1000000 - 1;
    microseconds_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    microseconds_timer.Init.Period = UINT32_MAX;
    microseconds_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    return HAL_TIM_Base_Init(&microseconds_timer) == HAL_OK &&
           HAL_TIM_Base_Start(&microseconds_timer) == HAL_OK;
}

bool configureConsole() {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
    console_uart.Instance = USART2;
    console_uart.Init.BaudRate = ihm08::UART_BAUD;
    console_uart.Init.WordLength = UART_WORDLENGTH_8B;
    console_uart.Init.StopBits = UART_STOPBITS_1;
    console_uart.Init.Parity = UART_PARITY_NONE;
    console_uart.Init.Mode = UART_MODE_TX_RX;
    console_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    console_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&console_uart) != HAL_OK) return false;
    console_rx_dma.Instance = DMA1_Stream5;
    console_rx_dma.Init.Channel = DMA_CHANNEL_4;
    console_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    console_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    console_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    console_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    console_rx_dma.Init.Mode = DMA_CIRCULAR;
    console_rx_dma.Init.Priority = DMA_PRIORITY_LOW;
    console_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&console_rx_dma) != HAL_OK) return false;
    __HAL_LINKDMA(&console_uart, hdmarx, console_rx_dma);
    if (HAL_UART_Receive_DMA(&console_uart, const_cast<uint8_t*>(rx_buffer), sizeof(rx_buffer)) != HAL_OK)
        return false;
    // RX is consumed in the foreground; no per-byte or half-buffer interrupt.
    __HAL_DMA_DISABLE_IT(&console_rx_dma, DMA_IT_HT | DMA_IT_TC);
    uart_ready = true;
    return true;
}
} // namespace

// Accessible through SWD, including in optimized builds. Cycle counts include
// the current controller and OSC capture, but exclude exception entry/exit.
extern "C" {
struct F410Diagnostics {
    uint32_t samples;
    uint32_t max_current_loop_cycles;
    uint32_t overruns;
    uint32_t faults;
    uint32_t hall_captures;
    uint32_t hall_overcaptures;
};
volatile F410Diagnostics f410_diagnostics{};
volatile F410TimingCapture f410_timing_capture{};
#ifdef PWM_TIMING_BENCH
volatile F410BenchInputs f410_bench_inputs{{2122, 2122, 2122}, 2028, 0, 1};
#endif
}

static void trip(uint32_t reason) {
    libecu::CriticalSection lock;
    pwm_driver.trip();
    f410_diagnostics.faults |= reason;
}

uint32_t time_us() { return TIM5->CNT; }

extern "C" void disable_interrupts() {
    const uint32_t prior = __get_PRIMASK();
    __disable_irq();
    if (interrupt_depth++ == 0) saved_primask = prior;
    __DMB();
}
extern "C" void enable_interrupts() {
    __DMB();
    if (interrupt_depth != 0 && --interrupt_depth == 0) __set_PRIMASK(saved_primask);
}

extern "C" int board_console_write(const char* data, int length) {
    if (!uart_ready || length < 0 || length > UINT16_MAX) return -1;
    return HAL_UART_Transmit(&console_uart, reinterpret_cast<const uint8_t*>(data),
                             static_cast<uint16_t>(length), 1000) == HAL_OK ? length : -1;
}
extern "C" int board_console_read() {
    // F4 clears ORE by reading SR then DR (there is no G4-style ICR register).
    if (USART2->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        __HAL_UART_CLEAR_OREFLAG(&console_uart);
    }
    const uint16_t position = (sizeof(rx_buffer) - __HAL_DMA_GET_COUNTER(&console_rx_dma)) % sizeof(rx_buffer);
    if (position == rx_index) return -1;
    const int result = rx_buffer[rx_index];
    rx_index = (rx_index + 1) % sizeof(rx_buffer);
    return result;
}

extern "C" void board_break_interrupt() {
    if (TIM1->SR & TIM_SR_BIF) {
        trip(BREAK); // Hardware has already cleared MOE asynchronously.
        // BKIN may remain low: disable its IRQ after latching to avoid an IRQ storm.
        CLEAR_BIT(TIM1->DIER, TIM_DIER_BIE);
        TIM1->SR = ~TIM_SR_BIF;
    }
}

extern "C" void board_adc_interrupt() {
    const uint32_t started = DWT->CYCCNT;
    const uint32_t flags = ADC1->SR;
    if (flags & ADC_SR_OVR) {
        ADC1->SR = ~ADC_SR_OVR;
        trip(ADC_FAILURE);
    }
    if (!(flags & ADC_SR_JEOC)) return;
    ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    adc_driver.captureInjected();
    ++f410_diagnostics.samples;
    if (!controller) return;
#ifdef PWM_TIMING_BENCH
    // Exercise deferred Hall/PLL processing too (~5.95 mechanical RPS with MOTOR_1).
    // This extra work is included in timing, giving a conservative ISR estimate.
    if (f410_diagnostics.samples % 28 == 0) {
        f410_bench_inputs.hall_position =
            (f410_bench_inputs.hall_position + f410_bench_inputs.hall_direction) % 6;
        controller->hallSensorInterruptHandler();
    }
#endif
    if (controller->getDriveMode() != libecu::DriveMode::NEUTRAL) {
        for (unsigned phase = 0; phase < 3; ++phase) {
            const int32_t delta = int32_t(adc_driver.getRawAdcValue(static_cast<libecu::PwmChannel>(phase)))
                                - zero_counts[phase];
            if (delta > CURRENT_TRIP_COUNTS || delta < -CURRENT_TRIP_COUNTS) trip(OVERCURRENT);
        }
        if (adc_driver.readNativeBusVoltage() <
            libecu::constant<libecu::ActiveNumericProfile::Voltage>(ihm08::MIN_BUS_VOLTAGE)) trip(UNDERVOLTAGE);
    }
    if (pwm_driver.faulted() && controller->getDriveMode() != libecu::DriveMode::NEUTRAL)
        controller->setDriveMode(libecu::DriveMode::NEUTRAL);
    controller->pwmInterruptHandler();
    console->captureNativeOscSample(controller->getNativeStatus());
    const uint32_t cycles = DWT->CYCCNT - started;
    if (cycles > f410_diagnostics.max_current_loop_cycles) f410_diagnostics.max_current_loop_cycles = cycles;
    const bool overrun = cycles >= SystemCoreClock / libecu::ActiveTiming::CURRENT_HZ || (ADC1->SR & ADC_SR_JEOC);
    if (f410_timing_capture.remaining) {
        f410_timing_capture.total_cycles += cycles;
        if (!f410_timing_capture.samples || cycles < f410_timing_capture.min_cycles)
            f410_timing_capture.min_cycles = cycles;
        if (cycles > f410_timing_capture.max_cycles) f410_timing_capture.max_cycles = cycles;
        if (overrun) ++f410_timing_capture.overruns;
        ++f410_timing_capture.samples;
        --f410_timing_capture.remaining;
    }
    if (overrun) {
        ++f410_diagnostics.overruns;
        trip(LOOP_OVERRUN);
    }
}

extern "C" void DMA2_Stream0_IRQHandler() { adc_driver.dmaInterrupt(); }
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*) { trip(ADC_FAILURE); }
extern "C" void board_hall_timer_interrupt() {
    if (__HAL_TIM_GET_FLAG(&microseconds_timer, TIM_FLAG_CC1OF)) {
        ++f410_diagnostics.hall_overcaptures;
        __HAL_TIM_CLEAR_FLAG(&microseconds_timer, TIM_FLAG_CC1OF);
    }
    HAL_TIM_IRQHandler(&microseconds_timer);
}
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* timer) {
    if (timer->Instance == TIM5 && timer->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ++f410_diagnostics.hall_captures;
        if (controller) controller->hallSensorInterruptHandler();
    }
}
extern "C" void board_systick() {
    static uint32_t divider = 0;
    static uint32_t last_samples = 0;
    if (!control_enabled || ++divider < 1000 / libecu::ActiveTiming::SPEED_HZ) return;
    divider = 0;
    const uint32_t samples = adc_driver.samples();
    if (samples == last_samples) trip(ADC_FAILURE);
    last_samples = samples;
    const uint32_t temperature = adc_driver.getRawTemperatureAdcValue();
    if (controller->getDriveMode() != libecu::DriveMode::NEUTRAL &&
        (temperature == 0 || temperature >= 4095)) trip(NTC_FAILURE);
    if (pwm_driver.faulted() && controller->getDriveMode() != libecu::DriveMode::NEUTRAL)
        controller->setDriveMode(libecu::DriveMode::NEUTRAL);
    controller->update();
    adc_driver.startRegular();
    control_tick = true;
}

extern "C" void Error_Handler() {
    __disable_irq();
    // Also works before TIM1 is initialized. Never leave a spinning bridge on a CPU fault.
    if (__HAL_RCC_TIM1_IS_CLK_ENABLED()) CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    libecu::Stm32Pwm::parkPins();
    while (true) {}
}
/* USER CODE END Private variables */

namespace libecu {
namespace {
class UartAtBridge final : public AtCommandProcessor {
public:
    explicit UartAtBridge(BldcController* motor) noexcept : AtCommandProcessor(motor) {}
    int32_t read() override { return board_console_read(); }
    void write(const char* data, std::size_t length) override { board_console_write(data, int(length)); }
};

class Stm32Board final : public Board {
public:
    bool initialize() noexcept override {
        if (HAL_Init() != HAL_OK) return false;
        Stm32Pwm::parkPins();
        if (!configureClocks() || !configureTimebase() || !configureConsole()) return false;
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitTypeDef gpio{};
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pin = GPIO_PIN_13; // B1, active low
        HAL_GPIO_Init(GPIOC, &gpio);
        // Shield aliases: PB15 is tied to PA7, PA11/PB14 to PA6, PA5/PB13/PC5
        // can share BEMF3. They stay inputs; never initialize a stock SPI/LED pin group.
        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_5;
        HAL_GPIO_Init(GPIOA, &gpio);
        gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4;
        HAL_GPIO_Init(GPIOB, &gpio);
        gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_9;
        HAL_GPIO_Init(GPIOC, &gpio);

        printf("NUCLEO-F410RB + X-NUCLEO-IHM08M1, MOTOR_1, VCP %lu 8N1\n", (unsigned long)ihm08::UART_BAUD);
        printf("Clock: HSE/ST-LINK quartz 8 MHz -> 100 MHz\n");
#ifdef PWM_TIMING_BENCH
        printf("TIMING BENCH: synthetic currents/Vbus/Hall, bridge outputs permanently OFF\n");
#endif
        if (!pwm_driver.initialize(ActiveTiming::CURRENT_HZ, ihm08::DEAD_TIME_NS) || !hall_sensor.initialize())
            return false;
        printf("Hall: TIM5 PB12/PC10/PC11, CKD=2 IC1F=0xF (5.12 us), Morpho jumpers required\n");
        info_ = {ActiveTiming::CURRENT_HZ, ActiveTiming::SPEED_HZ, ihm08::NOMINAL_BUS_VOLTAGE,
                 ihm08::DEAD_TIME_NS, pwm_driver.getActualDeadTimeNs(), ihm08::MAX_DUTY,
                 ihm08::MAX_MODULATION, ihm08::MAX_BUS_VOLTAGE, ihm08::LOW_SIDE_CLAMPED_PWM};
        CurrentSensorCalibration calibration{ihm08::SHUNT_OHMS, ihm08::CURRENT_GAIN, ihm08::ADC_REFERENCE, 12};
        VoltageSensorParameters voltage{float(ihm08::BUS_R_UP), float(ihm08::BUS_R_DOWN)};
        TemperatureSensorParameters temperature{4700.0f, 10000.0f, 3435.0f};
        if (!adc_driver.initialize(calibration, voltage, temperature) || !adc_driver.initializeHardware() ||
            !pwm_driver.startTimebase()) return false;
        HAL_Delay(100);
        if (adc_driver.samples() < 100 || !adc_driver.offsetsPlausible() || !adc_driver.calibrateZeroOffset()) {
            trip(ADC_CALIBRATION);
            printf("ADC calibration failed: raw U/V/W=%lu/%lu/%lu; bridge latched OFF, console available\n",
                   (unsigned long)ADC1->JDR1, (unsigned long)ADC1->JDR2, (unsigned long)ADC1->JDR3);
            return true; // Peripherals run; retain diagnostics without accepting an invalid zero.
        }
        const auto& offsets = adc_driver.getCalibration();
        const float values[3] = {offsets.offset_voltage_u, offsets.offset_voltage_v, offsets.offset_voltage_w};
        for (unsigned i = 0; i < 3; ++i) zero_counts[i] = int32_t(values[i] * 4095.0f / ihm08::ADC_REFERENCE + 0.5f);
        printf("PWM %lu Hz, dead time %u ns, DPWMMIN duty <=95%%; ADC zeros %.3f %.3f %.3f V\n",
               (unsigned long)info_.pwm_frequency_hz, info_.actual_dead_time_ns, values[0], values[1], values[2]);
        return true;
    }
    const BoardInfo& info() const noexcept override { return info_; }
    PwmInterface& pwm() noexcept override { return pwm_driver; }
    HallInterface& hall() noexcept override { return hall_sensor; }
    AdcInterface* adc() noexcept override { return &adc_driver; }
    AtCommandProcessor& createConsole(BldcController& motor) noexcept override {
        static UartAtBridge bridge(&motor);
        return bridge;
    }
    bool startCurrentLoop(BldcController& motor, AtCommandProcessor& processor) noexcept override {
        CriticalSection lock;
        console = &processor;
        controller = &motor;
        return true; // ADC JEOC IRQ has been running since calibration.
    }
    bool startControlLoop() noexcept override {
        // Keep HAL timeouts at 1 ms; divide SysTick rather than changing HAL's timebase.
        control_enabled = true;
        return true;
    }
    bool takeControlTick() noexcept override {
        bool ready;
        {
            CriticalSection lock;
            ready = control_tick;
            control_tick = false;
        }
        if (reported_faults_ != f410_diagnostics.faults) {
            reported_faults_ = f410_diagnostics.faults;
            printf("Board fault mask 0x%lx; bridge latched OFF, reset to re-arm\n", (unsigned long)reported_faults_);
        }
        return ready;
    }
    float readThrottle(float maximum) noexcept override { return adc_driver.convertAdcToThrottle(readPot()) * maximum; }
    ActiveNumericProfile::Duty readNativeThrottle() noexcept override {
        return adc_driver.convertNativeThrottle(readPot());
    }
    bool brakeEngaged() noexcept override { return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET; }
    [[noreturn]] void fail() noexcept override { Error_Handler(); while (true) {} }
private:
    uint32_t readPot() noexcept { return pwm_driver.faulted() ? 0 : adc_driver.readRawPotentiometer(); }
    BoardInfo info_{};
    uint32_t reported_faults_ = 0;
};
} // namespace
Board& createBoard() noexcept { static Stm32Board board; return board; }
} // namespace libecu
