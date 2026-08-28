/**
 * @file board.cpp
 * @brief STM32G431 port of libecu::Board
 *
 * Everything in this firmware that knows about STM32G4 hardware lives here or
 * under libecu/hal/stm32g4: the peripheral handles, the clock tree, the
 * MX_*_Init() functions, the interrupt entry points and the console
 * transport. The ECU itself is libecu/src/main.cpp and never sees any of it.
 */

#include "../Inc/main.h"
#include "../../libecu/include/board.hpp"
#include "../../libecu/include/bldc_controller.hpp"
#include "../../libecu/include/critical_section.hpp"
#include "../../libecu/hal/stm32g4/stm32_pwm.hpp"
#include "../../libecu/hal/stm32g4/stm32_hall_sensor.hpp"
#include "../../libecu/hal/stm32g4/stm32_adc.hpp"
#include "uart_at_bridge.hpp"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* ---- Board hardware description ----------------------------------------- */

#define PERIODIC_TIMER_FREQ 100
#define PWM_TIMER_FREQ 20000
/* Nominal DC link voltage. Used only to tune the current-loop PI gains; the
 * loops themselves run off the measured bus voltage. */
#define BLDC_NOMINAL_VBUS 31.3f
/* Dead time, nanoseconds. MEASURED optimum - see docs/FOC_HANDOFF.md section 15.
 *
 * This board ran 235 ns for a long time (a units bug: a requested "200" became
 * 235), and 235 ns was too short - the two switches in a leg still overlapped
 * on every transition. Sweeping DC input power against dead time put the
 * optimum at 350 ns, which at 4.95 RPS took FOC from 0.66 A to 0.55 A and
 * six-step from 0.57 A to 0.54 A. That is ~1 W per *modulating* half-bridge -
 * three legs under FOC, one under six-step - and it is what the whole
 * "FOC looks less efficient" puzzle turned out to be.
 *
 * The loss curve is steep below the knee (cross-conduction) and shallow above
 * it (body-diode conduction plus the dV = Vbus*t_dead*f_sw distortion, 0.22 V
 * here). Re-sweep if the FETs, gate driver or PWM frequency change; the boot
 * line prints what the hardware actually received, which is not the request.
 */
#define BLDC_DEAD_TIME_NS 350

/* ---- Peripheral handles -------------------------------------------------- */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;  /* For TIM4 Hall Sensor Interface IRQ handler */

UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_usart2_rx;
uint8_t dma_rx_buffer[256];
volatile uint16_t dma_rx_index = 0;

/* ---- Drivers and ISR targets --------------------------------------------- */

static libecu::Stm32Pwm pwm_driver(&htim1);
static libecu::HallGpioConfig hall_config{A__GPIO_Port, A__Pin, B__Pin, Z__Pin};
static libecu::Stm32TimHallSensor hall_sensor(hall_config);
static libecu::Stm32Adc adc_driver;
static libecu::BldcController* motor_controller = nullptr;
static libecu::AtCommandProcessor* g_at_processor = nullptr;
static volatile bool control_tick = false;

/* Throttle sampling.
 *
 * The potentiometer on PB12 shares ADC1's regular group with the NTC on PB14,
 * and Stm32Adc::readRegularChannel() selects the channel and reads DR as two
 * separate steps - so the two reads must never interleave. The NTC is read
 * from BldcController::update(), i.e. from HAL_SYSTICK_Callback(), so the
 * potentiometer is sampled from there as well: same context, and SysTick
 * cannot preempt itself. The main loop only ever reads the cached result.
 *
 * Sampling starts on the first readThrottle() call and not before, so a build
 * that never asks for the throttle - which is every build with
 * THROTTLE_BRAKE_CONTROL off - pays no conversion in the control tick.
 *
 * Both are single aligned words, so the ISR/main-loop handoff needs no lock:
 * the main loop sees either the previous sample or the current one.
 */
static volatile bool  throttle_sampling = false;
static volatile float throttle_fraction = 0.0f;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_USART2_Init(void);

extern "C" {
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 180);
    return (int)ch;
}

int __io_getchar(void)
{
    int ch = EOF;

    disable_interrupts();

    /* Read DMA write position from CNDTR (bytes remaining) */
    uint16_t cndtr = hdma_usart2_rx.Instance->CNDTR;
    uint16_t write_pos = (256 - cndtr) % 256;

    /* Clear overrun error flag to prevent RX stall at high baud rates */
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE);

    /* Check if new data is available */
    if (dma_rx_index != write_pos) {
        ch = (int)dma_rx_buffer[dma_rx_index];
        dma_rx_index = (dma_rx_index + 1) % 256;
    }

    enable_interrupts();

    return ch;
}
}

uint32_t time_us()
{
    return htim2.Instance->CNT;
}

// Use CMSIS intrinsics for interrupt control
// These functions are used by libecu library for critical sections
extern "C" void disable_interrupts() {
    __asm volatile ("cpsid i" : : : "memory");
}

extern "C" void enable_interrupts() {
    __asm volatile ("cpsie i" : : : "memory");
}

/**
 * @brief Read brake button state
 * @return true if button is pressed (active LOW)
 */
static bool readBrakeButton(void)
{
    return HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET;
}

/**
 * @brief C-linkage wrapper for Hall sensor interrupt handler
 * This function is called from C code (stm32g4xx_it.c) and delegates to the C++ motor controller
 */
extern "C" void motor_controller_hall_interrupt_handler(void)
{
    if (motor_controller != nullptr) {
        motor_controller->hallSensorInterruptHandler();
    }
}

/**
 * @brief C-linkage wrapper for PWM interrupt handler (current control loop)
 * This function is called from C code (stm32g4xx_it.c) at PWM_TIMER_FREQ for current control
 */
extern "C" void motor_controller_pwm_interrupt_handler(void)
{
    if (motor_controller != nullptr) {
        motor_controller->pwmInterruptHandler();
        g_at_processor->captureOscSample(motor_controller->getStatus());
    }
}

/**
 * @brief SysTick interrupt handler - sets control tick flag
 * Frequency: PERIODIC_TIMER_FREQ
 */
void HAL_SYSTICK_Callback(void) {
    control_tick = true;
    if (motor_controller) {
        // Update motor controller
        motor_controller->update();
    }
    // After update(), so the control loop is not held up behind the regular
    // conversion: an injected trigger aborts and restarts an in-flight regular
    // conversion, and one arrives every 50 us, so this read can take a few
    // attempts. Normalised here; readThrottle() applies the caller's scale.
    if (throttle_sampling) {
        throttle_fraction = adc_driver.readPotentiometer(1.0f);
    }
}

namespace libecu {
namespace {

/**
 * @brief The b-g431b-esc1 style three-shunt inverter board
 */
class Stm32Board final : public Board {
public:
    bool initialize() noexcept override
    {
        /* MCU Configuration--------------------------------------------------*/

        /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
        HAL_Init();

        /* Configure the system clock */
        SystemClock_Config();

        /* Initialize all configured peripherals */
        MX_GPIO_Init();
        /* TIM1 is used as PWM timer in pwm_driver */
        MX_TIM2_Init();
        MX_USART2_UART_Init();

        // Initialize motor control components
        if (!pwm_driver.initialize(PWM_TIMER_FREQ, BLDC_DEAD_TIME_NS)) {
            return false;
        }
        info_.pwm_frequency_hz       = PWM_TIMER_FREQ;
        info_.control_frequency_hz   = PERIODIC_TIMER_FREQ;
        info_.nominal_bus_voltage    = BLDC_NOMINAL_VBUS;
        info_.requested_dead_time_ns = BLDC_DEAD_TIME_NS;
        info_.actual_dead_time_ns    = pwm_driver.getActualDeadTimeNs();

        // DTG is quantised, so print what the hardware really got rather than
        // what was asked for - the two disagreed by 18 % until the conversion
        // was fixed.
        printf("PWM: %u Hz, dead time requested %u ns, programmed %u ns\n",
               (unsigned)info_.pwm_frequency_hz,
               (unsigned)info_.requested_dead_time_ns,
               (unsigned)info_.actual_dead_time_ns);

        if (!hall_sensor.initialize()) {
            return false;
        }

        // Initialize ADC for current sensing
        CurrentSensorCalibration adc_calibration;
        adc_calibration.shunt_resistance_ohms = 0.003f;  // 3 milliohm shunts
        adc_calibration.opamp_gain = 16.0f;              // PGA gain
        adc_calibration.adc_reference_voltage = 3.3f;    // 3.3V ADC reference
        adc_calibration.adc_resolution_bits = 12;        // 12-bit ADC
        // Configure voltage sensor (resistor divider for Vbus measurement)
        VoltageSensorParameters voltage_params;
        voltage_params.r_up = 169000.0f;   // 169kOhm upper resistor
        voltage_params.r_down = 18000.0f;  // 18kOhm lower resistor
        // NTC divider on PB14 (ADC1_IN5):
        //   +3.3V -- [10k NTC] -- PB14 -- [4.7k] -- GND
        // so the pin voltage rises with temperature.
        TemperatureSensorParameters temp_params;
        temp_params.r_pulldown_ohms = 4700.0f;
        temp_params.ntc_r25_ohms = 10000.0f;
        temp_params.ntc_beta_k = 3435.0f;   // B25/85 of the fitted thermistor

        if (!adc_driver.initialize(adc_calibration, voltage_params, temp_params)) {
            return false;
        }

        // Initialize ADC and OPAMP hardware (including calibration and starting conversions)
        if (!adc_driver.initializeHardware()) {
            return false;
        }

        // Start TIM1 to generate TRGO2 triggers for ADC (must be before ADC start)
        if (HAL_TIM_Base_Start(&htim1) != HAL_OK) {
            return false;
        }

        // Wait for stable ADC readings
        HAL_Delay(100);

        // Calibrate zero-current offset (motor must be stationary)
        // Must be AFTER ADC is started and TIM1 is generating triggers
        if (!adc_driver.calibrateZeroOffset()) {
            return false;
        }
        printf("zero offsets: %f %f %f\n", adc_driver.getCalibration().offset_voltage_u,
                                           adc_driver.getCalibration().offset_voltage_v,
                                           adc_driver.getCalibration().offset_voltage_w);

        return true;
    }

    const BoardInfo& info() const noexcept override { return info_; }

    PwmInterface&  pwm() noexcept override  { return pwm_driver; }
    HallInterface& hall() noexcept override { return hall_sensor; }
    AdcInterface*  adc() noexcept override  { return &adc_driver; }

    AtCommandProcessor& createConsole(BldcController& controller) noexcept override
    {
        static UartAtBridge at_processor(&controller);
        return at_processor;
    }

    bool startCurrentLoop(BldcController& controller,
                          AtCommandProcessor& console) noexcept override
    {
        motor_controller = &controller;
        g_at_processor = &console;

        /* Configure interrupt priorities for real-time control
         * Lower preempt priority number = higher priority (can preempt higher numbers)
         * Priority 0: TIM1 (PWM_TIMER_FREQ current loop) - highest priority, time-critical
         * Priority 1: Hall sensors (EXTI9_5) - medium priority, already configured in MX_GPIO_Init
         * Priority 2: SysTick (control loop) - lowest priority
         */
        HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

        // Enable TIM1 update interrupt for PWM_TIMER_FREQ current control loop
        // (TIM1 base is already started earlier for ADC calibration)
        return HAL_TIM_Base_Start_IT(&htim1) == HAL_OK;
    }

    bool startControlLoop() noexcept override
    {
        // Setup PERIODIC_TIMER_FREQ control loop with SysTick
        if (HAL_SYSTICK_Config(SystemCoreClock / PERIODIC_TIMER_FREQ) != 0) {
            return false;
        }
        // SysTick priority must be lower than TIM1 (higher number = lower priority)
        HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
        return true;
    }

    bool takeControlTick() noexcept override
    {
        if (!control_tick) {
            return false;
        }
        control_tick = false;
        return true;
    }

    [[noreturn]] void fail() noexcept override
    {
        Error_Handler();
        while (1) {
        }
    }

    float readThrottle(float max_value) noexcept override
    {
        // Arms sampling in the control tick; the first call returns 0, which
        // is the safe direction (no demand), and every later one returns a
        // sample at most one control period old.
        throttle_sampling = true;
        return throttle_fraction * max_value;
    }

    bool brakeEngaged() noexcept override { return readBrakeButton(); }

private:
    BoardInfo info_{};
};

} // namespace

Board& createBoard() noexcept
{
    static Stm32Board board;
    return board;
}

} // namespace libecu

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 170;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_Base_Start(&htim2);
}

/**
  * @brief USART2 DMA Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA_USART2_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMAMUX1 clock enable - required for DMA request routing on STM32G4 */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    /* Start DMA reception */
    if (HAL_UART_Receive_DMA(&huart2, dma_rx_buffer, 256) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 2000000;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
        Error_Handler();
    }

    /* Initialize USART2 DMA for RX */
    MX_DMA_USART2_Init();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pins : BUTTON_Pin */
    GPIO_InitStruct.Pin = BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
