
#include "../Inc/main.h"
#include "../../libecu/include/libecu.hpp"
#include "../../libecu/include/bldc_controller.hpp"
#include "../../libecu/hal/stm32g4/stm32_pwm.hpp"
#include "../../libecu/hal/stm32g4/stm32_hall_sensor.hpp"
#include "../../libecu/hal/stm32g4/stm32_adc.hpp"
#include "../../libecu/include/algorithms/commutation_controller.hpp"
#include "../../libecu/include/algorithms/bemf_observer.hpp"
#include "../../libecu/include/algorithms/pid_controller.hpp"
#include "../../libecu/include/platform/critical_section.hpp"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "uart_at_bridge.hpp"

#define PERIODIC_TIMER_FREQ 100
#define PWM_TIMER_FREQ 20000
#define BLDC_NUM_POLES 40 // 8
#define BLDC_MAX_CURRENT 18.0f //  6.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 20.0f // 200.0f
#define BLDC_MAX_ACCELERATION 5.0f // 100.0f
#define BLDC_INVERTION false // true

//#define LEGACY_POT_CONTROL

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;  /* For TIM4 Hall Sensor Interface IRQ handler */

UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_usart2_rx;
uint8_t dma_rx_buffer[256];
volatile uint16_t dma_rx_index = 0;

static libecu::Stm32Pwm pwm_driver(&htim1);
static libecu::HallGpioConfig hall_config{A__GPIO_Port, A__Pin, B__Pin, Z__Pin};
static libecu::Stm32TimHallSensor hall_sensor(hall_config);
static libecu::Stm32Adc adc_driver;
static libecu::BemfObserver bemf_observer(PWM_TIMER_FREQ);
static libecu::CommutationController* commutation_controller = nullptr;
static libecu::BldcController* motor_controller = nullptr;
static libecu::UartAtBridge* g_at_processor = nullptr;
static volatile bool control_tick = false;

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
bool readBrakeButton(void)
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
        libecu::MotorStatus status = motor_controller->getStatus();
        g_at_processor->captureOscSample(
            static_cast<uint8_t>(status.duty_cycle * 100.0f),
            status.target_current, status.measured_current,
            status.bemf_voltage, status.measured_position);
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

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
    if (!pwm_driver.initialize(PWM_TIMER_FREQ, 200)) {  // PWM_TIMER_FREQ PWM, 200ns dead-time
        Error_Handler();
    }

    if (!hall_sensor.initialize()) {
        Error_Handler();
    }

    // Initialize ADC for current sensing
    libecu::CurrentSensorCalibration adc_calibration;
    adc_calibration.shunt_resistance_ohms = 0.003f;  // 3 milliohm shunts
    adc_calibration.opamp_gain = 16.0f;              // PGA gain
    adc_calibration.adc_reference_voltage = 3.3f;    // 3.3V ADC reference
    adc_calibration.adc_resolution_bits = 12;        // 12-bit ADC
    // Configure voltage sensor (resistor divider for Vbus measurement)
    libecu::VoltageSensorParameters voltage_params;
    voltage_params.r_up = 169000.0f;   // 169kOhm upper resistor
    voltage_params.r_down = 18000.0f;  // 18kOhm lower resistor

    if (!adc_driver.initialize(adc_calibration, voltage_params)) {
        Error_Handler();
    }

    // BEMF phase voltage divider: 10kOhm / 2.2kOhm
    libecu::BemfVoltageSensorParameters bemf_voltage_params;
    bemf_voltage_params.r_up = 10000.0f;
    bemf_voltage_params.r_down = 2200.0f;
    adc_driver.initializeBemf(bemf_voltage_params);

    // Initialize ADC and OPAMP hardware (including calibration and starting conversions)
    if (!adc_driver.initializeHardware()) {
        Error_Handler();
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
        Error_Handler();
    }
    printf("zero offsets: %f %f %f\n",  adc_driver.getCalibration().offset_voltage_u,
                                        adc_driver.getCalibration().offset_voltage_v,
                                        adc_driver.getCalibration().offset_voltage_w);

    // Create component instances
    // Using 8 pole pairs for commutation
    commutation_controller = new libecu::CommutationController(pwm_driver, hall_sensor, BLDC_NUM_POLES);

    libecu::MotorControlParams motor_params;
    motor_params.max_duty_cycle = 0.95f;
    motor_params.max_current = BLDC_MAX_CURRENT;
    motor_params.min_current = BLDC_MIN_CURRENT;
    motor_params.max_voltage = 36.0f;
    motor_params.max_speed_rps = BLDC_MAX_SPEED;
    motor_params.acceleration_rate = BLDC_MAX_ACCELERATION;  // RPS/s
    motor_params.target_speed_lpf_alpha = 0.0f;  // LPF smoothing for noisy potentiometer input
    motor_params.measured_speed_lpf_alpha = 0.5f; // LPF smoothing for noisy velocity measurement
    motor_params.control_frequency = PERIODIC_TIMER_FREQ;
    motor_params.pid_voltage_mode = {0.005f, 0.005f}; // Speed PID controller parameters for VOLTAGE_MODE (outputs duty cycle 0.0-1.0)
    motor_params.pid_current_mode = {0.025f, 0.05f}; // Speed PID controller parameters for CURRENT_MODE (outputs current from negative to positive values)
    motor_params.pid_current_regulator = {0.1f, 50.0f}; // Current PID controller parameters for CURRENT_MODE (outputs duty cycle 0..1.0)
    motor_params.useInverseCommTable = BLDC_INVERTION;

    // BEMF sensorless observer parameters
    motor_params.bemf_transition_speed_low = 500.0f;    // steps/sec: below = Hall only
    motor_params.bemf_transition_speed_high = 800.0f;   // steps/sec: above = BEMF only
    motor_params.bemf_blanking_cycles = 5.0f;           // PWM cycles blanking after commutation
    motor_params.bemf_zc_threshold = 0.5f;              // ZC at Vbus/2

    libecu::BemfObserverParams bemf_params;
    bemf_params.blanking_cycles = motor_params.bemf_blanking_cycles;
    bemf_params.zc_threshold = motor_params.bemf_zc_threshold;
    bemf_params.transition_speed_low = motor_params.bemf_transition_speed_low;
    bemf_params.transition_speed_high = motor_params.bemf_transition_speed_high;
    bemf_observer.setParameters(bemf_params);

    motor_controller = new libecu::BldcController(
        pwm_driver, hall_sensor, *commutation_controller,
        motor_params, &adc_driver);

    motor_controller->setBemfObserver(&bemf_observer);

    if (!motor_controller->initialize()) {
        Error_Handler();
    }

    static libecu::UartAtBridge at_processor(motor_controller);
        g_at_processor = &at_processor;

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
    HAL_TIM_Base_Start_IT(&htim1);

    motor_controller->start();

    // Setup PERIODIC_TIMER_FREQ control loop with SysTick
    HAL_SYSTICK_Config(SystemCoreClock / PERIODIC_TIMER_FREQ);
    // SysTick priority must be lower than TIM1 (higher number = lower priority)
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);

    printf("ECU started\n");
    while (1) {
        // motor control loop
        if (control_tick) {
            control_tick = false;

            // Update motor controller status
            libecu::MotorStatus status;
            {
                libecu::CriticalSection cs;
                status = motor_controller->getStatus();
            }
#ifdef LEGACY_POT_CONTROL
            // Read potentiometer and update target speed (runs in main loop)
            if (status.control_mode == libecu::ControlMode::CLOSED_LOOP_VELOCITY ||
                    status.control_mode == libecu::ControlMode::OPEN_LOOP) {
                float target_speed = readBrakeButton()? 0.0 : adc_driver.readPotentiometer(motor_params.max_speed_rps);
                motor_controller->setTargetSpeed(target_speed);
            } else if (status.control_mode == libecu::ControlMode::CLOSED_LOOP_TORQUE) {
                if (status.electric_mode == libecu::ElectricMode::CURRENT_MODE) {
                    float target_current = readBrakeButton()? 0.0 : adc_driver.readPotentiometer(motor_params.max_current);
                    motor_controller->setCurrent(target_current);
                } else if (status.electric_mode == libecu::ElectricMode::VOLTAGE_MODE) {
                    float target_duty_cycle = readBrakeButton()? 0.0 : adc_driver.readPotentiometer(1.0f);
                    motor_controller->setDutyCycle(target_duty_cycle);
                }
            }
#endif

            if (at_processor.isTelemetryEnabled()) {
                at_processor.sendTelemetry(status);
            }
            if (at_processor.isPllTelemetryEnabled()) {
                libecu::MotorPLL::PllInfo pll_info = motor_controller->getPllInfo();
                at_processor.sendPllTelemetry(pll_info);
            }
        }

        at_processor.process();
        at_processor.processOscOutput();
    }
}

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

    /* Enable EXTI interrupts for Hall sensor pins */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
