
#include "../Inc/main.h"
#include "../../libecu/include/libecu.hpp"
#include "../../libecu/include/bldc_controller.hpp"
#include "../../libecu/hal/stm32g4/stm32_pwm.hpp"
#include "../../libecu/hal/stm32g4/stm32_hall_sensor.hpp"
#include "../../libecu/hal/stm32g4/stm32_adc.hpp"
#include "../../libecu/include/algorithms/commutation_controller.hpp"
#include "../../libecu/include/algorithms/pid_controller.hpp"
#include "../../libecu/include/safety/safety_monitor.hpp"
#include "../../libecu/include/platform/critical_section.hpp"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define PERIODIC_TIMER_FREQ 1000
#define PWM_TIMER_FREQ 40000

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

static libecu::Stm32Pwm pwm_driver(&htim1);
static libecu::HallGpioConfig hall_config{A__GPIO_Port, A__Pin, B__Pin, Z__Pin};
static libecu::Stm32HallSensor hall_sensor(hall_config);
static libecu::Stm32Adc adc_driver;
static libecu::CommutationController* commutation_controller = nullptr;
static libecu::SafetyMonitor* safety_monitor = nullptr;
static libecu::BldcController* motor_controller = nullptr;
static volatile bool control_tick = false;
//static uint32_t control_counter = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

extern "C" {
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 180);
    return (int)ch;
}

int __io_getchar(void)
{
    uint8_t byte;

    if (HAL_UART_Receive(&huart2, &byte, 1, 0) == HAL_OK)
        return (int)byte;
    else
        return (int)EOF;
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
 * @brief Read potentiometer and convert to output
 * @param max_value Maximum speed corresponding to 3.3V
 * @return output (0 to max_value)
 */
float readPotentiometer(float max_value)
{
    // Start ADC regular conversion
    HAL_ADC_Start(&hadc1);

    // Wait for conversion complete (should be very fast)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

        // Convert ADC value (0-4095) to (0-max_value)
        // Linear mapping: output = (adc_value / 4095.0) * max_value
        float speed_rpm = (static_cast<float>(adc_value) / 4095.0f) * max_value;

        return speed_rpm;
    }

    // If conversion failed, return 0
    return 0.0f;
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
 * This function is called from C code (stm32g4xx_it.c) at 20kHz for current control
 */
extern "C" void motor_controller_pwm_interrupt_handler(void)
{
    if (motor_controller != nullptr) {
        motor_controller->pwmInterruptHandler();
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
    if (!pwm_driver.initialize(PWM_TIMER_FREQ, 100)) {  // 20kHz PWM, 100ns dead-time
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

    if (!adc_driver.initialize(adc_calibration)) {
        Error_Handler();
    }

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
    commutation_controller = new libecu::CommutationController(pwm_driver, hall_sensor, 40);

    // Speed PID controller parameters for VOLTAGE_MODE (outputs duty cycle 0.0-1.0)
    libecu::PidParameters speed_pid_params_voltage;
    speed_pid_params_voltage.kp = 0.01f;
    speed_pid_params_voltage.ki = 0.1f;
    speed_pid_params_voltage.kd = 0.0f;
    speed_pid_params_voltage.max_output = 1.0f;    // Max duty cycle
    speed_pid_params_voltage.min_output = 0.0f;

    // Speed PID controller parameters for CURRENT_MODE (outputs current 0.0-5.4A)
    libecu::PidParameters speed_pid_params_current;
    speed_pid_params_current.kp = 0.05f;     // Higher gain for current control
    speed_pid_params_current.ki = 1.0f;     // Different integral for current
    speed_pid_params_current.kd = 0.0f;
    speed_pid_params_current.max_output = 5.4f;    // Max current (A)
    speed_pid_params_current.min_output = 0.0f;

    // Current PID controller parameters for CURRENT_MODE (outputs duty cycle 0..1.0)
    libecu::PidParameters current_pid_params;
    current_pid_params.kp = 0.1f;
    current_pid_params.ki = 50.0f;
    current_pid_params.kd = 0.0f;
    current_pid_params.max_output = 1.0f;
    current_pid_params.min_output = 0.0f;
    current_pid_params.sample_time_s = 1.0f / PWM_TIMER_FREQ;

    libecu::SafetyLimits safety_limits;
    safety_limits.max_current =  6.0f;      // 6A max current
    safety_limits.max_temperature = 85.0f;  // 85°C max temp
    safety_monitor = new libecu::SafetyMonitor(safety_limits);

    libecu::MotorControlParams motor_params;
    motor_params.max_duty_cycle = 0.85f;
    motor_params.max_current = 6.0f;          // 6A max current (matches safety limit)
    motor_params.max_speed_rpm = 150.0f;
    motor_params.acceleration_rate = 100.0f;  // RPM/s
    motor_params.target_speed_lpf_alpha = 0.1f;  // LPF smoothing for noisy potentiometer input
    motor_params.control_frequency = PERIODIC_TIMER_FREQ;
    motor_params.pid_voltage_mode = speed_pid_params_voltage;
    motor_params.pid_current_mode = speed_pid_params_current;
    motor_params.pid_current_regulator = current_pid_params;

    motor_controller = new libecu::BldcController(
        pwm_driver, hall_sensor, *commutation_controller,
        *safety_monitor, motor_params,
        &adc_driver);
    
    if (!motor_controller->initialize()) {
        Error_Handler();
    }

    /* Configure interrupt priorities for real-time control
     * Lower preempt priority number = higher priority (can preempt higher numbers)
     * Priority 0: TIM1 (20kHz current loop) - highest priority, time-critical
     * Priority 1: Hall sensors (EXTI9_5) - medium priority, already configured in MX_GPIO_Init
     * Priority 2: SysTick (control loop) - lowest priority
     */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    // Enable TIM1 update interrupt for 20kHz current control loop
    // (TIM1 base is already started earlier for ADC calibration)
    HAL_TIM_Base_Start_IT(&htim1);

    // Set control mode (mechanical) and electric mode (electrical)
    motor_controller->setControlMode(libecu::ControlMode::CLOSED_LOOP_VELOCITY);
    motor_controller->setElectricMode(libecu::ElectricMode::CURRENT_MODE);

    // This setting is for (CLOSED_LOOP_TORQUE or OPEN_LOOP) and VOLTAGE_MODE mode only
    motor_controller->setDutyCycle(0.3f);

    // This setting is for (CLOSED_LOOP_TORQUE or OPEN_LOOP) and CURRENT_MODE mode only
    motor_controller->setCurrent(1.0f);

    motor_controller->start();

    // Setup PERIODIC_TIMER_FREQ control loop with SysTick
    HAL_SYSTICK_Config(SystemCoreClock / PERIODIC_TIMER_FREQ);
    // SysTick priority must be lower than TIM1 (higher number = lower priority)
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);

    motor_controller->setDriveMode(libecu::DriveMode::FORWARD);

    printf("ECU started\n");
    while (1) {
        // motor control loop
        if (control_tick) {
            control_tick = false;

            if (motor_controller) {
                // Update motor controller
                
                libecu::MotorStatus status;
                {
                    libecu::CriticalSection cs;
                    status = motor_controller->getStatus();
                }
                // Read potentiometer and update target speed (runs in main loop)
                if (status.control_mode == libecu::ControlMode::CLOSED_LOOP_VELOCITY ||
                        status.control_mode == libecu::ControlMode::OPEN_LOOP) {
                    float target_speed = readPotentiometer(motor_params.max_speed_rpm);
                    motor_controller->setTargetSpeed(target_speed);
                } else if (status.control_mode == libecu::ControlMode::CLOSED_LOOP_TORQUE) {
                    if (status.electric_mode == libecu::ElectricMode::CURRENT_MODE) {
                        float target_current = readPotentiometer(motor_params.max_current);
                        motor_controller->setCurrent(target_current);
                    } else if (status.electric_mode == libecu::ElectricMode::VOLTAGE_MODE) {
                        float target_duty_cycle = readPotentiometer(1.0f);
                        motor_controller->setDutyCycle(target_duty_cycle);
                    }
                }
                printf("%u->%u: %.2f %.2f %.2f %.2f\n", status.measured_position, status.target_position,
                                            status.target_speed_rpm,
                                            status.current_speed_rpm,
                                            status.duty_cycle,
                                            status.measured_current);
            }
            /*if (safety_monitor) {
                // Basic safety check every 10 control cycles
                if ((control_counter % 10) == 0) {
                    // Collect safety data
                    libecu::SafetyData safety_data = {0};

                    // Read actual phase currents from ADC
                    adc_driver.readAllCurrents(
                        safety_data.phase_u_current,
                        safety_data.phase_v_current,
                        safety_data.phase_w_current
                    );

                    safety_data.temperature = 25.0f;     // TODO: Read from temp sensor
                    safety_data.bus_voltage = 24.0f;     // TODO: Read from voltage sensor
                    safety_data.emergency_stop = false;  // TODO: Read from E-stop button
                    safety_data.hall_fault = false;      // TODO: Check hall sensors
                    motor_controller->monitor(safety_data);
                    libecu::MotorStatus status = motor_controller->getStatus();
                    if (status.active_fault != libecu::SafetyFault::NONE) {
                        motor_controller->emergencyStop();
                        // Could add fault handling here
                    }
                }

                control_counter++;
            }*/
        }

#ifdef DEBUG_PWM_ISR
        // Process debug buffer output (one sample per loop iteration)
        if (motor_controller) {
            motor_controller->processDebugOutput();
        }
#endif
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
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
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pins : PA1 PA3 PA5 PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB2 PB12 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : A__Pin B__Pin Z__Pin */
    GPIO_InitStruct.Pin = A__Pin|B__Pin|Z__Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
