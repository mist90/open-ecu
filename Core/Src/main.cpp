/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "../Inc/main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../libecu/include/libecu.hpp"
#include "../../libecu/include/bldc_controller.hpp"
#include "../../libecu/hal/stm32g4/stm32_pwm.hpp"
#include "../../libecu/hal/stm32g4/stm32_hall_sensor.hpp"
#include "../../libecu/hal/stm32g4/stm32_adc.hpp"
#include "../../libecu/include/algorithms/commutation_controller.hpp"
#include "../../libecu/include/algorithms/pid_controller.hpp"
#include "../../libecu/include/algorithms/current_controller.hpp"
#include "../../libecu/include/safety/safety_monitor.hpp"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIODIC_TIMER_FREQ 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
static libecu::Stm32Pwm pwm_driver(&htim1);
static libecu::HallGpioConfig hall_config{A__GPIO_Port, A__Pin, B__Pin, Z__Pin};
static libecu::Stm32HallSensor hall_sensor(hall_config);
static libecu::Stm32Adc adc_driver(&hadc1, &hdma_adc1);
static libecu::CommutationController* commutation_controller = nullptr;
static libecu::PidController* pid_controller = nullptr;
static libecu::SafetyMonitor* safety_monitor = nullptr;
static libecu::CurrentController* current_controller = nullptr;
static libecu::BldcController* motor_controller = nullptr;
static volatile bool control_tick = false;
static uint32_t control_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  /* Start OPAMPs for current sensing */
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

    /* USER CODE BEGIN 2 */
    // Initialize motor control components
    if (!pwm_driver.initialize(20000, 100)) {  // 20kHz PWM, 100ns dead-time
        Error_Handler();
    }

    if (!hall_sensor.initialize()) {
        Error_Handler();
    }

    // Calibrate ADCs (must be done before starting conversions)
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    // Initialize ADC for current sensing
    libecu::CurrentSensorCalibration adc_calibration;
    adc_calibration.shunt_resistance_ohms = 0.003f;  // 3 milliohm shunts
    adc_calibration.opamp_gain = 16.0f;              // PGA gain
    adc_calibration.adc_reference_voltage = 3.3f;    // 3.3V ADC reference
    adc_calibration.adc_resolution_bits = 12;        // 12-bit ADC
    adc_calibration.offset_voltage = 1.65f;          // Mid-supply offset (will be calibrated)

    if (!adc_driver.initialize(adc_calibration)) {
        Error_Handler();
    }

    // Calibrate zero-current offset (motor must be stationary)
    if (!adc_driver.calibrateZeroOffset()) {
        Error_Handler();
    }

    // Start ADC injected conversions (triggered by TIM1_TRGO2)
    // In dual-mode simultaneous, only start the master (ADC1)
    // ADC1 (master): Phase U (VOPAMP1) + Phase W (IN12/OPAMP3)
    // ADC2 (slave): Phase V (VOPAMP2) - automatically started by master
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // Create component instances
    // Using 8 pole pairs for commutation
    commutation_controller = new libecu::CommutationController(pwm_driver, hall_sensor, 8);

    libecu::PidParameters pid_params;
    pid_params.kp = 0.01f;
    pid_params.ki = 0.05f;
    pid_params.kd = 0.0f;
    pid_params.max_output = 1.0f;
    pid_params.min_output = 0.0f;
    pid_params.max_integral = 20.0f;
    pid_controller = new libecu::PidController(pid_params);

    // Create current controller for current control mode
    libecu::CurrentControllerParameters current_params;
    current_params.kp = 0.5f;                  // Current loop proportional gain
    current_params.ki = 50.0f;                 // Current loop integral gain
    current_params.max_output = 1.0f;          // Max duty cycle
    current_params.min_output = 0.0f;          // Min duty cycle
    current_params.max_integral = 10.0f;       // Anti-windup limit
    current_params.sample_time_s = 1.0f / 20000.0f;  // 20kHz (50μs)
    current_params.max_current = 5.4f;         // 5.4A maximum current
    current_controller = new libecu::CurrentController(current_params);

    libecu::SafetyLimits safety_limits;
    safety_limits.max_current =  6.0f;      // 6A max current
    safety_limits.max_temperature = 85.0f;  // 85°C max temp
    safety_monitor = new libecu::SafetyMonitor(safety_limits);

    libecu::MotorControlParams motor_params;
    motor_params.max_duty_cycle = 0.85f;
    motor_params.max_speed_rpm = 150.0f;
    motor_params.acceleration_rate = 1000.0f; // 1000 RPM/s accel
    motor_params.control_frequency = PERIODIC_TIMER_FREQ;

    motor_controller = new libecu::BldcController(
        pwm_driver, hall_sensor, *commutation_controller,
        *pid_controller, *safety_monitor, motor_params,
        &adc_driver, current_controller);
    
    if (!motor_controller->initialize()) {
        Error_Handler();
    }

    /* Configure interrupt priorities for real-time control
     * Lower preempt priority number = higher priority (can preempt higher numbers)
     * Priority 0: TIM1 (20kHz current loop) - highest priority, time-critical
     * Priority 2: SysTick (5kHz control loop) - medium priority
     * Priority 5: Hall sensors (EXTI9_5) - lowest priority, already configured in MX_GPIO_Init
     */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    // Enable TIM1 update interrupt for 20kHz current control loop
    HAL_TIM_Base_Start_IT(&htim1);

    motor_controller->setControlMode(libecu::ControlMode::CLOSED_LOOP);

    // This setting is for libecu::ControlMode::OPEN_LOOP mode only
    motor_controller->setDutyCycle(0.3);

    motor_controller->start();

    // Setup 5kHz control loop with SysTick
    HAL_SYSTICK_Config(SystemCoreClock / PERIODIC_TIMER_FREQ);
    // SysTick priority must be lower than TIM1 (higher number = lower priority)
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);

    // This setting is for libecu::ControlMode::CLOSED_LOOP mode only
    motor_controller->setTargetSpeed(10.0f);
    motor_controller->setDirection(libecu::RotationDirection::CLOCKWISE);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      // motor control loop
      if (control_tick) {
          control_tick = false;
          
          if (motor_controller && safety_monitor) {
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

              // Update motor controller with safety data
              motor_controller->update(safety_data);
              
              // Basic safety check every 10 control cycles
              if ((control_counter % 10) == 0) {
                  libecu::MotorStatus status = motor_controller->getStatus();
                  if (status.active_fault != libecu::SafetyFault::NONE) {
                      motor_controller->emergencyStop();
                      // Could add fault handling here
                  }
              }
              
              control_counter++;
          }
      }
    }
    /* USER CODE END 3 */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;  // Enable scan for injected channels
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;  // EOC after sequence
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode for simultaneous injected conversions
  */
  multimode.Mode = ADC_DUALMODE_INJECSIMULT;  // Simultaneous injected mode
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel (for diagnostics - not used in current control)
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel 1: Phase U current (OPAMP1)
  */
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;  // Fast sampling for 20kHz
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;  // 2 injected channels on ADC1
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;  // TIM1 TRGO2 trigger
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;

  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel 2: Phase W current (OPAMP3 via ADC1_IN12)
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;  // OPAMP3_OUT internally connected
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;  // Single injected channel
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel (for diagnostics - not used in current control)
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel 1: Phase V current (OPAMP2)
  */
  ADC_InjectionConfTypeDef sConfigInjected2 = {0};
  sConfigInjected2.InjectedChannel = ADC_CHANNEL_VOPAMP2;
  sConfigInjected2.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected2.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;  // Match ADC1 timing
  sConfigInjected2.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected2.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected2.InjectedOffset = 0;
  sConfigInjected2.InjectedNbrOfConversion = 1;  // 1 injected channel on ADC2
  sConfigInjected2.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected2.AutoInjectedConv = DISABLE;
  sConfigInjected2.QueueInjectedContext = DISABLE;
  sConfigInjected2.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;  // TIM1 TRGO2 trigger
  sConfigInjected2.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected2.InjecOversamplingMode = DISABLE;

  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;  // Not used in PGA mode
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.InvertingInputSecondary = OPAMP_SEC_INVERTINGINPUT_IO0;
  hopamp1.Init.NonInvertingInputSecondary = OPAMP_SEC_NONINVERTINGINPUT_IO0;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  hopamp1.Init.TrimmingValueP = 0;
  hopamp1.Init.TrimmingValueN = 0;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;  // Not used in PGA mode
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.InvertingInputSecondary = OPAMP_SEC_INVERTINGINPUT_IO0;
  hopamp2.Init.NonInvertingInputSecondary = OPAMP_SEC_NONINVERTINGINPUT_IO0;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  hopamp2.Init.TrimmingValueP = 0;
  hopamp2.Init.TrimmingValueN = 0;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;  // Not used in PGA mode
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.InvertingInputSecondary = OPAMP_SEC_INVERTINGINPUT_IO0;
  hopamp3.Init.NonInvertingInputSecondary = OPAMP_SEC_NONINVERTINGINPUT_IO0;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  hopamp3.Init.TrimmingValueP = 0;
  hopamp3.Init.TrimmingValueN = 0;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  * @note TIM1 is now fully initialized by pwm_driver.initialize() in libecu
  *       This function is kept as a stub for compatibility with HAL structure
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  /* USER CODE BEGIN TIM1_Init 1 */
  // TIM1 initialization moved to libecu::Stm32Pwm::initialize()
  // This includes:
  //   - Basic timer setup (prescaler, period, counter mode)
  //   - Master synchronization configuration
  //   - GPIO configuration (HAL_TIM_MspPostInit)
  //   - PWM channel configuration
  //   - Dead-time configuration
  /* USER CODE END TIM1_Init 1 */

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

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

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A__Pin B__Pin Z__Pin */
  GPIO_InitStruct.Pin = A__Pin|B__Pin|Z__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Enable EXTI interrupts for Hall sensor pins */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/* USER CODE BEGIN 4 */

#ifdef STM32G4
/**
 * @brief SysTick interrupt handler for 100Hz control loop
 */
void HAL_SYSTICK_Callback(void) {
    control_tick = true;
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
