
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

#define MOTOR_1

#ifdef MOTOR_1
#define PERIODIC_TIMER_FREQ 100
#define PWM_TIMER_FREQ 20000
#define BLDC_NUM_POLES 40
#define BLDC_MAX_CURRENT 18.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 20.0f
#define BLDC_MAX_ACCELERATION 5.0f
#define BLDC_INVERTION false
// Hall -> BEMF handover, in RPS. Measured with tests/test_bemf_replay over an
// 8 s capture at each speed (utility/capture_log.py); the numbers are the
// zero-crossing jitter against the Hall mid-step, in electrical degrees:
//
//   5.16 RPS (619 st/s) 12.5     6.56 RPS (787 st/s)  5.1
//   5.43 RPS (652 st/s) 10.5     6.91 RPS (829 st/s)  6.2
//   6.18 RPS (742 st/s)  4.6     8.00 RPS (960 st/s)  1.5
//
// The detector goes from unusable to good between 652 and 742 steps/s, so both
// thresholds sit above that knee: the observer never drives commutation in the
// 10-degree-jitter region.  transition_speed_low is the one that matters -
// once BEMF has the loop it keeps it all the way down to that speed.
#define BLDC_BEMF_LOW_RPS   6.0f
#define BLDC_BEMF_HIGH_RPS  6.9f
// Electrical model for the current feed-forward, identified by
// tests/test_bemf_replay over 14 captured operating points (0.30 V rms fit).
//
// Electrical model of one phase, used to compute the current-loop PI gains
// (see libecu/include/algorithms/current_loop_tuning.hpp).
//
// ke and R come from tests/test_bemf_replay over 14 captured operating points.
// L is NOT the demag-window figure (5.2 mH) - that quantises to whole PWM
// cycles and over-estimated by an order of magnitude. This value is backed out
// of the measured closed-loop current step: L = kp*Vbus/(2*w_measured), with
// the step settling in ~1.05 ms at kp = 0.1.
//
// R is an upper bound: a no-load speed sweep cannot separate winding resistance
// from the dead-time and diode drops, so ki (which scales with R) is the less
// trustworthy of the two gains.
#define BLDC_KE_V_S_PER_RAD_E   1.2336e-2f
#define BLDC_R_PHASE_OHM        0.658f
#define BLDC_L_PHASE_H          0.0007f
// Current-loop bandwidth. Measured sweep against the old hand-tuned gains
// (which are themselves ~356 Hz by the same formula):
//   500 Hz: mean |Ierr| -15%/-9% at 5/8 RPS, worst case and commutation swing
//           unchanged, duty activity 1.4x
//   800 Hz: mean |Ierr| -35%/-24%, but at 8 RPS the commutation swing grows
//           26% and p95 11%, and duty activity is 2.3x
// 500 Hz ships because it never regresses; raise it if mean accuracy matters
// more than worst-case excursion.
#define BLDC_ILOOP_BW_HZ        500.0f
#define BLDC_NOMINAL_VBUS       31.3f
#else
#define PERIODIC_TIMER_FREQ 100
#define PWM_TIMER_FREQ 20000
#define BLDC_NUM_POLES 8
#define BLDC_MAX_CURRENT 6.0f
#define BLDC_MIN_CURRENT  -6.0f
#define BLDC_MAX_SPEED 200.0f
#define BLDC_MAX_ACCELERATION 100.0f
#define BLDC_INVERTION true
// Not measured on this motor - placeholders at 20%/30% of max speed. Re-run
// tests/test_bemf_replay against captures from this motor before trusting them.
#define BLDC_BEMF_LOW_RPS   40.0f
#define BLDC_BEMF_HIGH_RPS  60.0f
// Not identified for this motor - run tests/test_bemf_replay on captures from
// it before trusting these. Zero L or R falls back to the hand-tuned gains.
#define BLDC_KE_V_S_PER_RAD_E   0.0f
#define BLDC_R_PHASE_OHM        0.0f
#define BLDC_L_PHASE_H          0.0f
// Current-loop bandwidth. Measured sweep against the old hand-tuned gains
// (which are themselves ~356 Hz by the same formula):
//   500 Hz: mean |Ierr| -15%/-9% at 5/8 RPS, worst case and commutation swing
//           unchanged, duty activity 1.4x
//   800 Hz: mean |Ierr| -35%/-24%, but at 8 RPS the commutation swing grows
//           26% and p95 11%, and duty activity is 2.3x
// 500 Hz ships because it never regresses; raise it if mean accuracy matters
// more than worst-case excursion.
#define BLDC_ILOOP_BW_HZ        500.0f
#define BLDC_NOMINAL_VBUS       24.0f
#endif

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
            status.bemf_voltage_u, status.bemf_voltage_v, status.bemf_voltage_w,
            status.measured_position);
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

    // BEMF phase voltage divider: 22kOhm / 2.2kOhm -> ratio 11.0, full scale
    // 3.3 V * 11.0 = 36.3 V, just above the 36 V over-voltage trip.
    libecu::BemfVoltageSensorParameters bemf_voltage_params;
    bemf_voltage_params.r_up = 22000.0f;
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
    motor_params.measured_current_lpf_alpha = 0.005f; // 10 ms at 20 kHz; telemetry only
    motor_params.control_frequency = PERIODIC_TIMER_FREQ;
    motor_params.pid_voltage_mode = {0.1f, 0.2f}; // Speed PID for VOLTAGE_MODE (outputs duty cycle 0.0-1.0)
    motor_params.pid_voltage_mode.integral_max = 0.8f;
    motor_params.pid_voltage_mode.integral_min = 0.0f;
    motor_params.pid_voltage_mode.kb = 2.0f;

    motor_params.pid_current_mode = {1.0f, 2.0f}; // Speed PID for CURRENT_MODE (outputs current, A)
    motor_params.pid_current_mode.integral_max = 12.0f;
    motor_params.pid_current_mode.integral_min = -4.0f;
    motor_params.pid_current_mode.kb = 2.0f;

    // Current-loop PI from the electrical model: kp = 2*L*w_c/Vbus,
    // ki = 2*R*w_c/Vbus, which places the PI zero on the plant pole (Ti = L/R)
    // and puts the crossover at BLDC_ILOOP_BW_HZ. Falls back to the previous
    // hand-tuned gains if the motor has not been identified.
    {
        libecu::CurrentLoopModel iloop;
        iloop.l_phase_h    = BLDC_L_PHASE_H;
        iloop.r_phase_ohm  = BLDC_R_PHASE_OHM;
        iloop.bus_voltage  = BLDC_NOMINAL_VBUS;
        iloop.bandwidth_hz = BLDC_ILOOP_BW_HZ;
        libecu::PidParameters ip = libecu::tuneCurrentPi(iloop);
        if (ip.kp <= 0.0f) {
            ip.kp = 0.1f;   // un-identified motor: previous hand-tuned values
            ip.ki = 50.0f;
        }
        motor_params.pid_current_regulator = ip;
    }
    motor_params.pid_current_regulator.min_output = 0.0f;
    motor_params.pid_current_regulator.max_output = 1.0f;
    motor_params.pid_current_regulator.integral_max = 1.0f;
    motor_params.pid_current_regulator.integral_min = 0.0f;
    motor_params.pid_current_regulator.kb = 5.0f;
    motor_params.useInverseCommTable = BLDC_INVERTION;

    // BEMF sensorless observer parameters.
    // Thresholds are declared per motor in RPS and converted here: one
    // electrical step is 60 degrees, so steps/sec = RPS * num_poles * 3.  The
    // old values were hard-coded in steps/sec outside the per-motor block,
    // which made them meaningless if BLDC_NUM_POLES changed.
    motor_params.bemf_transition_speed_low =
        BLDC_BEMF_LOW_RPS  * BLDC_NUM_POLES * 3.0f;   // below: Hall only
    motor_params.bemf_transition_speed_high =
        BLDC_BEMF_HIGH_RPS * BLDC_NUM_POLES * 3.0f;   // above: BEMF only
    motor_params.bemf_blanking_cycles = 2.0f;           // PWM cycles: demagnetization floor
    motor_params.bemf_blanking_fraction = 0.20f;        // ...or 20% of the step, whichever is longer
    motor_params.bemf_min_duty = 0.15f;            // ON-time sensing needs >6% duty; 15% with margin
    motor_params.bemf_zc_deadband_volts = 0.0f;    // 0 = auto: 1% of Vbus
    motor_params.bemf_zc_confirm_samples = 2;      // reject single-sample noise spikes
    motor_params.bemf_integrator_limit_vs = 0.0f;  // 0 = learn it from Hall-driven steps
    motor_params.bemf_phase_advance = 0.0f;        // no advance

    libecu::BemfObserverParams bemf_params = bemf_observer.getParameters();
    bemf_params.blanking_cycles = motor_params.bemf_blanking_cycles;
    bemf_params.blanking_fraction = motor_params.bemf_blanking_fraction;
    bemf_params.min_duty = motor_params.bemf_min_duty;
    bemf_params.zc_deadband_volts = motor_params.bemf_zc_deadband_volts;
    bemf_params.zc_confirm_samples = motor_params.bemf_zc_confirm_samples;
    bemf_params.transition_speed_low = motor_params.bemf_transition_speed_low;
    bemf_params.transition_speed_high = motor_params.bemf_transition_speed_high;
    bemf_params.is_inverse_commutation = motor_params.useInverseCommTable;
    bemf_params.timing_mode = libecu::BemfTimingMode::FLUX_INTEGRATE;
    bemf_params.integrator_limit_vs = motor_params.bemf_integrator_limit_vs;
    bemf_params.phase_advance = motor_params.bemf_phase_advance;
    bemf_params.auto_learn_limit = true;
    // The 22k/2.2k dividers reach 36.3 V full scale, so the phase sitting at
    // Vbus no longer rails out and (Vu+Vv+Vw)/3 is a usable neutral.  It
    // travels the same divider path as the floating phase, so divider
    // tolerance mismatch and common-mode pickup cancel instead of showing up
    // as a fixed zero-crossing offset.  The observer falls back to Vbus/2 by
    // itself on any sample where a phase tap looks clipped.
    bemf_params.use_virtual_neutral = true;
    bemf_observer.setParameters(bemf_params);

    // Hall health thresholds, sized from a measurement on this hardware
    // (AT+HSTATUS with the detectors disabled, swept 2-9 RPS):
    //
    //   speed    illegal/s   max standing time   step period
    //   6 RPS         0            0 us            1389 us
    //   8 RPS        44            0 us            1042 us
    //   9 RPS      3532           50 us             926 us
    //
    // Illegal codes are common on intact wiring here - Hall bounce, strongly
    // speed dependent - but always transient, because a bouncing line keeps
    // raising EXTI and the next reading is valid. A stuck line has nothing left
    // to raise an edge, so its illegal code stands for a whole step. Counting
    // cannot separate those (benign is 20x the fault rate); duration can.
    libecu::HallMonitorParams hall_params;
    hall_params.decay_time_s           = 0.5f;
    hall_params.invalid_threshold      = 0.0f;    // counting is useless here
    hall_params.invalid_debounce       = 1;
    hall_params.invalid_persist_time_s = 0.0003f; // 6x the benign max, 3x below the fault
    // Fraction, not a count: the count scales with edge rate. A loaded run at
    // 6 RPS / 2 A measured a healthy fraction of ~0.06 while an absolute
    // threshold of 20 tripped on it (surplus 21 of 351 edges).
    hall_params.erratic_fraction       = 0.35f;
    hall_params.erratic_min_edges      = 10.0f;
    hall_params.require_drive_active   = true;

    motor_controller = new libecu::BldcController(
        pwm_driver, hall_sensor, *commutation_controller,
        motor_params, &adc_driver);

    //motor_controller->setBemfObserver(&bemf_observer);

    motor_controller->setHallMonitorParams(hall_params);

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
            if (at_processor.isHallTelemetryEnabled()) {
                at_processor.sendHallTelemetry(motor_controller->getHallInfo());
            }
            if (at_processor.isBemfTelemetryEnabled()) {
                at_processor.sendBemfTelemetry(motor_controller->getBemfInfo(),
                                               motor_controller->getBemfAmplitude());
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
