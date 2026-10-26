/** @file stm32_hall_sensor.cpp
 * @brief TIM5 XOR Hall interface with a 5.12 us digital input filter
 */
#include "stm32f4xx_hal.h"
#include "stm32_hall_sensor.hpp"
#include "pwm_timing_bench.hpp"
#include "board_config.hpp"
namespace libecu {
bool Stm32HallSensor::initialize() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    // Morpho jumpers: PA15->PB12, PB3->PC10, PB10->PC11. Keep the shield's
    // original pads high impedance; all edge detection belongs to TIM5.
    GPIO_InitTypeDef gpio{};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF2_TIM5;
    gpio.Pull = GPIO_NOPULL; // JP3: shield pull-ups to 3V3, after its RC/clamp network
    gpio.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOC, &gpio);

    // TIM5 already supplies time_us(). Reconfigure it before ADC calibration,
    // then preserve its 32-bit free-running counter during every Hall edge.
    if (HAL_TIM_Base_Stop(&timer_) != HAL_OK) return false;
    // CKD divides only the filter/dead-time clock, not CK_CNT. The timebase
    // remains 1 MHz with PSC=99 while the digital filter sees fDTS=50 MHz.
    timer_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    TIM_HallSensor_InitTypeDef hall{};
    hall.IC1Polarity = TIM_ICPOLARITY_RISING;
    hall.IC1Prescaler = TIM_ICPSC_DIV1;
    hall.IC1Filter = ihm08::HALL_FILTER;
    if (HAL_TIMEx_HallSensor_Init(&timer_, &hall) != HAL_OK) return false;

    // HAL Hall mode normally resets CNT at every TI1F_ED trigger. Keep capture
    // on TRC (both filtered XOR edges), but disable slave reset so time_us()
    // remains monotonic. No commutation is generated in hardware.
    TIM_SlaveConfigTypeDef slave{};
    slave.SlaveMode = TIM_SLAVEMODE_DISABLE;
    slave.InputTrigger = TIM_TS_TI1F_ED;
    slave.TriggerFilter = ihm08::HALL_FILTER;
    if (HAL_TIM_SlaveConfigSynchro(&timer_, &slave) != HAL_OK) return false;
    TIM_MasterConfigTypeDef master{};
    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer_, &master) != HAL_OK) return false;
    __HAL_TIM_CLEAR_FLAG(&timer_, TIM_FLAG_CC1 | TIM_FLAG_CC1OF | TIM_FLAG_UPDATE | TIM_FLAG_TRIGGER);
    HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    return HAL_TIMEx_HallSensor_Start_IT(&timer_) == HAL_OK;
}
uint8_t Stm32HallSensor::getPosition() {
#ifdef PWM_TIMING_BENCH
    return static_cast<uint8_t>(f410_bench_inputs.hall_position);
#else
    static constexpr uint8_t positions[8] = {0xFF, 0, 2, 1, 4, 5, 3, 0xFF};
    const uint32_t b = GPIOB->IDR;
    const uint32_t c = GPIOC->IDR;
    const uint32_t code = ((b >> 12) & 1) | (((c >> 10) & 1) << 1) | (((c >> 11) & 1) << 2);
    return positions[code];
#endif
}
} // namespace libecu
