/** @file stm32_pwm.cpp
 * @brief IHM08M1 complementary PWM, break shutdown and injected-ADC trigger
 */
#include "stm32_pwm.hpp"
#include "board_config.hpp"
#include <cmath>
#include <initializer_list>

namespace libecu {

void Stm32Pwm::parkPins() noexcept {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
    GPIO_InitTypeDef gpio{};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

bool Stm32Pwm::initialize(uint32_t frequency, uint16_t dead_time_ns) {
    // This port intentionally supports one clock tree and the linear DTG range.
    if (!frequency || ihm08::TIMER_CLOCK_HZ % (2 * frequency) != 0) return false;
    period_ = ihm08::TIMER_CLOCK_HZ / (2 * frequency); // centre-aligned period is 2*ARR
    const uint32_t dtg = (uint64_t(dead_time_ns) * ihm08::TIMER_CLOCK_HZ + 999999999) / 1000000000;
    if (period_ > 65535 || period_ <= ihm08::ADC_LEAD_TICKS || dtg > 127) return false;
    frequency_ = frequency;
    actual_dead_time_ns_ = dtg * (1000000000 / ihm08::TIMER_CLOCK_HZ);
    parkPins();
    __HAL_RCC_TIM1_CLK_ENABLE();
    timer_.Instance = TIM1;
    timer_.Init.Prescaler = 0;
    timer_.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    timer_.Init.Period = period_;
    timer_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer_.Init.RepetitionCounter = 0;
    timer_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&timer_) != HAL_OK) return false;

    TIM_OC_InitTypeDef oc{};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    // L6398 LIN is active LOW: invert the physical CHxN output after dead time.
    oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_SET;
    for (uint32_t channel : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3}) {
        if (HAL_TIM_PWM_ConfigChannel(&timer_, &oc, channel) != HAL_OK) return false;
    }
    // OC4REF falls once per full period, 2 us BEFORE the centre of all-low time.
    // ADC JEXTSEL=TIM1_TRGO, falling edge; CH4 is internal, PA11 stays Hi-Z (BKIN net!).
    oc.Pulse = period_ - ihm08::ADC_LEAD_TICKS;
    if (HAL_TIM_PWM_ConfigChannel(&timer_, &oc, TIM_CHANNEL_4) != HAL_OK) return false;
    TIM_MasterConfigTypeDef master{};
    master.MasterOutputTrigger = TIM_TRGO_OC4REF;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer_, &master) != HAL_OK) return false;

    TIM_BreakDeadTimeConfigTypeDef bdtr{};
    bdtr.OffStateRunMode = TIM_OSSR_ENABLE;
    bdtr.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bdtr.DeadTime = dtg;
    bdtr.BreakState = TIM_BREAK_ENABLE;
    bdtr.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    bdtr.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&timer_, &bdtr) != HAL_OK ||
        HAL_TIMEx_ConfigCommutEvent(&timer_, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE) != HAL_OK) return false;
    setNeutral();
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&timer_);

    GPIO_InitTypeDef gpio{};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_6; // TIM1_BKIN, shield R78, comparator outputs active low
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    __HAL_TIM_CLEAR_FLAG(&timer_, TIM_FLAG_BREAK);
    __HAL_TIM_ENABLE_IT(&timer_, TIM_IT_BREAK);
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    return true;
}

void Stm32Pwm::setChannelState(PwmChannel channel, PwmState state) {
    const uint32_t index = static_cast<uint32_t>(channel);
    if (index > 2) return;
    const uint32_t shift = 4 * index;
    // Keep CCxE ON even for OFF. With OSSR=1 and CCxNE=0, forced-inactive
    // OCxREF drives HIN=0 AND LIN=1. Disabling both outputs would float LIN.
    const uint32_t enables = TIM_CCER_CC1E | TIM_CCER_CC1NP |
                            (state == PwmState::OFF ? 0 : TIM_CCER_CC1NE);
    const uint32_t mode = state == PwmState::UP ? TIM_OCMODE_PWM1 : TIM_OCMODE_FORCED_INACTIVE;
    volatile uint32_t& ccmr = index == 2 ? TIM1->CCMR2 : TIM1->CCMR1;
    const uint32_t mode_shift = index == 1 ? 8 : 0;
    MODIFY_REG(ccmr, TIM_CCMR1_OC1M << mode_shift, mode << mode_shift);
    MODIFY_REG(TIM1->CCER, 0xFU << shift, enables << shift);
}

void Stm32Pwm::updateDutyCycle(float u, float v, float w) {
    if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(w)) { trip(); return; }
    updateNativeDutyCycle(numberCast<NativeDuty>(u), numberCast<NativeDuty>(v), numberCast<NativeDuty>(w));
}

void Stm32Pwm::updateNativeDutyCycle(NativeDuty u, NativeDuty v, NativeDuty w) {
    constexpr NativeDuty maximum = constant<NativeDuty>(ihm08::MAX_DUTY);
    const uint32_t cu = dutyToCompare(numericClamp(u, NativeDuty{}, maximum), period_);
    const uint32_t cv = dutyToCompare(numericClamp(v, NativeDuty{}, maximum), period_);
    const uint32_t cw = dutyToCompare(numericClamp(w, NativeDuty{}, maximum), period_);
    // Block only the reload, not the running counter/OC4REF ADC trigger.
    SET_BIT(TIM1->CR1, TIM_CR1_UDIS);
    TIM1->CCR1 = cu;
    TIM1->CCR2 = cv;
    TIM1->CCR3 = cw;
    CLEAR_BIT(TIM1->CR1, TIM_CR1_UDIS);
}

void Stm32Pwm::enable(bool enabled) {
#ifdef PWM_TIMING_BENCH
    // Timing firmware cannot energize the inverter, regardless of AT commands.
    enabled = false;
#endif
    if (!enabled || faulted_) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&timer_);
    } else if (!(GPIOA->IDR & GPIO_PIN_6) || (TIM1->SR & TIM_SR_BIF)) {
        trip();
    } else {
        __HAL_TIM_MOE_ENABLE(&timer_);
    }
}

void Stm32Pwm::apply() { HAL_TIM_GenerateEvent(&timer_, TIM_EVENTSOURCE_COM); }
bool Stm32Pwm::startTimebase() noexcept { return HAL_TIM_Base_Start(&timer_) == HAL_OK; }
void Stm32Pwm::trip() noexcept {
    faulted_ = true;
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&timer_);
}
} // namespace libecu
