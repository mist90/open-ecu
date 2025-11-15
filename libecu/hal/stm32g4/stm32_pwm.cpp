/**
 * @file stm32_pwm.cpp
 * @brief STM32G4 TIM1-based PWM implementation for 3-phase motor control
 */

#include "stm32_pwm.hpp"
#include "../../Core/Inc/main.h"
extern TIM_HandleTypeDef htim1;

namespace libecu {

Stm32Pwm::Stm32Pwm(void* htim) 
    : htim_(htim), frequency_(20000), period_(0), dead_time_ns_(100), enabled_(false) {
}

bool Stm32Pwm::initialize(uint32_t frequency, uint16_t dead_time_ns) {
    frequency_ = frequency;
    dead_time_ns_ = dead_time_ns;

    // Calculate timer settings
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq() * 2;
    uint32_t prescaler = 0;
    period_ = (timer_clock / frequency) - 1;
    
    while (period_ > 65535) {
        prescaler++;
        period_ = (timer_clock / ((prescaler + 1) * frequency)) - 1;
    }
    
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    tim_handle->Init.Period = period_;
    tim_handle->Init.Prescaler = prescaler;
    
    if (HAL_TIM_PWM_Init(tim_handle) != HAL_OK) {
        return false;
    }
    
    // Configure PWM channels
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    if (HAL_TIM_PWM_ConfigChannel(tim_handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(tim_handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(tim_handle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        return false;
    }
    
    // Configure dead-time for complementary PWM outputs
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    
    // Calculate dead-time register value
    // TIM1 clock is typically 170MHz, dead-time units depend on TDTS
    uint32_t tim_clock = HAL_RCC_GetPCLK2Freq() * 2;  // 170MHz typically
    uint32_t dead_time_ticks = (dead_time_ns_ * tim_clock) / 1000000000UL;
    
    // STM32 dead-time has different ranges based on DTG[7:0] value
    if (dead_time_ticks <= 127) {
        sBreakDeadTimeConfig.DeadTime = dead_time_ticks;
    } else if (dead_time_ticks <= 254) {
        sBreakDeadTimeConfig.DeadTime = 0x80 | ((dead_time_ticks - 128) / 2);
    } else if (dead_time_ticks <= 504) {
        sBreakDeadTimeConfig.DeadTime = 0xC0 | ((dead_time_ticks - 256) / 8);
    } else {
        sBreakDeadTimeConfig.DeadTime = 0xE0 | ((dead_time_ticks - 512) / 16);
    }
    
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    
    if (HAL_TIMEx_ConfigBreakDeadTime(tim_handle, &sBreakDeadTimeConfig) != HAL_OK) {
        return false;
    }

    return true;
}

void Stm32Pwm::setChannelState(PwmChannel channel, PwmState state, float duty_cycle) {
    // Clamp duty_cycle to valid range
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;

    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    TIM_TypeDef* tim_instance = (TIM_TypeDef*)tim_handle->Instance;
    uint32_t tim_channel = getTimChannel(channel);
    uint32_t compare_value = calculateCompareValue(duty_cycle);

    // Determine which channel we're configuring (0=CH1, 1=CH2, 2=CH3)
    uint32_t channel_index = static_cast<uint32_t>(channel);

    // CCER register bit positions for polarity control
    // CCxP = bit 1 + 4*x (main output polarity)
    // CCxNP = bit 3 + 4*x (complementary output polarity)
    uint32_t ccxp_bit = (1UL << (1 + 4 * channel_index));   // CCxP
    uint32_t ccxnp_bit = (1UL << (3 + 4 * channel_index));  // CCxNP

    switch (state) {
        case PwmState::OFF:
            // High impedance - disable both high-side and low-side outputs
            HAL_TIM_PWM_Stop(tim_handle, tim_channel);
            HAL_TIMEx_PWMN_Stop(tim_handle, tim_channel);
            __HAL_TIM_SET_COMPARE(tim_handle, tim_channel, 0);
            break;

        case PwmState::UP:
            // Non-inverse PWM: High-side active for duty_cycle, low-side complementary
            // Set normal polarity (active high) by clearing CCxP and CCxNP bits
            tim_instance->CCER &= ~(ccxp_bit | ccxnp_bit);

            // Set compare value and start outputs
            __HAL_TIM_SET_COMPARE(tim_handle, tim_channel, compare_value);
            HAL_TIM_PWM_Start(tim_handle, tim_channel);
            HAL_TIMEx_PWMN_Start(tim_handle, tim_channel);
            break;

        case PwmState::DOWN:
            // Inverse PWM: Low-side active for duty_cycle, high-side complementary
            // Set inverted polarity (active low) by setting CCxP and CCxNP bits
            tim_instance->CCER |= (ccxp_bit | ccxnp_bit);

            // Set compare value and start outputs
            __HAL_TIM_SET_COMPARE(tim_handle, tim_channel, compare_value);
            HAL_TIM_PWM_Start(tim_handle, tim_channel);
            HAL_TIMEx_PWMN_Start(tim_handle, tim_channel);
            break;
    }
}

void Stm32Pwm::setState(PwmChannel channel, PwmState state) {
    // Legacy method - use setChannelState with default duty_cycle
    setChannelState(channel, state, 0.0f);
}

void Stm32Pwm::enable(bool enable) {
    enabled_ = enable;

    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    if (enable) {
        HAL_TIM_PWM_Start(tim_handle, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(tim_handle, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(tim_handle, TIM_CHANNEL_3);
        
        HAL_TIMEx_PWMN_Start(tim_handle, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(tim_handle, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(tim_handle, TIM_CHANNEL_3);
    } else {
        emergencyStop();
    }
}

void Stm32Pwm::emergencyStop() {
    enabled_ = false;
    
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    HAL_TIM_PWM_Stop(tim_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(tim_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(tim_handle, TIM_CHANNEL_3);
    
    HAL_TIMEx_PWMN_Stop(tim_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(tim_handle, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(tim_handle, TIM_CHANNEL_3);
    
    __HAL_TIM_SET_COMPARE(tim_handle, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(tim_handle, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(tim_handle, TIM_CHANNEL_3, 0);
}

uint32_t Stm32Pwm::getFrequency() const {
    return frequency_;
}

uint32_t Stm32Pwm::getTimChannel(PwmChannel channel) {
    switch (channel) {
        case PwmChannel::PHASE_U: return TIM_CHANNEL_1;
        case PwmChannel::PHASE_V: return TIM_CHANNEL_2;
        case PwmChannel::PHASE_W: return TIM_CHANNEL_3;
        default: return TIM_CHANNEL_1;
    }
}

uint32_t Stm32Pwm::calculateCompareValue(float duty_cycle) {
    uint32_t compare_value = static_cast<uint32_t>(duty_cycle * period_);
    // Limit to 95% to ensure proper PWM operation
    uint32_t max_value = static_cast<uint32_t>(period_ * 0.95f);
    return (compare_value > max_value) ? max_value : compare_value;
}

} // namespace libecu