/**
 * @file stm32_pwm.cpp
 * @brief STM32G4 TIM1-based PWM implementation for 3-phase motor control
 */

#include "stm32_pwm.hpp"

#ifdef STM32G4
#include "../../Core/Inc/main.h"
extern TIM_HandleTypeDef htim1;
#else
struct TIM_HandleTypeDef_Impl {
    void* Instance;
    struct {
        uint32_t Period;
        uint32_t Prescaler;
    } Init;
};
static TIM_HandleTypeDef_Impl mock_timer;
#endif

namespace libecu {

Stm32Pwm::Stm32Pwm(void* htim) 
    : htim_(htim), frequency_(20000), period_(0), enabled_(false) {
}

bool Stm32Pwm::initialize(uint32_t frequency) {
    frequency_ = frequency;
    
#ifdef STM32G4
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
#else
    period_ = 4249;  // Mock value for testing
#endif
    
    return true;
}

void Stm32Pwm::setDutyCycle(PwmChannel channel, float duty_cycle) {
    if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
        return;
    }
    
    uint32_t compare_value = calculateCompareValue(duty_cycle);
    uint32_t tim_channel = getTimChannel(channel);
    
#ifdef STM32G4
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    __HAL_TIM_SET_COMPARE(tim_handle, tim_channel, compare_value);
#endif
}

void Stm32Pwm::setState(PwmChannel channel, PwmState state) {
#ifdef STM32G4
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    uint32_t tim_channel = getTimChannel(channel);
    
    switch (state) {
        case PwmState::OFF:
            HAL_TIM_PWM_Stop(tim_handle, tim_channel);
            HAL_TIMEx_PWMN_Stop(tim_handle, tim_channel);
            break;
            
        case PwmState::HIGH_SIDE:
            HAL_TIM_PWM_Start(tim_handle, tim_channel);
            HAL_TIMEx_PWMN_Stop(tim_handle, tim_channel);
            break;
            
        case PwmState::LOW_SIDE:
            HAL_TIM_PWM_Stop(tim_handle, tim_channel);
            HAL_TIMEx_PWMN_Start(tim_handle, tim_channel);
            break;
            
        case PwmState::FLOATING:
            HAL_TIM_PWM_Stop(tim_handle, tim_channel);
            HAL_TIMEx_PWMN_Stop(tim_handle, tim_channel);
            break;
    }
#endif
}

void Stm32Pwm::enable(bool enable) {
    enabled_ = enable;
    
#ifdef STM32G4
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
#endif
}

void Stm32Pwm::emergencyStop() {
    enabled_ = false;
    
#ifdef STM32G4
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
#endif
}

uint32_t Stm32Pwm::getFrequency() const {
    return frequency_;
}

uint32_t Stm32Pwm::getTimChannel(PwmChannel channel) {
#ifdef STM32G4
    switch (channel) {
        case PwmChannel::PHASE_U: return TIM_CHANNEL_1;
        case PwmChannel::PHASE_V: return TIM_CHANNEL_2;
        case PwmChannel::PHASE_W: return TIM_CHANNEL_3;
        default: return TIM_CHANNEL_1;
    }
#else
    return 0;
#endif
}

uint32_t Stm32Pwm::calculateCompareValue(float duty_cycle) {
    uint32_t compare_value = static_cast<uint32_t>(duty_cycle * period_);
    // Limit to 95% to ensure proper PWM operation
    uint32_t max_value = static_cast<uint32_t>(period_ * 0.95f);
    return (compare_value > max_value) ? max_value : compare_value;
}

} // namespace libecu