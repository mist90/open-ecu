#include "../include/hal/stm32g4/stm32_pwm.hpp"

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

STM32PWM::STM32PWM() : timer_handle_(nullptr), pwm_frequency_(20000), max_duty_cycle_(4250) {
#ifdef STM32G4
    timer_handle_ = static_cast<void*>(&htim1);
#else
    timer_handle_ = static_cast<void*>(&mock_timer);
#endif
}

STM32PWM::~STM32PWM() {
    emergencyStop();
}

bool STM32PWM::initialize(uint32_t frequency) {
    pwm_frequency_ = frequency;
    calculateTimerSettings();
    
#ifdef STM32G4
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(timer_handle_);
    tim_handle->Init.Period = timer_period_;
    tim_handle->Init.Prescaler = timer_prescaler_;
    
    if (HAL_TIM_PWM_Init(tim_handle) != HAL_OK) {
        return false;
    }
    
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
#endif
    
    return true;
}

void STM32PWM::setDutyCycle(PwmChannel channel, float duty_cycle) {
    if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
        return;
    }
    
#ifdef STM32G4
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(timer_handle_);
    uint32_t pulse_value = static_cast<uint32_t>(duty_cycle * timer_period_);
    pulse_value = (pulse_value > max_duty_cycle_) ? max_duty_cycle_ : pulse_value;
    
    uint32_t tim_channel = getTimerChannel(channel);
    __HAL_TIM_SET_COMPARE(tim_handle, tim_channel, pulse_value);
#endif
}

void STM32PWM::setState(PwmChannel channel, PwmState state) {
#ifdef STM32G4
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(timer_handle_);
    uint32_t tim_channel = getTimerChannel(channel);
    
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

void STM32PWM::enable(bool enable) {
#ifdef STM32G4
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(timer_handle_);
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

void STM32PWM::emergencyStop() {
#ifdef STM32G4
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(timer_handle_);
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

uint32_t STM32PWM::getFrequency() const {
    return pwm_frequency_;
}

void STM32PWM::calculateTimerSettings() {
#ifdef STM32G4
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq() * 2;
    
    timer_prescaler_ = 0;
    timer_period_ = (timer_clock / pwm_frequency_) - 1;
    
    while (timer_period_ > 65535) {
        timer_prescaler_++;
        timer_period_ = (timer_clock / ((timer_prescaler_ + 1) * pwm_frequency_)) - 1;
    }
    
    max_duty_cycle_ = static_cast<uint32_t>(timer_period_ * 0.95f);
#else
    timer_prescaler_ = 0;
    timer_period_ = 4249;  // Example for 20kHz at 170MHz
    max_duty_cycle_ = 4000;
#endif
}

uint32_t STM32PWM::getTimerChannel(PwmChannel channel) const {
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

}