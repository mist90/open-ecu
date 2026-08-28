/**
 * @file stm32_pwm.cpp
 * @brief STM32G4 TIM1-based PWM implementation for 3-phase motor control
 */

#include "stm32_pwm.hpp"
#include "main.h"  // board pin map, supplied by the port's include path

namespace libecu {

namespace {

/// Encode a dead time in nanoseconds into the TIM DTG[7:0] field.
///
/// DTG is piecewise: four ranges with steps of 1, 2, 8 and 16 tDTS. Rounding
/// happens on the *tick* count, and the caller should read the value back with
/// dtgToNanoseconds() rather than assume it got what it asked for.
uint32_t nanosecondsToDtg(uint16_t ns, uint32_t tim_clock) noexcept {
    // ticks = round(ns * f_tim / 1e9). 65535 ns * 170e6 = 1.1e13 needs 64 bits.
    // Computing an integer tDTS *period* first and dividing by it - which this
    // used to do - truncates 5.882 ns to 5 and inflates every dead time by 18 %.
    const uint32_t ticks = static_cast<uint32_t>(
        (static_cast<uint64_t>(ns) * tim_clock + 500000000ULL) / 1000000000ULL);

    if (ticks <= 127)  return ticks;                          // step 1
    if (ticks <= 254)  return 0x80U | ((ticks - 128U) / 2U);  // step 2
    if (ticks <= 504)  return 0xC0U | ((ticks - 256U) / 8U);  // step 8
    const uint32_t dtg = (ticks - 512U) / 16U;                // step 16
    return 0xE0U | (dtg > 31U ? 31U : dtg);
}

/// Decode DTG[7:0] back to nanoseconds - what the hardware will actually do.
uint16_t dtgToNanoseconds(uint32_t dtg, uint32_t tim_clock) noexcept {
    uint32_t ticks;
    if      ((dtg & 0x80U) == 0U)    ticks = dtg;                          // step 1
    else if ((dtg & 0xC0U) == 0x80U) ticks = (64U + (dtg & 0x3FU)) * 2U;   // step 2
    else if ((dtg & 0xE0U) == 0xC0U) ticks = (32U + (dtg & 0x1FU)) * 8U;   // step 8
    else                             ticks = (32U + (dtg & 0x1FU)) * 16U;  // step 16
    return static_cast<uint16_t>(
        (static_cast<uint64_t>(ticks) * 1000000000ULL + tim_clock / 2U) / tim_clock);
}

} // namespace

Stm32Pwm::Stm32Pwm(void* htim) noexcept
    : htim_(htim), period_(0), dead_time_ns_(100), actual_dead_time_ns_(0), enabled_(false) {
    frequency_ = 20000;
}

bool Stm32Pwm::initialize(uint32_t frequency, uint16_t dead_time_ns) {
    frequency_ = frequency;
    dead_time_ns_ = dead_time_ns;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    // Calculate timer settings for center-aligned mode
    // In center-aligned mode, counter goes 0→ARR→0, so period = 2*ARR
    // To get desired frequency: ARR = timer_clock / (2 * frequency)
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();
    uint32_t prescaler = 0;
    period_ = (timer_clock / (2 * frequency)) - 1;

    while (period_ > 65535) {
        prescaler++;
        period_ = (timer_clock / (2 * (prescaler + 1) * frequency)) - 1;
    }

    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);

    // Full TIM1 initialization (moved from MX_TIM1_Init)
    tim_handle->Instance = TIM1;
    tim_handle->Init.Prescaler = prescaler;
    tim_handle->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;  // Center-aligned mode 1
    tim_handle->Init.Period = period_;
    tim_handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim_handle->Init.RepetitionCounter = 0;
    tim_handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  // Enable ARR shadow register

    if (HAL_TIM_PWM_Init(tim_handle) != HAL_OK) {
        return false;
    }

    // ADC trigger at the peak (CNT = ARR) — the middle of the OFF-time, furthest
    // from the UP phase switching edges (at CNT = CCR). Static: the trigger no
    // longer tracks duty cycle, so nothing rewrites CCR4 per commutation.
    TIM_OC_InitTypeDef sConfigOC4 = {0};
    sConfigOC4.OCMode = TIM_OCMODE_PWM1;
    sConfigOC4.Pulse = period_ - 1;
    sConfigOC4.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC4.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC4.OCIdleState = TIM_OCIDLESTATE_RESET;

    if (HAL_TIM_OC_ConfigChannel(tim_handle, &sConfigOC4, TIM_CHANNEL_4) != HAL_OK) {
        return false;
    }

    // Enable CCR4 preload (shadow register)
    __HAL_TIM_ENABLE_OCxPRELOAD(tim_handle, TIM_CHANNEL_4);

    // TRGO2 = OC4REF triggers the ADCs mid-OFF-time, while both low-side
    // switches freewheel and the shunt of the DOWN phase carries the full
    // phase current.
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;  // Trigger ADCs on OC4 match (peak)
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(tim_handle, &sMasterConfig) != HAL_OK) {
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

    // CCRx preload stays ENABLED (HAL_TIM_PWM_ConfigChannel sets OCxPE).
    //
    // It was previously disabled so that "CCR changes take effect immediately
    // for the current control loop".  What that actually bought was a race: the
    // current loop rewrites CCR from the TIM1 ISR, and with preload off that
    // write lands in the *live* compare register while the counter is running.
    // When the counter happens to be crossing the compare value at that moment
    // the pulse for that period comes out short, long or split, and whether it
    // does depends on where in the 50 us period the ISR finished - which moves
    // with the optimisation level.
    //
    // Note on how much this was worth: it is NOT what caused the -Os/-O0
    // "vibration" difference.  That turned out to be the demagnetisation window
    // (see MotorControlParams::current_blanking_cycles), and preloading on its
    // own only moved the number around.  Preload is here because the race is
    // real, because it makes the PWM edge independent of ISR execution time,
    // and because the CCR/phase-state split it forces is the shape sinusoidal
    // modulation needs.
    //
    // Preloading only works because setChannelState() no longer encodes the
    // DOWN phase as CCR=0.  Phase state now lives entirely in OCxM/CCxE/CCxNE,
    // which CCPC preloads and the COM event applies; duty lives entirely in
    // CCR, which OCxPE preloads and the update event applies.  The two are
    // independent, so it does not matter that they latch on different events.
    // (This split is what the VESC firmware does; encoding DOWN as CCR=0 while
    // preloading CCR leaves a freshly commutated phase running on the previous
    // phase's duty until the next update event.)
    //
    // Cost: a duty change waits for the next update event - in centre-aligned
    // mode with RCR=0 that is at most half a PWM period, 25 us, about 4.5
    // degrees of phase at the 500 Hz current-loop bandwidth.

    // Configure dead-time for complementary PWM outputs
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;

    // Dead time. See nanosecondsToDtg(); DTG is quantised, so what the hardware
    // receives is read back with dtgToNanoseconds() and reported by
    // getActualDeadTimeNs() rather than assumed.
    uint32_t tim_clock = HAL_RCC_GetPCLK2Freq();  // 170 MHz typically
    sBreakDeadTimeConfig.DeadTime = nanosecondsToDtg(dead_time_ns_, tim_clock);
    actual_dead_time_ns_ = dtgToNanoseconds(sBreakDeadTimeConfig.DeadTime, tim_clock);

    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(tim_handle, &sBreakDeadTimeConfig) != HAL_OK) {
        return false;
    }

    HAL_TIM_PWM_Start(tim_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(tim_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(tim_handle, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(tim_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(tim_handle, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(tim_handle, TIM_CHANNEL_3);

    // Enable CCPC (Capture/Compare Preload Control) for atomic commutation
    // CCxE, CCxNE, OCxM bits are preloaded — only applied on COM event
    if (HAL_TIMEx_ConfigCommutEvent(tim_handle, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE) != HAL_OK) {
        return false;
    }

    return true;
}

namespace {

/// OCxM output-compare mode field values (CCMRx), 4 bits wide including OCxM[3]
constexpr uint32_t OCM_PWM1            = 0x6U;  ///< 0110: OCxREF high while CNT < CCR
constexpr uint32_t OCM_FORCED_INACTIVE = 0x4U;  ///< 0100: OCxREF forced low, CCR ignored

/**
 * @brief Write the OCxM field for CH1..CH3 without touching the rest of CCMRx
 *
 * CCPC is enabled, so OCxM is preloaded here just like CCxE/CCxNE: the write
 * only takes effect on the next COM event, which is what makes a commutation
 * atomic across all three phases.
 */
void setOcMode(TIM_TypeDef* tim, uint32_t channel_index, uint32_t mode) noexcept {
    switch (channel_index) {
        case 0:  // CH1 -> CCMR1 OC1M at bit 4 (mask already covers OC1M[3] at bit 16)
            tim->CCMR1 = (tim->CCMR1 & ~TIM_CCMR1_OC1M) | (mode << TIM_CCMR1_OC1M_Pos);
            break;
        case 1:  // CH2 -> CCMR1 OC2M at bit 12
            tim->CCMR1 = (tim->CCMR1 & ~TIM_CCMR1_OC2M) | (mode << TIM_CCMR1_OC2M_Pos);
            break;
        default: // CH3 -> CCMR2 OC3M at bit 4
            tim->CCMR2 = (tim->CCMR2 & ~TIM_CCMR2_OC3M) | (mode << TIM_CCMR2_OC3M_Pos);
            break;
    }
}

/// Clamp a duty request into the range the bridge can actually switch
constexpr float clampDuty(float d) noexcept {
    return d < 0.0f ? 0.0f : (d > 0.95f ? 0.95f : d);
}

} // namespace

void Stm32Pwm::setChannelState(PwmChannel channel, PwmState state) {
    // Deliberately NOT gated on enabled_. It used to be, and that made
    // setNeutral() a no-op at boot: the OFF states were dropped, so CCER kept
    // the channel-enable bits HAL_TIM_PWM_Start() had set, and the bridge was
    // armed the instant anything configured the pins as alternate function.
    // The register writes below are preloaded (CCPC) and reach no pin while the
    // GPIOs are still inputs, so applying them early is free; what it buys is
    // that "off" is recorded in the hardware rather than merely intended.
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    TIM_TypeDef* tim_instance = (TIM_TypeDef*)tim_handle->Instance;

    // Determine which channel we're configuring (0=CH1, 1=CH2, 2=CH3)
    uint32_t channel_index = static_cast<uint32_t>(channel);

    // CCER register bit positions for polarity control
    // CCxP = bit 1 + 4*x (main output polarity)
    // CCxNP = bit 3 + 4*x (complementary output polarity)
    uint32_t ccxp_bit = (1UL << (1 + 4 * channel_index));   // CCxP
    uint32_t ccxnp_bit = (1UL << (3 + 4 * channel_index));  // CCxNP
    uint32_t ccxe_bit = (1UL << (0 + 4 * channel_index));   // CCxE
    uint32_t ccxne_bit = (1UL << (2 + 4 * channel_index));  // CCxNE

    // No CCR is touched here - see the preload note in initialize().  The DOWN
    // phase is expressed as "force OCxREF inactive" rather than "compare
    // against zero", which is the same waveform (high side off, low side on
    // through the complementary output) but leaves the compare register free to
    // belong entirely to the current loop.
    switch (state) {
        case PwmState::OFF:
            tim_instance->CCER &= ~(ccxe_bit | ccxne_bit);
            break;

        case PwmState::UP:
            tim_instance->CCER &= ~(ccxp_bit | ccxnp_bit);
            setOcMode(tim_instance, channel_index, OCM_PWM1);
            tim_instance->CCER |= (ccxe_bit | ccxne_bit);
            break;

        case PwmState::DOWN:
            tim_instance->CCER &= ~(ccxp_bit | ccxnp_bit);
            setOcMode(tim_instance, channel_index, OCM_FORCED_INACTIVE);
            tim_instance->CCER |= (ccxe_bit | ccxne_bit);
            break;
    }
}

void Stm32Pwm::updateDutyCycle(float duty_u, float duty_v, float duty_w)
{
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    TIM_TypeDef* tim_instance = (TIM_TypeDef*)tim_handle->Instance;

    uint32_t ccr_u = calculateCompareValue(clampDuty(duty_u));
    uint32_t ccr_v = calculateCompareValue(clampDuty(duty_v));
    uint32_t ccr_w = calculateCompareValue(clampDuty(duty_w));

    // UDIS blocks the update event for the duration of the write, so the three
    // compare registers reload as one set.  Without it an update event landing
    // between two of these stores latches a mix of the old and the new duty on
    // different phases.  A suppressed update event is not queued - it is simply
    // skipped, so the reload waits for the next one, at most half a period
    // later.  Kept as short as possible for that reason.
    tim_instance->CR1 |= TIM_CR1_UDIS;
    tim_instance->CCR1 = ccr_u;
    tim_instance->CCR2 = ccr_v;
    tim_instance->CCR3 = ccr_w;
    tim_instance->CR1 &= ~TIM_CR1_UDIS;
}

void Stm32Pwm::enable(bool enable) {
    if (!(enable ^ enabled_))
        return;

    enabled_ = enable;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (enable) {
        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    } else {
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_13);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12);
    }
}

uint32_t Stm32Pwm::getTimChannel(PwmChannel channel) noexcept {
    switch (channel) {
        case PwmChannel::PHASE_U: return TIM_CHANNEL_1;
        case PwmChannel::PHASE_V: return TIM_CHANNEL_2;
        case PwmChannel::PHASE_W: return TIM_CHANNEL_3;
        default: return TIM_CHANNEL_1;
    }
}

uint32_t Stm32Pwm::calculateCompareValue(float duty_cycle) noexcept {
    uint32_t compare_value = static_cast<uint32_t>(duty_cycle * period_);
    // Limit to 95% to ensure proper PWM operation
    uint32_t max_value = static_cast<uint32_t>(period_ * 0.95f);
    return (compare_value > max_value) ? max_value : compare_value;
}

void Stm32Pwm::apply() {
    TIM_HandleTypeDef* tim_handle = static_cast<TIM_HandleTypeDef*>(htim_);
    HAL_TIM_GenerateEvent(tim_handle, TIM_EVENTSOURCE_COM);
}

} // namespace libecu
