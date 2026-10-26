/** @file board_config.hpp
 * @brief Fixed electrical configuration of the F410RB/IHM08M1 stack
 */
#ifndef OPEN_ECU_STM32F410_BOARD_CONFIG_HPP
#define OPEN_ECU_STM32F410_BOARD_CONFIG_HPP
#include <cstdint>
namespace ihm08 {
constexpr uint32_t TIMER_CLOCK_HZ = 100000000;
constexpr uint32_t UART_BAUD = 2000000;
constexpr uint16_t DEAD_TIME_NS = 1000;
// CKD=DIV2, IC1F=1111: fDTS/32, N=8 at fDTS=50 MHz -> 5.12 us.
// The 1.6 us setting admitted PWM-correlated pulses on this motor cable.
constexpr uint32_t HALL_FILTER = 0x0F;
constexpr float NOMINAL_BUS_VOLTAGE = 31.3f;
constexpr float MIN_BUS_VOLTAGE = 10.0f;
constexpr float MAX_BUS_VOLTAGE = 42.0f;
// DPWMMIN: the two lower duties never exceed sqrt(3)/2 * 0.95 = 0.82273.
// Their low-side windows are >=8.86 us at 20 kHz; reconstruct the highest-duty phase.
constexpr float MAX_DUTY = 0.95f;
constexpr float MAX_MODULATION = MAX_DUTY;
constexpr bool LOW_SIDE_CLAMPED_PWM = true;
constexpr uint32_t ADC_LEAD_TICKS = 100; // TRGO falling edge 1 us before CNT=ARR
constexpr uint32_t CURRENT_SETTLING_NS = 2000; // UM1996: T-noise + T-rise; excludes timer dead time
constexpr float SHUNT_OHMS = 0.010f;
constexpr float CURRENT_GAIN = 57.0f / 11.0f;
constexpr float ADC_REFERENCE = 3.3f;
constexpr uint32_t BUS_R_UP = 169000;
constexpr uint32_t BUS_R_DOWN = 9310;
} // namespace ihm08
#endif
