/** @file board_config.hpp
 * @brief Fixed electrical configuration of the F410RB/IHM08M1 stack
 */
#ifndef OPEN_ECU_STM32F410_BOARD_CONFIG_HPP
#define OPEN_ECU_STM32F410_BOARD_CONFIG_HPP
#include <cstdint>
namespace ihm08 {
constexpr uint32_t TIMER_CLOCK_HZ = 100000000;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint16_t DEAD_TIME_NS = 500;
// CKD=DIV2, IC1F=1111: fDTS/32, N=8 at fDTS=50 MHz -> 5.12 us.
// The 1.6 us setting admitted PWM-correlated pulses on this motor cable.
constexpr uint32_t HALL_FILTER = 0x0F;
constexpr float NOMINAL_BUS_VOLTAGE = 31.3f;
constexpr float MIN_BUS_VOLTAGE = 10.0f;
// 10 us all-low window at 20 kHz: 2.5 us settling, 3.24 us U/V/W scan.
constexpr float MAX_DUTY = 0.80f;
constexpr float MAX_MODULATION = 0.60f;
constexpr uint32_t ADC_LEAD_TICKS = 200; // TRGO falling edge 2 us before CNT=ARR
constexpr float SHUNT_OHMS = 0.010f;
constexpr float CURRENT_GAIN = 57.0f / 11.0f;
constexpr float ADC_REFERENCE = 3.3f;
constexpr uint32_t BUS_R_UP = 169000;
constexpr uint32_t BUS_R_DOWN = 9310;
} // namespace ihm08
#endif
