/** @file ecu_config.hpp
 * @brief HAL-independent timing and arithmetic for NUCLEO-F410RB + IHM08M1
 */
#ifndef OPEN_ECU_STM32F410_CONFIG_HPP
#define OPEN_ECU_STM32F410_CONFIG_HPP
#include "control_timing.hpp"
#include "numeric_profiles.hpp"
#ifndef LIBECU_SPEED_PI_HZ
#define LIBECU_SPEED_PI_HZ 1000
#endif
namespace libecu {
// HAL_IncTick and the regular ADC/DMA service share the 1 ms HAL SysTick.
using ActiveTiming = ControlTiming<20000, 100, LIBECU_SPEED_PI_HZ, 1000>;
#ifdef ECU_FIXED_NUMERIC
using ActiveNumericProfile = FixedProfile;
using ActivePiProfile = FixedProfile;
inline constexpr const char* CONTROL_PROFILE_NAME = "fixed";
inline constexpr const char* PI_PROFILE_NAME = "fixed";
inline constexpr const char* PI_FIRMWARE_VERSION = "1.2.0-f410-fixed";
#else
using ActiveNumericProfile = FloatProfile;
using ActivePiProfile = FloatProfile;
inline constexpr const char* CONTROL_PROFILE_NAME = "float";
inline constexpr const char* PI_PROFILE_NAME = "float";
inline constexpr const char* PI_FIRMWARE_VERSION = "1.2.0-f410-float";
#endif
} // namespace libecu
#endif
