/**
 * @file ecu_config.hpp
 * @brief HAL-independent compile-time configuration supplied by this board port
 */
#ifndef OPEN_ECU_STM32G431_CONFIG_HPP
#define OPEN_ECU_STM32G431_CONFIG_HPP

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
inline constexpr const char* CONTROL_PROFILE_NAME = "fixed";
#else
using ActiveNumericProfile = FloatProfile;
inline constexpr const char* CONTROL_PROFILE_NAME = "float";
#endif
#if defined(ECU_FIXED_PI) || defined(ECU_FIXED_NUMERIC)
using ActivePiProfile = FixedProfile;
inline constexpr const char* PI_PROFILE_NAME = "fixed";
#ifdef ECU_FIXED_NUMERIC
inline constexpr const char* PI_FIRMWARE_VERSION = "1.2.0-control-fixed";
#else
inline constexpr const char* PI_FIRMWARE_VERSION = "1.2.0-pi-fixed";
#endif
#else
using ActivePiProfile = FloatProfile;
inline constexpr const char* PI_PROFILE_NAME = "float";
inline constexpr const char* PI_FIRMWARE_VERSION = "1.2.0-control-float";
#endif
} // namespace libecu

#endif // OPEN_ECU_STM32G431_CONFIG_HPP
