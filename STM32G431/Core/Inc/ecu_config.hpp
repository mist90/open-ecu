/**
 * @file ecu_config.hpp
 * @brief HAL-independent compile-time configuration supplied by this board port
 */
#ifndef OPEN_ECU_STM32G431_CONFIG_HPP
#define OPEN_ECU_STM32G431_CONFIG_HPP

#include "control_timing.hpp"
#include "numeric_profiles.hpp"

namespace libecu {
using ActiveTiming = ControlTiming<20000, 100>;
#ifdef ECU_FIXED_PI
using ActivePiProfile = FixedProfile;
inline constexpr const char* PI_PROFILE_NAME = "fixed";
inline constexpr const char* PI_FIRMWARE_VERSION = "1.0.0-pi-fixed";
#else
using ActivePiProfile = FloatProfile;
inline constexpr const char* PI_PROFILE_NAME = "float";
inline constexpr const char* PI_FIRMWARE_VERSION = "1.0.0-pi-float";
#endif
} // namespace libecu

#endif // OPEN_ECU_STM32G431_CONFIG_HPP
