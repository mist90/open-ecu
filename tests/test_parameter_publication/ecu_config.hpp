/** @file ecu_config.hpp
 * @brief Exercise publication with each supported current clock and numeric profile
 */
#ifndef TEST_PARAMETER_PUBLICATION_CONFIG_HPP
#define TEST_PARAMETER_PUBLICATION_CONFIG_HPP
#include "control_timing.hpp"
#include "numeric_profiles.hpp"
#ifndef TEST_CURRENT_HZ
#define TEST_CURRENT_HZ 20000
#endif
namespace libecu {
using ActiveTiming = ControlTiming<TEST_CURRENT_HZ, 100>;
#if defined(TEST_NARROW_PLL_STEP)
struct NarrowStepProfile : FixedProfile { using PllStepGain = FixedPointType<30>; };
using ActiveNumericProfile = NarrowStepProfile;
inline constexpr const char* CONTROL_PROFILE_NAME = "fixed-narrow-step";
#elif defined(ECU_FIXED_NUMERIC)
using ActiveNumericProfile = FixedProfile;
inline constexpr const char* CONTROL_PROFILE_NAME = "fixed";
#else
using ActiveNumericProfile = FloatProfile;
inline constexpr const char* CONTROL_PROFILE_NAME = "float";
#endif
#if defined(ECU_FIXED_NUMERIC)
using ActivePiProfile = FixedProfile;
inline constexpr const char* PI_PROFILE_NAME = "fixed";
#else
using ActivePiProfile = FloatProfile;
inline constexpr const char* PI_PROFILE_NAME = "float";
#endif
inline constexpr const char* PI_FIRMWARE_VERSION = "parameter-publication-test";
}
#endif
