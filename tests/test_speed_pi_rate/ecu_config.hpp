/** @file ecu_config.hpp @brief Fixed scooter host fixture; only the outer PI clock varies. */
#ifndef TEST_SPEED_PI_RATE_CONFIG_HPP
#define TEST_SPEED_PI_RATE_CONFIG_HPP

#include "control_timing.hpp"
#include "numeric_profiles.hpp"

namespace libecu {
using ActiveTiming = ControlTiming<16000, 100, TEST_SPEED_PI_HZ, 500>;
using ActiveNumericProfile = FixedProfile;
using ActivePiProfile = FixedProfile;
}
#endif
