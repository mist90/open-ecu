/**
 * @file ecu_config.hpp
 * @brief HAL-independent compile-time configuration supplied by this board port
 */
#ifndef OPEN_ECU_STM32G431_CONFIG_HPP
#define OPEN_ECU_STM32G431_CONFIG_HPP

#include "control_timing.hpp"

namespace libecu {
using ActiveTiming = ControlTiming<20000, 100>;
} // namespace libecu

#endif // OPEN_ECU_STM32G431_CONFIG_HPP
