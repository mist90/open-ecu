/**
 * @file libecu.hpp
 * @brief Main header for libecu - Platform-independent BLDC motor control library
 */

#ifndef LIBECU_HPP
#define LIBECU_HPP

// Core interfaces
#include "interfaces/pwm_interface.hpp"
#include "interfaces/hall_interface.hpp"

// Motor control algorithms
#include "algorithms/commutation_controller.hpp"
#include "algorithms/pid_controller.hpp"

// Safety monitoring
#include "safety/safety_monitor.hpp"

/**
 * @namespace libecu
 * @brief Platform-independent BLDC motor control library
 *
 * The libecu library provides a complete set of algorithms and interfaces
 * for BLDC motor control that can be ported to different microcontrollers
 * by implementing the hardware abstraction layer (HAL) interfaces.
 *
 * Key features:
 * - 6-step commutation algorithm with Hall sensor feedback
 * - PID speed controller with anti-windup
 * - Comprehensive safety monitoring
 * - Real-time performance optimized for embedded systems
 * - Clean separation between algorithms and hardware
 */

namespace libecu {

/**
 * @brief Library version information
 */
constexpr const char* VERSION = "1.0.0";
constexpr uint32_t VERSION_MAJOR = 1;
constexpr uint32_t VERSION_MINOR = 0;
constexpr uint32_t VERSION_PATCH = 0;

} // namespace libecu

#endif // LIBECU_HPP