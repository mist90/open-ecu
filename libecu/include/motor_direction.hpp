/** @file motor_direction.hpp
 * @brief Electrical drive direction, independent of numerical representation
 */
#ifndef LIBECU_MOTOR_DIRECTION_HPP
#define LIBECU_MOTOR_DIRECTION_HPP
#include <cstdint>
namespace libecu {
enum class DriveMode : uint8_t { FORWARD = 0, REVERSE = 1, NEUTRAL = 2 };
} // namespace libecu
#endif
