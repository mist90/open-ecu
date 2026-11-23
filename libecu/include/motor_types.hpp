/** @file motor_types.hpp
 * @brief Electrical algorithms shared by every numerical profile
 */
#ifndef LIBECU_MOTOR_TYPES_HPP
#define LIBECU_MOTOR_TYPES_HPP
#include "motor_direction.hpp"
namespace libecu {
enum class DriveAlgorithm : uint8_t { SIX_STEP = 0, FOC = 1 };
} // namespace libecu
#endif
