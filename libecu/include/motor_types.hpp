/** @file motor_types.hpp
 * @brief Electrical modes shared by every numerical profile
 */
#ifndef LIBECU_MOTOR_TYPES_HPP
#define LIBECU_MOTOR_TYPES_HPP
#include "motor_direction.hpp"
namespace libecu {
enum class ElectricMode : uint8_t { VOLTAGE_MODE = 0, CURRENT_MODE = 1 };
enum class DriveAlgorithm : uint8_t { SIX_STEP = 0, FOC = 1 };
} // namespace libecu
#endif
