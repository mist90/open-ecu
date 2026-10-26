/** @file motor_direction.hpp
 * @brief Electrical drive direction, independent of numerical representation
 */
#ifndef LIBECU_MOTOR_DIRECTION_HPP
#define LIBECU_MOTOR_DIRECTION_HPP
#include <cstdint>
namespace libecu {
enum class DriveMode : uint8_t { FORWARD = 0, REVERSE = 1, NEUTRAL = 2 };
/** @brief Motor-specific six-step torque lead relative to a Hall-sector angle. */
constexpr int commutationOffset(DriveMode mode, bool inverse) noexcept {
    if (mode == DriveMode::FORWARD) return inverse ? -2 : 1;
    if (mode == DriveMode::REVERSE) return inverse ? 1 : -2;
    return 0;
}
} // namespace libecu
#endif
