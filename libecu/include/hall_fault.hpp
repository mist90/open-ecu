/** @file hall_fault.hpp
 * @brief Hall sensor status codes independent of numerical representation
 */
#ifndef LIBECU_HALL_FAULT_HPP
#define LIBECU_HALL_FAULT_HPP
#include <cstdint>
namespace libecu {
inline constexpr uint8_t HALL_POSITION_INVALID = 0xff;
enum class HallFault : uint8_t { NONE = 0, INVALID_CODE, ERRATIC_SEQUENCE };
} // namespace libecu
#endif
