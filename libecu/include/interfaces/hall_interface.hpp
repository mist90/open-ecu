/**
 * @file hall_interface.hpp
 * @brief Platform-independent Hall sensor interface
 */

#ifndef LIBECU_HALL_INTERFACE_HPP
#define LIBECU_HALL_INTERFACE_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Abstract interface for Hall sensor reading
 */
class HallInterface {
public:
    virtual ~HallInterface() = default;

    /**
     * @brief Initialize Hall sensor hardware
     * @return true if initialization successful
     */
    virtual bool initialize() = 0;

    /**
     * @brief Get motor position from Hall state
     * @param state Hall sensor state
     * @return Motor position
     */
    virtual uint8_t getPosition() = 0;
};

} // namespace libecu

#endif // LIBECU_HALL_INTERFACE_HPP