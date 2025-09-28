/**
 * @file hall_interface.hpp
 * @brief Platform-independent Hall sensor interface
 */

#ifndef LIBECU_HALL_INTERFACE_HPP
#define LIBECU_HALL_INTERFACE_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Hall sensor state representation
 */
struct HallState {
    bool hall_a;    ///< Hall sensor A state
    bool hall_b;    ///< Hall sensor B state  
    bool hall_c;    ///< Hall sensor C state
    
    /**
     * @brief Get 3-bit Hall state value
     * @return Hall state as 3-bit value (0-7)
     */
    uint8_t getValue() const {
        return (hall_c << 2) | (hall_b << 1) | hall_a;
    }
};

/**
 * @brief Motor position based on Hall sensors
 */
enum class MotorPosition : uint8_t {
    INVALID = 0,
    POSITION_1 = 1,  ///< Hall state 001
    POSITION_2 = 2,  ///< Hall state 010  
    POSITION_3 = 3,  ///< Hall state 011
    POSITION_4 = 4,  ///< Hall state 100
    POSITION_5 = 5,  ///< Hall state 101
    POSITION_6 = 6   ///< Hall state 110
};

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
     * @brief Read current Hall sensor state
     * @return Current Hall sensor state
     */
    virtual HallState readState() = 0;

    /**
     * @brief Get motor position from Hall state
     * @param state Hall sensor state
     * @return Motor position
     */
    virtual MotorPosition getPosition(const HallState& state) = 0;

    /**
     * @brief Check if Hall state is valid
     * @param state Hall sensor state
     * @return true if valid (not 000 or 111)
     */
    virtual bool isValidState(const HallState& state) = 0;
};

} // namespace libecu

#endif // LIBECU_HALL_INTERFACE_HPP