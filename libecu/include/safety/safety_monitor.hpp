/**
 * @file safety_monitor.hpp
 * @brief Safety monitoring system for BLDC motor control
 */

#ifndef LIBECU_SAFETY_MONITOR_HPP
#define LIBECU_SAFETY_MONITOR_HPP

#include <cstdint>

namespace libecu {

/**
 * @brief Safety fault types
 */
enum class SafetyFault : uint8_t {
    NONE = 0,
    OVERCURRENT = 1,
    OVERTEMPERATURE = 2,
    UNDERVOLTAGE = 3,
    OVERVOLTAGE = 4,
    HALL_SENSOR_FAULT = 5,
    EMERGENCY_STOP = 6
};

/**
 * @brief Safety limits configuration
 */
struct SafetyLimits {
    float max_current;      ///< Maximum phase current (A)
    float max_temperature;  ///< Maximum temperature (°C)
};

/**
 * @brief Safety monitoring data
 */
struct SafetyData {
    float phase_u_current;  ///< Phase U current (A)
    float phase_v_current;  ///< Phase V current (A)
    float phase_w_current;  ///< Phase W current (A)
    float temperature;      ///< Temperature (°C)
    float bus_voltage;      ///< Bus voltage (V)
    bool emergency_stop;    ///< Emergency stop signal
    bool hall_fault;        ///< Hall sensor fault
};

/**
 * @brief Safety monitoring system
 */
class SafetyMonitor {
public:
    /**
     * @brief Constructor
     * @param limits Safety limits
     */
    explicit SafetyMonitor(const SafetyLimits& limits);

    /**
     * @brief Update safety monitoring
     * @param data Safety monitoring data
     * @return Active safety fault (NONE if no fault)
     */
    SafetyFault update(const SafetyData& data);

    /**
     * @brief Clear safety fault
     * @param fault Fault to clear
     */
    void clearFault(SafetyFault fault);

    /**
     * @brief Check if system is safe
     * @return true if no active faults
     */
    bool isSafe() const { return active_fault_ == SafetyFault::NONE; }

    /**
     * @brief Get active fault
     * @return Current active fault
     */
    SafetyFault getActiveFault() const { return active_fault_; }

    /**
     * @brief Get fault count for specific fault type
     * @param fault Fault type
     * @return Number of times this fault occurred
     */
    uint32_t getFaultCount(SafetyFault fault) const;

    /**
     * @brief Reset all fault counters
     */
    void resetFaultCounters();

    /**
     * @brief Set emergency stop
     * @param active true to activate emergency stop
     */
    void setEmergencyStop(bool active) { emergency_stop_active_ = active; }

private:
    SafetyLimits limits_;
    SafetyFault active_fault_;
    uint32_t fault_counters_[7]; // One for each fault type
    uint32_t fault_timestamp_;
    bool emergency_stop_active_;

    /**
     * @brief Check for overcurrent fault
     * @param data Safety data
     * @return true if overcurrent detected
     */
    bool checkOvercurrent(const SafetyData& data);

    /**
     * @brief Check for temperature fault
     * @param data Safety data
     * @return true if overtemperature detected
     */
    bool checkTemperature(const SafetyData& data);

    /**
     * @brief Set active fault
     * @param fault Fault to set as active
     */
    void setFault(SafetyFault fault);
};

} // namespace libecu

#endif // LIBECU_SAFETY_MONITOR_HPP