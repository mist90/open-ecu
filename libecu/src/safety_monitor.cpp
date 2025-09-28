/**
 * @file safety_monitor.cpp
 * @brief Implementation of safety monitoring system
 */

#include "../include/safety/safety_monitor.hpp"
#include <cmath>
#include <algorithm>

namespace libecu {

SafetyMonitor::SafetyMonitor(const SafetyLimits& limits)
    : limits_(limits)
    , active_fault_(SafetyFault::NONE)
    , fault_timestamp_(0)
    , emergency_stop_active_(false)
{
    resetFaultCounters();
}

SafetyFault SafetyMonitor::update(const SafetyData& data)
{
    // Check emergency stop first (highest priority)
    if (data.emergency_stop || emergency_stop_active_) {
        setFault(SafetyFault::EMERGENCY_STOP);
        return active_fault_;
    }
    
    // Check Hall sensor fault
    if (data.hall_fault) {
        setFault(SafetyFault::HALL_SENSOR_FAULT);
        return active_fault_;
    }
    
    // Check overcurrent
    if (checkOvercurrent(data)) {
        setFault(SafetyFault::OVERCURRENT);
        return active_fault_;
    }
    
    // Check overtemperature
    if (checkTemperature(data)) {
        setFault(SafetyFault::OVERTEMPERATURE);
        return active_fault_;
    }
    
    // Check voltage faults
    if (checkVoltage(data)) {
        // Fault type is set by checkVoltage function
        return active_fault_;
    }
    
    // No faults detected
    if (active_fault_ != SafetyFault::NONE) {
        active_fault_ = SafetyFault::NONE;
    }
    
    return active_fault_;
}

void SafetyMonitor::clearFault(SafetyFault fault)
{
    if (active_fault_ == fault) {
        active_fault_ = SafetyFault::NONE;
        fault_timestamp_ = 0;
    }
}

uint32_t SafetyMonitor::getFaultCount(SafetyFault fault) const
{
    uint8_t index = static_cast<uint8_t>(fault);
    if (index < 7) {
        return fault_counters_[index];
    }
    return 0;
}

void SafetyMonitor::resetFaultCounters()
{
    for (int i = 0; i < 7; i++) {
        fault_counters_[i] = 0;
    }
}

bool SafetyMonitor::checkOvercurrent(const SafetyData& data)
{
    // Check each phase current
    float u_current = std::abs(data.phase_u_current);
    float v_current = std::abs(data.phase_v_current);
    float w_current = std::abs(data.phase_w_current);
    
    float max_current = std::max(u_current, std::max(v_current, w_current));
    
    return max_current > limits_.max_current;
}

bool SafetyMonitor::checkTemperature(const SafetyData& data)
{
    return data.temperature > limits_.max_temperature;
}

bool SafetyMonitor::checkVoltage(const SafetyData& data)
{
    if (data.bus_voltage < limits_.min_voltage) {
        setFault(SafetyFault::UNDERVOLTAGE);
        return true;
    }
    
    if (data.bus_voltage > limits_.max_voltage) {
        setFault(SafetyFault::OVERVOLTAGE);
        return true;
    }
    
    return false;
}

void SafetyMonitor::setFault(SafetyFault fault)
{
    if (active_fault_ != fault) {
        active_fault_ = fault;
        fault_timestamp_ = 0; // Reset timestamp for new fault
        
        // Increment fault counter
        uint8_t index = static_cast<uint8_t>(fault);
        if (index < 7) {
            fault_counters_[index]++;
        }
    }
}

} // namespace libecu
