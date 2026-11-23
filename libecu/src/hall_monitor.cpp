/** @file hall_monitor.cpp
 * @brief Float-facing boundary of the numerical Hall monitor core
 */
#include "../include/hall_monitor.hpp"

namespace libecu {

// The float API's 0.35f rounds differently from exact 35/100 in Q30.
HallMonitor::HallMonitor() noexcept : core_(constant<Core::Fraction>(0.35f)),
    params_{500000, 0.0f, 1, 300, 0.35f, 10.0f, true} {}

LIBECU_SLOW_PATH
bool HallMonitor::prepareParameters(const HallMonitorParams& params, PreparedParameters& prepared) noexcept {
    Core::Params typed{};
    typed.decay_time_us = params.decay_time_us;
    typed.invalid_debounce = params.invalid_debounce;
    typed.invalid_persist_time_us = params.invalid_persist_time_us;
    typed.require_drive_active = params.require_drive_active;
    if (!tryNumber(params.invalid_threshold, typed.invalid_threshold) ||
        !tryNumber(params.erratic_fraction, typed.erratic_fraction) ||
        !tryNumber(params.erratic_min_edges, typed.erratic_min_edges)) return false;
    if (!Core::prepareParameters(typed, prepared.core)) return false;
    prepared.params = params;
    return true;
}

bool HallMonitor::setParameters(const HallMonitorParams& params) noexcept {
    PreparedParameters prepared;
    if (!prepareParameters(params, prepared)) return false;
    applyPrepared(prepared);
    return true;
}

void HallMonitor::tick(bool active) noexcept { core_.tick(active); }
void HallMonitor::onPosition(uint8_t position, bool active) noexcept { core_.onPosition(position, active); }
void HallMonitor::clearFault() noexcept { core_.clearFault(); }
void HallMonitor::reset() noexcept { core_.reset(); }

HallMonitor::Info HallMonitor::getInfo() const noexcept {
    return describe(snapshot());
}
HallMonitor::Info HallMonitor::describe(const Core::Snapshot& state) noexcept {
    return Core::describe<Info>(state);
}

} // namespace libecu
