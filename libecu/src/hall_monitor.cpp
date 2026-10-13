/** @file hall_monitor.cpp
 * @brief Float-facing boundary of the numerical Hall monitor core
 */
#include "../include/hall_monitor.hpp"

namespace libecu {

HallMonitor::HallMonitor() noexcept {
    setParameters(HallMonitorParams{500000, 0.0f, 1, 300, 0.35f, 10.0f, true});
}

bool HallMonitor::setParameters(const HallMonitorParams& params) noexcept {
    Core::Params typed{};
    typed.decay_time_us = params.decay_time_us;
    typed.invalid_debounce = params.invalid_debounce;
    typed.invalid_persist_time_us = params.invalid_persist_time_us;
    typed.require_drive_active = params.require_drive_active;
    if (!tryNumber(params.invalid_threshold, typed.invalid_threshold) ||
        !tryNumber(params.erratic_fraction, typed.erratic_fraction) ||
        !tryNumber(params.erratic_min_edges, typed.erratic_min_edges)) return false;
    if (!core_.setParameters(typed)) return false;
    params_ = params;
    return true;
}

void HallMonitor::tick(bool active) noexcept { core_.tick(active); }
void HallMonitor::onPosition(uint8_t position, bool active) noexcept { core_.onPosition(position, active); }
void HallMonitor::clearFault() noexcept { core_.clearFault(); }
void HallMonitor::reset() noexcept { core_.reset(); }

HallMonitor::Info HallMonitor::getInfo() const noexcept {
    const auto info = core_.getInfo();
    return {numberCast<float>(info.invalid_score), numberCast<float>(info.erratic_score),
            numberCast<float>(info.edge_accum), numberCast<float>(info.reversal_accum),
            info.invalid_events, info.invalid_run, info.invalid_time_us, info.edges,
            info.last_position, info.fault};
}

} // namespace libecu
