/** @file arm.cpp @brief Executable Cortex-M3 probes of the fixed limiter. */
#include "polygon_voltage_limit.hpp"
using namespace libecu;
using P = FixedProfile;
static PolygonVoltageLimit<P> limiter;

extern "C" __attribute__((used, externally_visible)) void prepare_limit(int32_t limit) {
    limiter.prepare(P::Voltage::fromRaw(limit));
}
extern "C" __attribute__((used, externally_visible)) int32_t query_limit(int32_t d) {
    return limiter.qLimit(P::Voltage::fromRaw(d)).raw();
}
extern "C" __attribute__((used, externally_visible)) void apply_limit(int32_t d, int32_t q, int32_t* out) {
    const auto result = limiter.apply(P::VoltageRequest::fromRaw(d), P::VoltageRequest::fromRaw(q));
    out[0] = result.d.raw();
    out[1] = result.q.raw();
    out[2] = result.limited;
}
