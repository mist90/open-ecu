/** @file measured_dq_monitor.hpp @brief Signed, slow telemetry from the measured d/q feedback. */
#ifndef LIBECU_MEASURED_DQ_MONITOR_HPP
#define LIBECU_MEASURED_DQ_MONITOR_HPP

#include <cstdint>

namespace libecu {
class BldcController;

class MeasuredDqMonitor {
public:
    enum class Path { Unavailable, GatesOff, Measuring };
    struct Pair { int32_t d, q; }; // Signed Q20 amperes
    static constexpr Pair unavailable{INT32_MIN, INT32_MIN};

    /** The board reports its live ADC/bridge state; called under the short snapshot lock. */
    explicit MeasuredDqMonitor(Path (*path)() noexcept) noexcept : board_path_(path) {}
    /** Foreground reader holds its snapshot lock. Reading never adds a sample. */
    Pair read(const BldcController& motor, Path hardware) noexcept;

private:
    friend class BldcController;
    static Path path(const BldcController& motor, Path hardware) noexcept;
    void sample(const BldcController& motor) noexcept;
    void invalidate() noexcept { continuous_ = false; }

    Path (*const board_path_)() noexcept;
    // Only the 100 Hz service writes the ring, indices and mean. A drive transition
    // (including a higher-priority carrier park) only invalidates continuity.
    static constexpr unsigned window = 8;
    Pair samples_[window]{}, mean_{};
    uint8_t next_ = 0, count_ = 0;
    volatile bool continuous_ = false;
};
} // namespace libecu
#endif
