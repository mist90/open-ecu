/** @file hall_period_speed.hpp
 * @brief Mechanical speed over one electrical turn. The carrier remains the sole Hall/PLL owner.
 */
#ifndef LIBECU_HALL_PERIOD_SPEED_HPP
#define LIBECU_HALL_PERIOD_SPEED_HPP

#include "numeric_ops.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <limits>

namespace libecu {

class HallPeriodSpeed {
    // Single Hall IRQ producer, one slow reader, bounded overwrite. Leaving spare slots
    // lets the producer continue during the unmasked scan of the latest 8 or 9 marks.
    static constexpr uint32_t CAPACITY = 16, MASK = CAPACITY - 1;
    static constexpr uint32_t MAX_GAP_US = 50000;
    static_assert(std::atomic<uint32_t>::is_always_lock_free);
    struct Slot { std::atomic<uint32_t> us{}, tag{}; };
    struct Mark {
        uint32_t us, tag;
        uint8_t sector() const noexcept { return uint8_t(tag & 7u); }
        bool matches(uint32_t ordinal) const noexcept { return (tag >> 3) == (ordinal & (UINT32_MAX >> 3)); }
    };
    Slot slots_[CAPACITY]{};
    std::atomic<uint32_t> written_{0};
    uint32_t oldest_ = 0;        // Slow reader's expiry/reset boundary.
    uint8_t last_irq_code_ = 7;  // IRQ-owned; a duplicate cannot refresh an edge.

    Mark read(uint32_t index) const noexcept {
        const auto& slot = slots_[index & MASK];
        return {slot.us.load(std::memory_order_relaxed), slot.tag.load(std::memory_order_relaxed)};
    }
    static int direction(uint8_t from, uint8_t to) noexcept {
        if (from > 5 || to > 5) return 0;
        const int d = int(to) - from;
        return d == 1 || d == -5 ? 1 : d == -1 || d == 5 ? -1 : 0;
    }

public:
    struct Window { uint32_t end, accepted; uint8_t sector; };

    /** @brief Record a real, decoded Hall IRQ only. No mask, arithmetic on speeds, or PWM-frame work. */
    void capture(uint32_t us, uint8_t sector, uint32_t accepted) noexcept {
        const uint8_t code = sector <= 5 ? sector : 7;
        if (last_irq_code_ == code) return;
        last_irq_code_ = code;
        const uint32_t i = written_.load(std::memory_order_relaxed);
        auto& slot = slots_[i & MASK];
        slot.us.store(us, std::memory_order_relaxed);
        slot.tag.store((accepted << 3) | code, std::memory_order_relaxed);
        written_.store(i + 1, std::memory_order_release);
    }

    /** @brief Pin the IRQ cursor alongside the carrier's accepted ordinal/sector under the reader's lock. */
    Window pin(uint32_t accepted, uint8_t sector) const noexcept {
        return {written_.load(std::memory_order_acquire), accepted, sector};
    }
    /** @brief Drop pre-reseed timestamps without touching the producer or clearing an array. */
    void discard() noexcept { oldest_ = written_.load(std::memory_order_relaxed); }

    /** @brief Unmasked slow tick. PLL and result are signed in the physical Hall sequence. */
    template<class Profile, uint32_t CarrierHz>
    typename Profile::Speed select(Window w, uint32_t now, uint8_t poles, typename Profile::Speed max_speed,
                                   typename Profile::Speed pll) noexcept {
        using Speed = typename Profile::Speed;
        using Fraction = typename Profile::Fraction;
        constexpr uint32_t frame_us = 1u + 999999u / CarrierHz;
        const uint32_t available = std::min(w.end - oldest_, CAPACITY);
        oldest_ = w.end - available;
        if (!available) return pll;
        uint32_t i = w.end - 1;
        Mark edge = read(i);
        if (now - edge.us > MAX_GAP_US || w.sector > 5) {
            oldest_ = w.end; // A long park (including one or more timer wraps) cannot revive this window.
            return pll;
        }
        if (available < 8 || !poles) return pll;
        Mark prev = read(i - 1);
        const int sign = direction(prev.sector(), edge.sector());
        if (!sign) return pll;
        if (w.sector != edge.sector()) {
            if (w.sector == prev.sector() && edge.matches(w.accepted + 1)) {
                // Latest IRQ precedes its PWM frame. Use the last *accepted* edge instead.
                if (available < 9) return pll;
                --i;
                edge = prev;
                prev = read(i - 1);
            } else if (direction(edge.sector(), w.sector) == sign) {
                // Carrier saw the next physical state before its EXTI ran.
                --w.accepted;
            } else return pll;
        }
        const uint32_t newest = edge.us;
        uint32_t longest_gap = 0;
        // Seven confirmed same-direction crossings bound six complete sector intervals.
        // Their predecessor must also be a confirmed frame; an EXTI-only bounce cannot
        // establish the direction of the first measured crossing after a reversal.
        for (unsigned n = 0; n < 7; ++n) {
            if (!edge.matches(w.accepted - n) || direction(prev.sector(), edge.sector()) != sign) return pll;
            if (n == 6) break;
            const uint32_t gap = edge.us - prev.us;
            if (!gap || gap > MAX_GAP_US) return pll;
            longest_gap = std::max(longest_gap, gap);
            edge = prev;
            --i;
            prev = read(i - 1);
        }
        if (!prev.matches(w.accepted - 7)) return pll;
        const uint32_t span = newest - edge.us; // Each of its six intervals is bounded above.
        const uint32_t fresh = std::min(MAX_GAP_US, 3u * span / 8u + frame_us);
        if (now - newest > fresh) {
            oldest_ = w.end; // Never reuse a turn already observed to have stopped.
            return pll;
        }
        if (longest_gap > fresh) return pll;
        // An EXTI can preempt the scan. Do not use slots it could have started overwriting.
        std::atomic_thread_fence(std::memory_order_acquire);
        if (written_.load(std::memory_order_relaxed) - w.end >= CAPACITY - 9) return pll;

        // 2e6/(span*poles) <= 2*max_speed allows real overspeed. Reject implausible
        // turns before dividing, even if their quotient wouldn't fit the Speed format.
        const uint32_t pole_us = span * poles; // At most 6 * 50000 * 255: fits int32.
        if constexpr (std::is_same_v<Speed, float>) {
            if (!(max_speed * pole_us >= 1000000.f)) return pll;
        } else {
            constexpr int64_t us_scale = 1000000 * int64_t(Speed::SCALE);
            constexpr auto max_raw = std::numeric_limits<typename Speed::StorageType>::max();
            constexpr auto min_pole_us = (2 * us_scale + max_raw - 1) / max_raw;
            if (pole_us < min_pole_us || fixed_detail::wideProduct(max_speed.raw(), int32_t(pole_us)) < us_scale)
                return pll;
        }
        using Hertz = std::conditional_t<std::is_same_v<Speed, float>, float, FixedPointType<0>>;
        const Speed magnitude = divideByInteger<Speed>(number_v<Hertz, 2000000>, pole_us);
        if (magnitude <= number_v<Speed, 1, 2>) return pll;
        const Speed candidate = sign > 0 ? magnitude : -magnitude;
        if (magnitude >= number_v<Speed, 1>) return candidate;
        const auto weight = numberCast<Fraction>(magnitude + magnitude - number_v<Speed, 1>);
        return pll + multiplyTo<Speed>(weight, candidate - pll);
    }
};

} // namespace libecu
#endif // LIBECU_HALL_PERIOD_SPEED_HPP
