/**
 * @file order_feedback.hpp
 * @brief Bounded diagnostic feedback at wheel order 144, sampled at 1 kHz.
 */
#ifndef LIBECU_ORDER_FEEDBACK_HPP
#define LIBECU_ORDER_FEEDBACK_HPP

#include <algorithm>
#include <cstdint>

namespace libecu {

/** @brief Track two current quadratures using relative wheel angle. Owned by one controller's SysTick. */
class Order144Feedback {
public:
    /**
     * @brief Return a current trim; leaving the test speed or load range clears the observer.
     * @param angle Native PLL angle in Q24 Hall sectors, wrapped into [0, 6).
     * @param speed Signed Hall sectors/s in Q14.
     * @param demand Requested q current in Q20 amperes.
     * @param iq Measured Q20 q current from the same snapshot as angle.
     * @param gain_q6 Signed Q6 feedback fraction. Zero is the selected bench setting.
     * @return Signed trim in Q20 amperes, limited to 1/8 A and 1/8 of the available demand.
     */
    int32_t step(int32_t angle, int32_t speed, int32_t demand, int32_t iq, int32_t gain_q6) noexcept {
        constexpr int32_t u = 1 << 14, a = 1 << 20, s = 1 << 24;
        if (speed <= 64 * u || speed >= 92 * u || demand <= 0 || demand >= 3 * a) {
            valid_ = false;
            return 0;
        }
        const int32_t input = std::clamp(iq, -4 * a, 4 * a) / 32;
        int32_t delta = angle - last_;
        last_ = angle;
        if (delta > 3 * s) delta -= 6 * s;
        if (delta < -3 * s) delta += 6 * s;
        if (!valid_ || delta < -s / 8 || delta > s / 4) {
            valid_ = true;
            phase_ = 0;
            mean_ = input;
            i_ = q_ = 0;
            return 0;
        }
        // At 20 pole pairs, order 144 makes 6/5 cycles per Hall sector. Five sectors
        // are an integer six cycles. Both mixing directions share the arbitrary initial phase.
        phase_ += delta;
        if (phase_ >= 5 * s) phase_ -= 5 * s;
        if (phase_ < 0) phase_ += 5 * s;
        const uint32_t ph = uint32_t(phase_ * 6 / 5);
        const int32_t x = input - mean_;
        mean_ += x / 16;
        const int32_t si = sine(ph), co = sine(ph + (1u << 22));
        i_ += ((x * si / 1024) - i_) / 64;
        q_ += ((x * co / 1024) - q_) / 64;

        // Predict the midpoint of the held sample. Use a wide sum for both quadratures.
        const uint32_t mid = ph + uint32_t((int64_t(speed) * 384 + 312) / 625);
        const int32_t estimate = int32_t((int64_t(i_) * sine(mid) + int64_t(q_) * sine(mid + (1u << 22))) / 512);
        const int32_t fade = std::min<int32_t>({256, (speed - 64 * u) >> 9, (92 * u - speed) >> 9});
        const int32_t limit = std::min({a / 8, demand / 8, (3 * a - demand) / 8});
        const int32_t base = int32_t(std::clamp<int64_t>(
            int64_t(estimate) * std::clamp<int32_t>(gain_q6, -64, 64) / 2, -limit, limit));
        return base * fade / 256;
    }

private:
    static int32_t sine(uint32_t phase) noexcept {
        const int32_t t = (int32_t((phase + (1u << 23)) & 0xffffffu) - (1 << 23)) >> 11;
        return t * (4096 - (t < 0 ? -t : t)) / 4096;
    }
    int32_t last_{}, phase_{}, mean_{}, i_{}, q_{};
    bool valid_{};
};

} // namespace libecu
#endif // LIBECU_ORDER_FEEDBACK_HPP
