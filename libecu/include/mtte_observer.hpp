/**
 * @file mtte_observer.hpp
 * @brief Rolling-wheel MTTE (Yin & Hori, doi:10.5772/6962, E6/E8/E17).
 * No board I/O. The result is an upper bound for a speed PI, never a torque command.
 */
#ifndef LIBECU_MTTE_OBSERVER_HPP
#define LIBECU_MTTE_OBSERVER_HPP

#include "motor_direction.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace libecu {

class MtteObserver {
public:
    static constexpr unsigned RATE_HZ = 100;
    static constexpr float DT = 1.0f / RATE_HZ;
    static constexpr float TWO_PI = 6.2831853071795864769f;
    static constexpr unsigned PROBE_ON_TICKS = 4, PROBE_PERIOD_TICKS = 64;

    struct Parameters {
        float wheel_inertia_kg_m2{}, kt_nm_per_a{}, radius_m{}, mass_kg{};
        float relaxation{};                  ///< E7: chassis / wheel peripheral acceleration, (0,1].
        float pll_kp{}, pll_ki{};            ///< Gains of the locked integral-speed Hall PLL.
        float filter_tau_s{};                ///< Additional common LPF for speed and measured torque.
        float min_rolling_rps{};             ///< No standstill launch; both signals need a qualified handoff.
        uint32_t settling_ticks{};           ///< Contiguous valid intervals after seeding an endpoint.
        float rise_gain_s{}, rise_max_a{};   ///< E17; explicit time gain and independent stimulus bound.
        float probe_a{};                     ///< Optional rolling re-acquisition allowance; zero disables.
    };

    struct Frame {
        DriveMode direction{DriveMode::NEUTRAL};
        uint32_t speed_tick{}, iq_tick{}; ///< Common clock: speed endpoint k, current averaged over (k-1,k].
        float hall_rps{};                 ///< Signed mechanical PLL speed, before speed-loop LPF.
        float mean_iq_a{};                ///< Signed measured Park q, not demand, battery current or |Iq|.
        bool hall_valid{};                ///< Locked, legal sequence, sufficiently recent physical edge.
        bool iq_complete{};               ///< Every contributing interval sample valid, same drive direction.
    };

    enum class State { Unavailable, Settling, Tracking };

    struct Result {
        State state = State::Unavailable;
        float acceleration_rad_s2{}, motor_nm{}, force_n{}, tmax_nm{}; // Signed estimates only in Tracking.
        float rise_a{}, probe_a{};     // Policy allowances, explicitly not measured grip.
        float positive_ceiling_a{};    // Zero in Unavailable/Settling. PI retains its braking limit.
    };

    /** @brief Install a complete SI calibration. A rejected configuration disables the observer. */
    bool configure(const Parameters& p) noexcept {
        reset();
        configured_ = false;
        const auto positive = [](float v) { return std::isfinite(v) && v > 0.0f; };
        const auto nonnegative = [](float v) { return std::isfinite(v) && v >= 0.0f; };
        if (!positive(p.wheel_inertia_kg_m2) || !positive(p.kt_nm_per_a) || !positive(p.radius_m) ||
            !positive(p.mass_kg) || !(p.relaxation > 0.0f && p.relaxation <= 1.0f) ||
            !positive(p.pll_kp) || !positive(p.pll_ki) || !positive(p.filter_tau_s) ||
            !positive(p.min_rolling_rps) || p.settling_ticks == 0 ||
            !nonnegative(p.rise_gain_s) || !nonnegative(p.rise_max_a) || !nonnegative(p.probe_a)) return false;

        // A speed read from MotorPLLCore is its integral, not instantaneous shaft speed.
        // Match the nominal locked transfer Ki/(s² + Kp*s + Ki) on the interval-mean
        // current stream (bilinear, 100 Hz). This does not undo tracking loss or sector errors.
        constexpr float c = 2.0f * RATE_HZ;
        const float denominator = c*c + c*p.pll_kp + p.pll_ki;
        pll_b_ = p.pll_ki / denominator;
        pll_a1_ = 2.0f * (p.pll_ki - c*c) / denominator;
        pll_a2_ = (c*c - c*p.pll_kp + p.pll_ki) / denominator;
        // Reject numerically singular/unstable digital filters, including coefficient underflow.
        if (!positive(pll_b_) || !(1.0f + pll_a1_ + pll_a2_ > 0.0f &&
            1.0f - pll_a1_ + pll_a2_ > 0.0f && std::abs(pll_a2_) < 1.0f)) return false;
        const float reflected = p.relaxation * p.mass_kg * p.radius_m * p.radius_m;
        if (!positive(reflected)) return false;
        tmax_gain_ = 1.0f + p.wheel_inertia_kg_m2 / reflected;
        lpf_ = DT / (p.filter_tau_s + DT);
        if (!positive(tmax_gain_) || !positive(lpf_)) return false;
        params_ = p;
        configured_ = true;
        return true;
    }

    /** @brief Discard all history at a mode/algorithm/clock handoff; retain calibration. */
    void reset() noexcept {
        primed_ = false;
        intervals_ = 0;
        x1_ = x2_ = y1_ = y2_ = accel_ = motor_nm_ = 0.0f;
    }

    /**
     * @param now Current 100-Hz tick (wrap allowed); call once per velocity interval.
     * @param f Coherent pair of measurements in the same native rotor frame.
     * @param ordinary_a Signed PI demand before MTTE, subject to existing limits.
     * @param existing_max_a Independently available positive current, including any prior limiter.
     * @return Positive ceiling for the PI's own saturation/anti-windup. No adhesion estimate on stale data.
     */
    Result update(uint32_t now, const Frame& f, float ordinary_a, float existing_max_a) noexcept {
        Result out;
        if (!configured_ || (f.direction != DriveMode::FORWARD && f.direction != DriveMode::REVERSE) ||
            f.speed_tick != now || f.iq_tick != now || !f.hall_valid || !f.iq_complete ||
            !std::isfinite(f.hall_rps) || !std::isfinite(f.mean_iq_a) ||
            !std::isfinite(ordinary_a) || !std::isfinite(existing_max_a) || existing_max_a < 0.0f) {
            reset();
            return out;
        }
        const bool reverse = f.direction == DriveMode::REVERSE;
        const float n = reverse ? -f.hall_rps : f.hall_rps;
        const float iq = reverse ? -f.mean_iq_a : f.mean_iq_a;
        if (n < params_.min_rolling_rps) {
            reset();
            return out;
        }
        const float demand = std::clamp(ordinary_a, 0.0f, existing_max_a);
        out.state = State::Settling;
        if (!primed_ || f.direction != direction_ || now - last_tick_ != 1u) {
            reset(); // Never differentiate across a gap, direction change or first observation.
            primed_ = true;
            direction_ = f.direction;
            last_tick_ = now;
            last_rps_ = n;
            last_demand_ = demand;
            return out;
        }
        const float aligned_iq = pll_b_ * (iq + 2.0f*x1_ + x2_) - pll_a1_*y1_ - pll_a2_*y2_;
        x2_ = x1_; x1_ = iq;
        y2_ = y1_; y1_ = aligned_iq;
        motor_nm_ += lpf_ * (params_.kt_nm_per_a * aligned_iq - motor_nm_);
        // D(LPF(n)) == LPF(D(n)) for this fixed sample clock; seed derivative and torque at zero.
        accel_ += lpf_ * (TWO_PI * RATE_HZ * (n - last_rps_) - accel_);
        const float contact_nm = motor_nm_ - params_.wheel_inertia_kg_m2 * accel_;
        const float force_n = contact_nm / params_.radius_m; // E6, signed.
        const float tmax_nm = tmax_gain_ * contact_nm;       // E8, Fdr=0.
        const float slope = (demand - last_demand_) * RATE_HZ;
        last_demand_ = demand;
        last_rps_ = n;
        last_tick_ = now;
        const float tmax_a = tmax_nm / params_.kt_nm_per_a;
        if (!std::isfinite(force_n) || !std::isfinite(tmax_a) || !std::isfinite(slope)) {
            reset();
            return {};
        }
        if (intervals_ < params_.settling_ticks) ++intervals_;
        if (intervals_ < params_.settling_ticks) return out;

        out.state = State::Tracking;
        out.acceleration_rad_s2 = accel_;
        out.motor_nm = motor_nm_;
        out.force_n = force_n;
        out.tmax_nm = tmax_nm;
        if (demand > 0.0f) {
            // E17 in amperes (constant Kt cancels), on rising positive demand only.
            out.rise_a = std::min(params_.rise_max_a, params_.rise_gain_s * std::max(0.0f, slope));
            // Optional bounded excitation on a fixed clock, never a direct current request.
            // At steady demand and zero force, E8 and E17 alone cannot discover new grip.
            // 4/64 ticks, including across uint32 wrap; no stale/invalid/launch probe.
            if (now % PROBE_PERIOD_TICKS < PROBE_ON_TICKS) out.probe_a = params_.probe_a;
            const float e8_e17_a = std::max(0.0f, tmax_a + out.rise_a);
            out.positive_ceiling_a = std::min(existing_max_a, std::max(e8_e17_a, out.probe_a));
        }
        return out;
    }

private:
    Parameters params_{};
    float pll_b_{}, pll_a1_{}, pll_a2_{}, lpf_{}, tmax_gain_{};
    float x1_{}, x2_{}, y1_{}, y2_{}, accel_{}, motor_nm_{}, last_rps_{}, last_demand_{};
    uint32_t last_tick_{}, intervals_{};
    DriveMode direction_{DriveMode::NEUTRAL};
    bool configured_{}, primed_{};
};

} // namespace libecu
#endif // LIBECU_MTTE_OBSERVER_HPP
