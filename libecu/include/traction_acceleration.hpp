/** @file traction_acceleration.hpp
 * @brief Positive torque ceiling from measured mechanical speed, on the velocity clock.
 */
#ifndef LIBECU_TRACTION_ACCELERATION_HPP
#define LIBECU_TRACTION_ACCELERATION_HPP

#include "numeric_ops.hpp"
#include <algorithm>

namespace libecu {

/** @brief Track the last applied current, so unused acceleration authority never accumulates. */
template<class Profile>
class TractionAcceleration {
    using Speed = typename Profile::Speed;
    using Current = typename Profile::Current;

public:
    struct Recovery {
        Speed lag{}, gain{}; // Mechanical RPS behind the ramp, A/(rev/s) of additional speed loss.
        Current max_rise_step{}; // Total A per slow tick. Zero keeps the original single-slope ceiling.
    };

    void reset() noexcept { primed_ = false; }

    /** @param speed Filtered, signed mechanical RPS relative to the selected drive direction.
     *  @param step Maximum positive RPS change in one velocity tick; zero bypasses.
     *  @param applied Last speed PI output (A), after all output limits.
     *  @param maximum Configured current ceiling (A).
     *  @param gain Current correction per unit of speed error (A/(rev/s)).
     */
    Current ceiling(Speed speed, Speed step, Current applied, Current maximum, Speed gain,
                    Speed lag = {}, Recovery recovery = {}) noexcept {
        const Speed change = primed_ ? speed - previous_ : Speed{};
        previous_ = speed;
        primed_ = true;
        if (step <= Speed{}) return maximum;

        // Integrate signed acceleration error in speed units: dt already cancels.
        // Half-wave rectification mistakes periodic Hall/torque ripple for sustained
        // acceleration, so a loaded wheel can never recover its holding current.
        // A falling speed relaxes the ceiling; only the speed PI can request torque.
        const Speed error = step - change;
        Current correction = multiplyTo<Current>(error, gain);
        if (recovery.max_rise_step > Current{}) {
            // Ignore ordinary Hall ripple near the target. On a sudden load step grant headroom,
            // never current directly. A release can still take all positive torque away in one tick.
            if (lag > recovery.lag && change < -step)
                correction += multiplyTo<Current>(-change - step, recovery.gain);
            correction = std::min(correction, recovery.max_rise_step);
        }
        return numericClamp(std::max(Current{}, applied) + correction, Current{}, maximum);
    }

private:
    Speed previous_{};
    bool primed_{};
};

} // namespace libecu
#endif // LIBECU_TRACTION_ACCELERATION_HPP
