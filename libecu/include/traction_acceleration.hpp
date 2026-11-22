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
    void reset() noexcept { primed_ = false; }

    /** @param speed Filtered, signed mechanical RPS relative to the selected drive direction.
     *  @param step Maximum positive RPS change in one velocity tick; zero bypasses.
     *  @param applied Last speed PI output (A), after all output limits.
     *  @param maximum Configured current ceiling (A).
     */
    Current ceiling(Speed speed, Speed step, Current applied, Current maximum) noexcept {
        const Speed change = primed_ ? speed - previous_ : Speed{};
        previous_ = speed;
        primed_ = true;
        if (step <= Speed{}) return maximum;

        // Integrate signed acceleration error in speed units: dt already cancels.
        // Half-wave rectification mistakes periodic Hall/torque ripple for sustained
        // acceleration, so a loaded wheel can never recover its holding current.
        // A falling speed relaxes the ceiling; only the speed PI can request torque.
        const Speed error = step - change;
        const Current correction = multiplyTo<Current>(error, number_v<Speed, 2>); // A/(rev/s)
        return numericClamp(std::max(Current{}, applied) + correction, Current{}, maximum);
    }

private:
    Speed previous_{};
    bool primed_{};
};

} // namespace libecu
#endif // LIBECU_TRACTION_ACCELERATION_HPP
