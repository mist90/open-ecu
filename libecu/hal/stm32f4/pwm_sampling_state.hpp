/** @file pwm_sampling_state.hpp
 * @brief TIM1 duty-generation bookkeeping, independent of hardware and ADC timing
 */
#ifndef LIBECU_STM32F4_PWM_SAMPLING_STATE_HPP
#define LIBECU_STM32F4_PWM_SAMPLING_STATE_HPP

#include <array>
#include <cstdint>

namespace libecu {

/** @brief One serialized PWM writer and one in-flight ADC sampling window.
 *  Caller supplies a coherent UIF/live-preload observation. Only reset() may
 *  establish known shadows; an unsupported CCR write invalidates that proof.
 */
struct PwmSamplingState {
    using Compare = std::array<uint32_t, 3>;
    struct Window {
        Compare opening{}, closing{};
        uint32_t generation = 0;
        bool known = false;
    };

    bool known = false;
    Compare active{}, preload{}, max_since_arm{};
    uint32_t generation = 0;

    /** @brief Establish actual shadows after initialization has loaded these preloads. */
    void reset(const Compare& initial = {}) noexcept {
        active = preload = max_since_arm = initial;
        ++generation; // Reinitialization also invalidates any older window.
        known = true;
    }

    /** @brief Observe UIF before clearing it; no preload writer may interleave. */
    void observeUpdate(bool update_seen, const Compare& live) noexcept {
        validateLive(live);
        if (known && update_seen) active = preload;
    }

    /** @brief Record one atomic preload write, after observing UIF with reload gated. */
    void recordWrite(const Compare& next) noexcept {
        include(active);
        include(next);
        preload = next;
        ++generation; // Counts even identical writes and A -> B -> A sequences.
    }

    /** @brief Arm the next source; update_before_source means an extremum precedes it.
     *  If UIF was false, the caller must not clear a subsequently arriving UIF.
     */
    Window armWindow(bool update_seen, bool update_before_source, const Compare& live) noexcept {
        observeUpdate(update_seen, live);
        Window window{update_before_source ? preload : active, preload, generation, known};
        if (known) {
            max_since_arm = window.opening;
            include(window.closing);
        }
        return window;
    }

    /** @brief Verify the saved generation; otherwise use conservative history/legacy guards. */
    bool matches(const Window& window, const Compare& live) noexcept {
        validateLive(live);
        return known && window.known && generation == window.generation && equal(live, window.closing);
    }

private:
    static bool equal(const Compare& a, const Compare& b) noexcept {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    }
    void include(const Compare& value) noexcept {
        for (unsigned phase = 0; phase < 3; ++phase)
            if (value[phase] > max_since_arm[phase]) max_since_arm[phase] = value[phase];
    }
    void validateLive(const Compare& live) noexcept {
        if (!equal(live, preload)) {
            include(active);
            include(preload);
            include(live);
            if (known) ++generation;
            known = false;
        }
    }
};

// ADC-only fixtures remain unknown and keep their conservative legacy path.
inline PwmSamplingState pwm_sampling_state{};

} // namespace libecu
#endif // LIBECU_STM32F4_PWM_SAMPLING_STATE_HPP
