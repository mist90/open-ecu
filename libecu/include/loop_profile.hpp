/** @file loop_profile.hpp @brief Optional board-owned current-loop timing markers. */
#ifndef LIBECU_LOOP_PROFILE_HPP
#define LIBECU_LOOP_PROFILE_HPP

namespace libecu {
enum class LoopStage : unsigned {
    Start, MarkerCost, Adc, Protection, HallGates, Currents, Hall, Pll, Dispatch,
    FocTrig, FocPark, FocFilter, FocPi, FocLimit, FocIntegral, FocInvPark,
    FocSvpwm, FocOutput, Pwm, Algorithm, Status, Capture, End, Count
};
#if LIBECU_LOOP_PROFILING
// The board supplies the clock/storage. Disabled builds have no calls or data.
void recordLoopStage(LoopStage stage) noexcept;
#else
[[gnu::always_inline]] inline void recordLoopStage(LoopStage) noexcept {}
#endif
}
#endif
