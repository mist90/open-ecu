/** @file diagnostics.cpp @brief Count lock losses across an entire long observation window. */
#include "motor_pll_core.hpp"
#include <cmath>
#include <cstdio>

static unsigned losses, reseeds, failures;
namespace libecu {
void recordPllPhaseReset(bool lost_sync) noexcept {
    ++reseeds;
    if (lost_sync) ++losses;
}
}
static void check(bool ok, const char* message) {
    if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
}

template<class Profile>
static void exercise() {
    using namespace libecu;
    MotorPLLCore<Profile, 16000> pll(number<typename Profile::PllSpeed>(4800));
    pll.setUsePLL(true);
    losses = reseeds = 0;
    double phase = 0;
    auto healthy_tick = [&] {
        phase = std::fmod(phase + 190.0 / 16000, 6.0); // 95 mechanical RPM, 20 pole pairs.
        pll.updateHall(static_cast<uint8_t>(phase));
        pll.updateTick();
    };
    for (unsigned tick = 0; tick < 32000; ++tick) healthy_tick();
    check(pll.getInfo().is_sync, "healthy Hall sequence establishes PLL lock");
    const unsigned before_losses = losses, before_reseeds = reseeds;
    for (unsigned tick = 0; tick < 160000; ++tick) healthy_tick();
    check(losses == before_losses && reseeds == before_reseeds,
          "ten seconds of healthy 95-RPM tracking produces no hidden phase reseeds");
    unsigned transitions = 0;
    for (unsigned tick = 0; tick < 8000; ++tick) {
        const bool locked = pll.getInfo().is_sync;
        const unsigned old_losses = losses, old_reseeds = reseeds;
        const uint8_t opposite = (static_cast<unsigned>(numberCast<float>(pll.getAngle())) + 3) % 6;
        pll.updateHall(opposite);
        pll.updateTick();
        const auto after = pll.getInfo();
        const bool lost = locked && !after.is_sync;
        transitions += lost;
        check(losses - old_losses == unsigned(lost), "each lock loss is counted exactly once");
        check(reseeds - old_reseeds <= 1, "a tick cannot report two phase reseeds");
        if (reseeds != old_reseeds)
            check(after.angle == number<typename Profile::Angle>(opposite),
                  "phase-reset event corresponds to an actual Hall-angle reseed");
    }
    check(transitions && losses - before_losses == transitions && reseeds > before_reseeds,
          "sustained contradictory Hall readings exercise the loss/reset diagnostics");
}

int main() {
    exercise<libecu::FloatProfile>();
    exercise<libecu::FixedProfile>();
    std::printf("PLL event diagnostics float/fixed: %u failures\n", failures);
    return failures != 0;
}
