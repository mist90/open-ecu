/** @file main.cpp
 * @brief Configuration publication preserves ISR-owned history and coefficient rounding
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <tuple>
#include "ecu_config.hpp"
#include "numeric_ops.hpp"
// Compare every live field, including sub-LSB accumulators and debounce/lock
// history that a rounded telemetry snapshot cannot expose.
#define private public
#include "bldc_controller.hpp"
#undef private

using namespace libecu;
static unsigned checks = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}

static unsigned lock_depth = 0, lock_count = 0;
static void (*before_lock)() = nullptr;
extern "C" void disable_interrupts() {
    if (before_lock) before_lock(); // An ISR arriving immediately before masking.
    ++lock_depth;
    ++lock_count;
}
extern "C" void enable_interrupts() {
    check(lock_depth == 1, "setter uses one non-nested publication critical section");
    --lock_depth;
}
uint32_t time_us() { return 0; }

template<class Params> static auto paramsTuple(const Params& p) {
    return std::make_tuple(p.decay_time_us, p.invalid_threshold, p.invalid_debounce,
                          p.invalid_persist_time_us, p.erratic_fraction, p.erratic_min_edges,
                          p.require_drive_active);
}
template<class Core> static auto hallConfig(const Core& c) {
    return std::make_tuple(paramsTuple(c.params_), c.alpha_, c.keep_, c.persistence_ticks_);
}
template<class Core> static auto hallLive(const Core& c) {
    return std::make_tuple(c.invalid_score_, c.edges_accum_, c.reversals_accum_, c.invalid_ticks_,
                          c.invalid_events_, c.edges_, c.invalid_run_, c.last_position_, c.last_sign_,
                          c.have_last_, c.last_invalid_, c.invalid_counted_, HallFault(c.fault_));
}
template<class Core> static auto pllConfig(const Core& c) { return std::make_tuple(c.kp_, c.ki_, c.ki_step_); }
template<class Core> static auto pllLive(const Core& c) {
    return std::make_tuple(c.hall_, c.inverse_, c.synchronized_, c.use_pll_, c.integral_limit_,
                          c.phase_, c.speed_, c.integral_, c.lock_error_);
}

template<class Profile, uint32_t Hz> static void hallCore() {
    using Core = HallMonitorCore<Profile, Hz>;
    using Fraction = typename Core::Fraction;
    Core monitor;
    auto original = monitor.getParameters();
    original.erratic_fraction = Fraction{};
    original.invalid_persist_time_us = 1000000;
    check(monitor.setParameters(original), "initial Hall tuning accepted");
    monitor.onPosition(1, true); monitor.onPosition(2, true); monitor.tick(true);
    const auto config_before = hallConfig(monitor);
    const auto live_before = hallLive(monitor);

    auto candidate = original;
    candidate.decay_time_us = 123457;
    candidate.invalid_debounce = 2;
    candidate.invalid_persist_time_us = 1;
    typename Core::PreparedParameters prepared;
    check(Core::prepareParameters(candidate, prepared), "prepare Hall parameters");
    check(hallConfig(monitor) == config_before && hallLive(monitor) == live_before,
          "Hall preparation changes neither live state nor active configuration");

    // IRQ work between preparation and publication: reversal history, scored
    // invalid episode, persistence age and a last valid position all change.
    monitor.onPosition(4, true); monitor.onPosition(3, true);
    monitor.onPosition(255, true); monitor.onPosition(255, true);
    for (unsigned i = 0; i < 3; ++i) monitor.tick(true);
    const auto live_at_commit = hallLive(monitor);
    check(live_at_commit != live_before, "Hall preemption actually advances live history");
    monitor.applyPrepared(prepared);
    check(hallLive(monitor) == live_at_commit, "Hall commit retains every ISR-owned field");
    check(paramsTuple(monitor.getParameters()) == paramsTuple(candidate), "Hall commit installs the candidate");
    check(!monitor.isFaulted(), "publishing a shorter persistence does not evaluate or reset live state");
    monitor.tick(true);
    check(monitor.faultCause() == HallFault::INVALID_CODE, "next tick applies new persistence to preserved age");
    const auto faulted = hallLive(monitor);
    check(monitor.setParameters(original) && hallLive(monitor) == faulted, "convenience setter preserves a latched fault");

    for (uint32_t tau : {0u, 1u, 33u, 1000u, 123457u, 500000u}) {
        candidate.decay_time_us = tau;
        candidate.invalid_persist_time_us = UINT32_MAX;
        check(Core::prepareParameters(candidate, prepared), "Hall coefficient boundary accepted");
        const uint64_t window = uint64_t(Hz) * (tau ? tau : 1000u);
        if constexpr (std::is_same<Fraction, float>::value) {
            const float alpha = 1000000.0f / (float(Hz) * (tau ? tau : 1000u));
            check(prepared.alpha == alpha && prepared.keep == std::clamp(1.0f - alpha, 0.0f, 1.0f),
                  "float Hall coefficients retain the original operation order and clamp");
        } else {
            const uint64_t raw = (uint64_t(1000000) * Fraction::SCALE + window / 2) / window;
            const int64_t alpha = std::min<uint64_t>(raw, Fraction::SCALE);
            check(prepared.alpha.raw() == alpha && prepared.keep.raw() == int64_t(Fraction::SCALE) - alpha,
                  "fixed Hall coefficients match independent rational rounding");
        }
        check(prepared.persistence_ticks == std::min<uint64_t>(UINT32_MAX,
              (uint64_t(UINT32_MAX) * Hz + 999999) / 1000000), "persistence still rounds up in sample-clock units");
    }
    if constexpr (!std::is_same<typename Core::Count, float>::value) {
        constexpr uint64_t max_events = uint64_t(std::numeric_limits<typename Core::Count::StorageType>::max())
                                        / Core::Count::SCALE;
        candidate.decay_time_us = max_events * 1000000 / Hz;
        check(Core::prepareParameters(candidate, prepared), "largest fixed Hall event window accepted");
        const auto output_before = std::make_tuple(paramsTuple(prepared.params), prepared.alpha,
                                                   prepared.keep, prepared.persistence_ticks);
        const auto active_before = hallConfig(monitor);
        ++candidate.decay_time_us;
        check(!Core::prepareParameters(candidate, prepared), "oversized Hall event window rejected");
        check(std::make_tuple(paramsTuple(prepared.params), prepared.alpha, prepared.keep,
                              prepared.persistence_ticks) == output_before, "invalid prepare leaves its output unchanged");
        check(!monitor.setParameters(candidate) && hallConfig(monitor) == active_before && hallLive(monitor) == faulted,
              "invalid Hall convenience transaction changes no configuration or live state");
    }
}

template<class Profile, uint32_t Hz> static void pllCore() {
    using Core = MotorPLLCore<Profile, Hz>;
    using Gain = typename Core::Gain;
    using StepGain = typename Core::StepGain;
    Core pll(number<typename Core::Speed>(2400), true);
    pll.setUsePLL(true);
    pll.updateHall(1); pll.updateTick();
    const auto before = pllLive(pll);
    const auto gains_before = pllConfig(pll);
    const auto prepared = Core::prepareGains(number<Gain>(151, 2), number<Gain>(10001, 2));
    check(pllLive(pll) == before && pllConfig(pll) == gains_before, "PLL preparation is independent of live state");
    for (unsigned tick = 0; tick < 317; ++tick) {
        pll.updateHall((tick / 53) % 6); pll.updateTick();
    }
    const auto at_commit = pllLive(pll);
    check(at_commit != before, "PLL preemption advances phase, integral and lock history");
    pll.applyPrepared(prepared);
    check(pllLive(pll) == at_commit, "PLL gain commit retains every tracking field and mode");
    check(pllConfig(pll) == std::make_tuple(prepared.kp, prepared.ki, prepared.ki_step), "PLL publishes all three gains");

    // Re-publishing the same gains must be indistinguishable from no setter,
    // including high precision state that becomes visible only on later ticks.
    Core uninterrupted = pll;
    const auto same = Core::prepareGains(pll.kp_, pll.ki_);
    for (unsigned tick = 0; tick < 400; ++tick) {
        const uint8_t hall = (tick / 37 + 2) % 6;
        pll.updateHall(hall); uninterrupted.updateHall(hall);
        pll.updateTick(); uninterrupted.updateTick();
        if (tick == 79) pll.applyPrepared(same);
        check(pllLive(pll) == pllLive(uninterrupted), "PLL publication retains the exact future trajectory");
    }
    for (int numerator : {-10001, -1, 0, 1, 10001, 250001}) {
        const Gain ki = number<Gain>(numerator, 2);
        const auto gains = Core::prepareGains(number<Gain>(100), ki);
        if constexpr (std::is_same<Gain, float>::value) {
            check(gains.ki_step == ki / float(Hz), "float PLL step coefficient retains the original division");
        } else {
            const int64_t value = int64_t(ki.raw()) * (StepGain::SCALE / Gain::SCALE);
            const int64_t rounded = value < 0 ? -((-value + Hz / 2) / Hz) : (value + Hz / 2) / Hz;
            check(gains.ki_step.raw() == rounded, "fixed PLL step uses exact signed rounding, not a float round trip");
        }
    }
}

static void wrappers() {
    HallMonitor monitor;
    monitor.onPosition(0, true); monitor.onPosition(1, true);
    auto params = monitor.getParameters();
    params.decay_time_us = 135791;
    HallMonitor::PreparedParameters hall_prepared;
    check(HallMonitor::prepareParameters(params, hall_prepared), "wrapper prepares Hall coefficients");
    monitor.onPosition(255, true); monitor.tick(true);
    const auto hall_before = hallLive(monitor.core_);
    monitor.applyPrepared(hall_prepared);
    check(hallLive(monitor.core_) == hall_before && paramsTuple(monitor.getParameters()) == paramsTuple(params),
          "Hall wrapper publishes float-facing params without replacing the core");
    const auto hall_config_before = hallConfig(monitor.core_);
    const auto hall_output_before = paramsTuple(hall_prepared.params);
    for (float invalid : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        auto rejected = params;
        rejected.erratic_fraction = invalid;
        check(!HallMonitor::prepareParameters(rejected, hall_prepared) && !monitor.setParameters(rejected),
              "non-finite Hall parameter rejected before publication");
        check(hallLive(monitor.core_) == hall_before && hallConfig(monitor.core_) == hall_config_before &&
              paramsTuple(hall_prepared.params) == hall_output_before, "invalid Hall wrapper transaction is unchanged");
    }
    params.invalid_threshold = -1.0f;
    params.erratic_fraction = -0.5f;
    params.erratic_min_edges = -1.0f;
    check(monitor.setParameters(params), "existing acceptance of finite negative Hall thresholds is retained");

    MotorPLL pll(2400.0f, true);
    MotorPLL::PreparedGains gains;
    check(MotorPLL::prepareGains(-13.25f, 5000.125f, gains), "wrapper prepares continuous PLL gains");
    pll.setUsePLL(true); pll.updateHall(3); pll.updateTick();
    const auto pll_before = pllLive(pll.core_);
    pll.applyPrepared(gains);
    check(pllLive(pll.core_) == pll_before, "PLL wrapper does not reset tracking when installing gains");
    const auto gains_before = pllConfig(pll.core_);
    const auto prepared_before = std::make_tuple(gains.kp, gains.ki, gains.ki_step);
    for (float invalid : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        check(!MotorPLL::prepareGains(invalid, 1000.0f, gains) && !pll.setGains(1.0f, invalid),
              "non-finite PLL gains rejected");
        check(pllLive(pll.core_) == pll_before && pllConfig(pll.core_) == gains_before &&
              std::make_tuple(gains.kp, gains.ki, gains.ki_step) == prepared_before,
              "invalid PLL transaction leaves gain output, active gains and tracking unchanged");
    }
#ifdef ECU_FIXED_NUMERIC
    check(!pll.setGains(131072.0f, 1.0f) && !pll.setGains(1.0f, 131072.0f), "fixed PLL gain range checks retained");
    params.invalid_threshold = 32768.0f;
    check(!monitor.setParameters(params), "fixed Hall count range check retained");
#endif
#ifdef TEST_NARROW_PLL_STEP
    check(!MotorPLL::prepareGains(100.0f, 3.0f * ActiveTiming::CURRENT_HZ, gains),
          "StepGain representability check remains even when continuous Ki fits");
    check(std::make_tuple(gains.kp, gains.ki, gains.ki_step) == prepared_before,
          "StepGain range failure is transactional");
#endif
}

#ifndef TEST_NARROW_PLL_STEP
class Pwm final : public PwmInterface {
public:
    bool initialize(uint32_t, uint16_t) override { return true; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float, float, float) override {}
    void enable(bool) override {}
};
class Hall final : public HallInterface {
public:
    bool initialize() override { return true; }
    uint8_t getPosition() override { return 0; }
};
static BldcController* boundary_controller = nullptr;
static void advanceAtBoundary() {
    check(lock_depth == 0, "ISR runs before the setter masks interrupts");
    auto& hall = boundary_controller->hall_monitor_;
    auto& pll = boundary_controller->motor_pll_;
    check(hall.getParameters().decay_time_us == 500000, "Hall preparation has not prematurely published");
    check(pll.core_.kp_ == number<MotorPLL::Core::Gain>(100), "PLL preparation has not prematurely published");
    hall.onPosition(4, true); hall.onPosition(3, true); hall.onPosition(255, true); hall.tick(true);
    pll.updateHall(3); pll.updateTick();
}
static void controllerBoundary() {
    Pwm pwm;
    Hall hall;
    MotorControlParams params{};
    params.num_poles = 20;
    params.max_speed_rps = 10.0f;
    BldcController controller(pwm, hall, params);
    boundary_controller = &controller;
    before_lock = advanceAtBoundary;
    auto tuning = controller.hall_monitor_.getParameters();
    tuning.decay_time_us = 123457;
    const unsigned locks_before = lock_count;
    check(controller.setHallMonitorParams(tuning), "controller Hall setter accepts prepared tuning");
    before_lock = nullptr;
    check(lock_count == locks_before + 1 && lock_depth == 0, "Hall controller masks only one commit");
    check(controller.hall_monitor_.core_.last_position_ == 3 && controller.hall_monitor_.core_.invalid_ticks_ == 1 &&
          controller.hall_monitor_.core_.invalid_events_ == 1, "Hall controller retains the ISR at the commit boundary");

    controller.hall_monitor_.setParameters(HallMonitorParams{500000, 0.0f, 1, 300, 0.35f, 10.0f, true});
    before_lock = advanceAtBoundary;
    const auto pll_before = pllLive(controller.motor_pll_.core_);
    check(controller.setPllGains(123.0f, 4567.0f), "controller PLL setter accepts prepared gains");
    before_lock = nullptr;
    check(lock_count == locks_before + 2 && lock_depth == 0, "PLL controller masks only one commit");
    check(pllLive(controller.motor_pll_.core_) != pll_before && controller.motor_pll_.getHallStateRaw() == 3,
          "PLL controller retains tracking advanced at the commit boundary");
    const auto hall_state = hallLive(controller.hall_monitor_.core_);
    const auto pll_state = pllLive(controller.motor_pll_.core_);
    tuning.invalid_threshold = std::numeric_limits<float>::quiet_NaN();
    check(!controller.setHallMonitorParams(tuning) &&
          !controller.setPllGains(std::numeric_limits<float>::infinity(), 1.0f), "controller rejects invalid transactions");
    check(lock_count == locks_before + 2 && hallLive(controller.hall_monitor_.core_) == hall_state &&
          pllLive(controller.motor_pll_.core_) == pll_state, "invalid controller parameters never enter a critical section");
}
#endif

int main() {
    hallCore<FloatProfile, 8000>(); hallCore<FloatProfile, 16000>(); hallCore<FloatProfile, 20000>();
    hallCore<FixedProfile, 8000>(); hallCore<FixedProfile, 16000>(); hallCore<FixedProfile, 20000>();
    pllCore<FloatProfile, 8000>(); pllCore<FloatProfile, 16000>(); pllCore<FloatProfile, 20000>();
    pllCore<FixedProfile, 8000>(); pllCore<FixedProfile, 16000>(); pllCore<FixedProfile, 20000>();
    wrappers();
#ifndef TEST_NARROW_PLL_STEP
    controllerBoundary();
#endif
    std::printf("Parameter publication %s, %u Hz: %u checks passed\n",
                CONTROL_PROFILE_NAME, unsigned(ActiveTiming::CURRENT_HZ), checks);
}
