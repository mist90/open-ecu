/** @file numeric.cpp
 * @brief Exact Hall score, lifecycle and fault-timing tests against a rational model
 */
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>
#include "hall_fault.hpp"
#include "control_timing.hpp"
#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"
// Seed full accumulator states, including public-count overflow, without
// millions of synthetic events. Do not change the production interface.
#define private public
#include "hall_monitor_core.hpp"
#undef private

using namespace libecu;
static uint64_t checks = 0;

static void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "Hall oracle failure at check %llu: %s\n",
                     static_cast<unsigned long long>(checks), message);
        std::exit(1);
    }
}

// Independent rational rounding/saturation. No production conversion,
// multiplication or shift helper participates in the expected results.
template<class T>
static int64_t rational(__int128 numerator, __int128 denominator = 1) {
    const __int128 result = numerator < 0 ? -((-numerator + denominator / 2) / denominator)
                                         : (numerator + denominator / 2) / denominator;
    using Raw = typename T::StorageType;
    return static_cast<int64_t>(std::clamp<__int128>(result, std::numeric_limits<Raw>::min(),
                                                  std::numeric_limits<Raw>::max()));
}

static uint64_t randomWord(uint64_t& seed) {
    seed ^= seed << 13;
    seed ^= seed >> 7;
    seed ^= seed << 17;
    return seed;
}

template<class Profile, uint32_t Hz>
struct Reference {
    using Monitor = HallMonitorCore<Profile, Hz>;
    using Count = typename Profile::Count;
    using State = typename Profile::CountAccumulator;
    using Fraction = typename Profile::Fraction;
    using Params = typename Monitor::Params;
    Params params{};
    int64_t alpha = 0;
    int64_t scores[3]{}; // Invalid, edges, reversals, in raw accumulator units.
    uint32_t persist = 0, ticks = 0, events = 0, edges = 0;
    uint8_t run = 0, position = 0;
    int8_t direction = 0;
    bool have_position = false, invalid = false, counted = false;
    HallFault fault = HallFault::NONE;

    Reference() { configure(Params{}); }
    bool configure(const Params& next) {
        const uint64_t window = uint64_t(Hz) * (next.decay_time_us ? next.decay_time_us : 1000);
        const uint64_t maximum = uint64_t(std::numeric_limits<typename Count::StorageType>::max())
                               / Count::SCALE;
        if (window > maximum * 1000000) return false;
        alpha = std::clamp<int64_t>(rational<Fraction>(__int128(1000000) * Fraction::SCALE, window),
                                   0, rational<Fraction>(Fraction::SCALE));
        persist = static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX,
                    (uint64_t(next.invalid_persist_time_us) * Hz + 999999) / 1000000));
        params = next;
        return true;
    }
    static int64_t count(int64_t score) {
        return rational<Count>(__int128(score) * Count::SCALE, State::SCALE);
    }
    int64_t decayed(int64_t score) const {
        const int64_t leak = rational<State>(__int128(alpha) * count(score) * State::SCALE,
                                            __int128(Fraction::SCALE) * Count::SCALE);
        return std::max<int64_t>(0, score - leak);
    }
    static int64_t added(int64_t score) {
        return rational<State>(__int128(score) + rational<State>(State::SCALE));
    }
    void judge() {
        if (fault != HallFault::NONE) return;
        if (params.invalid_threshold.raw() > 0 && scores[0] >=
            rational<State>(__int128(params.invalid_threshold.raw()) * State::SCALE, Count::SCALE)) {
            fault = HallFault::INVALID_CODE;
            return;
        }
        const int64_t e = count(scores[1]), r = count(scores[2]);
        if (params.erratic_fraction.raw() > 0 && r > 0 && e > 0 && e >= params.erratic_min_edges.raw() &&
            __int128(r) * Fraction::SCALE >= __int128(params.erratic_fraction.raw()) * e)
            fault = HallFault::ERRATIC_SEQUENCE;
    }
    void tick(bool active) {
        for (auto& score : scores) score = decayed(score);
        if (invalid && (active || !params.require_drive_active)) {
            ticks += ticks != UINT32_MAX;
            if (persist && ticks >= persist && fault == HallFault::NONE) fault = HallFault::INVALID_CODE;
        }
        judge();
    }
    void read(uint8_t next, bool active) {
        // An independently tabulated direction for every pair, including the
        // convention that both ambiguous half-turns count as positive.
        static constexpr int8_t directions[6][6] = {
            {0, 1, 1, 1, -1, -1}, {-1, 0, 1, 1, 1, -1}, {-1, -1, 0, 1, 1, 1},
            {1, -1, -1, 0, 1, 1}, {1, 1, -1, -1, 0, 1}, {1, 1, 1, -1, -1, 0}
        };
        const bool accumulating = active || !params.require_drive_active;
        if (next >= 6) {
            invalid = true;
            if (run != 255) ++run;
            if (!counted && run >= std::max<int>(1, params.invalid_debounce)) {
                counted = true;
                ++events;
                if (accumulating) scores[0] = added(scores[0]);
            }
        } else {
            invalid = false;
            run = 0;
            ticks = 0;
            counted = false;
            if (have_position && next != position) {
                const int8_t next_direction = directions[position][next];
                ++edges;
                if (accumulating) {
                    scores[1] = added(scores[1]);
                    if (direction && direction != next_direction) scores[2] = added(scores[2]);
                }
                direction = next_direction;
            }
            position = next;
            have_position = true;
        }
        judge();
    }
    void clear() {
        fault = HallFault::NONE;
        for (auto& score : scores) score = 0;
        direction = 0;
        run = 0;
        ticks = 0;
        counted = false;
    }
    void reset() {
        clear();
        have_position = false;
        position = 0;
        invalid = false;
        events = edges = 0;
    }
    void seed(Monitor& actual) const {
        actual.invalid_score_ = State::fromRaw(scores[0]);
        actual.edges_accum_ = State::fromRaw(scores[1]);
        actual.reversals_accum_ = State::fromRaw(scores[2]);
        actual.invalid_ticks_ = ticks;
        actual.invalid_events_ = events;
        actual.edges_ = edges;
        actual.invalid_run_ = run;
        actual.last_position_ = position;
        actual.last_sign_ = direction;
        actual.have_last_ = have_position;
        actual.last_invalid_ = invalid;
        actual.invalid_counted_ = counted;
        actual.fault_ = fault;
    }
    void verify(const Monitor& actual) const {
        const auto info = actual.getInfo();
        const int64_t e = count(scores[1]), r = count(scores[2]);
        const int64_t ratio = e > 0 && e >= params.erratic_min_edges.raw()
                            ? rational<Fraction>(__int128(r) * Fraction::SCALE, e) : 0;
        check(actual.invalid_score_.raw() == scores[0] && actual.edges_accum_.raw() == scores[1] &&
              actual.reversals_accum_.raw() == scores[2], "exact wide score history");
        check(info.invalid_score.raw() == count(scores[0]) && info.edge_accum.raw() == e &&
              info.reversal_accum.raw() == r && info.erratic_score.raw() == ratio, "public rounded scores");
        check(info.fault == fault && actual.faultCause() == fault &&
              actual.isFaulted() == (fault != HallFault::NONE), "same fault on the same operation");
        check(info.invalid_events == events && info.invalid_run == run && info.edges == edges &&
              info.last_position == position && info.invalid_time_us == std::min<uint64_t>(UINT32_MAX,
              uint64_t(ticks) * 1000000 / Hz), "counter and duration telemetry");
        check(actual.invalid_ticks_ == ticks && actual.last_sign_ == direction &&
              actual.have_last_ == have_position && actual.last_invalid_ == invalid &&
              actual.invalid_counted_ == counted, "lifecycle and debounce history");
        check(actual.alpha_.raw() == alpha && actual.persistence_ticks_ == persist &&
              actual.getParameters().decay_time_us == params.decay_time_us, "accepted tuning retained");
    }
};

// Exercise the generic paths as well: coarse alpha can quantize to zero at a
// valid long window, so an actual trace can exceed the public count range.
struct CoarseFractionProfile : FixedProfile { using Fraction = FixedPointType<8>; };
struct CompactProfile : FixedProfile {
    using Count = FixedPointType<4, int16_t>;
    using Fraction = FixedPointType<12, int16_t>;
    using CountAccumulator = FixedPointType<12>;
};
struct WideProfile : FixedProfile {
    using Count = FixedPointType<32, int64_t>;
    using Fraction = FixedPointType<8, int64_t>;
};

template<class Profile, uint32_t Hz>
static void arithmetic() {
    using Model = Reference<Profile, Hz>;
    using Monitor = typename Model::Monitor;
    using Count = typename Model::Count;
    using State = typename Model::State;
    using Fraction = typename Model::Fraction;
    constexpr int64_t maximum = std::numeric_limits<typename State::StorageType>::max();
    std::vector<int64_t> inputs{0, 1, maximum};
    const __int128 anchors[] = {0, State::SCALE / 2, State::SCALE,
        __int128(std::numeric_limits<typename Count::StorageType>::max()) * State::SCALE / Count::SCALE,
        maximum, __int128(maximum) - State::SCALE};
    for (const auto anchor : anchors) {
        for (const int offset : {-2, -1, 0, 1, 2}) {
            for (const int half : {-1, 0, 1}) {
                const __int128 value = anchor + offset + half * __int128(State::SCALE / Count::SCALE / 2);
                if (value >= 0 && value <= maximum) inputs.push_back(static_cast<int64_t>(value));
            }
        }
    }
    uint64_t random = 0xc9348ab56e1273ULL;
    for (unsigned i = 0; i < 20000; ++i) {
        const uint64_t mask = i % 2 ? uint64_t(maximum) : uint64_t(maximum) / 65536;
        inputs.push_back(static_cast<int64_t>(randomWord(random) & mask));
    }
    Monitor actual;
    Model expected;
    for (const auto raw : inputs) {
        const State state = State::fromRaw(raw);
        check(Monitor::toCount(state).raw() == Model::count(raw), "nonnegative narrowing including saturation");
        State added = state;
        Monitor::addEvent(added);
        check(added.raw() == Model::added(raw), "add-one at every accumulator boundary");
        for (const int64_t alpha : {int64_t(0), int64_t(1), int64_t(Fraction::SCALE / 8000),
                                   int64_t(Fraction::SCALE / 2), int64_t(Fraction::SCALE - 1),
                                   int64_t(Fraction::SCALE)}) {
            actual.alpha_ = Fraction::fromRaw(alpha);
            expected.alpha = alpha;
            State decayed = state;
            actual.decay(decayed);
            check(decayed.raw() == expected.decayed(raw), "decay matches rational leak and zero clamp");
        }
    }
}

template<class Profile, uint32_t Hz>
static void traces() {
    using Model = Reference<Profile, Hz>;
    using Monitor = typename Model::Monitor;
    using Count = typename Model::Count;
    using State = typename Model::State;
    using Fraction = typename Model::Fraction;
    Monitor actual;
    Model expected;
    auto configure = [&](const typename Monitor::Params& params) {
        check(actual.setParameters(params) == expected.configure(params), "window acceptance agrees");
        expected.verify(actual);
    };
    auto tick = [&](bool active) { actual.tick(active); expected.tick(active); expected.verify(actual); };
    auto read = [&](uint8_t pos, bool active) {
        actual.onPosition(pos, active); expected.read(pos, active); expected.verify(actual);
    };
    auto clear = [&] { actual.clearFault(); expected.clear(); expected.verify(actual); };
    auto reset = [&] { actual.reset(); expected.reset(); expected.verify(actual); };
    // Every decoded value and legal prior position; sign includes the first
    // transition after clearFault, and inactive transitions still update it.
    for (uint8_t last = 0; last < 6; ++last) {
        for (unsigned next = 0; next < 256; ++next) {
            for (const int8_t direction : {-1, 0, 1}) {
                expected.reset();
                expected.have_position = true;
                expected.position = last;
                expected.direction = direction;
                expected.seed(actual);
                read(next, next % 2);
            }
        }
    }
    reset();
    auto params = expected.params;
    params.invalid_persist_time_us = 301;
    configure(params);
    read(255, true);
    for (uint32_t i = 0; i < expected.persist; ++i) tick(true);
    check(actual.faultCause() == HallFault::INVALID_CODE, "standing bad Hall trips on its exact tick");
    clear(); // The standing invalid reading survives clearing the latch.
    for (uint32_t i = 0; i < expected.persist; ++i) tick(true);
    check(actual.isFaulted(), "clearFault re-arms persistence for the same bad reading");
    reset();
    params.invalid_persist_time_us = 0;
    params.erratic_fraction = Fraction{};
    params.invalid_debounce = 255;
    configure(params);
    for (unsigned i = 0; i < 300; ++i) read(255, true);
    expected.ticks = UINT32_MAX - 1;
    expected.edges = expected.events = UINT32_MAX;
    expected.seed(actual);
    tick(true); tick(true); read(1, true); read(2, true); read(255, true);
    // Configuration changes must be judged on the next onPosition even when
    // that reading changes no accumulator. No stale "already evaluated" flag.
    reset();
    expected.scores[0] = int64_t(State::SCALE) * 2;
    expected.seed(actual);
    params.invalid_threshold = number<Count>(1);
    configure(params);
    read(1, false);
    check(actual.faultCause() == HallFault::INVALID_CODE, "new threshold applies to unchanged reading");
    reset();
    params.invalid_threshold = Count{};
    params.erratic_fraction = number<Fraction>(1, 2);
    params.erratic_min_edges = number<Count>(10);
    configure(params);
    expected.scores[1] = int64_t(State::SCALE) * 9;
    expected.scores[2] = int64_t(State::SCALE) * 5;
    expected.have_position = true;
    expected.position = 1;
    expected.direction = 1;
    expected.seed(actual);
    read(2, true);
    check(actual.faultCause() == HallFault::ERRATIC_SEQUENCE, "forward edge can cross the minimum-edge gate");

    uint64_t random = 0x843720bcda9e513fULL;
    constexpr uint64_t max_events = uint64_t(std::numeric_limits<typename Count::StorageType>::max())
                                  / Count::SCALE;
    const uint32_t max_tau = static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX, max_events * 1000000 / Hz));
    for (unsigned step = 0; step < 60000; ++step) {
        const auto word = randomWord(random);
        if (step % 997 == 0) reset();
        else if (step % 101 == 0) clear();
        if (step % 127 == 0) {
            auto next = expected.params;
            const uint32_t windows[] = {0, 1, 1000, 500000, max_tau, max_tau + 1, UINT32_MAX};
            next.decay_time_us = windows[(word >> 16) % 7];
            next.invalid_debounce = static_cast<uint8_t>(word >> 24);
            const uint32_t durations[] = {0, 1, 300, 301, UINT32_MAX};
            next.invalid_persist_time_us = durations[(word >> 32) % 5];
            next.require_drive_active = word & 1;
            next.invalid_threshold = Count::fromRaw(static_cast<typename Count::StorageType>(word));
            next.erratic_fraction = Fraction::fromRaw(static_cast<typename Fraction::StorageType>(word >> 32));
            next.erratic_min_edges = Count::fromRaw(static_cast<typename Count::StorageType>(word >> 16));
            configure(next);
        }
        if (step % 251 == 0) {
            for (auto& score : expected.scores)
                score = static_cast<int64_t>(randomWord(random) &
                        uint64_t(std::numeric_limits<typename State::StorageType>::max()));
            expected.seed(actual);
        }
        const bool active = word & 8;
        tick(active);
        // Include invalid bursts, same-position reads, quiet ticks and changes.
        if (word & 1) read((word & 16) ? uint8_t(word % 6) : uint8_t(word >> 32), active);
    }
    reset();
    params = typename Monitor::Params{};
    params.invalid_persist_time_us = 0;
    params.erratic_fraction = Fraction{};
    params.decay_time_us = max_tau;
    configure(params);
    for (unsigned step = 0; step < 100000; ++step) { tick(true); read(step % 6, true); }
    if constexpr (std::is_same<Profile, CoarseFractionProfile>::value) {
        check(expected.scores[1] > int64_t(max_events + 1) * int64_t(State::SCALE),
              "valid quantized-alpha trace really exceeds the public count range");
        check(actual.getInfo().edge_accum.raw() == INT32_MAX, "overflowing history still reports saturated Count");
    }
}

int main() {
    // The pre-existing wide-profile event path must still compile and retain
    // bits above bit 31. Its generic multiplication limits do not permit tick.
    {
        HallMonitorCore<WideProfile, 16000> monitor;
        auto params = monitor.getParameters();
        params.erratic_fraction = number<WideProfile::Fraction>(1, 2);
        params.erratic_min_edges = number<WideProfile::Count>(2);
        check(monitor.setParameters(params), "wide-profile event tuning remains supported");
        monitor.onPosition(0, true);
        monitor.onPosition(1, true);
        check(!monitor.isFaulted(), "wide-profile first forward edge stays healthy");
        monitor.onPosition(0, true);
        check(monitor.faultCause() == HallFault::ERRATIC_SEQUENCE,
              "wide counts retain their upper word in the generic cross product");
    }
    arithmetic<FixedProfile, 16000>();
    arithmetic<CoarseFractionProfile, 16000>();
    arithmetic<CompactProfile, 1000>();
    traces<FixedProfile, 8000>();
    traces<FixedProfile, 16000>();
    traces<FixedProfile, 40000>();
    traces<CoarseFractionProfile, 16000>();
    traces<CompactProfile, 1000>();
    std::printf("Numeric Hall: %llu exact rational/lifecycle checks passed\n",
                static_cast<unsigned long long>(checks));
}
