/**
 * @file main.cpp
 * @brief Regression tests for HallMonitor.
 *
 * The hard requirement is separating two things that both produce bursts of
 * Hall edges: benign bounce on healthy hardware, which must never trip, and a
 * broken line, which must trip quickly. Both are modelled here from the raw
 * 3-bit code, so the tests exercise the same decode path the firmware uses.
 *
 * Build/run: ./run.sh
 */

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <random>
#include <algorithm>
#include <initializer_list>

#include "../../libecu/include/algorithms/hall_monitor.hpp"

using libecu::HallFault;
using libecu::HallMonitor;
using libecu::HALL_POSITION_INVALID;

namespace {

constexpr float PWM_FREQ = 20000.0f;
constexpr float DT       = 1.0f / PWM_FREQ;

int failures = 0;

void check(bool ok, const char* what) {
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

/// Same table as Stm32TimHallSensor::POSITION_TABLE.
const uint8_t POSITION_TABLE[8] = {0xFF, 0, 2, 1, 4, 5, 3, 0xFF};

/// Healthy 3-bit Gray sequence, in electrical order.
const uint8_t CODE_SEQ[6] = {0b001, 0b011, 0b010, 0b110, 0b100, 0b101};

/// Apply a stuck line to a raw code. bit < 0 means "healthy".
uint8_t applyFault(uint8_t code, int stuck_bit, bool stuck_high) {
    if (stuck_bit < 0) return code;
    return stuck_high ? static_cast<uint8_t>(code |  (1u << stuck_bit))
                      : static_cast<uint8_t>(code & ~(1u << stuck_bit));
}

struct Rotor {
    float   sector = 0.0f;               ///< fractional electrical sector, wraps at 6
    int     stuck_bit = -1;
    bool    stuck_high = false;
    uint8_t last_reported = 0xFE;        ///< last decoded value handed to the monitor

    uint8_t sample() const {
        int s = static_cast<int>(sector) % 6;
        if (s < 0) s += 6;
        return POSITION_TABLE[applyFault(CODE_SEQ[s], stuck_bit, stuck_high) & 0x07];
    }
};

/// Drive the monitor the way the firmware does: tick() every PWM cycle, and
/// onPosition() only when the decoded reading actually changes - that is what
/// raises a Hall EXTI and produces a deferred read.
void feed(HallMonitor& m, uint8_t& last, uint8_t now, bool drive_active) {
    m.tick(drive_active);
    if (now != last) {
        last = now;
        m.onPosition(now, drive_active);
    }
}

/// Spin at `steps_per_sec` for `seconds`.
void spin(HallMonitor& m, Rotor& r, float steps_per_sec, float seconds,
          bool drive_active = true) {
    const uint32_t ticks = static_cast<uint32_t>(seconds / DT);
    for (uint32_t k = 0; k < ticks; ++k) {
        r.sector += steps_per_sec * DT;
        if (r.sector >= 6.0f) r.sector -= 6.0f;
        feed(m, r.last_reported, r.sample(), drive_active);
    }
}

/// Rotor dithering back and forth across one sector boundary - the "impact"
/// bounce that healthy hardware shows when commutation is rough. Produces
/// genuine edges and zero net progress.
void bounce(HallMonitor& m, Rotor& r, int edges, float edge_period_s) {
    const uint32_t per = static_cast<uint32_t>(edge_period_s / DT);
    // Straddle the nearest sector boundary so every half-cycle really does
    // produce an edge; nudging the fractional sector is not enough.
    const float boundary = std::floor(r.sector) + 1.0f;
    for (int e = 0; e < edges; ++e) {
        r.sector = boundary + ((e % 2 == 0) ? 0.2f : -0.2f);
        if (r.sector < 0.0f)  r.sector += 6.0f;
        if (r.sector >= 6.0f) r.sector -= 6.0f;
        for (uint32_t k = 0; k < per; ++k) {
            feed(m, r.last_reported, r.sample(), true);
        }
    }
}

} // namespace

int main() {
    printf("HallMonitor regression\n\n");

    // ---- 1. healthy running must never fault ------------------------------
    {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        for (float spd : {60.0f, 600.0f, 1200.0f, 2400.0f}) {
            spin(m, r, spd, 2.0f);
        }
        check(!m.isFaulted(), "healthy running, 60-2400 steps/s, no fault");
    }

    // ---- 2. benign bounce must not fault ----------------------------------
    // Several separated bursts, each of a size a real rough commutation makes.
    {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        spin(m, r, 600.0f, 0.5f);
        float peak = 0.0f;
        for (int burst = 0; burst < 6; ++burst) {
            bounce(m, r, 8, 0.002f);          // 8 edges at 500 Hz
            peak = std::max(peak, m.getInfo().erratic_score);
            spin(m, r, 600.0f, 0.4f);          // recover
        }
        printf("       peak erratic fraction over 6 bursts of 8 edges: %.3f "
               "(threshold %.2f)\n", peak, m.getParameters().erratic_fraction);
        check(!m.isFaulted(), "six separated bounce bursts do not fault");
    }

    // ---- 3. sustained chatter must fault ----------------------------------
    {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        spin(m, r, 600.0f, 0.3f);
        bounce(m, r, 400, 0.002f);             // 500 Hz chatter, sustained
        check(m.isFaulted() && m.faultCause() == HallFault::ERRATIC_SEQUENCE,
              "sustained chatter faults as ERRATIC_SEQUENCE");
    }

    // ---- 4. a broken line must fault, at every speed and polarity ----------
    printf("\n  broken line, time to detect:\n");
    for (int bit = 0; bit < 3; ++bit) {
        for (int high = 0; high < 2; ++high) {
            for (float spd : {120.0f, 600.0f, 1200.0f}) {
                HallMonitor m(PWM_FREQ);
                Rotor r;
                spin(m, r, spd, 0.3f);         // healthy first
                if (m.isFaulted()) { check(false, "faulted before the break"); continue; }
                r.stuck_bit = bit; r.stuck_high = (high != 0);

                float t = 0.0f;
                const float limit = 2.0f;
                while (!m.isFaulted() && t < limit) {
                    r.sector += spd * DT;
                    if (r.sector >= 6.0f) r.sector -= 6.0f;
                    feed(m, r.last_reported, r.sample(), true);
                    t += DT;
                }
                if (spd == 600.0f) {
                    printf("     bit%d stuck %-4s @ %4.0f steps/s -> %6.1f ms  (%s)\n",
                           bit, high ? "high" : "low", spd, t * 1000.0f,
                           m.faultCause() == HallFault::INVALID_CODE ? "INVALID_CODE" : "ERRATIC");
                }
                char msg[128];
                std::snprintf(msg, sizeof(msg),
                              "bit%d stuck %s detected at %.0f steps/s in %.1f ms",
                              bit, high ? "high" : "low", spd, t * 1000.0f);
                check(m.isFaulted(), msg);
            }
        }
    }

    // ---- 4b. transient illegal codes at the measured healthy rate ---------
    // Hardware showed 3532 illegal readings/sec at 9 RPS on intact wiring,
    // each standing for at most one PWM period before a valid reading
    // followed. That must not fault.
    {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        spin(m, r, 1080.0f, 0.3f);
        const uint32_t glitches = 3532;
        const uint32_t gap = static_cast<uint32_t>(PWM_FREQ / glitches);
        for (uint32_t g = 0; g < glitches && !m.isFaulted(); ++g) {
            // one illegal reading, then a valid one a single PWM period later
            m.onPosition(HALL_POSITION_INVALID, true);
            m.tick(true);
            m.onPosition(r.last_reported, true);
            for (uint32_t k = 0; k < gap; ++k) {
                r.sector += 1080.0f * DT;
                if (r.sector >= 6.0f) r.sector -= 6.0f;
                feed(m, r.last_reported, r.sample(), true);
            }
        }
        printf("       %u transient illegal codes at 1 PWM period each\n", glitches);
        check(!m.isFaulted(), "measured healthy glitch rate does not fault");
    }

    // ---- 4c. loaded running with real rotor dither -------------------------
    // From a captured false positive: at 6 RPS under 2 A, about 6% of Hall
    // transitions failed to advance - genuine dither at the sector boundary.
    // The absolute-count threshold of 20 tripped on this; the fraction must not.
    {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        float peak = 0.0f;
        for (uint32_t k = 0; k < static_cast<uint32_t>(4.0f / DT); ++k) {
            r.sector += 720.0f * DT;
            if (r.sector >= 6.0f) r.sector -= 6.0f;
            uint8_t now = r.sample();
            // 6% of the time, emit the previous sector once more before moving
            // on - a rotor stepping back over the boundary and returning.
            if (now != r.last_reported && u(rng) < 0.06f) {
                const uint8_t back = static_cast<uint8_t>((r.last_reported + 5) % 6);
                m.tick(true); m.onPosition(back, true);
                m.tick(true); m.onPosition(r.last_reported, true);
            }
            feed(m, r.last_reported, now, true);
            peak = std::max(peak, m.getInfo().erratic_score);
        }
        printf("       loaded dither: peak erratic fraction %.3f (threshold %.2f)\n",
               peak, m.getParameters().erratic_fraction);
        check(!m.isFaulted(), "6%% non-advancing edges under load does not fault");
    }

    // ---- 5. disconnected loom (all lines at the pull level) ---------------
    for (int high = 0; high < 2; ++high) {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        spin(m, r, 600.0f, 0.3f);
        // The loom going open produces one settling edge and then silence -
        // no further events at all. Only the persistence timer can catch this.
        float t = 0.0f;
        feed(m, r.last_reported, HALL_POSITION_INVALID, true);
        while (!m.isFaulted() && t < 1.0f) {
            m.tick(true);
            t += DT;
        }
        char msg[96];
        std::snprintf(msg, sizeof(msg),
                      "disconnected loom (all %s) detected in %.1f ms",
                      high ? "111" : "000", t * 1000.0f);
        check(m.isFaulted() && m.faultCause() == HallFault::INVALID_CODE, msg);
    }

    // ---- 6. parked motor must not fault, however long ---------------------
    {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        spin(m, r, 600.0f, 1.0f);
        spin(m, r, 0.0f, 60.0f, /*drive_active=*/true);   // held still, bridge on
        check(!m.isFaulted(), "parked motor does not fault after 60 s");
    }

    // ---- 7. the fault latches until cleared -------------------------------
    {
        HallMonitor m(PWM_FREQ);
        Rotor r;
        spin(m, r, 600.0f, 0.3f);
        r.stuck_bit = 0;
        spin(m, r, 600.0f, 0.5f);
        const bool latched = m.isFaulted();
        spin(m, r, 0.0f, 2.0f);                            // condition removed
        check(latched && m.isFaulted(), "fault latches while the condition persists");
        m.clearFault();
        check(!m.isFaulted(), "clearFault() releases it");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASS",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
