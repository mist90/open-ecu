/** @file main.cpp @brief Mechanical Hall feedback, independent of the stock carrier and its PLL. */
#include "bldc_controller.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace libecu;
static uint32_t clock_us;
static unsigned masks, clock_reads, checks;
uint32_t time_us() { ++clock_reads; return clock_us; }
extern "C" void disable_interrupts() { ++masks; }
extern "C" void enable_interrupts() {}

namespace {
using Profile = ActiveNumericProfile;
using Speed = Profile::Speed;
constexpr uint32_t HZ = ActiveTiming::CURRENT_HZ;
constexpr Speed MAX_SPEED = number_v<Speed, 12>;
constexpr uint32_t widths[] = {1250, 2250, 1750, 2500, 2000, 2750}; // 12.5 ms / 40 poles = 4 RPS.

void check(bool condition, const char* msg) {
    ++checks;
    if (!condition) { std::fprintf(stderr, "%s at %u us\n", msg, clock_us); std::exit(1); }
}
void near(Speed sample, float expected, const char* msg) {
    if (std::fabs(numberCast<float>(sample) - expected) > 0.0001f)
        std::fprintf(stderr, "actual %.7f, expected %.7f: ", numberCast<float>(sample), expected);
    check(std::fabs(numberCast<float>(sample) - expected) <= 0.0001f, msg);
}
uint8_t next(uint8_t s, int dir) { return uint8_t((int(s) + 6 + dir) % 6); }

void windowChecks() {
    // Both the microsecond timer and the carrier transition count wrap within the measured turn.
    for (int dir : {-1, 1}) {
        HallPeriodSpeed h;
        uint32_t t = UINT32_MAX - 5200, count = UINT32_MAX - 4;
        uint8_t s = 0;
        const auto pll = number_v<Speed, -2>;
        const auto observe = [&](uint32_t now) {
            return h.select<Profile, HZ>(h.pin(count, s), now, 40, MAX_SPEED, pll);
        };
        for (unsigned i = 0; i < 26; ++i) {
            t += widths[s]; ++count; s = next(s, dir);
            h.capture(t, s, count);
            near(observe(t), i < 7 ? -2.f : 4.f * dir, "six intervals, each ending sector, counter wrap");
        }
        h.capture(t + 4000, s, count); // Duplicate EXTI: not an edge, even if notified again much later.
        near(observe(t + 4000), 4.f * dir, "do not date a duplicate");
        near(observe(t + 5000), -2, "speed-relative expiry");
        near(observe(t + 50001), -2, "absolute expiry");
        near(observe(t + 1), -2, "a complete timer wrap parked cannot revive a stale window");
        s = next(s, dir); h.capture(t + 1000, s, ++count);
        near(observe(t + 1000), -2, "first edge after long park cannot reuse old turns");
    }

    // The speed magnitude, not the PLL speed or commanded direction, selects the 0.5..1 RPS blend.
    for (const auto span : {120000u, 100000u, 62500u, 50000u, 20000u}) {
        for (int dir : {-1, 1}) {
            HallPeriodSpeed h;
            uint32_t t = 0;
            uint8_t s = 0;
            for (unsigned i = 0; i <= 7; ++i) {
                t = i * span / 6; s = next(s, dir);
                h.capture(t, s, i);
            }
            const float magnitude = 50000.f / span;
            const float blend = std::clamp((magnitude - 0.5f) * 2.f, 0.f, 1.f);
            const auto window = h.pin(7, s);
            near(h.select<Profile, HZ>(window, t, 40, MAX_SPEED, number_v<Speed, -2>),
                 -2 + blend * (dir * magnitude + 2), "signed speed blend at crawl, ramp and full authority");
            near(h.select<Profile, HZ>(window, t, 0, MAX_SPEED, number_v<Speed, -2>), -2, "zero poles");
            h.discard();
            near(h.select<Profile, HZ>(h.pin(7, s), t, 40, MAX_SPEED, number_v<Speed, -2>), -2, "explicit reset");
        }
    }

    for (auto bad_interval : {0u, 40000u, 51000u}) {
        HallPeriodSpeed h;
        uint32_t t = 0;
        for (unsigned i = 0; i <= 7; ++i) {
            t += i == 4 ? bad_interval : 2000;
            h.capture(t, i % 6, i);
        }
        near(h.select<Profile, HZ>(h.pin(7, 1), t, 40, MAX_SPEED, number_v<Speed, -2>), -2,
             "zero, interrupted or excessively long sector interval");
    }
    // A different motor's 10 RPS rating. Test exactly twice that limit and either side
    // of it in both directions, including a turn too fast to represent in Q16 at two poles.
    for (int dir : {-1, 1}) {
        for (uint32_t span : {3125u, 2500u, 2499u, 30u, 6u}) {
            HallPeriodSpeed h;
            uint8_t s = 0;
            uint32_t t = 0;
            for (unsigned i = 0; i <= 7; ++i) {
                t = i * span / 6; s = next(s, dir);
                h.capture(t, s, i);
            }
            const uint8_t poles = span == 6 ? 2 : 40;
            near(h.select<Profile, HZ>(h.pin(7, s), t, poles, number_v<Speed, 10>, number_v<Speed, -2>),
                 span < 2500 ? -2.f : dir * 50000.f / span, "overspeed accepted to 2x rating, then PLL (not clamp)");
            if (span == 30) {
                // 33333 RPS is within 2x this artificial rating but outside Q16.
                near(h.select<Profile, HZ>(h.pin(7, s), t, 2, number_v<Speed, 20000>, number_v<Speed, -2>),
                     std::is_same_v<Speed, float> ? dir * (1000000.f / span) : -2.f,
                     "reject an unrepresentable quotient rather than saturating it");
            }
        }
    }
    HallPeriodSpeed busy;
    for (unsigned i = 0; i < 12; ++i) busy.capture(i * 2000, i % 6, i);
    const auto old = busy.pin(11, 5);
    for (unsigned i = 12; i < 32; ++i) busy.capture(i * 2000, i % 6, i);
    near(busy.select<Profile, HZ>(old, 62000, 40, MAX_SPEED, number_v<Speed, -2>), -2,
         "IRQ overran a pinned slow window");
    near(busy.select<Profile, HZ>(busy.pin(31, 1), 62000, 40, MAX_SPEED, number_v<Speed, -2>),
         50.f / 12, "overwritten IRQ buffer recovers at the latest complete turn");
}

struct Hardware final : HallInterface, AdcInterface, PwmInterface {
    uint8_t sector = 0;
    bool enabled = false;
    bool initialize(uint32_t, uint16_t) override { return true; }
    bool initialize() override { return true; }
    uint8_t getPosition() override { return sector; }
    void enable(bool on) override { enabled = on; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float, float, float) override {}
    uint32_t getRawTemperatureAdcValue() override { return 0; }
    uint32_t getRawAdcValue(PwmChannel) override { return 0; }
    uint32_t getRawAdcValue() override { return 0; }
    NativeVoltage readNativeBusVoltage() noexcept override { return number_v<NativeVoltage, 24>; }
    void readNativeCurrents(NativeCurrent& u, NativeCurrent& v, NativeCurrent& w) noexcept override { u = v = w = {}; }
};

MotorControlParams parameters() {
    MotorControlParams p{};
    p.num_poles = 40; p.max_speed_rps = 12; p.max_duty_cycle = 0.95f;
    p.max_current = 45; p.min_current = -6; p.max_voltage = 60;
    p.speed_pi = {1, 0, 45, -6, 45, -6, 0};
    return p;
}

void rejectedEdges() {
    Hardware hw;
    hw.sector = 1;
    clock_us = 0;
    BldcController motor(hw, hw, parameters(), &hw);
    check(motor.setPllGains(0, 0) && motor.initialize(), "known PLL fallback");
    motor.start();
    motor.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    motor.setDriveMode(DriveMode::FORWARD);
    motor.pwmInterruptHandler(); // Carrier baseline at sector 1.
    const auto edge = [&](uint8_t sector, uint32_t dt, bool accept = true) {
        clock_us += dt;
        hw.sector = sector;
        motor.hallEdgeInterruptHandler();
        if (accept) motor.pwmInterruptHandler();
    };
    const auto checkSpeed = [&](float expected, const char* msg) {
        motor.update();
        near(motor.io().in.velocity_feedback_rps, LIBECU_HALL_PERIOD_SPEED ? expected : 0.f, msg);
        check(motor.getHallFault() == HallFault::NONE && hw.enabled, "guard still accepts the driving frame");
    };
    edge(0, 2000);
    edge(5, 2000, false); edge(0, 2000, false); // No PWM observation during the entire 0→5→0 excursion.
    for (unsigned i = 0; i < 6; ++i) edge(next(hw.sector, 1), i ? 2000 : 10);
    checkSpeed(0, "an unaccepted predecessor cannot witness a six-interval turn");
    edge(1, 2000);
    checkSpeed(50.f / 12, "first complete real turn after the reversal");

    for (unsigned i = 0; i < 9; ++i) edge(next(hw.sector, 1), 63);
    checkSpeed(0, "nine accepted legal 63-us Hall transitions are implausible on a 12 RPS motor");
    for (unsigned i = 0; i < 7; ++i) edge(next(hw.sector, 1), 2000);
    checkSpeed(50.f / 12, "normal timing recovers without a Hall fault reset");
}

void acquisition() {
    Hardware hw;
    BldcController motor(hw, hw, parameters(), &hw);
    // Known independent fallback: leave the real PLL and commutation running with zero gains.
    check(motor.setPllGains(0, 0) && motor.initialize(), "controller initialization");
    motor.start();
    motor.pwmInterruptHandler();
    const auto slow = [&](float expected, const char* msg) {
        motor.update();
        near(motor.io().in.current_speed_rps, 0, "legacy mechanical telemetry remains the PLL");
        near(motor.io().in.velocity_feedback_rps, LIBECU_HALL_PERIOD_SPEED ? expected : 0.f, msg);
    };
    const auto irq = [&] {
        const unsigned before = masks;
        motor.hallEdgeInterruptHandler();
        check(before == masks, "Hall timestamp capture must not mask ADC/BRK");
    };
    unsigned tick = 0;
    const auto advance = [&] { clock_us = uint32_t(uint64_t(++tick) * 1000000 / HZ); };
    const auto edge = [&](int dir, bool irq_first, bool inspect, float expected) {
        const unsigned ticks = widths[hw.sector] * HZ / 1000000;
        for (unsigned k = 1; k < ticks; ++k) {
            advance();
            const unsigned before = clock_reads;
            motor.pwmInterruptHandler();
            check(clock_reads == before, "no timestamp in a carrier frame");
        }
        advance(); hw.sector = next(hw.sector, dir);
        if (irq_first) {
            irq(); if (inspect) slow(expected, "latest EXTI not yet qualified: use preceding accepted turn");
            motor.pwmInterruptHandler();
        } else {
            motor.hallSensorInterruptHandler(); motor.pwmInterruptHandler(); // ADC saw pins before EXTI.
            if (inspect) slow(expected, "latest accepted sector still waiting for its EXTI");
            irq();
        }
        irq(); // Later IRQ for the same sector, including an EXTI pending across the ADC frame.
        slow(expected, "physical edge phasing");
    };

    // Coast in NEUTRAL: the healthy monitor's drive-only error accumulator is not a motion clock.
    for (unsigned i = 0; i < 28; ++i) edge(1, i % 2, i > 8, i >= 7 ? 4.f : 0.f);
    check(!hw.enabled && motor.io().in.target_current == Profile::Current{}, "parked observation cannot make torque");
    motor.setDriveMode(DriveMode::REVERSE);
    slow(-4, "opposite-command motion has negative feedback");
    motor.setDriveMode(DriveMode::FORWARD);
    slow(4, "with-command motion");
    near(numberCast<Speed>(motor.io().in.target_current), LIBECU_HALL_PERIOD_SPEED ? -4.f : 0.f,
         "speed PI consumes the selected speed, including zero-target braking");
    const auto held = motor.io().in.velocity_feedback_rps;
    const unsigned before = clock_reads;
    motor.updateSpeedPi();
    check(held == motor.io().in.velocity_feedback_rps && before == clock_reads,
          "intermediate PI only reads held speed");
    motor.setDriveMode(DriveMode::NEUTRAL);
    for (unsigned i = 0; i < 10; ++i) edge(-1, i % 2, false, i >= 6 ? -4.f : 0.f);
    motor.setDriveMode(DriveMode::REVERSE);
    slow(4, "reverse drive, reverse physical sequence");
    motor.setDriveMode(DriveMode::NEUTRAL);

    const uint8_t original = hw.sector;
    hw.sector = 0xff; motor.hallSensorInterruptHandler(); motor.pwmInterruptHandler(); // only carrier saw illegal code
    slow(0, "instant invalid carrier history");
    hw.sector = original; motor.pwmInterruptHandler();
    slow(0, "cleared illegal run is not speed history");
    for (unsigned i = 0; i < 10; ++i) edge(1, true, false, i >= 7 ? 4.f : 0.f);
    const auto before_glitch = hw.sector;
    hw.sector = 0xff; irq(); // rejected raw EXTI, never delivered to the carrier
    hw.sector = before_glitch; irq(); motor.pwmInterruptHandler();
    slow(0, "illegal EXTI breaks the edge window");
    for (unsigned i = 0; i < 10; ++i) edge(1, true, false, i >= 6 ? 4.f : 0.f);

    advance(); hw.sector = next(next(hw.sector, 1), 1); irq(); motor.pwmInterruptHandler();
    slow(0, "a legal but nonadjacent accepted code is not one sector interval");
    for (unsigned i = 0; i < 8; ++i) edge(1, true, false, i >= 6 ? 4.f : 0.f);
    HallMonitorParams monitor{500000, 0, 2, 300, 0.35f, 10, true};
    check(motor.setHallMonitorParams(monitor), "alternative debounce config");
    for (unsigned i = 0; i < 10; ++i) edge(1, true, false, 0);
    monitor.invalid_debounce = 1;
    check(motor.setHallMonitorParams(monitor), "restore single-read illegal-code evidence");
    slow(0, "unqualified anti-bounce history cannot be replayed after changing configuration");
    for (unsigned i = 0; i < 10; ++i) edge(1, true, false, i >= 7 ? 4.f : 0.f);

    // The raw IRQ stream looks ideal, but the carrier coalesces two successive sectors.
    hw.sector = next(hw.sector, 1); clock_us += 200; irq();
    hw.sector = next(hw.sector, 1); clock_us += 200; irq(); motor.pwmInterruptHandler();
    slow(0, "one missing accepted sector must invalidate apparently regular EXTI timestamps");
    // More than a full turn of EXTI activity, no accepted frame at all.
    for (int i = 0; i < 10; ++i) { clock_us += 200; hw.sector = next(hw.sector, 1); irq(); }
    slow(0, "unconfirmed EXTI-only revolution");
    motor.stop(); motor.start();
    slow(0, "restart cannot replay a previously accepted turn");
    check(motor.getHallFault() == HallFault::NONE && !hw.enabled,
          "observer failures leave protection and neutral alone");
}

void shadowAndFilter() {
    Hardware a, b;
    auto p = parameters();
    p.algorithm = DriveAlgorithm::FOC;
    p.measured_speed_lpf_alpha = 0.5f;
    BldcController selected(a, a, p, &a), legacy(b, b, p, &b);
    for (auto* m : {&selected, &legacy}) {
        check(m->initialize() && m->setPllGains(BLDC_PLL_KP, BLDC_PLL_KI), "reference controller");
        m->start(); m->setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
        m->setDriveMode(DriveMode::FORWARD); m->setCurrent(1);
    }
    const auto frames = HZ / 500; // Constant 2 ms sectors; 12 ms per electrical turn.
    for (unsigned k = 0; k <= 12 * frames; ++k) {
        clock_us = uint32_t(uint64_t(k) * 1000000 / HZ);
        if (k && k % frames == 0) {
            b.sector = a.sector = next(a.sector, 1);
            selected.hallEdgeInterruptHandler(); legacy.hallSensorInterruptHandler();
        }
        check(selected.pwmInterruptHandler() && legacy.pwmInterruptHandler(), "both torque frames complete");
        const auto& x = selected.io().out;
        const auto& y = legacy.io().out;
        check(x.pll.angle_steps == y.pll.angle_steps && x.pll.speed_steps_s == y.pll.speed_steps_s &&
              x.electrical.target_position == y.electrical.target_position &&
              x.electrical.v_q == y.electrical.v_q && x.electrical.v_d == y.electrical.v_d &&
              x.electrical.duty_u == y.electrical.duty_u && x.electrical.duty_v == y.electrical.duty_v &&
              x.electrical.duty_w == y.electrical.duty_w, "one physical PLL, unchanged electrical FOC and duties");
    }
    selected.update(); legacy.update();
    const auto& in = selected.io().in;
    check(in.current_speed_rps == legacy.io().in.current_speed_rps, "shadow and legacy filtered PLL are identical");
    const float first = numberCast<float>(in.velocity_feedback_rps);
    near(in.velocity_feedback_rps,
         LIBECU_HALL_PERIOD_SPEED ? 0.5f * 50.f / 12 : numberCast<float>(in.current_speed_rps),
         "existing outer-speed filter applied once to selected source");
    clock_us += 60000; // No new carrier or EXTI at all.
    selected.update(); legacy.update();
    const float raw_pll = numberCast<float>(selected.io().out.pll.speed_steps_s) / (3 * p.num_poles);
    near(in.velocity_feedback_rps, 0.5f * (first + raw_pll), "stale source returns through the same 100-Hz LPF");
    check(in.current_speed_rps == legacy.io().in.current_speed_rps, "parallel stock LPF never inherits Hall history");
    selected.cancelPwmTick();
    check(!selected.pwmInterruptHandler(), "preempting break cancels a live frame");
    selected.parkFromCarrier(); selected.updateSpeedPi(); selected.update();
    check(!a.enabled && in.target_current == Profile::Current{}, "slow observation cannot undo a carrier park");
}
} // namespace

int main() {
    windowChecks();
    rejectedEdges();
    acquisition();
    shadowAndFilter();
    std::printf("Hall period speed (%s, %u Hz, enabled=%d): %u checks passed\n",
                CONTROL_PROFILE_NAME, HZ, LIBECU_HALL_PERIOD_SPEED, checks);
}
