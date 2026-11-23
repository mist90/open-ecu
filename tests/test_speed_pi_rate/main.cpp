/**
 * @file main.cpp
 * @brief Closed-loop scooter speed PI cadence A/B. Only Iq tracking and mechanics are simulated.
 */
#include "bldc_controller.hpp"
#include "motor_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace libecu;
static unsigned carrier_tick;
uint32_t time_us() { return uint32_t(uint64_t(carrier_tick) * 1000000 / ActiveTiming::CURRENT_HZ); }
extern "C" void disable_interrupts() {}
extern "C" void enable_interrupts() {}

namespace {
constexpr unsigned CARRIER_HZ = ActiveTiming::CURRENT_HZ, SLOW_HZ = ActiveTiming::SPEED_HZ;
constexpr unsigned PI_HZ = ActiveTiming::SPEED_PI_HZ, SECTORS = 3 * BLDC_NUM_POLES;
constexpr unsigned SYSTICK_HZ = ActiveTiming::SYSTICK_HZ;
constexpr unsigned SECONDS = 40, TAIL_SECONDS = 5;
constexpr double TARGET_RPS = 155.0 / 60.0, TURN = 6.2831853071795864769;
constexpr double DT = 1.0 / CARRIER_HZ, RADIUS_M = 0.760 / TURN;
constexpr double KT = 1.5 * (BLDC_NUM_POLES / 2) * BLDC_KE_V_S_PER_RAD_E;
static_assert(CARRIER_HZ == 16000 && SYSTICK_HZ == 500 && SLOW_HZ == 100 && CARRIER_HZ % SYSTICK_HZ == 0);
static_assert(SECTORS == 120 && BLDC_MAX_CURRENT == 45 && BLDC_MIN_CURRENT == -6);

void require(bool ok, const char* message) {
    if (!ok) { std::fprintf(stderr, "FAIL at carrier tick %u: %s\n", carrier_tick, message); std::exit(1); }
}

struct Io final : PwmInterface, HallInterface, AdcInterface {
    uint8_t position = 0;
    unsigned ntc_reads = 0;
    bool initialize(uint32_t, uint16_t) override { return true; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float, float, float) override {}
    void enable(bool) override {}
    bool initialize() override { return true; }
    uint8_t getPosition() override { return position; }
    uint32_t getRawTemperatureAdcValue() override { ++ntc_reads; return 1309; } // ~25 C
    uint32_t getRawAdcValue(PwmChannel) override { return 2048; }
    uint32_t getRawAdcValue() override { return 3500; }
};

struct Tuning { float kp, ki, traction; };

MotorControlParams parameters(Tuning t) {
    MotorControlParams p{};
    p.num_poles = BLDC_NUM_POLES;
    p.max_speed_rps = BLDC_MAX_SPEED;
    p.max_current = BLDC_MAX_CURRENT;
    p.min_current = BLDC_MIN_CURRENT;
    p.max_voltage = 60;
    p.max_duty_cycle = 0.92f;
    p.max_temperature_c = BLDC_MAX_TEMPERATURE;
    p.algorithm = DriveAlgorithm::FOC;
    p.useInverseCommTable = BLDC_INVERTION;
    p.acceleration_rate = motor_config::soft_start_rps2; // Mechanical rev/s² slew, for every run.
    p.traction_acceleration_rps2 = t.traction;
    p.measured_speed_lpf_alpha = 0.5f;
    p.speed_pi = {t.kp, t.ki, BLDC_MAX_CURRENT, BLDC_MIN_CURRENT, BLDC_MAX_CURRENT, -4, 2};
    return p;
}

// Host truth in double; no speed, angle, acceleration or ideal current is injected into libecu.
// Inertia and bearing estimates from test_speed_loop; bench wheel circumference is 760 mm.
struct Plant {
    explicit Plant(double mass_kg)
        : inertia(0.00625 + mass_kg * RADIUS_M * RADIUS_M)
        , dry_torque(KT * 0.656 + 0.008 * mass_kg * 9.80665 * RADIUS_M) {}
    const double inertia, dry_torque;
    const double current_gain = -std::expm1(-TURN * 250 * DT);
    const double mean_factor = current_gain / (TURN * 250 * DT);
    double speed = 0, iq = 0, angle_steps = 0.13;
    int sector = 0; // Unwrapped, signed Hall count.

    void step(double command_a) {
        // Exact average and endpoint of a first-order 250-Hz current response.
        const double iq_mean = command_a + (iq - command_a) * mean_factor;
        iq += current_gain * (command_a - iq);
        const double free_speed = speed + DT * KT * (iq_mean - 0.0484 * speed) / (TURN * inertia);
        const double friction_step = DT * dry_torque / (TURN * inertia);
        const double next = free_speed - std::clamp(free_speed, -friction_step, friction_step);
        angle_steps += 0.5 * (speed + next) * DT * SECTORS;
        speed = next;
        sector = int(std::floor(angle_steps + 0.5)); // centres are integer steps
    }
};

struct Sample { double wheel, feedback; int sector; };
struct Metrics {
    static constexpr unsigned W = SLOW_HZ / 5; // 200 ms; advance all windows by 10 ms
    std::array<Sample, 2 * W + 1> history{};
    double launch_wheel200 = 0, launch_feedback200 = 0;
    double wheel10 = 0, feedback10 = 0, wheel200 = 0, feedback200 = 0, hall200 = 0;
    double max_speed = 0, t10 = -1, t90 = -1, tail_min = 1e9, tail_max = -1e9, tail_sum = 0;
    double tail_feedback_sum = 0;

    void add(unsigned k, Sample s) {
        const auto past = [&](unsigned lag) { return history[(k - lag) % history.size()]; };
        if (k) {
            wheel10 = std::max(wheel10, (s.wheel - past(1).wheel) * SLOW_HZ);
            feedback10 = std::max(feedback10, (s.feedback - past(1).feedback) * SLOW_HZ);
        }
        constexpr double inv_window = double(SLOW_HZ) / W;
        if (k >= W) {
            const double a_wheel = (s.wheel - past(W).wheel) * inv_window;
            const double a_feedback = (s.feedback - past(W).feedback) * inv_window;
            if (t90 < 0) { // Includes the sample where the physical wheel first crosses 90%.
                launch_wheel200 = std::max(launch_wheel200, a_wheel);
                launch_feedback200 = std::max(launch_feedback200, a_feedback);
            }
            wheel200 = std::max(wheel200, a_wheel);
            feedback200 = std::max(feedback200, a_feedback);
        }
        if (k >= 2 * W) {
            // Two neighbouring 200-ms mean speeds from raw Hall edges; 400-ms support.
            const int diff = s.sector - 2 * past(W).sector + past(2 * W).sector;
            hall200 = std::max(hall200, diff * inv_window * inv_window / SECTORS);
        }
        history[k % history.size()] = s;
        max_speed = std::max(max_speed, s.wheel);
        if (t10 < 0 && s.wheel >= 0.1 * TARGET_RPS) t10 = double(k) / SLOW_HZ;
        if (t90 < 0 && s.wheel >= 0.9 * TARGET_RPS) t90 = double(k) / SLOW_HZ;
        if (k >= (SECONDS - TAIL_SECONDS) * SLOW_HZ) {
            tail_min = std::min(tail_min, s.wheel);
            tail_max = std::max(tail_max, s.wheel);
            tail_sum += s.wheel;
            tail_feedback_sum += s.feedback;
        }
    }
};

double run(double mass_kg, Tuning t, FILE* trace) {
    carrier_tick = 0;
    Io io;
    require(io.AdcInterface::initialize({0.003f, 16, 3.3f, 12}, {169000, 18000}), "ADC fixture setup");
    BldcController motor(io, io, parameters(t), &io);
    require(motor.setPllGains(BLDC_PLL_KP, BLDC_PLL_KI) && motor.initialize(), "scooter controller setup");
    motor.start();
    motor.setDriveMode(DriveMode::FORWARD);
    motor.setTargetSpeed(float(TARGET_RPS));

    Plant plant(mass_kg);
    Metrics m;
    unsigned mid_ticks = 0, mid_changes = 0;
    const auto& state = motor.io().in;
    const char* load = mass_kg ? "80kg" : "free";
    // t=0 is the first service boundary after arming. Before it, no PI-only tick has valid inputs.
    uint32_t phase = SYSTICK_HZ / SLOW_HZ - 1;
    for (; carrier_tick < SECONDS * CARRIER_HZ; ++carrier_tick) {
        if (carrier_tick % (CARRIER_HZ / SYSTICK_HZ) == 0) {
            const auto due = ActiveTiming::nextSysTick(phase);
            if (due == ControlTick::Service) {
                motor.update(); // Slow frame, including exactly one coincident PI sample.
                m.add(carrier_tick / (CARRIER_HZ / SLOW_HZ),
                      {plant.speed, state.current_speed_rps.toFloat(), plant.sector});
            } else if (due == ControlTick::SpeedPiOnly) {
                require(io.ntc_reads > 0, "PI-only tick before first service frame");
                const auto before = state.target_current;
                const auto feedback = state.current_speed_rps;
                motor.updateSpeedPi(); // Publish immediately; the next plant substep sees this new Iq.
                ++mid_ticks;
                mid_changes += state.target_current != before;
                require(state.current_speed_rps == feedback, "intermediate tick refreshed slow feedback");
            }
        }
        const float request = state.target_current.toFloat();
        require(request >= BLDC_MIN_CURRENT && request <= BLDC_MAX_CURRENT, "current request outside scooter rails");
        // Common 2-ms trace grid for both rates, after any update, BEFORE advancing the plant.
        if (trace && carrier_tick % (CARRIER_HZ / SYSTICK_HZ) == 0)
            std::fprintf(trace, "%u,%s,%g,%g,%g,%.3f,%.8f,%.8f,%d,%.8f,%.8f\n", PI_HZ, load,
                         t.kp, t.ki, t.traction, carrier_tick * DT, plant.speed,
                         state.current_speed_rps.toFloat(), plant.sector, request, plant.iq);

        const int old_sector = plant.sector;
        plant.step(request);
        if (plant.sector != old_sector) {
            require(std::abs(plant.sector - old_sector) == 1, "more than one Hall edge in a carrier tick");
            io.position = uint8_t((plant.sector % 6 + 6) % 6);
            motor.hallSensorInterruptHandler();
        }
        // The plant integrates the requested Iq; the real carrier still consumes one qualified sample.
        motor.pwmInterruptHandler();
    }
    require(io.ntc_reads == SECONDS * SLOW_HZ, "temperature was not sampled exactly at 100 Hz");
    require(mid_ticks == SECONDS * (PI_HZ - SLOW_HZ), "wrong number of intermediate PI ticks");
    require(motor.getHallFault() == HallFault::NONE && motor.getDriveMode() == DriveMode::FORWARD,
            "controller fault or unexpected park");
    require(m.t90 > m.t10 && m.t10 >= 0, "10-90% speed crossing missing");
    require(std::abs(m.tail_sum - m.tail_feedback_sum) / (TAIL_SECONDS * SLOW_HZ) < 0.05,
            "late mean Hall feedback lost the moving wheel");
    require(PI_HZ == SLOW_HZ || mid_changes > 0, "no intermediate PI command reached the plant");
    std::printf("%u,%s,%g,%g,%g,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                PI_HZ, load, t.kp, t.ki, t.traction, m.launch_wheel200, m.launch_feedback200,
                m.wheel200, m.feedback200, m.hall200, m.wheel10, m.feedback10,
                m.t90 - m.t10, m.t90, 60 * std::max(0.0, m.max_speed - TARGET_RPS),
                60 * m.tail_sum / (TAIL_SECONDS * SLOW_HZ), 60 * (m.tail_max - m.tail_min));
    return m.launch_wheel200;
}
} // namespace

int main(int argc, char** argv) {
    if (argc > 2) { std::fprintf(stderr, "Usage: %s [trace.csv]\n", argv[0]); return 2; }
    FILE* trace = nullptr;
    if (argc == 2) {
        trace = std::fopen(argv[1], "w");
        require(trace, "cannot open trace");
        std::fputs("pi_hz,load,kp,ki,traction_rps2,t_s,wheel_rps,feedback_rps,hall_count,request_a,iq_a\n", trace);
    }
    for (double mass : {0.0, 80.0}) {
        constexpr float traction = motor_config::traction_acceleration_rps2;
        // Preserve all historical comparison arms regardless of the motor's boot PI selection.
        run(mass, {1.25f, 5, traction}, trace);
        const double cap_on = run(mass, {3, 24, traction}, trace);
        run(mass, {6, 24, traction}, trace);
        const double cap_off = run(mass, {3, 24, 0}, trace);
        require(cap_on > 0 && cap_on < cap_off, "3/24 traction control did not attenuate this model's launch");
    }
    if (trace) require(std::fclose(trace) == 0, "could not write trace");
}
