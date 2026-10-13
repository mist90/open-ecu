/** @file main.cpp
 * @brief Control-clock, Hall duration and PI command integration regressions
 */
#include "../../libecu/include/bldc_controller.hpp"
#include "../../libecu/include/at_command_processor.hpp"
#include "../../libecu/include/crc16.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static uint32_t fake_time_us = 0;
uint32_t time_us() { return fake_time_us; }
extern "C" void disable_interrupts() {}
extern "C" void enable_interrupts() {}

using namespace libecu;
static int checks = 0;
static int failures = 0;
static void check(bool condition, const char* message) {
    ++checks;
    if (!condition) { ++failures; std::printf("FAIL: %s\n", message); }
}

class Pwm final : public PwmInterface {
public:
    Pwm() { frequency_ = ActiveTiming::CURRENT_HZ; }
    bool initialize(uint32_t frequency, uint16_t) override { frequency_ = frequency; return true; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float, float, float) override {}
    void enable(bool) override {}
};
class Hall final : public HallInterface {
public:
    bool initialize() override { return true; }
    uint8_t getPosition() override { return 0; }
};
class Console final : public AtCommandProcessor {
public:
    explicit Console(BldcController& controller) : AtCommandProcessor(&controller) {}
    int32_t read() override {
        return position_ < input_.size() ? static_cast<uint8_t>(input_[position_++]) : -1;
    }
    void write(const char* data, std::size_t length) override { output_.append(data, length); }
    std::string command(const char* text) {
        char suffix[12];
        const auto crc = crc16_compute(reinterpret_cast<const uint8_t*>(text), std::strlen(text));
        std::snprintf(suffix, sizeof(suffix), "*%04X\r\n", crc);
        input_ = std::string(text) + suffix;
        position_ = 0;
        output_.clear();
        process();
        return output_;
    }
private:
    std::string input_;
    std::string output_;
    std::size_t position_ = 0;
};

static MotorControlParams makeParams() {
    MotorControlParams params{};
    params.num_poles = 40;
    params.max_speed_rps = 12.0f;
    params.max_current = 18.0f;
    params.min_current = -6.0f;
    params.max_voltage = 36.0f;
    params.max_duty_cycle = 0.95f;
    params.max_temperature_c = 100.0f;
    params.algorithm = DriveAlgorithm::SIX_STEP;
    params.pi_current_mode.ki = 1.0f;
    params.pi_current_mode.integral_min = -4.0f;
    params.pi_current_mode.integral_max = 12.0f;
    params.pi_voltage_mode.integral_max = 1.0f;
    params.pi_current_regulator.integral_max = 1.0f;
    return params;
}

static void testSpeedClock() {
    Pwm pwm;
    Hall hall;
    BldcController controller(pwm, hall, makeParams());
    controller.initialize();
    controller.start();
    controller.setDriveMode(DriveMode::FORWARD);
    controller.setTargetSpeed(1.0f);
    for (uint32_t tick = 0; tick < ActiveTiming::SPEED_HZ / 2; ++tick) {
        // Wall-clock jumps must not turn a fixed-frequency sample into a long integration step.
        fake_time_us += tick % 2 ? 1000000u : 1u;
        controller.update();
    }
    const float expected = 0.5f - 0.5f / ActiveTiming::SPEED_HZ;
    check(std::abs(controller.getStatus().target_current - expected) < 2e-6f,
          "outer PI uses the speed clock rather than PWM or wall-clock intervals");

    controller.setDriveMode(DriveMode::NEUTRAL);
    controller.update();
    fake_time_us += 60000000u;
    controller.setDriveMode(DriveMode::FORWARD);
    controller.setTargetSpeed(1.0f);
    controller.update();
    check(std::abs(controller.getStatus().target_current - 0.5f / ActiveTiming::SPEED_HZ) < 1e-6f,
          "re-engagement does not integrate the parked interval");
}

static void testSlewClock() {
    Pwm pwm;
    Hall hall;
    auto params = makeParams();
    params.pi_current_mode.kp = 1.0f;
    params.pi_current_mode.ki = 0.0f;
    params.acceleration_rate = 2.0f;
    BldcController controller(pwm, hall, params);
    controller.initialize();
    controller.start();
    controller.setDriveMode(DriveMode::FORWARD);
    controller.setTargetSpeed(10.0f);
    for (int tick = 0; tick < 10; ++tick) controller.update();
    check(std::abs(controller.getStatus().target_current - 20.0f / ActiveTiming::SPEED_HZ) < 1e-6f,
          "acceleration limit advances by rate/frequency per sample");
    controller.setAccelerationRate(4.0f);
    controller.update();
    check(std::abs(controller.getStatus().target_current - 24.0f / ActiveTiming::SPEED_HZ) < 1e-6f,
          "changing acceleration refreshes the cached increment");
}

static void testCommands() {
    Pwm pwm;
    Hall hall;
    BldcController controller(pwm, hall, makeParams());
    Console console(controller);
    for (const char* prefix : {"SPID", "CPID", "FPID", "PLLID"}) {
        char text[64];
        std::snprintf(text, sizeof(text), "AT+%s=0.25,2.5", prefix);
        check(console.command(text) == "OK\r\n", "PI setter accepts two finite coefficients");
        std::snprintf(text, sizeof(text), "AT+%s?", prefix);
        const std::string response = console.command(text);
        const std::size_t first = response.find(',');
        check(first != std::string::npos && response.find(',', first + 1) == std::string::npos,
              "PI query publishes exactly two coefficients");
        for (const char* args : {"0.5,3,0", "0.5,3,0.01", "0.5", "0.5,", ",3", "nan,3", "1,inf", "1,2x"}) {
            std::snprintf(text, sizeof(text), "AT+%s=%s", prefix, args);
            check(console.command(text) == "ERROR\r\n", "malformed or three-parameter PI setting is rejected");
        }
        std::snprintf(text, sizeof(text), "AT+%s?", prefix);
        check(console.command(text) == response, "rejected command does not change gains");
    }
    controller.setSpeedPi(0.75f, 3.0f);
    controller.setElectricMode(ElectricMode::VOLTAGE_MODE);
    controller.setElectricMode(ElectricMode::CURRENT_MODE);
    float kp, ki;
    controller.getSpeedPiGains(kp, ki);
    check(kp == 0.75f && ki == 3.0f, "speed tuning survives a mode round-trip");
}

static void testHallDuration() {
    HallMonitor monitor;
    auto params = monitor.getParameters();
    params.invalid_persist_time_us = 301;
    monitor.setParameters(params);
    monitor.onPosition(HALL_POSITION_INVALID, true);
    const uint32_t limit = ActiveTiming::CurrentClock::ticksFromUs(301);
    for (uint32_t tick = 1; tick < limit; ++tick) monitor.tick(true);
    check(!monitor.isFaulted(), "Hall persistence never trips before the rounded-up threshold");
    monitor.tick(true);
    check(monitor.faultCause() == HallFault::INVALID_CODE, "Hall persistence trips on the exact sample");
    check(monitor.getInfo().invalid_time_us == ActiveTiming::CurrentClock::ticksToUs(limit),
          "Hall duration telemetry is derived from integer ticks");
    monitor.reset();
    monitor.onPosition(HALL_POSITION_INVALID, false);
    for (int tick = 0; tick < 100; ++tick) monitor.tick(false);
    check(!monitor.isFaulted() && monitor.getInfo().invalid_time_us == 0,
          "inactive drive does not accumulate Hall persistence");
}

int main() {
    testSpeedClock();
    testSlewClock();
    testCommands();
    testHallDuration();
    std::printf("Control integration: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
