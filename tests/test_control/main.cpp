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
    void updateNativeDutyCycle(NativeDuty u, NativeDuty v, NativeDuty w) override {
        ++native_writes;
        check(u >= NativeDuty{} && v >= NativeDuty{} && w >= NativeDuty{}, "native PWM is nonnegative");
    }
    void enable(bool value) override { enabled = value; }
    unsigned native_writes = 0;
    bool enabled = false;
};
class Hall final : public HallInterface {
public:
    bool initialize() override { return true; }
    uint8_t getPosition() override { return 0; }
};
class Adc final : public AdcInterface {
public:
    Adc() {
        check(initialize(CurrentSensorCalibration{0.003f, 16.0f, 3.3f, 12, 1.65f, 1.65f, 1.65f, 2},
                         VoltageSensorParameters{169000.0f, 18000.0f}), "ADC calibration initializes");
    }
    uint32_t getRawAdcValue(PwmChannel) override { return 4107; }
    uint32_t getRawAdcValue() override { return bus; }
    uint32_t getRawTemperatureAdcValue() override { return 1000; }
    uint32_t bus = 5000;
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
    std::string drainOsc() {
        output_.clear();
        for (unsigned i = 0; i < OSC_BUFFER_SIZE; ++i) processOscOutput();
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
#if defined(ECU_FIXED_PI) || defined(ECU_FIXED_NUMERIC)
    // The speed-facing conversion rounds to Q16 before applying the unit gain.
    constexpr float tolerance = 1.0f / FixedProfile::Speed::SCALE;
#else
    constexpr float tolerance = 1e-6f;
#endif
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
    check(std::abs(controller.getStatus().target_current - 20.0f / ActiveTiming::SPEED_HZ) < tolerance,
          "acceleration limit advances by rate/frequency per sample");
    controller.setAccelerationRate(4.0f);
    controller.update();
    check(std::abs(controller.getStatus().target_current - 24.0f / ActiveTiming::SPEED_HZ) < tolerance,
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
#if defined(ECU_FIXED_PI) || defined(ECU_FIXED_NUMERIC)
    check(console.command("AT+SPID=1000,3") == "ERROR\r\n", "speed-gain format overflow is rejected");
    check(console.command("AT+CPID=0.15,40000") == "ERROR\r\n", "current-gain format overflow is rejected");
    controller.getSpeedPiGains(kp, ki);
    check(kp == 0.75f && ki == 3.0f, "rejected fixed-point gains preserve the previous tuning");
#endif
#ifdef ECU_FIXED_NUMERIC
    const auto foc_before = console.command("AT+FPID?");
    check(console.command("AT+FPID=1,40000") == "ERROR\r\n", "FOC gain overflow is rejected");
    check(console.command("AT+FPID?") == foc_before, "rejected FOC gains are transactional");
    const auto pll_before = console.command("AT+PLLID?");
    check(console.command("AT+PLLID=100,200000") == "ERROR\r\n", "PLL gain overflow is rejected");
    check(console.command("AT+PLLID?") == pll_before, "rejected PLL gains are transactional");
    auto bad = makeParams();
    bad.foc_current_regulator.ki = 40000.0f;
    BldcController invalid_foc(pwm, hall, bad);
    check(!invalid_foc.initialize(), "unrepresentable FOC boot configuration prevents initialization");
#endif
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
#ifdef ECU_FIXED_NUMERIC
    const auto previous = monitor.getParameters();
    auto oversized = previous;
    oversized.decay_time_us = 2000000;
    check(!monitor.setParameters(oversized), "Hall window exceeding the count range is rejected");
    check(monitor.getParameters().decay_time_us == previous.decay_time_us,
          "rejected Hall tuning preserves the running configuration");
#endif
}

static void testNativePath() {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto params = makeParams();
    params.max_voltage = 24.0f;
    BldcController controller(pwm, hall, params, &adc);
    check(controller.initialize(), "native controller initializes");
    controller.start();
    controller.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    controller.setElectricMode(ElectricMode::VOLTAGE_MODE);
    controller.setDriveMode(DriveMode::FORWARD);
    controller.setDutyCycle(0.1f);
    const unsigned initial_writes = pwm.native_writes;
    controller.pwmInterruptHandler();
    controller.pwmInterruptHandler();
    check(pwm.native_writes == initial_writes + 2, "both PWM samples use the native interface");
    check(std::abs(controller.getStatus().measured_current - 0.1007326f) < 0.0001f,
          "integer ADC sum reaches six-step current telemetry with the right physical scale");
    controller.setAlgorithm(DriveAlgorithm::FOC);
    controller.pwmInterruptHandler();
    controller.update();
    check(std::abs(controller.getStatus().duty_cycle - 0.095f) < 0.00003f,
          "FOC modulation telemetry is computed from a slow coherent native snapshot");
    adc.bus = 8190;
    controller.pwmInterruptHandler();
    check(!pwm.enabled && controller.getDriveMode() == DriveMode::NEUTRAL,
          "native bus measurement still trips overvoltage protection");
    check(controller.getStatus().duty_cycle == 0.0f && controller.getStatus().target_current == 0.0f,
          "protection parks command and telemetry together");

    Console console(controller);
    NativeMotorStatus sample{};
    using P = ActiveNumericProfile;
    sample.target_current = number_v<P::Current, 50>;
    sample.measured_current = number_v<P::Current, -50>;
    sample.i_d = number_v<P::Current, -1, 8>;
    sample.i_q = number_v<P::Current, 1, 8>;
    sample.angle_e_rad = -number_v<P::Angle, 1570796326795LL, 1000000000000LL>;
    sample.duty_cycle = number_v<P::Duty, 95, 100>;
    sample.measured_position = 4;
    console.startOscilloscope();
    for (unsigned i = 0; i < Console::OSC_BUFFER_SIZE; ++i) console.captureNativeOscSample(sample);
    check(console.drainOsc().find("+OSC:0,-32768,32767,95,4,-125,125,270\r\n") == 0,
          "native OSC preserves signs, saturates milliamps and wraps negative angles");
}

#if defined(ECU_FIXED_PI) || defined(ECU_FIXED_NUMERIC)
static void testRejectedModeTuning() {
    Pwm pwm;
    Hall hall;
    auto params = makeParams();
    params.pi_voltage_mode.kp = 200.0f; // Outside the speed gain format's range.
    BldcController controller(pwm, hall, params);
    check(controller.initialize(), "valid initial current mode initializes");
    check(!controller.setElectricMode(ElectricMode::VOLTAGE_MODE), "unrepresentable alternate tuning is rejected");
    check(controller.getStatus().electric_mode == ElectricMode::CURRENT_MODE,
          "failed retuning keeps the original output units and mode");
}
#endif

int main() {
    testSpeedClock();
    testSlewClock();
    testCommands();
    testHallDuration();
    testNativePath();
#if defined(ECU_FIXED_PI) || defined(ECU_FIXED_NUMERIC)
    testRejectedModeTuning();
#endif
    std::printf("Control integration: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
