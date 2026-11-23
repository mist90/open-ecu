/** @file main.cpp
 * @brief Control-clock, Hall duration and PI command integration regressions
 */
#include "../../libecu/include/bldc_controller.hpp"
#include "../../libecu/include/at_command_processor.hpp"
#include "../../libecu/include/crc16.hpp"
#include "../../libecu/include/critical_section.hpp"
#include "../../libecu/include/traction_acceleration.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static uint32_t fake_time_us = 0;
uint32_t time_us() { return fake_time_us; }
static unsigned mask_depth = 0, mask_entries = 0;
static void (*before_mask_hook)() = nullptr;
static void (*unmasked_hook)() = nullptr;
extern "C" void disable_interrupts() {
    if (!mask_depth && before_mask_hook) {
        const auto hook = before_mask_hook;
        before_mask_hook = nullptr;
        hook();
    }
    ++mask_depth;
    ++mask_entries;
}
extern "C" void enable_interrupts() {
    if (--mask_depth == 0 && unmasked_hook) {
        const auto hook = unmasked_hook;
        unmasked_hook = nullptr;
        hook();
    }
}

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
    void setChannelState(PwmChannel, PwmState) override { ++calls; }
    void updateDutyCycle(float, float, float) override { ++calls; }
    void updateNativeDutyCycle(NativeDuty u, NativeDuty v, NativeDuty w) override {
        ++calls;
        ++native_writes;
        check(u >= NativeDuty{} && v >= NativeDuty{} && w >= NativeDuty{}, "native PWM is nonnegative");
        if (on_native_write) on_native_write(context);
    }
    void enable(bool value) override { ++calls; enabled = value; if (value) ++enables; }
    void apply() override { ++calls; }
    unsigned calls = 0;
    unsigned enables = 0;
    unsigned native_writes = 0;
    bool enabled = false;
    void (*on_native_write)(void*) = nullptr;
    void* context = nullptr;
};
class Hall final : public HallInterface {
public:
    bool initialize() override { return true; }
    uint8_t getPosition() override { ++reads; if (on_read) on_read(context); return position; }
    void (*on_read)(void*) = nullptr;
    void* context = nullptr;
    uint8_t position = 0;
    unsigned reads = 0;
};
class Adc : public AdcInterface {
public:
    Adc() {
        check(initialize(CurrentSensorCalibration{0.003f, 16.0f, 3.3f, 12, 1.65f, 1.65f, 1.65f, 2},
                         VoltageSensorParameters{169000.0f, 18000.0f}), "ADC calibration initializes");
    }
    uint32_t getRawAdcValue(PwmChannel channel) override { ++current_reads; return currents[unsigned(channel)]; }
    uint32_t getRawAdcValue() override { ++bus_reads; return bus; }
    uint32_t getRawTemperatureAdcValue() override { ++temperature_reads; return 1000; }
    unsigned reads() const { return current_reads + bus_reads + temperature_reads; }
    unsigned current_reads = 0;
    unsigned bus_reads = 0;
    unsigned temperature_reads = 0;
    uint32_t currents[3] = {4107, 4107, 4107};
    uint32_t bus = 5000;
};
class CapturedAdc final : public Adc {
public:
    void readNativeCurrents(NativeCurrent& u, NativeCurrent& v, NativeCurrent& w) noexcept override {
        ++frame_reads;
        u = convertNativeAdcToCurrent(currents[0], PwmChannel::PHASE_U);
        v = convertNativeAdcToCurrent(currents[1], PwmChannel::PHASE_V);
        w = convertNativeAdcToCurrent(currents[2], PwmChannel::PHASE_W);
    }
    unsigned frame_reads = 0;
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
        CommandProcessor& processor = *this;
        processor.process();
        return output_;
    }
    std::string feed(const std::string& bytes) {
        input_ = bytes;
        position_ = 0;
        output_.clear();
        CommandProcessor& processor = *this;
        processor.process();
        return output_;
    }
    const std::string& output() const { return output_; }
    void clearOutput() { output_.clear(); }
    std::string drainOsc() {
        output_.clear();
        CommandProcessor& processor = *this;
        for (unsigned i = 0; i < OSC_BUFFER_SIZE; ++i) processor.processOutput();
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
    params.speed_pi.ki = 1.0f;
    params.speed_pi.integral_min = -4.0f;
    params.speed_pi.integral_max = 12.0f;
    params.pi_current_regulator.integral_max = 1.0f;
    return params;
}

// One interval starting with a fresh 100 Hz sample and ending just before the next one.
static void serviceInterval(BldcController& drive) {
    drive.update();
    for (uint32_t i = 1; i < ActiveTiming::SPEED_PI_HZ / ActiveTiming::SPEED_HZ; ++i)
        drive.updateSpeedPi();
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
        serviceInterval(controller);
    }
    const float expected = 0.5f - 0.5f / ActiveTiming::SPEED_PI_HZ;
    // Float integrates up to 500 rounded increments in this window at 1 kHz.
    check(std::abs(numberCast<float>(controller.io().in.target_current) - expected) < 5e-6f,
          "outer PI uses the speed clock rather than PWM or wall-clock intervals");

    controller.setDriveMode(DriveMode::NEUTRAL);
    controller.update();
    fake_time_us += 60000000u;
    controller.setDriveMode(DriveMode::FORWARD);
    controller.setTargetSpeed(1.0f);
    controller.update();
    check(std::abs(numberCast<float>(controller.io().in.target_current) - 0.5f / ActiveTiming::SPEED_PI_HZ) < 1e-6f,
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
    params.speed_pi.kp = 1.0f;
    params.speed_pi.ki = 0.0f;
    params.acceleration_rate = 2.0f;
    BldcController controller(pwm, hall, params);
    controller.initialize();
    controller.start();
    controller.setDriveMode(DriveMode::FORWARD);
    controller.setTargetSpeed(10.0f);
    for (int tick = 0; tick < 10; ++tick) controller.update();
    check(std::abs(numberCast<float>(controller.io().in.target_current) - 20.0f / ActiveTiming::SPEED_HZ) < tolerance,
          "acceleration limit advances by rate/frequency per sample");
    controller.setAccelerationRate(4.0f);
    controller.update();
    check(std::abs(numberCast<float>(controller.io().in.target_current) - 24.0f / ActiveTiming::SPEED_HZ) < tolerance,
          "changing acceleration refreshes the cached increment");
}

static void testIntermediatePi() {
    if constexpr (ActiveTiming::SPEED_PI_HZ == ActiveTiming::SPEED_HZ) return;
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto params = makeParams();
    params.acceleration_rate = 2.0f;
    params.speed_pi.kp = 0;
    params.speed_pi.ki = 24;
    BldcController motor(pwm, hall, params, &adc);
    check(motor.initialize(), "multirate controller initialized");
    motor.start();
    motor.setDriveMode(DriveMode::FORWARD);
    motor.setTargetSpeed(3);
    motor.updateSpeedPi();
    check(motor.io().in.target_current == BldcController::Current{}, "no PI before first reference sample");
    motor.update();
    const auto first_current = motor.io().in.target_current;
    const auto first_speed = motor.io().in.current_speed_rps;
    const auto reads = adc.reads(), writes = pwm.calls;
    motor.setTargetSpeed(4); // New positive command waits for next slow ramp; the PI still uses the old sample.
    motor.updateSpeedPi();
    const auto& next = motor.io().in;
    check(std::abs(numberCast<float>(next.target_current - first_current) - 24.f * .02f / ActiveTiming::SPEED_PI_HZ) < 2e-6f &&
          next.current_speed_rps == first_speed && reads == adc.reads() && writes == pwm.calls,
          "intermediate PI integrates the held ramp once, without auxiliary acquisition or PWM writes");
    // A higher-priority carrier parks after PI computation, immediately before committing the request.
    static BldcController* interrupted;
    interrupted = &motor;
    before_mask_hook = [] { interrupted->parkFromCarrier(); };
    motor.updateSpeedPi();
    check(!before_mask_hook && motor.getDriveMode() == DriveMode::NEUTRAL &&
          motor.io().in.target_current == BldcController::Current{} && !pwm.enabled,
          "PI publication cannot reinstate a demand calculated across a carrier park");
    motor.setDriveMode(DriveMode::FORWARD);
    motor.setTargetSpeed(3);
    motor.updateSpeedPi();
    check(motor.io().in.target_current == BldcController::Current{},
          "park and re-arm between service ticks cannot replay an old current demand");
    motor.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    motor.setCurrent(-3);
    motor.updateSpeedPi();
    check(numberCast<float>(motor.io().in.target_current) == -3, "intermediate speed PI never takes ownership of torque mode");
}

static void testTractionAcceleration() {
    using Profile = ActiveNumericProfile;
    using S = Profile::Speed;
    using C = Profile::Current;
    TractionAcceleration<Profile> limiter;
    const auto amp = [](auto c) { return numberCast<float>(c); };
    const auto step = number_v<S, 2, 100>;
    const auto eight = number_v<C, 8>;
    const auto max = number_v<C, 45>;
    const auto gain = number_v<S, 2>; // A per rev/s
    // Same current command and speed setpoint, only the *measured* trajectory differs.
    const C first = limiter.ceiling(number_v<S, 3>, step, eight, max, gain);
    const C brisk = limiter.ceiling(number_v<S, 31, 10>, step, eight, max, gain);
    check(std::abs(amp(first) - 8.04f) < 0.001f && std::abs(amp(brisk) - 7.84f) < 0.001f,
          "real acceleration folds back positive torque; a coasting engagement has no fictitious impulse");
    check(limiter.ceiling(number_v<S, 4>, step, C{}, max, gain) == C{},
          "external positive acceleration can remove traction but cannot demand braking");
    check(limiter.ceiling(number_v<S, 4>, S{}, C{}, max, gain) == max,
          "disabled feedback preserves the original current ceiling");
    limiter.reset();
    check(limiter.ceiling(number_v<S, -4>, step, eight, max, gain) == first,
          "reversing the direction frame does not differentiate across directions");
    check(std::abs(amp(limiter.ceiling(number_v<S, -4>, step, eight, max, number_v<S, 3, 4>)) - 8.015f) < 0.001f,
          "the explicit feedback gain scales the change in applied torque");
    // A periodic Hall speed error has zero net acceleration. Rectifying the
    // changes here would ratchet the torque ceiling down on every full period.
    limiter.reset();
    C applied = limiter.ceiling(number_v<S, 2>, step, eight, max, gain);
    for (int i = 0; i < 10; ++i) {
        applied = limiter.ceiling(number_v<S, 21, 10>, step, applied, max, gain);
        applied = limiter.ceiling(number_v<S, 2>, step, applied, max, gain);
    }
    check(std::abs(amp(applied) - 8.84f) < 0.002f, "alternating signed speed error does not consume net headroom");
    using Pi = PiFloatInterface<ActivePiProfile::SpeedPiTypes, ActiveTiming::SPEED_PI_HZ>;
    Pi pi;
    PiParameters limits{};
    limits.kp = 6; limits.ki = 24; limits.kb = 0;
    limits.min_output = -6; limits.max_output = 45;
    limits.integral_min = -4; limits.integral_max = 12;
    check(pi.setParameters(limits), "configure bounded speed PI");
    for (int i = 0; i < 100; ++i) {
        pi.updateNativeCapped(number_v<Pi::Input, 2>, Pi::Input{}, 0,
                              number_v<Pi::Output, 12>, number_v<Pi::Output, 1>);
    }
    check(pi.getOutput() == 1.0f && pi.getIntegral() < 0.25f && pi.saturationSign() == 1,
          "without Kb, a positive traction limit blocks continuing integral windup");
    for (int i = 1; i <= 50; ++i) {
        pi.updateNativeCapped(number_v<Pi::Input, 2>, Pi::Input{}, 0, number_v<Pi::Output, 12>,
                              numberCast<Pi::Output>(1.0f + i * 0.04f));
    }
    check(pi.getIntegral() < 0.25f && std::abs(pi.getOutput() - 3.0f) < 0.00001f,
          "rising but still binding ceiling does not accumulate positive integral without Kb");
    const auto brake = pi.updateNativeCapped(Pi::Input{}, number_v<Pi::Input, 2>, 0,
                                              number_v<Pi::Output, 12>, Pi::Output{});
    check(amp(brake) == -6.0f && pi.saturationSign() == -1,
          "zero traction authority does not mask speed PI braking or the hard current floor");
#ifdef MOTOR_1
    Pwm pwm;
    Hall hall;
    auto params = makeParams();
    params.traction_acceleration_rps2 = 2.0f;
    params.speed_pi.kp = 6.0f;
    params.speed_pi.ki = 24.0f;
    params.speed_pi.kb = 2.0f;
    BldcController drive(pwm, hall, params);
    check(drive.initialize(), "scooter controller accepts the fixture's measured-acceleration rate");
    drive.start();
    drive.setDriveMode(DriveMode::FORWARD);
    drive.setTargetSpeed(4.0f);
    drive.update();
    check(std::abs(numberCast<float>(drive.io().in.target_current) - 0.04f) < 0.001f,
          "unramped speed step is bounded at the speed PI output");
    for (unsigned i = 0; i < 25; ++i) serviceInterval(drive);
    check(std::abs(numberCast<float>(drive.io().in.target_current) - 1.04f) < 0.002f,
          "a loaded stationary rotor recovers more torque without bypassing the ceiling");
    drive.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    drive.setCurrent(-3.0f);
    drive.update();
    check(numberCast<float>(drive.io().in.target_current) == -3.0f, "torque mode still accepts the commanded braking current");
    drive.setDriveMode(DriveMode::NEUTRAL);
    drive.update();
    check(!pwm.enabled && numberCast<float>(drive.io().in.target_current) == 0, "park immediately removes torque");
    drive.setControlMode(ControlMode::CLOSED_LOOP_VELOCITY);
    drive.setDriveMode(DriveMode::REVERSE);
    drive.setTargetSpeed(4.0f);
    drive.update();
    check(std::abs(numberCast<float>(drive.io().in.target_current) - 0.04f) < 0.001f, "reverse launch starts without stored torque");
#endif
}

static void testLiveSpeedPiGains() {
    Pwm pwm, reference_pwm;
    Hall hall;
    auto params = makeParams();
    params.speed_pi.kp = 6.0f;
    params.speed_pi.ki = 24.0f;
    params.speed_pi.integral_max = 18.0f;
    BldcController motor(pwm, hall, params), reference(reference_pwm, hall, params);
    for (auto* drive : {&motor, &reference}) {
        check(drive->initialize(), "speed retune fixture initializes");
        drive->start();
        drive->setDriveMode(DriveMode::FORWARD);
        drive->setTargetSpeed(1.0f);
        for (unsigned i = 0; i < 25; ++i) serviceInterval(*drive);
    }
    const float before = numberCast<float>(motor.io().in.target_current);
    check(std::abs(before - (12.0f - 12.0f / ActiveTiming::SPEED_PI_HZ)) < 0.00006f,
          "250 ms at the selected PI frequency accumulates the expected trapezoidal integral");
    check(motor.setSpeedPi(6, 24), "repeating speed gains succeeds under load");
    motor.update();
    reference.update();
    const auto held = motor.io().in.target_current;
    check(held == reference.io().in.target_current, "identical gains preserve the untouched PI trajectory");
    check(motor.setSpeedPi(4, 0), "changed speed gains have a feasible integral rebase");
    motor.update();
    check(motor.io().in.target_current == held, "changing Kp with zero Ki holds the exact last command");
    motor.setTargetSpeed(1.5f);
    motor.update();
    check(std::abs(numberCast<float>(motor.io().in.target_current) - (numberCast<float>(held) + 2.0f)) < 0.000003f,
          "subsequent speed error uses the new proportional gain");
    check(motor.setSpeedPi(4, 10), "Ki-only speed edit succeeds");
    motor.setTargetSpeed(1.0f);
    motor.update();
    check(std::abs(numberCast<float>(motor.io().in.target_current) - (numberCast<float>(held) + 12.5f / ActiveTiming::SPEED_PI_HZ)) < 0.000003f,
          "Ki-only edit retains I and both trapezoidal error endpoints");

    const auto command = motor.io().in.target_current;
    check(!motor.setSpeedPi(100, 5), "unreachable compensated integral is rejected");
    float kp, ki;
    motor.getSpeedPiGains(kp, ki);
    check(kp == 4 && ki == 10 && motor.io().in.target_current == command,
          "rejected transfer preserves gains and the active command");
#ifdef MOTOR_1
    check(!motor.setSpeedPi(0, 0), "stationary scooter rejects a rebase above 12 A but below the static limit");
#endif
    static BldcController* interrupted;
    interrupted = &motor;
    unmasked_hook = [] { interrupted->setDriveMode(DriveMode::NEUTRAL); };
    check(!motor.setSpeedPi(5, 10) && !pwm.enabled && motor.io().in.target_current == BldcController::Current{},
          "parking between snapshot and publish cancels a retune and keeps zero torque");
    motor.getSpeedPiGains(kp, ki);
    check(kp == 4 && ki == 10, "a cancelled retune cannot change stored gains");

    motor.setDriveMode(DriveMode::FORWARD);
    motor.setTargetSpeed(10.0f);
    motor.update();
    check(numberCast<float>(motor.io().in.target_current) == params.max_current && !motor.setSpeedPi(2, 5) &&
          motor.setSpeedPi(4, 10), "active output rail refuses changed gains but accepts an identical write");
    std::printf("Speed PI 6/24: %.5f -> %.5f A (unchanged twin); live Kp retune held %.5f A\n",
                before, numberCast<float>(reference.io().in.target_current), numberCast<float>(held));
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
    for (const char* command : {"AT+EMODE?", "AT+EMODE=0", "AT+EMODE=1", "AT+DUT?", "AT+DUT=0.5"})
        check(console.command(command) == "ERROR\r\n", "retired electrical commands are rejected");
    check(console.command("AT+STATUS?").find("+STATUS:1,1,") == 0,
          "legacy status retains current-regulation wire position");
    float kp, ki;
    controller.getSpeedPiGains(kp, ki);
    check(kp == 0.75f && ki == 3.0f, "obsolete requests do not alter speed tuning");
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

static void testFragmentedCommands() {
    Pwm pwm;
    Hall hall;
    BldcController controller(pwm, hall, makeParams());
    Console console(controller);
    const char* text = "AT+SPID=0.25,2.5";
    const auto crc = crc16_compute(reinterpret_cast<const uint8_t*>(text), std::strlen(text));
    char suffix[8];
    std::snprintf(suffix, sizeof(suffix), "*%04X", crc);
    const std::string frame = std::string(text) + suffix;

    // Every possible UART-buffer boundary, including each of the four CRC digits.
    for (std::size_t split = 1; split < frame.size(); ++split) {
        controller.setSpeedPi(0.5f, 3.0f);
        check(console.feed(frame.substr(0, split)).empty(), "partial AT frame has no response");
        float kp, ki;
        controller.getSpeedPiGains(kp, ki);
        check(kp == 0.5f && ki == 3.0f, "partial AT frame does not change the controller");
        check(console.feed(frame.substr(split)) == "OK\r\n", "fragmented CRC completes once");
        controller.getSpeedPiGains(kp, ki);
        check(kp == 0.25f && ki == 2.5f, "fragmented command reaches the controller");
    }
    for (std::size_t i = 0; i < frame.size(); ++i) {
        const std::string response = console.feed(frame.substr(i, 1));
        check(response == (i + 1 == frame.size() ? "OK\r\n" : ""),
              "one-byte UART fragments preserve the CRC accumulator");
    }
    std::string bad = frame;
    bad.back() = bad.back() == '0' ? '1' : '0';
    controller.setSpeedPi(0.5f, 3.0f);
    check(console.feed(bad) == "ERROR\r\n", "bad CRC is rejected");
    float kp, ki;
    controller.getSpeedPiGains(kp, ki);
    check(kp == 0.5f && ki == 3.0f, "bad CRC cannot change gains");
    check(console.feed(frame + "\r\n" + frame + "\r\n") == "OK\r\nOK\r\n",
          "parser recovers after bad CRC and handles concatenated frames");
}

static void testAtLifecycle() {
    Pwm pwm;
    Hall hall;
    BldcController controller(pwm, hall, makeParams());
    Console console(controller);
    CommandProcessor& processor = console;
    check(console.isTelemetryEnabled() && !console.isPllTelemetryEnabled() &&
          !console.isHallTelemetryEnabled(), "AT telemetry defaults are retained");
    check(!processor.wantsCurrentSample(), "AT capture is opt-in");
    processor.onStarted();
    const std::string greeting = std::string("ECU started, control: ") + CONTROL_PROFILE_NAME +
                                 ", PI: " + PI_PROFILE_NAME + "\n";
    check(console.output() == greeting, "generic startup hook preserves the AT greeting");
    console.clearOutput();
    processor.onControlTick();
    const std::string default_output = console.output();
    check(default_output.find("+TM:") == 0 && default_output.find("+PLL:") == std::string::npos &&
          default_output.find("+HSTATUS:") == std::string::npos, "default tick sends only motor telemetry");
    console.setTelemetryEnabled(false);
    console.clearOutput();
    processor.onControlTick();
    check(console.output().empty(), "disabled telemetry leaves a silent control tick");
    console.setPllTelemetryEnabled(true);
    console.setHallTelemetryEnabled(true);
    processor.onControlTick();
    check(console.output().find("+PLL:") == 0 && console.output().find("+HSTATUS:") != std::string::npos &&
          console.output().find("+TM:") == std::string::npos, "PLL and Hall tick output remains independently enabled");

    console.clearOutput();
    console.startOscilloscope();
    check(processor.wantsCurrentSample(), "OSC start requests native ISR samples");
    MotorIo sample{};
    for (std::size_t i = 0; i < Console::OSC_BUFFER_SIZE; ++i) processor.captureCurrentSample(sample);
    check(console.output().empty(), "ISR capture never transmits");
    check(!processor.wantsCurrentSample(), "full OSC buffer stops requesting snapshots");
    const std::string capture = console.drainOsc();
    check(capture.find("+OSC:0,") == 0 && capture.substr(capture.size() - 7) == "+OSC:\r\n",
          "generic output hook drains the configured-size OSC buffer");
    check(processor.wantsCurrentSample(), "drained OSC buffer resumes capture");
    console.stopOscilloscope();
    console.clearOutput();
    processor.captureCurrentSample(sample);
    processor.processOutput();
    check(!processor.wantsCurrentSample() && console.output().empty(), "OSC stop disables capture and output");
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
    constexpr auto too_many = uint32_t(INT32_MAX / FixedProfile::Count::SCALE + 1);
    oversized.decay_time_us = ActiveTiming::CurrentClock::ticksToUs(too_many);
    check(!monitor.setParameters(oversized), "Hall window exceeding the count range is rejected");
    check(monitor.getParameters().decay_time_us == previous.decay_time_us,
          "rejected Hall tuning preserves the running configuration");
#endif
}

static void testInvalidHallRetry() {
    for (bool direct : {false, true}) {
        for (bool persistent : {false, true}) {
            Pwm pwm;
            Hall hall;
            Adc adc;
            auto params = makeParams();
            params.algorithm = DriveAlgorithm::FOC;
            BldcController controller(pwm, hall, params, &adc);
            check(controller.initialize(), "invalid-Hall retry controller initializes");
            HallMonitor defaults;
            auto monitor = defaults.getParameters();
            monitor.invalid_threshold = 0.0f;
            monitor.invalid_persist_time_us = 300;
            check(controller.setHallMonitorParams(monitor), "configure unchanged 300us invalid-Hall hold");
            controller.start();
            controller.setDriveMode(DriveMode::FORWARD);
            const auto tick = [&] {
                if (direct) controller.pwmInterruptHandler(adc, pwm);
                else controller.pwmInterruptHandler();
            };
            tick(); // Consume the healthy Hall sample requested by drive entry.
            hall.position = HALL_POSITION_INVALID;
            controller.hallSensorInterruptHandler();
            tick();
            check(controller.getHallFault() == HallFault::NONE && controller.getHallInfo().invalid_time_us == 0,
                  "one transient illegal reading starts the hold rather than faulting immediately");
            const unsigned reads = hall.reads;
            const auto limit = ActiveTiming::CurrentClock::ticksFromUs(monitor.invalid_persist_time_us);
            if (!persistent) {
                hall.position = 1; // GPIO settles without another filtered Hall edge/interrupt.
                tick();
                check(hall.reads == reads + 1 && controller.io().out.measured_position == 1 &&
                      controller.getHallInfo().invalid_time_us == 0,
                      "next control tick re-reads the illegal GPIO and observes recovery without a new edge");
                for (uint32_t sample = 0; sample < limit + 2; ++sample) tick();
                check(controller.getHallFault() == HallFault::NONE && pwm.enabled &&
                      controller.getDriveMode() == DriveMode::FORWARD && hall.reads == reads + 1,
                      "recovered Hall stops retries and never ages the stale illegal reading into a fault");
            } else {
                for (uint32_t sample = 1; sample < limit; ++sample) {
                    tick();
                    check(controller.getHallFault() == HallFault::NONE && hall.reads == reads + sample &&
                          controller.getHallInfo().invalid_time_us == ActiveTiming::CurrentClock::ticksToUs(sample),
                          "persistent illegal GPIO is retried without resetting or shortening its hold");
                }
                tick();
                check(controller.getHallFault() == HallFault::INVALID_CODE && !pwm.enabled &&
                      controller.getDriveMode() == DriveMode::NEUTRAL && hall.reads == reads + limit &&
                      controller.getHallInfo().invalid_time_us == ActiveTiming::CurrentClock::ticksToUs(limit),
                      "persistent illegal GPIO still trips on the exact configured hold tick without new edges");
            }
        }
    }
}

static void testCapturedCurrentFrames() {
    for (auto algorithm : {DriveAlgorithm::SIX_STEP, DriveAlgorithm::FOC}) {
        Pwm raw_pwm, captured_pwm;
        Hall raw_hall, captured_hall;
        Adc raw;
        CapturedAdc captured;
        auto params = makeParams();
        params.algorithm = algorithm;
        params.max_voltage = 24.0f;
        BldcController reference(raw_pwm, raw_hall, params, &raw);
        BldcController controller(captured_pwm, captured_hall, params, &captured);
        for (auto* next : {&reference, &controller}) {
            check(next->initialize(), "captured-frame controller initializes");
            next->start();
            next->setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
            next->setDriveMode(DriveMode::FORWARD);
            next->setCurrent(1.5f);
        }
        // The common loop sees normalized frames, whether from injected x1/x2,
        // hardware oversampling or DMA. Actual scan count stays below this API.
        const uint32_t frames[][3] = {
            {2000u * 2, 2060u * 2, 2080u * 2},
            {2000u + 2001u, 2060u + 2059u, 2080u + 2081u},
            {2100u * 2, 2020u * 2, 2010u * 2},
        };
        for (const auto& frame : frames) {
            for (unsigned i = 0; i < 3; ++i) raw.currents[i] = captured.currents[i] = frame[i];
            const unsigned reads = captured.frame_reads, writes = captured_pwm.native_writes;
            const unsigned bus_reads = captured.bus_reads;
            reference.pwmInterruptHandler();
            controller.pwmInterruptHandler();
            check(captured.frame_reads == reads + 1 && captured.current_reads == 0 &&
                  captured.bus_reads == bus_reads + 1 && captured.temperature_reads == 0,
                  "one controller tick uses the coherent native override and a separate cached bus read");
            check(captured_pwm.native_writes == writes + 1,
                  "one completed current frame produces one PWM update regardless of scan count");
            const auto& expected = reference.io().out.electrical;
            const auto& actual = controller.io().out.electrical;
            check(actual.measured_current == expected.measured_current &&
                  actual.i_d == expected.i_d && actual.i_q == expected.i_q &&
                  actual.v_d == expected.v_d && actual.v_q == expected.v_q &&
                  actual.duty_u == expected.duty_u && actual.duty_v == expected.duty_v &&
                  actual.duty_w == expected.duty_w && actual.bus_voltage == expected.bus_voltage,
                  "native captured frames and raw getter frames produce identical control outputs");
        }
        captured.bus = 8190;
        controller.pwmInterruptHandler();
        check(controller.getDriveMode() == DriveMode::NEUTRAL && !captured_pwm.enabled,
              "independently updated cached Vbus still protects a native captured-current path");
    }
}

// A standalone PLL is the oracle for ownership: one update per qualified ADC tick,
// no updates from Hall EXTI, command publication, or the slow loop.
static void testCurrentTick() {
    for (auto algorithm : {DriveAlgorithm::FOC, DriveAlgorithm::SIX_STEP}) {
        for (auto mode : {ControlMode::OPEN_LOOP, ControlMode::CLOSED_LOOP_TORQUE}) {
            Pwm pwm;
            Hall hall;
            CapturedAdc adc;
            auto params = makeParams();
            params.algorithm = algorithm;
            params.pi_current_regulator.kp = .1f;
            params.foc_current_regulator.kp = 1;
            BldcController motor(pwm, hall, params, &adc);
            check(motor.initialize(), "ADC owner fixture initializes");
            motor.start();
            motor.setControlMode(mode);
            motor.setDriveMode(DriveMode::FORWARD);
            motor.setTargetSpeed(2);
            MotorPLL reference(params.max_speed_rps * params.num_poles * 3, params.useInverseCommTable);
            reference.setUsePLL(true);
            reference.updateHall(hall.position);
            reference.reset();
            auto& io = motor.io();
            for (unsigned n = 0; n < 100; ++n) {
                const uint8_t sector = (n / 8) % 6;
                const auto angle = io.out.pll.angle_steps;
                const auto applied = io.out.electrical.duty_u;
                motor.hallSensorInterruptHandler();
                motor.setCurrent(n % 2 ? 2 : 1);
                check(io.out.pll.angle_steps == angle && io.out.electrical.duty_u == applied,
                      "Hall notification and demand publication do not change the previous result");
                const unsigned masks = mask_entries, writes = pwm.native_writes, reads = hall.reads;
                check(motor.pwmInterruptHandler(adc, pwm, sector), "one qualified tick completes");
                reference.updateHall(sector);
                reference.updateTick();
                check(mask_entries == masks && hall.reads == reads && io.out.measured_position == sector,
                      "predecoded Hall is consumed without reread or masking the carrier");
                check(io.out.pll.angle_steps == reference.angle_steps &&
                      io.out.pll.speed_steps_s == reference.speed_steps_s,
                      "coalesced EXTI advances the real rotor exactly once per current tick");
                check(adc.frame_reads == n + 1 && adc.bus_reads == n + 1 && pwm.native_writes == writes + 1,
                      "one current triple drives exactly one electrical update");
            }
            motor.setDriveMode(DriveMode::NEUTRAL);
            const auto writes = pwm.native_writes;
            check(motor.pwmInterruptHandler(adc, pwm, 5) && !pwm.enabled && pwm.native_writes == writes,
                  "parked acquisition continues without applying duty");
            const auto& e = io.out.electrical;
            check(e.i_d == BldcController::Current{} && e.i_q == BldcController::Current{} &&
                  e.v_q == BldcController::Voltage{} && e.v_d == BldcController::Voltage{} &&
                  e.duty_u == BldcController::Duty{} && e.duty_v == BldcController::Duty{} &&
                  e.duty_w == BldcController::Duty{}, "parked output contains no stale drive results");
        }
    }
}

static void testBreakCancellation() {
    Pwm pwm;
    Hall hall;
    CapturedAdc adc;
    auto params = makeParams();
    params.algorithm = DriveAlgorithm::FOC;
    BldcController motor(pwm, hall, params, &adc);
    check(motor.initialize(), "cancellation fixture initializes");
    motor.start();
    motor.setDriveMode(DriveMode::FORWARD);
    struct Break {
        Pwm& pwm;
        BldcController& motor;
    } source{pwm, motor};
    const auto interrupt = [](void* ptr) {
        auto& b = *static_cast<Break*>(ptr);
        check(!mask_depth, "BRK can preempt the current tick");
        b.pwm.enabled = false; // Hardware latch clears the gate before the ISR.
        b.motor.cancelPwmTick();
    };
    hall.context = &source;
    hall.on_read = interrupt;
    const auto writes = pwm.native_writes;
    check(!motor.pwmInterruptHandler() && !pwm.enabled && pwm.native_writes == writes && !adc.frame_reads,
          "break in unmasked Hall work cancels the tick before acquisition/PWM");
    hall.on_read = nullptr;
    check(!motor.pwmInterruptHandler() && pwm.native_writes == writes,
          "unparked software cannot clear a break on the next IRQ entry");
    motor.parkFromCarrier();
    check(motor.pwmInterruptHandler() && !pwm.enabled, "qualified neutral acquisition can resume tracking");
    motor.setDriveMode(DriveMode::FORWARD);
    pwm.context = &source;
    pwm.on_native_write = interrupt;
    const auto enables = pwm.enables;
    check(!motor.pwmInterruptHandler() && !pwm.enabled && pwm.enables == enables,
          "break during duty writes cannot re-enable the bridge or publish a scope sample");
}

static void testBorrowedTargetGuard() {
    Pwm pwm;
    Hall hall;
    BldcController controller(pwm, hall, makeParams());
    BldcController other(pwm, hall, makeParams());
    BldcController::PreparedTargetSpeed prepared, foreign, invalid;
    check(controller.prepareNativeTargetSpeed(number_v<BldcController::Speed, 3, 2>, prepared) &&
          other.prepareNativeTargetSpeed(number_v<BldcController::Speed, 2>, foreign),
          "prepare owner-bound commands without enabling the bridge");
    check(mask_depth == 0, "preparation does not hold an IRQ mask");
    {
        CriticalSection guard;
        const unsigned entries = mask_entries;
        check(!controller.commitNativeTargetSpeed(prepared, guard) && !pwm.enabled,
              "borrowed guard still rejects a freshly parked drive");
        check(mask_depth == 1 && mask_entries == entries, "rejected commit does not nest the mask");
    }
    controller.setDriveMode(DriveMode::FORWARD);
    {
        CriticalSection guard;
        const unsigned entries = mask_entries;
        check(controller.commitNativeTargetSpeed(prepared, guard),
              "borrowed guard accepts this controller's prepared command");
        check(!controller.commitNativeTargetSpeed(foreign, guard) &&
              !controller.commitNativeTargetSpeed(invalid, guard),
              "borrowed guard rejects foreign and invalid commands without replacing the target");
        check(controller.io().in.target_speed_rps == number_v<BldcController::Speed, 3, 2>,
              "failed commits preserve the accepted target");
        check(mask_depth == 1 && mask_entries == entries, "accepted commit does not nest the safety mask");
    }
    const unsigned entries = mask_entries;
    check(controller.commitNativeTargetSpeed(prepared) && mask_entries == entries + 1 && mask_depth == 0,
          "standalone commit still owns and restores exactly one mask");
    controller.setDriveMode(DriveMode::NEUTRAL);
}

static void testNativePath() {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto params = makeParams();
    params.max_voltage = 24.0f;
    params.foc_current_regulator.kp = 16; // Nontrivial modulation even with the scooter's 1/16 standstill weight.
    BldcController controller(pwm, hall, params, &adc);
    check(controller.initialize(), "native controller initializes");
    controller.start();
    controller.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    controller.setDriveMode(DriveMode::FORWARD);
    controller.setCurrent(2);
    const unsigned initial_writes = pwm.native_writes;
    controller.pwmInterruptHandler();
    controller.pwmInterruptHandler();
    check(pwm.native_writes == initial_writes + 2, "both PWM samples use the native interface");
    check(std::abs(numberCast<float>(controller.io().out.electrical.measured_current) - 0.1007326f) < 0.0001f,
          "integer ADC sum reaches six-step current telemetry with the right physical scale");
    controller.setAlgorithm(DriveAlgorithm::FOC);
    controller.pwmInterruptHandler();
    const auto& fast = controller.io().out.electrical;
    const float vd = numberCast<float>(fast.v_d), vq = numberCast<float>(fast.v_q);
    [[maybe_unused]] const auto duty_u = fast.duty_u;
    const float expected_modulation = std::hypot(vd, vq) *
                                      FOC_SQRT3 / numberCast<float>(fast.bus_voltage);
    controller.update();
#if LIBECU_FOC_DUTY_TELEMETRY
    check(expected_modulation > 0.08f &&
          std::abs(numberCast<float>(controller.io().reportedDuty()) - expected_modulation) < 0.00003f,
          "FOC modulation telemetry is computed from a slow coherent native snapshot");
#else
    check(expected_modulation > 0.08f && numberCast<float>(controller.io().reportedDuty()) == 0.0f &&
          numberCast<float>(fast.v_q) == vq && fast.duty_u == duty_u,
          "host-math telemetry omits only scalar FOC duty; voltage and PWM control are preserved");
#endif
    adc.bus = 8190;
    controller.pwmInterruptHandler();
    check(!pwm.enabled && controller.getDriveMode() == DriveMode::NEUTRAL,
          "native bus measurement still trips overvoltage protection");
    check(numberCast<float>(controller.io().reportedDuty()) == 0.0f && numberCast<float>(controller.io().in.target_current) == 0.0f,
          "protection parks command and telemetry together");

    Console console(controller);
    MotorIo sample{};
    using P = ActiveNumericProfile;
    sample.in.is_running = true;
    sample.in.drive_mode = DriveMode::FORWARD;
    sample.in.target_current = number_v<P::Current, 50>;
    sample.out.electrical.measured_current = number_v<P::Current, -50>;
    sample.out.electrical.i_d = number_v<P::Current, -1, 8>;
    sample.out.electrical.i_q = number_v<P::Current, 1, 8>;
    sample.out.electrical.angle_e_rad = -number_v<P::Angle, 1570796326795LL, 1000000000000LL>;
    sample.out.electrical.duty_cycle = number_v<P::Duty, 95, 100>;
    sample.out.measured_position = 4;
    console.startOscilloscope();
    for (unsigned i = 0; i < Console::OSC_BUFFER_SIZE; ++i) console.captureCurrentSample(sample);
    check(console.drainOsc().find("+OSC:0,-32768,32767,95,4,-125,125,270\r\n") == 0,
          "native OSC preserves signs, saturates milliamps and wraps negative angles");
#ifdef ECU_FIXED_NUMERIC
    console.startOscilloscope();
    constexpr auto period = number_v<P::Angle, 628318530718LL, 100000000000LL>;
    uint32_t random = 0x36a902c7u;
    std::string expected;
    for (unsigned i = 0; i < Console::OSC_BUFFER_SIZE; ++i) {
        random ^= random << 13; random ^= random >> 17; random ^= random << 5;
        sample.out.electrical.angle_e_rad = P::Angle::fromRaw(static_cast<int32_t>(random));
        console.captureCurrentSample(sample);
        int32_t wrapped = sample.out.electrical.angle_e_rad.raw() % period.raw();
        if (wrapped < 0) wrapped += period.raw();
        const uint32_t turn = (uint64_t(wrapped) << 16) / uint32_t(period.raw());
        char line[96];
        std::snprintf(line, sizeof(line), "+OSC:%u,-32768,32767,95,4,-125,125,%u\r\n",
                      i, turn * 360u / 65536u);
        expected += line;
    }
    expected += "+OSC:\r\n";
    check(console.drainOsc() == expected, "fixed OSC angles match exact division over the complete signed range");
#endif
}

// Generate physical Hall sectors from a prescribed mechanical coast, independently
// of controller estimates and bridge commands. Only real sector changes raise the
// Hall IRQ; only the PWM frame advances the PLL, including during a coast.
enum class RotorClock { PWM, STOPPED };
class RotorTrace {
public:
    RotorTrace(BldcController& controller, Hall& hall) : controller_(controller), hall_(hall) {}

    void advance(float from_rps, float to_rps, uint32_t ticks, RotorClock clock = RotorClock::PWM) {
        const float steps_per_rev = controller_.getParams().num_poles * 3.0f;
        for (uint32_t i = 0; i < ticks; ++i) {
            const float rps = from_rps + (to_rps - from_rps) * float(i + 1) / float(ticks);
            phase_ = std::fmod(phase_ + rps * steps_per_rev / ActiveTiming::CURRENT_HZ + 6.0f, 6.0f);
            const uint8_t position = static_cast<unsigned>(phase_ + 0.5f) % 6u;
            if (position != hall_.position) {
                hall_.position = position;
                controller_.hallSensorInterruptHandler();
            }
            fake_time_us += 1000000u / ActiveTiming::CURRENT_HZ;
            if (clock == RotorClock::PWM) controller_.pwmInterruptHandler();
            if (++ticks_ % (ActiveTiming::CURRENT_HZ / ActiveTiming::SPEED_HZ) == 0)
                controller_.update();
            else if (ticks_ % (ActiveTiming::CURRENT_HZ / ActiveTiming::SPEED_PI_HZ) == 0)
                controller_.updateSpeedPi();
        }
    }

private:
    BldcController& controller_;
    Hall& hall_;
    float phase_ = 0.0f;
    uint32_t ticks_ = 0;
};

static void checkCoastTelemetry(Console& console, BldcController& controller, float physical_rps) {
    const float tolerance = physical_rps == 0.0f ? 0.005f : 0.15f;
    float speed = 1000.0f;
    check(std::sscanf(console.command("AT+SPD?").c_str(), "+SPD:%f", &speed) == 1 &&
          std::abs(speed - physical_rps) < tolerance,
          "UART SPD? follows the signed physical coast while PWM continues");
    console.clearOutput();
    console.onControlTick();
    float instantaneous = 1000.0f, integral = 1000.0f, angle = -1.0f;
    unsigned hall = 255;
    int sync = -1;
    check(std::sscanf(console.output().c_str(), "+PLL:%f;%f;%f;%u;%d",
                      &instantaneous, &integral, &angle, &hall, &sync) == 5,
          "UART PLL telemetry remains available with the bridge disabled");
    const auto& out = controller.io().out;
    check(std::abs(integral / (controller.getParams().num_poles * 3) - physical_rps) < tolerance,
           "UART PLL integrator follows the changing physical coast speed");
    check(hall == out.measured_position && std::abs(angle - numberCast<float>(out.pll.angle_steps)) < 0.001f,
          "observer publishes native angle and Hall position used by UART telemetry");
    if (physical_rps == 0.0f)
        check(std::abs(instantaneous) < 0.1f, "PLL angle advance also settles at standstill");
}

static void testParkedPwmTracking(int direction, DriveAlgorithm algorithm) {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto params = makeParams();
    params.algorithm = algorithm;
    BldcController controller(pwm, hall, params, &adc);
    check(controller.initialize(), "coast controller initializes");
    controller.start();
    controller.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    controller.setDriveMode(direction > 0 ? DriveMode::FORWARD : DriveMode::REVERSE);
    controller.setCurrent(4);
    RotorTrace trace(controller, hall);
    trace.advance(direction * 8.08f, direction * 8.08f, 2 * ActiveTiming::CURRENT_HZ);
    const float running_speed = numberCast<float>(controller.io().in.current_speed_rps);
    check(pwm.enabled && std::abs(running_speed - 8.08f) < 0.15f && controller.getPllInfo().is_sync,
           "PWM tracks a real Hall sequence in either drive direction");

    // Disable torque, but keep the physical PWM time base and acquisition running.
    controller.setDriveMode(DriveMode::NEUTRAL);
    controller.update();
    check(!pwm.enabled && std::abs(numberCast<float>(controller.io().in.current_speed_rps) - direction * running_speed) < 0.0001f,
          "entering NEUTRAL retains physical signed rotor speed instead of faking zero");
    const unsigned enables = pwm.enables;
    const auto hall_before = controller.getHallInfo();
    trace.advance(direction * 8.08f, direction * 8.08f, 1);
    controller.update();
    check(std::abs(numberCast<float>(controller.io().in.current_speed_rps)) > 7.5f,
          "first parked PWM tick preserves the already-moving rotor estimate");

    Console console(controller);
    console.setTelemetryEnabled(false);
    check(console.command("AT+PLL=1") == "OK\r\n", "PLL telemetry enables in NEUTRAL");
    trace.advance(direction * 8.08f, direction * 6.0f, 3 * ActiveTiming::CURRENT_HZ / 5);
    checkCoastTelemetry(console, controller, direction * 6.0f);
    const float coast_fast = numberCast<float>(controller.io().in.current_speed_rps);
    check(controller.getHallInfo().edges > hall_before.edges &&
          controller.getHallInfo().edge_accum < hall_before.edge_accum &&
          controller.io().out.measured_position == hall.position,
          "parked PWM ticks consume real Hall edges and age Hall health");

    for (int repeat = 0; repeat < 3; ++repeat) {
        const auto before = controller.getPllInfo();
        for (const char* command : {"AT+DMODE=2", "AT+SPD=12", "AT+CUR=6", "AT+MODE=1"})
            check(console.command(command) == "OK\r\n", "parked demand command remains protocol-compatible");
        controller.update();
        const auto& parked = controller.io();
        check(!pwm.enabled && pwm.enables == enables && parked.in.target_current == BldcController::Current{} &&
              parked.in.target_speed_rps == BldcController::Speed{} && parked.reportedDuty() == BldcController::Duty{},
              "repeated neutral and AT demands cannot energize the parked bridge or wind up its speed loop");
        check(controller.getPllInfo().pll_integral == before.pll_integral &&
              controller.getPllInfo().angle == before.angle,
              "repeated neutral and AT demands cannot reset an actual coast estimate");
    }

    const unsigned duty_writes = pwm.native_writes;
    trace.advance(direction * 6.0f, direction * 2.0f, ActiveTiming::CURRENT_HZ);
    checkCoastTelemetry(console, controller, direction * 2.0f);
    const float coast_slow = numberCast<float>(controller.io().in.current_speed_rps);
    trace.advance(direction * 2.0f, 0.0f, ActiveTiming::CURRENT_HZ);
    trace.advance(0.0f, 0.0f, 2 * ActiveTiming::CURRENT_HZ);
    checkCoastTelemetry(console, controller, 0.0f);
    check(std::abs(numberCast<float>(controller.io().in.current_speed_rps)) < 0.005f,
          "speed reaches telemetry zero from stationary Hall feedback on PWM ticks");
    check(pwm.native_writes == duty_writes, "parked PWM frames do not apply new electrical outputs");
    check(!pwm.enabled && pwm.enables == enables && controller.getHallFault() == HallFault::NONE,
          "coast observation leaves the bridge disabled and preserves healthy Hall diagnostics");
    std::printf("Parked PWM %s %s: drive %.3f, coast %.3f -> %.3f -> %.6f RPS\n",
                algorithm == DriveAlgorithm::FOC ? "FOC" : "six-step", direction > 0 ? "forward" : "reverse",
                running_speed, coast_fast, coast_slow, numberCast<float>(controller.io().in.current_speed_rps));
}

static void testVelocityStandstillRelease(bool filtered) {
    Pwm pwm, fresh_pwm;
    Hall hall, fresh_hall;
    Adc adc, fresh_adc;
    auto params = makeParams();
    params.speed_pi.kp = 0.25f;
    params.speed_pi.ki = 0.5f;
    params.target_speed_lpf_alpha = params.measured_speed_lpf_alpha = filtered ? 0.2f : 0.0f;
    params.acceleration_rate = filtered ? 2.0f : 0.0f;
    const auto direction = filtered ? DriveMode::REVERSE : DriveMode::FORWARD;
    BldcController controller(pwm, hall, params, &adc);
    BldcController fresh(fresh_pwm, fresh_hall, params, &fresh_adc);
    for (auto* drive : {&controller, &fresh}) {
        check(drive->initialize(), "standstill release controller initializes");
        drive->start();
        drive->setDriveMode(direction);
    }
    controller.setTargetSpeed(1.0f);
    for (unsigned i = 0; i < ActiveTiming::SPEED_HZ; ++i) controller.update();
    check(numberCast<float>(controller.io().in.target_current) > 0.1f,
          "nonzero stationary speed demand builds real PI output before release");
    controller.setTargetSpeed(0.0f);
    controller.update();
    check(numberCast<float>(controller.io().in.target_current) == 0.0f,
          "zero command at standstill releases PI immediately, including an unfinished filtered ramp");
    bool stays_zero = true;
    for (unsigned i = 0; i < ActiveTiming::SPEED_HZ; ++i) {
        controller.update();
        stays_zero &= numberCast<float>(controller.io().in.target_current) == 0.0f;
    }
    check(stays_zero, "active velocity idle remains exactly zero rather than retaining PI history");

    // Below the standstill speed tolerance, but still a genuine user command.
    controller.setTargetSpeed(0.0005f);
    fresh.setTargetSpeed(0.0005f);
    bool same_restart = true;
    for (unsigned i = 0; i < 20; ++i) {
        controller.update();
        fresh.update();
        same_restart &= std::abs(numberCast<float>(controller.io().in.target_current) - numberCast<float>(fresh.io().in.target_current)) < 2e-6f;
    }
    check(numberCast<float>(controller.io().in.target_current) > 0.0f,
          "tiny nonzero stationary command is not swallowed by idle release");
    check(same_restart, "restart after release matches a fresh controller without stale integral or ramp");

    controller.setTargetSpeed(0.0f);
    controller.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    controller.setCurrent(0.125f);
    controller.update();
    check(std::abs(numberCast<float>(controller.io().in.target_current) - 0.125f) < 2e-6f,
          "zero speed command does not cancel an explicit standstill torque demand");
}

static void testVelocityBrakingRelease(int direction) {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto params = makeParams();
    params.speed_pi.kp = 0.25f;
    params.speed_pi.ki = 0.5f;
    params.measured_speed_lpf_alpha = 0.2f;
    BldcController controller(pwm, hall, params, &adc);
    check(controller.initialize(), "braking release controller initializes");
    controller.start();
    controller.setDriveMode(direction > 0 ? DriveMode::FORWARD : DriveMode::REVERSE);
    RotorTrace trace(controller, hall);
    trace.advance(direction * 2.0f, direction * 2.0f, ActiveTiming::CURRENT_HZ);
    check(numberCast<float>(controller.io().in.current_speed_rps) > 1.8f && numberCast<float>(controller.io().in.target_current) < -0.1f,
          "zero speed command retains braking current while the rotor moves in the selected direction");
    trace.advance(direction * 2.0f, 0.0f, ActiveTiming::CURRENT_HZ);
    trace.advance(0.0f, 0.0f, 3 * ActiveTiming::CURRENT_HZ);
    check(std::abs(controller.getPllInfo().pll_integral) < 0.1f,
          "decelerated Hall trace settles the real PLL into standstill");
    check(numberCast<float>(controller.io().in.target_current) == 0.0f,
          "braking integral releases to exact zero after real Hall deceleration");
    trace.advance(-direction * 2.0f, -direction * 2.0f, ActiveTiming::CURRENT_HZ);
    check(numberCast<float>(controller.io().in.current_speed_rps) < -1.8f && numberCast<float>(controller.io().in.target_current) > 0.1f,
          "zero command still applies opposite corrective current when the rotor moves against drive direction");
}

static void testOneFrameReconstruction() {
    using Core = FocAlgorithm::Core;
    using Current = Core::Current;
    for (bool clamped : {false, true}) {
        for (unsigned discarded = 0; discarded < 3; ++discarded) {
            Core core;
            Core::Params params{};
            params.angle_offset_rad = Core::Angle{};
            params.low_side_clamped_pwm = clamped;
            core.setParams(params);
            core.reset();
            Core::Input in{};
            in.bus_voltage = number_v<Core::Voltage, 24>;
            core.setCurrentPiGains(number_v<Core::Gain, 1>, Core::Gain{});
            in.target_current = number_v<Current, 16>;
            in.drive_mode = DriveMode::FORWARD;
            // A positive q voltage at 0 degrees gives V the highest duty;
            // rotating it by 120 degrees gives W the highest. Force a DIFFERENT
            // phase, reproducing the sampled-window vs latest-duty mismatch.
            in.angle_steps = discarded == 1 ? number_v<Core::Angle, 2> : Core::Angle{};
            const unsigned automatic = discarded == 1 ? 2u : 1u;
            const auto seeded = core.update(in);
            const Core::Duty duties[] = {seeded.duty_u, seeded.duty_v, seeded.duty_w};
            check(core.reconstructedPhase() == automatic && automatic != discarded &&
                  duties[automatic] > duties[(automatic + 1) % 3] &&
                  duties[automatic] > duties[(automatic + 2) % 3],
                  "physical voltage vector seeds an automatic reconstruction different from the invalid shunt");

            // Balanced physical motor currents: U=2 A, V=-0.5 A, W=-1.5 A.
            // ADC-facing signs are opposite. Expected dq is calculated from
            // that physical vector, independently of the reconstruction code.
            in.i_u = number_v<Current, -2>;
            in.i_v = number_v<Current, 1, 2>;
            in.i_w = number_v<Current, 3, 2>;
            const float angle = numberCast<float>(in.angle_steps) * FOC_RAD_PER_STEP;
            const float expected_d = 2.0f * std::cos(angle) + std::sin(angle) / std::sqrt(3.0f);
            const float expected_q = -2.0f * std::sin(angle) + std::cos(angle) / std::sqrt(3.0f);
            const auto accurate = [&](const Core::Output& out) {
                return std::abs(numberCast<float>(out.i_d) - expected_d) < 0.002f &&
                       std::abs(numberCast<float>(out.i_q) - expected_q) < 0.002f;
            };
            for (int poison : {15, -19}) {
                auto sampled = in;
                Current* phases[] = {&sampled.i_u, &sampled.i_v, &sampled.i_w};
                *phases[discarded] = numberCast<Current>(float(poison));
                Core default_only = core;
                check(!accurate(default_only.update(sampled)),
                      "negative control: highest-duty reconstruction consumes the poisoned invalid shunt");
                check(core.setNextReconstructedPhase(discarded) &&
                      core.reconstructedPhase() == discarded && core.reconstructedPhase() == discarded,
                      "explicit U/V/W choice is visible without consuming the one-frame override");
                check(!core.setNextReconstructedPhase(3) &&
                      !core.setNextReconstructedPhase(static_cast<unsigned>(-1)) &&
                      core.reconstructedPhase() == discarded,
                      "invalid reconstruction requests cannot corrupt a pending qualified choice");
                const auto out = core.update(sampled);
                check(accurate(out), "poisoning the discarded physical shunt cannot influence measured Id/Iq");
                check(sampled.i_u == (discarded == 0 ? numberCast<Current>(float(poison)) : in.i_u) &&
                      sampled.i_v == (discarded == 1 ? numberCast<Current>(float(poison)) : in.i_v) &&
                      sampled.i_w == (discarded == 2 ? numberCast<Current>(float(poison)) : in.i_w),
                      "reconstruction leaves all original raw phase inputs intact");
                check(core.reconstructedPhase() == automatic,
                      "one-frame override expires back to the next actual highest duty");
                sampled = in;
                *phases[automatic] = number_v<Current, 15>;
                check(accurate(core.update(sampled)),
                      "following frame really reconstructs the automatic phase rather than retaining the override");
            }
            check(core.setNextReconstructedPhase(discarded), "queue reconstruction before parameter replacement");
            core.setParams(params);
            check(core.reconstructedPhase() == automatic,
                  "parameter replacement clears the pending override while preserving automatic duty history");
            check(core.setNextReconstructedPhase(2), "queue reconstruction before reset");
            core.reset();
            check(core.reconstructedPhase() == 0, "reset clears the pending choice and restores the U tie rule");
            check(!core.setNextReconstructedPhase(256) && core.reconstructedPhase() == 0,
                  "large invalid phase cannot truncate into a valid override");
        }
    }
}

static void testControllerReconstruction() {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto params = makeParams();
    params.foc_current_regulator.kp = 1;
    BldcController controller(pwm, hall, params, &adc);
    check(controller.initialize(), "reconstruction controller initializes");
    controller.start();
    controller.setDriveMode(DriveMode::FORWARD);
    unsigned reads = adc.reads(), writes = pwm.calls;
    check(!controller.setCurrentReconstructedPhase(0) && controller.currentReconstructedPhase() == -1,
          "board reconstruction setter rejects six-step");
    check(adc.reads() == reads && pwm.calls == writes, "rejected reconstruction makes no hardware calls");
    controller.setAlgorithm(DriveAlgorithm::FOC);
    controller.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    controller.setCurrent(4);
    adc.currents[0] = 3895; adc.currents[1] = 4145; adc.currents[2] = 4245;
    controller.pwmInterruptHandler();
    const auto& e = controller.io().out.electrical;
    const auto healthy_d = e.i_d, healthy_q = e.i_q;
    check(controller.currentReconstructedPhase() == 1, "controller's current frame requests highest-duty V");
    adc.currents[0] = 6000; // U aperture was invalid even though V is the latest duty maximum.
    reads = adc.reads(); writes = pwm.calls;
    check(controller.setCurrentReconstructedPhase(0) && controller.currentReconstructedPhase() == 0,
          "board can select sampled-invalid U instead of latest-highest V for active FOC");
    check(!controller.setCurrentReconstructedPhase(3) && controller.currentReconstructedPhase() == 0,
          "controller rejects invalid phase without replacing qualified U");
    check(adc.reads() == reads && pwm.calls == writes, "successful reconstruction setter is also hardware-free");
    const unsigned current_reads = adc.current_reads;
    controller.pwmInterruptHandler();
    check(std::abs(numberCast<float>(e.i_d - healthy_d)) < 0.002f && std::abs(numberCast<float>(e.i_q - healthy_q)) < 0.002f,
          "board-to-controller-to-FOC path rejects poisoned U using qualified V/W");
    check(adc.currents[0] == 6000 && adc.currents[1] == 4145 && adc.currents[2] == 4245 &&
          adc.current_reads == current_reads + 3,
          "normal current handler still acquires all three original phase samples");
    check(controller.currentReconstructedPhase() == 1, "controller getter returns to automatic selection next frame");

    check(controller.setCurrentReconstructedPhase(2), "queue choice before an aborted frame parks the bridge");
    controller.setDriveMode(DriveMode::NEUTRAL);
    reads = adc.reads(); writes = pwm.calls;
    check(!controller.setCurrentReconstructedPhase(1) && controller.currentReconstructedPhase() == -1,
          "board reconstruction setter rejects NEUTRAL FOC");
    check(adc.reads() == reads && pwm.calls == writes, "neutral rejection has no ADC/PWM side effects");
    controller.setDriveMode(DriveMode::FORWARD);
    check(controller.currentReconstructedPhase() == 0, "re-entry cannot leak an aborted frame's reconstruction");
    check(controller.setCurrentReconstructedPhase(2) && controller.initialize(),
          "controller reinitialization accepts valid configuration");
    check(controller.currentReconstructedPhase() == 0, "reinitialization clears queued reconstruction");
    check(controller.setCurrentReconstructedPhase(2), "queue choice before algorithm handover");
    controller.setAlgorithm(DriveAlgorithm::SIX_STEP);
    check(!controller.setCurrentReconstructedPhase(1), "inactive FOC cannot accept a board reconstruction choice");
    controller.setAlgorithm(DriveAlgorithm::FOC);
    check(controller.currentReconstructedPhase() == 0, "algorithm handover cannot restore a stale FOC choice");

    FocAlgorithm standalone(pwm);
    check(standalone.setNextReconstructedPhase(2) && standalone.setParams(standalone.getParams()),
          "FOC wrapper forwards reconstruction and valid parameter replacement");
    check(standalone.reconstructedPhase() == 0, "FOC parameter replacement cancels the pending choice");
}

static void testFrozenRotorClock() {
    Pwm pwm;
    Hall hall;
    Adc adc;
    BldcController controller(pwm, hall, makeParams(), &adc);
    check(controller.initialize(), "frozen-clock regression setup initializes");
    controller.start();
    controller.setDriveMode(DriveMode::FORWARD);
    RotorTrace trace(controller, hall);
    trace.advance(8.08f, 8.08f, 2 * ActiveTiming::CURRENT_HZ);
    const float before = numberCast<float>(controller.io().in.current_speed_rps);
    controller.setDriveMode(DriveMode::NEUTRAL);
    const auto pll = controller.getPllInfo();
    const auto health = controller.getHallInfo();
    const unsigned reads = hall.reads;
    trace.advance(8.08f, 0.0f, 2 * ActiveTiming::CURRENT_HZ, RotorClock::STOPPED);
    trace.advance(0.0f, 0.0f, ActiveTiming::CURRENT_HZ, RotorClock::STOPPED);
    const auto after = controller.getPllInfo();
    check(before > 7.5f && numberCast<float>(controller.io().in.current_speed_rps) == before &&
          after.angle == pll.angle && after.pll_integral == pll.pll_integral &&
          after.hall_state_raw == pll.hall_state_raw &&
          hall.reads == reads && controller.getHallInfo().edges == health.edges,
          "Hall edges, speed PI, slow service and telemetry never advance tracking without PWM");
    controller.pwmInterruptHandler();
    check(hall.reads == reads + 1 && controller.getPllInfo().hall_state_raw == hall.position,
          "first real PWM frame consumes the pending Hall sector once");
}

int main() {
    testSpeedClock();
    testIntermediatePi();
    testLiveSpeedPiGains();
    testSlewClock();
    testTractionAcceleration();
    for (bool filtered : {false, true}) testVelocityStandstillRelease(filtered);
    for (int direction : {1, -1}) testVelocityBrakingRelease(direction);
    testCommands();
    testFragmentedCommands();
    testAtLifecycle();
    testHallDuration();
    testInvalidHallRetry();
    testNativePath();
    testBorrowedTargetGuard();
    testCapturedCurrentFrames();
    testCurrentTick();
    testBreakCancellation();
    testFrozenRotorClock();
    testParkedPwmTracking(1, DriveAlgorithm::FOC);
    testParkedPwmTracking(-1, DriveAlgorithm::FOC);
    testParkedPwmTracking(1, DriveAlgorithm::SIX_STEP);
    testOneFrameReconstruction();
    testControllerReconstruction();
    std::printf("Control integration: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
