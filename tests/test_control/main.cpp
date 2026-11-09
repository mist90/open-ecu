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
    void setChannelState(PwmChannel, PwmState) override { ++calls; }
    void updateDutyCycle(float, float, float) override { ++calls; }
    void updateNativeDutyCycle(NativeDuty u, NativeDuty v, NativeDuty w) override {
        ++calls;
        ++native_writes;
        check(u >= NativeDuty{} && v >= NativeDuty{} && w >= NativeDuty{}, "native PWM is nonnegative");
    }
    void enable(bool value) override { ++calls; enabled = value; if (value) ++enables; }
    void apply() override { ++calls; }
    unsigned calls = 0;
    unsigned enables = 0;
    unsigned native_writes = 0;
    bool enabled = false;
};
class Hall final : public HallInterface {
public:
    bool initialize() override { return true; }
    uint8_t getPosition() override { ++reads; return position; }
    uint8_t position = 0;
    unsigned reads = 0;
};
class Adc final : public AdcInterface {
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
    NativeMotorStatus sample{};
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
    const auto fast = controller.getStatus();
    const float expected_modulation = std::sqrt(fast.v_d * fast.v_d + fast.v_q * fast.v_q) *
                                      FOC_SQRT3 / fast.bus_voltage;
    controller.update();
    check(expected_modulation > 0.08f &&
          std::abs(controller.getStatus().duty_cycle - expected_modulation) < 0.00003f,
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
#ifdef ECU_FIXED_NUMERIC
    console.startOscilloscope();
    constexpr auto period = number_v<P::Angle, 628318530718LL, 100000000000LL>;
    uint32_t random = 0x36a902c7u;
    std::string expected;
    for (unsigned i = 0; i < Console::OSC_BUFFER_SIZE; ++i) {
        random ^= random << 13; random ^= random >> 17; random ^= random << 5;
        sample.angle_e_rad = P::Angle::fromRaw(static_cast<int32_t>(random));
        console.captureNativeOscSample(sample);
        int32_t wrapped = sample.angle_e_rad.raw() % period.raw();
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
// Hall IRQ; the selected fast clock is the sole source of PLL time progression.
enum class RotorClock { CURRENT, OBSERVATION, STOPPED };
class RotorTrace {
public:
    RotorTrace(BldcController& controller, Hall& hall, Adc& adc, Pwm& pwm)
        : controller_(controller), hall_(hall), adc_(adc), pwm_(pwm) {}

    void advance(float from_rps, float to_rps, uint32_t ticks, RotorClock clock) {
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
            if (clock == RotorClock::CURRENT) {
                controller_.pwmInterruptHandler();
            } else if (clock == RotorClock::OBSERVATION) {
                const unsigned reads = adc_.reads();
                const unsigned writes = pwm_.calls;
                controller_.rotorObservationInterruptHandler();
                observer_is_passive &= adc_.reads() == reads && pwm_.calls == writes;
            }
            if (++ticks_ % (ActiveTiming::CURRENT_HZ / ActiveTiming::SPEED_HZ) == 0)
                controller_.update();
        }
    }

    bool observer_is_passive = true;
private:
    BldcController& controller_;
    Hall& hall_;
    Adc& adc_;
    Pwm& pwm_;
    float phase_ = 0.0f;
    uint32_t ticks_ = 0;
};

static void checkCoastTelemetry(Console& console, BldcController& controller, float physical_rps) {
    const float tolerance = physical_rps == 0.0f ? 0.005f : 0.15f;
    float speed = 1000.0f;
    check(std::sscanf(console.command("AT+SPD?").c_str(), "+SPD:%f", &speed) == 1 &&
          std::abs(speed - physical_rps) < tolerance,
          "UART SPD? follows the signed physical coast after the current ISR stops");
    console.clearOutput();
    console.onControlTick();
    float instantaneous = 1000.0f, integral = 1000.0f, angle = -1.0f;
    unsigned hall = 255;
    int sync = -1;
    check(std::sscanf(console.output().c_str(), "+PLL:%f;%f;%f;%u;%d",
                      &instantaneous, &integral, &angle, &hall, &sync) == 5,
          "UART PLL telemetry remains available without current samples");
    const auto status = controller.getStatus();
    check(std::abs(integral / (controller.getParams().num_poles * 3) - physical_rps) < tolerance,
          "UART PLL integrator follows real coast speed rather than its pre-fault value");
    check(hall == status.measured_position && std::abs(angle - status.pll_angle) < 0.001f,
          "observer publishes native angle and Hall position used by UART telemetry");
    if (physical_rps == 0.0f)
        check(std::abs(instantaneous) < 0.1f, "PLL angle advance also settles at standstill");
}

static void testRotorObservation(int direction, DriveAlgorithm algorithm) {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto params = makeParams();
    params.algorithm = algorithm;
    BldcController controller(pwm, hall, params, &adc);
    check(controller.initialize(), "coast controller initializes");
    controller.start();
    controller.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    controller.setElectricMode(ElectricMode::VOLTAGE_MODE);
    controller.setDriveMode(direction > 0 ? DriveMode::FORWARD : DriveMode::REVERSE);
    controller.setDutyCycle(0.25f);
    RotorTrace trace(controller, hall, adc, pwm);
    trace.advance(direction * 8.08f, direction * 8.08f, 2 * ActiveTiming::CURRENT_HZ, RotorClock::CURRENT);
    const float running_speed = controller.getStatus().current_speed_rps;
    check(pwm.enabled && std::abs(running_speed - 8.08f) < 0.15f && controller.getPllInfo().is_sync,
          "healthy current loop tracks a real Hall sequence before failure in either drive direction");

    const auto active_pll = controller.getPllInfo();
    const unsigned active_reads = adc.reads(), active_writes = pwm.calls, active_hall_reads = hall.reads;
    controller.rotorObservationInterruptHandler();
    check(controller.getPllInfo().angle == active_pll.angle &&
          controller.getPllInfo().pll_integral == active_pll.pll_integral &&
          adc.reads() == active_reads && pwm.calls == active_writes && hall.reads == active_hall_reads,
          "observation handler is inert while drive mode is active");

    // The board's latched failure disables acquisition and selects NEUTRAL.
    // From here on this test never invokes pwmInterruptHandler() again.
    controller.setDriveMode(DriveMode::NEUTRAL);
    controller.update();
    check(!pwm.enabled && std::abs(controller.getStatus().current_speed_rps - direction * running_speed) < 0.0001f,
          "entering NEUTRAL retains physical signed rotor speed instead of faking zero");
    const unsigned current_reads = adc.current_reads, bus_reads = adc.bus_reads, enables = pwm.enables;
    const auto hall_before = controller.getHallInfo();
    trace.advance(direction * 8.08f, direction * 8.08f, 1, RotorClock::OBSERVATION);
    controller.update();
    check(std::abs(controller.getStatus().current_speed_rps) > 7.5f,
          "first observation tick preserves the already-moving rotor estimate");

    Console console(controller);
    console.setTelemetryEnabled(false);
    check(console.command("AT+PLL=1") == "OK\r\n", "PLL telemetry enables after current-loop failure");
    trace.advance(direction * 8.08f, direction * 6.0f, 3 * ActiveTiming::CURRENT_HZ / 5,
                  RotorClock::OBSERVATION);
    checkCoastTelemetry(console, controller, direction * 6.0f);
    const float coast_fast = controller.getStatus().current_speed_rps;
    check(controller.getHallInfo().edges > hall_before.edges &&
          controller.getHallInfo().edge_accum < hall_before.edge_accum &&
          controller.getStatus().measured_position == hall.position,
          "observation-only ticks consume real Hall edges and age Hall health");

    for (int repeat = 0; repeat < 3; ++repeat) {
        const auto before = controller.getPllInfo();
        for (const char* command : {"AT+DMODE=2", "AT+SPD=12", "AT+CUR=6", "AT+DUT=0.9", "AT+MODE=1"})
            check(console.command(command) == "OK\r\n", "parked demand command remains protocol-compatible");
        controller.update();
        const auto parked = controller.getStatus();
        check(!pwm.enabled && pwm.enables == enables && parked.target_current == 0.0f &&
              parked.target_speed_rps == 0.0f && parked.duty_cycle == 0.0f,
              "repeated neutral and AT demands cannot energize the parked bridge or wind up its speed loop");
        check(controller.getPllInfo().pll_integral == before.pll_integral &&
              controller.getPllInfo().angle == before.angle,
              "repeated neutral and AT demands cannot reset an actual coast estimate");
    }

    trace.advance(direction * 6.0f, direction * 2.0f, ActiveTiming::CURRENT_HZ, RotorClock::OBSERVATION);
    checkCoastTelemetry(console, controller, direction * 2.0f);
    const float coast_slow = controller.getStatus().current_speed_rps;
    trace.advance(direction * 2.0f, 0.0f, ActiveTiming::CURRENT_HZ, RotorClock::OBSERVATION);
    trace.advance(0.0f, 0.0f, 2 * ActiveTiming::CURRENT_HZ, RotorClock::OBSERVATION);
    checkCoastTelemetry(console, controller, 0.0f);
    check(std::abs(controller.getStatus().current_speed_rps) < 0.005f,
          "speed reaches telemetry zero from stationary Hall feedback without ADC or control ticks");
    check(trace.observer_is_passive && adc.current_reads == current_reads && adc.bus_reads == bus_reads,
          "every observation interrupt performs zero ADC reads and zero PWM calls, including at standstill");
    check(!pwm.enabled && pwm.enables == enables && controller.getHallFault() == HallFault::NONE,
          "coast observation leaves the bridge disabled and preserves healthy Hall diagnostics");
    std::printf("Rotor observation %s %s: drive %.3f, coast %.3f -> %.3f -> %.6f RPS\n",
                algorithm == DriveAlgorithm::FOC ? "FOC" : "six-step", direction > 0 ? "forward" : "reverse",
                running_speed, coast_fast, coast_slow, controller.getStatus().current_speed_rps);
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
            in.duty_command = number_v<Core::Duty, 1, 4>;
            in.drive_mode = DriveMode::FORWARD;
            in.electric_mode = ElectricMode::VOLTAGE_MODE;
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
    BldcController controller(pwm, hall, makeParams(), &adc);
    check(controller.initialize(), "reconstruction controller initializes");
    controller.start();
    controller.setDriveMode(DriveMode::FORWARD);
    unsigned reads = adc.reads(), writes = pwm.calls;
    check(!controller.setCurrentReconstructedPhase(0) && controller.currentReconstructedPhase() == -1,
          "board reconstruction setter rejects six-step");
    check(adc.reads() == reads && pwm.calls == writes, "rejected reconstruction makes no hardware calls");
    controller.setAlgorithm(DriveAlgorithm::FOC);
    controller.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    controller.setElectricMode(ElectricMode::VOLTAGE_MODE);
    controller.setDutyCycle(0.25f);
    adc.currents[0] = 3895; adc.currents[1] = 4145; adc.currents[2] = 4245;
    controller.pwmInterruptHandler();
    const auto healthy = controller.getStatus();
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
    const auto recovered = controller.getStatus();
    check(std::abs(recovered.i_d - healthy.i_d) < 0.002f && std::abs(recovered.i_q - healthy.i_q) < 0.002f,
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
    RotorTrace trace(controller, hall, adc, pwm);
    trace.advance(8.08f, 8.08f, 2 * ActiveTiming::CURRENT_HZ, RotorClock::CURRENT);
    const float before = controller.getStatus().current_speed_rps;
    controller.setDriveMode(DriveMode::NEUTRAL);
    trace.advance(8.08f, 0.0f, 2 * ActiveTiming::CURRENT_HZ, RotorClock::STOPPED);
    trace.advance(0.0f, 0.0f, ActiveTiming::CURRENT_HZ, RotorClock::STOPPED);
    check(before > 7.5f && controller.getStatus().current_speed_rps == before,
          "negative control reproduces frozen speed: Hall IRQs plus slow updates cannot replace the PLL clock");
}

static void testObservationWithoutAdc() {
    Pwm pwm;
    Hall hall;
    BldcController controller(pwm, hall, makeParams());
    check(controller.initialize(), "observation without an ADC initializes");
    controller.start();
    const unsigned writes = pwm.calls;
    hall.position = 1;
    controller.hallSensorInterruptHandler();
    for (unsigned tick = 0; tick < ActiveTiming::CURRENT_HZ / 10; ++tick)
        controller.rotorObservationInterruptHandler();
    check(controller.getStatus().measured_position == 1 && controller.getPllInfo().angle > 0.9f,
          "rotor observation does not depend on the current handler's ADC prerequisite");
    check(pwm.calls == writes && !pwm.enabled, "ADC-less observation cannot touch the bridge");
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
    testFragmentedCommands();
    testAtLifecycle();
    testHallDuration();
    testNativePath();
    testFrozenRotorClock();
    testRotorObservation(1, DriveAlgorithm::FOC);
    testRotorObservation(-1, DriveAlgorithm::FOC);
    testRotorObservation(1, DriveAlgorithm::SIX_STEP);
    testObservationWithoutAdc();
    testOneFrameReconstruction();
    testControllerReconstruction();
#if defined(ECU_FIXED_PI) || defined(ECU_FIXED_NUMERIC)
    testRejectedModeTuning();
#endif
    std::printf("Control integration: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
