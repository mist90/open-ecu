/**
 * @file main.cpp
 * @brief Execute the shared firmware main with a protocol that does not link AT
 */
#include "board.hpp"
#include "bldc_controller.hpp"
#include "command_processor.hpp"
#include "motor_config.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

uint32_t time_us() { return 0; }
extern "C" void disable_interrupts() {}
extern "C" void enable_interrupts() {}

namespace {
using namespace libecu;
unsigned checks = 0;
unsigned failures = 0;
void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

class SilentProcessor final : public CommandProcessor {
public:
    using CommandProcessor::CommandProcessor;
    int32_t read() override { return -1; }
    void write(const char*, std::size_t) override { ++writes; }
    void process() noexcept override { ++process_calls; }
    unsigned writes = 0;
    unsigned process_calls = 0;
};
static_assert(sizeof(SilentProcessor) < 128, "non-AT processors must not inherit capture storage");

void testDefaults(BldcController& controller) {
    SilentProcessor silent(&controller);
    CommandProcessor& processor = silent;
    const auto before = controller.getDriveMode();
    processor.onStarted();
    processor.onControlTick();
    processor.process();
    processor.processOutput();
    processor.captureCurrentSample(controller.getNativeStatus());
    check(!processor.wantsCurrentSample(), "capture is disabled by default");
    check(silent.writes == 0, "default lifecycle hooks emit no bytes");
    check(silent.process_calls == 1, "process dispatches through the protocol-neutral base");
    check(controller.getDriveMode() == before, "default hooks preserve drive state");
}

void testMotorDefaults(BldcController& controller) {
    // Exercise the real shared main's configuration assembly, before any
    // protocol can hide an incorrect boot default with runtime setters.
    const auto& params = controller.getParams();
    check(controller.getDriveMode() == DriveMode::NEUTRAL, "motor boots in neutral");
    check(controller.getAlgorithm() == DriveAlgorithm::FOC, "qualified motor boots with FOC selected");
    check(!params.foc.use_decoupling, "unqualified feed-forward stays disabled");
    float speed_kp{}, speed_ki{}, current_kp{}, current_ki{}, foc_kp{}, foc_ki{};
    controller.getSpeedPiGains(speed_kp, speed_ki);
    controller.getCurrentPiGains(current_kp, current_ki);
    controller.getFocCurrentPi(foc_kp, foc_ki);
#ifdef MOTOR_1
    check(params.num_poles == 40 && !params.useInverseCommTable, "scooter Hall configuration survives");
    check(std::fabs(controller.getFocAngleOffsetDeg() + 60.0f) < 0.001f,
          "scooter FOC alignment remains -60 degrees");
    check(std::fabs(speed_kp - BLDC_SPEED_CURRENT_KP) < 0.00001f &&
          std::fabs(speed_ki - BLDC_SPEED_CURRENT_KI) < 0.00001f,
          "scooter loads the shared speed PI tuning");
    check(params.max_current == 20.0f && params.min_current == -6.0f,
          "scooter current limits are -6/+20 A");
    check(std::fabs(foc_kp - 2.199115f) < 0.0001f && std::fabs(foc_ki - 2067.168f) < 0.002f,
          "scooter model still produces the qualified FOC gains");
    check(std::fabs(current_kp - 0.183260f) < 0.0001f && std::fabs(current_ki - 172.264f) < 0.002f,
          "scooter six-step PI still follows the model and the test board's 24 V bus");
#else
    check(params.num_poles == 8 && params.useInverseCommTable, "JK42 inverse Hall configuration loads");
    check(std::fabs(controller.getFocAngleOffsetDeg() - 120.0f) < 0.001f,
          "JK42 boots with the hardware-qualified inverse FOC alignment");
    check(std::fabs(speed_kp - 0.025f) < 0.00001f && std::fabs(speed_ki - 0.025f) < 0.00001f,
          "JK42 boots with the damped speed PI");
    check(std::fabs(foc_kp - 5.340708f) < 0.0001f && std::fabs(foc_ki - 5026.548f) < 0.002f,
          "JK42 model produces the hardware-qualified FOC gains without AT commands");
    check(std::fabs(current_kp - 0.15f) < 0.0001f && std::fabs(current_ki - 200.0f) < 0.002f,
          "JK42 FOC model does not replace the independently tuned six-step PI");
#endif
}

char events[32]{};
std::size_t event_count = 0;
void record(char event) {
    if (event_count + 1 < sizeof(events)) events[event_count++] = event;
    else check(false, "lifecycle event buffer did not overflow");
}

class Protocol final : public CommandProcessor {
public:
    using CommandProcessor::CommandProcessor;
    int32_t read() override { return -1; }
    void write(const char*, std::size_t) override { check(false, "main never writes protocol bytes directly"); }
    void onStarted() noexcept override { record('S'); }
    void onControlTick() noexcept override { record('T'); }
    void process() noexcept override { record('P'); }
    void processOutput() noexcept override { record('O'); }
};

class Pwm final : public PwmInterface {
public:
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

class TestBoard final : public Board {
public:
    bool initialize() noexcept override {
        record('I');
        return pwm_.initialize(info_.pwm_frequency_hz, info_.requested_dead_time_ns) && hall_.initialize();
    }
    const BoardInfo& info() const noexcept override { return info_; }
    PwmInterface& pwm() noexcept override { return pwm_; }
    HallInterface& hall() noexcept override { return hall_; }
    AdcInterface* adc() noexcept override { return nullptr; }
    CommandProcessor& createConsole(BldcController& controller) noexcept override {
        record('C');
        testDefaults(controller);
        testMotorDefaults(controller);
        static Protocol protocol(&controller);
        return protocol;
    }
    bool startCurrentLoop(BldcController&, CommandProcessor& protocol) noexcept override {
        record('F');
        check(!protocol.wantsCurrentSample(), "board can omit snapshots for a non-capturing protocol");
        return true;
    }
    bool startControlLoop() noexcept override { record('L'); return true; }
    bool takeControlTick() noexcept override {
        if (iterations_++ < 2) return true;
        check(std::strcmp(events, "ICFLSTPOTPO") == 0,
              "shared main initializes and dispatches every protocol lifecycle hook in order");
        std::printf("Command processor lifecycle: %u checks, %u failures\n", checks, failures);
        std::exit(failures == 0 ? 0 : 1);
    }
    [[noreturn]] void fail() noexcept override {
        std::fprintf(stderr, "FAIL: shared main rejected the fake board\n");
        std::exit(1);
    }
private:
    BoardInfo info_{ActiveTiming::CURRENT_HZ, ActiveTiming::SPEED_HZ, 24.0f, 350, 350};
    Pwm pwm_;
    Hall hall_;
    unsigned iterations_ = 0;
};
} // namespace

namespace libecu {
Board& createBoard() noexcept {
    static TestBoard board;
    return board;
}
} // namespace libecu
