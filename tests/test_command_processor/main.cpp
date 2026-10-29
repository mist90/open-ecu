/**
 * @file main.cpp
 * @brief Execute the shared firmware main with a protocol that does not link AT
 */
#include "board.hpp"
#include "bldc_controller.hpp"
#include "command_processor.hpp"
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
