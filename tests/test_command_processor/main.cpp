/**
 * @file main.cpp
 * @brief Execute the shared firmware main with a protocol that does not link AT
 */
#include "board.hpp"
#include "bldc_controller.hpp"
#include "command_processor.hpp"
#include "motor_config.hpp"
#include <algorithm>
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
    processor.captureCurrentSample(controller.io());
    check(!processor.wantsCurrentSample(), "capture is disabled by default");
    check(silent.writes == 0, "default lifecycle hooks emit no bytes");
    check(silent.process_calls == 1, "process dispatches through the protocol-neutral base");
    check(controller.getDriveMode() == before, "default hooks preserve drive state");
}

void testMotorDefaults(BldcController& controller, const BoardInfo& board) {
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
    float pll_kp{}, pll_ki{};
    controller.getPllBaseGains(pll_kp, pll_ki);
#ifdef MOTOR_1
    check(params.num_poles == 40 && !params.useInverseCommTable, "scooter Hall configuration survives");
    check(std::fabs(controller.getFocAngleOffsetDeg() + 60.0f) < 0.001f,
          "scooter FOC alignment remains -60 degrees");
#if MOTOR_1_LOADED
    check(speed_kp == 3.f && speed_ki == 24.f, "loaded scooter boots with 3/24 speed PI");
#else
    check(speed_kp == 1.25f && speed_ki == 5.f, "scooter boots with 1.25/5 speed PI by default");
#endif
    check(pll_kp == 50.f && pll_ki == 1250.f, "scooter Hall PLL uses 50/1250");
    check(params.traction_acceleration_rps2 == 1.f && motor_config::traction_gain_a_per_rps == 2.f,
          "both scooter PI selections keep the same measured-acceleration calibration");
    const float positive_limit = std::min(45.f, board.max_drive_current_a);
    check(params.speed_pi.integral_max == positive_limit, "rolling I headroom cannot exceed the drive rail");
    using FocGain = ActiveNumericProfile::Gain;
    check(foc_kp == numberCast<float>(constant<FocGain>(0.549775f)) &&
          foc_ki == numberCast<float>(constant<FocGain>(516.792f)),
          "scooter boots with FOC gains qualified with first-order dq filtering");
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
    check(pll_kp == 100.f && pll_ki == 5000.f, "JK42 keeps its own Hall PLL tuning");
    check(params.traction_acceleration_rps2 == 0.f, "JK42 boots without measured acceleration feedback");
    const float positive_limit = std::min(6.f, board.max_drive_current_a);
#endif
    check(params.max_current == positive_limit && params.speed_pi.max_output == positive_limit &&
          params.min_current == -6.f && params.speed_pi.min_output == -6.f,
          "positive PI rail respects both board and motor; the braking rail is independent");
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
        check(info_.max_drive_current_a == 20.f, "a board without an explicit drive rating defaults to 20 A");
        if (const char* limit = std::getenv("TEST_BOARD_DRIVE_LIMIT_A"))
            info_.max_drive_current_a = std::strtof(limit, nullptr);
        if (const char* cutoff = std::getenv("TEST_BOARD_CURRENT_FILTER_HZ"))
            info_.current_feedback_filter_hz = std::strtof(cutoff, nullptr);
        if (const char* hybrid = std::getenv("TEST_BOARD_HYBRID_IQ"))
            info_.hybrid_iq_control = std::strcmp(hybrid, "1") == 0;
        return pwm_.initialize(info_.pwm_frequency_hz, info_.requested_dead_time_ns) && hall_.initialize();
    }
    const BoardInfo& info() const noexcept override { return info_; }
    PwmInterface& pwm() noexcept override { return pwm_; }
    HallInterface& hall() noexcept override { return hall_; }
    AdcInterface* adc() noexcept override { return nullptr; }
    CommandProcessor& createConsole(BldcController& controller) noexcept override {
        record('C');
#ifdef MOTOR_1
        check(!info_.hybrid_iq_control, "a PI-only motor must reject a hybrid board before protocol startup");
#endif
        testDefaults(controller);
        testMotorDefaults(controller, info_);
        const auto& foc = controller.getParams().foc;
        const auto expected_order = info_.current_feedback_filter_hz > 0.0f
            ? CurrentFilterOrder::FIRST_ORDER : CurrentFilterOrder::OFF;
        check(foc.current_filter_order == expected_order,
              "board enables dq filtering independently of the selected motor");
        check(foc.current_filter_cutoff_hz == info_.current_feedback_filter_hz,
              "shared main loads the board's actual dq cutoff without a motor override");
        check(foc.hybrid_iq.enabled == info_.hybrid_iq_control &&
              foc.hybrid_iq.r_phase_ohm == BLDC_R_PHASE_OHM,
              "board selects hybrid strategy while the motor supplies its resistance");
        static Protocol protocol(&controller);
        return protocol;
    }
    bool startCurrentLoop(BldcController& controller, CommandProcessor& protocol) noexcept override {
        controller_ = &controller;
        record('F');
        check(!protocol.wantsCurrentSample(), "board can omit snapshots for a non-capturing protocol");
        return true;
    }
    bool startControlLoop() noexcept override { record('L'); return true; }
    bool takeControlTick() noexcept override {
        if (iterations_++ < 2) return true;
        check(std::strcmp(events, "ICFLSTPOTPO") == 0,
              "shared main initializes and dispatches every protocol lifecycle hook in order");
        controller_->setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
        controller_->setDriveMode(DriveMode::FORWARD);
        controller_->setCurrent(100.f);
        check(numberCast<float>(controller_->io().in.target_current) == std::min(BLDC_MAX_CURRENT, info_.max_drive_current_a),
              "a torque command cannot exceed the installed board/motor pair");
        controller_->setCurrent(-100.f);
        check(numberCast<float>(controller_->io().in.target_current) == -6.f, "board drive rating does not change the braking floor");
        controller_->setDriveMode(DriveMode::NEUTRAL);
        std::printf("Command processor lifecycle: %u checks, %u failures\n", checks, failures);
        std::exit(failures == 0 ? 0 : 1);
    }
    [[noreturn]] void fail() noexcept override {
#ifdef MOTOR_1
        if (info_.hybrid_iq_control) {
            check(std::strcmp(events, "I") == 0, "incompatible current control is rejected before arming loops");
            std::printf("Hybrid board / PI-only motor: boot rejected, %u failures\n", failures);
            std::exit(failures ? 1 : 0);
        }
#endif
        std::fprintf(stderr, "FAIL: shared main rejected the fake board\n");
        std::exit(1);
    }
private:
    BoardInfo info_{ActiveTiming::CURRENT_HZ, ActiveTiming::SPEED_HZ, 24.0f, 350, 350};
    BldcController* controller_ = nullptr;
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
