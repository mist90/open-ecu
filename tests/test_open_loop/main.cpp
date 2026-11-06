/** @file main.cpp
 * @brief Mechanical-RPS, PWM timing, direction and independent Hall-feedback regressions
 */
#include "bldc_controller.hpp"
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

using namespace libecu;
static uint32_t clock_us = 0;
uint32_t time_us() { return clock_us; }
extern "C" void disable_interrupts() {}
extern "C" void enable_interrupts() {}
static int checks = 0, failures = 0;
static void check(bool value, const char* message) {
    ++checks;
    if (!value) { ++failures; std::printf("FAIL: %s\n", message); }
}

class Pwm final : public PwmInterface {
public:
    Pwm() { frequency_ = ActiveTiming::CURRENT_HZ; }
    bool initialize(uint32_t, uint16_t) override { return true; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float, float, float) override {}
    void updateNativeDutyCycle(NativeDuty, NativeDuty, NativeDuty) override {}
    void enable(bool value) override { enabled = value; }
    bool enabled = false;
};
class Hall final : public HallInterface {
public:
    bool initialize() override { return true; }
    uint8_t getPosition() override { return position; }
    uint8_t position = 0;
};
class Adc final : public AdcInterface {
public:
    Adc() {
        initialize({0.003f, 16.0f, 3.3f, 12, 1.65f, 1.65f, 1.65f}, {169000.0f, 18000.0f});
    }
    uint32_t getRawAdcValue(PwmChannel) override { return 2048; }
    uint32_t getRawAdcValue() override { return 2500; }
    uint32_t getRawTemperatureAdcValue() override { return 1000; }
};
static MotorControlParams params(uint8_t poles = 40) {
    MotorControlParams result{};
    result.num_poles = poles;
    result.max_speed_rps = poles == 40 ? 12.0f : 200.0f;
    result.max_current = 18.0f;
    result.min_current = -6.0f;
    result.max_voltage = 36.0f;
    result.max_duty_cycle = 0.95f;
    result.max_temperature_c = 100.0f;
    result.measured_speed_lpf_alpha = 0.5f;
    result.foc.angle_offset_rad = -FOC_RAD_PER_STEP;
    return result;
}
static float angleDelta(float next, float previous) {
    float result = next - previous;
    if (result > FOC_PI) result -= FOC_TWO_PI;
    if (result < -FOC_PI) result += FOC_TWO_PI;
    return result;
}

template<class Profile, uint32_t Frequency, unsigned Column>
static void testPreparedRate() {
    using Phase = OpenLoopPhase<Profile, Frequency>;
    using Speed = typename Profile::Speed;
    using Rate = typename Profile::PllSpeed;
    using Angle = typename Profile::Angle;
    // Independently pinned old DDS outputs: Q16 inputs use exact rational
    // rounding; float inputs include binary32 rounding at each old operation.
    // In particular, float's +0.5 differs from fixed's rounded division at 2 RPS.
    struct Vector { float speed; uint8_t poles; uint32_t floating[3], fixed[3]; };
    const Vector vectors[] = {
        {0.0f, 40, {0, 0, 0}, {0, 0, 0}},
        {1.0f / 65536, 40, {82, 66, 41}, {82, 66, 41}},
        {0.125f, 8, {134218, 107374, 67109}, {134218, 107374, 67109}},
        {2.0f, 40, {10737420, 8589934, 5368710}, {10737418, 8589935, 5368709}},
        {3.0f, 40, {16106128, 12884902, 8053064}, {16106127, 12884902, 8053064}},
        {12.0f, 40, {64424512, 51539608, 32212256}, {64424509, 51539608, 32212255}},
        {200.0f, 8, {214748368, 171798688, 107374184}, {214748365, 171798692, 107374182}},
        {799.0f, 40, {4289598720u, 3431678720u, 2144799360u},
                     {4289598587u, 3431678870u, 2144799293u}}
    };
    typename Phase::PreparedRate prepared;
    for (const auto& vector : vectors) {
        check(Phase::prepareSpeed(numberCast<Speed>(vector.speed), vector.poles, prepared),
              "prepared DDS accepts previously representable rates");
        const uint32_t increment = std::is_same<Speed, float>::value
                                 ? vector.floating[Column] : vector.fixed[Column];
        check(prepared.increment == increment, "prepared DDS increment equals independent old-rate fixture");
        check(prepared.electrical_rate == numberCast<Rate>(vector.speed * vector.poles * 3),
              "prepared electrical steps per second retain mechanical pole scaling");
    }

    const auto previous = prepared;
    for (const uint8_t poles : {0, 1, 3, 255})
        check(!Phase::prepareSpeed(number_v<Speed, 1>, poles, prepared), "invalid poles reject preparation");
    check(!Phase::prepareSpeed(number_v<Speed, -1>, 40, prepared), "negative DDS magnitude rejects preparation");
    check(!Phase::prepareSpeed(number<Speed>(Frequency / 20), 40, prepared),
          "one electrical revolution per sample is outside DDS range");
    check(prepared.increment == previous.increment && prepared.electrical_rate == previous.electrical_rate,
          "failed standalone conversion retains the prior prepared rate");
    if constexpr (std::is_same<Speed, float>::value) {
        for (const float bad : {std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::infinity(),
                                -std::numeric_limits<float>::infinity()})
            check(!Phase::prepareSpeed(bad, 40, prepared), "nonfinite DDS speed rejects preparation");
    }

    Phase phase;
    phase.seed(number_v<Angle, 23, 4>);
    check(phase.setSpeed(number_v<Speed, 2>, 40), "old rate starts the phase");
    const Angle before = phase.angle();
    check(Phase::prepareSpeed(number_v<Speed, 3>, 40, prepared), "next rate prepares independently of phase");
    for (unsigned tick = 0; tick < 100; ++tick) phase.advance(DriveMode::FORWARD);
    const Angle advanced = phase.angle();
    check(advanced != before, "phase has advanced while the prepared command waited");
    phase.applyPreparedRate(prepared);
    check(phase.angle() == advanced && phase.speed(DriveMode::FORWARD) == number_v<Rate, 360> &&
          phase.speed(DriveMode::REVERSE) == number_v<Rate, -360> && phase.speed(DriveMode::NEUTRAL) == Rate{},
          "publishing a prepared rate preserves the advanced phase and direction semantics");
    phase.advance(DriveMode::FORWARD);
    float delta = numberCast<float>(phase.angle()) - numberCast<float>(advanced);
    if (delta < 0) delta += 6;
    check(std::abs(delta - 360.0f / Frequency) < 0.000002f, "next PWM sample uses the new rate");
    const Angle held = phase.angle();
    phase.advance(DriveMode::NEUTRAL);
    check(phase.angle() == held, "neutral still holds phase after a prepared-rate commit");
}

static void testPreparedRateBounds() {
    using Slow = OpenLoopPhase<FixedProfile, 1>;
    Slow::PreparedRate rate;
    check(Slow::prepareSpeed(FixedProfile::Speed::fromRaw(65535), 2, rate) && rate.increment == 4294901760u,
          "minimum PWM frequency accepts the last sub-turn Q16 rate");
    check(!Slow::prepareSpeed(number_v<FixedProfile::Speed, 1>, 2, rate),
          "minimum PWM frequency rejects a whole turn");
    using Fast = OpenLoopPhase<FixedProfile, 1000000>;
    Fast::PreparedRate fast;
    check(!Fast::prepareSpeed(number_v<FixedProfile::Speed, 2000>, 40, fast),
          "electrical-rate saturation rejects even when the DDS increment fits");
    check(!Fast::prepareSpeed(FixedProfile::Speed::fromRaw(INT32_MAX), 2, fast),
          "maximum native magnitude cannot saturate the electrical rate silently");
    using FloatSlow = OpenLoopPhase<FloatProfile, 1>;
    FloatSlow::PreparedRate floating;
    check(FloatSlow::prepareSpeed(std::nextafter(1.0f, 0.0f), 2, floating) &&
          floating.increment == 4294967040u, "binary32 rate just below a whole turn stays representable");
    check(!FloatSlow::prepareSpeed(1.0f, 2, floating), "binary32 rate at a whole turn remains invalid");
}

static void testPreparedController(DriveMode direction) {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto configuration = params();
    configuration.algorithm = DriveAlgorithm::FOC;
    BldcController motor(pwm, hall, configuration, &adc);
    check(motor.initialize(), "prepared-command controller initializes");
    motor.start();
    motor.setControlMode(ControlMode::OPEN_LOOP);
    motor.setElectricMode(ElectricMode::VOLTAGE_MODE);
    motor.setDriveMode(direction);
    motor.setTargetSpeed(2.0f);
    for (unsigned tick = 0; tick < 100; ++tick) motor.pwmInterruptHandler();
    const float before = motor.getStatus().angle_e_rad;
    BldcController::PreparedTargetSpeed prepared;
    check(motor.prepareNativeTargetSpeed(number_v<BldcController::Speed, 3>, prepared),
          "controller prepares rate without changing target");
    check(motor.getStatus().target_speed_rps == 2.0f, "preparation leaves the live target alone");
    for (unsigned tick = 0; tick < 100; ++tick) motor.pwmInterruptHandler();
    const float advanced = motor.getStatus().angle_e_rad;
    check(advanced != before && motor.commitNativeTargetSpeed(prepared),
          "controller accepts prepared target after intervening PWM interrupts");
    motor.pwmInterruptHandler();
    const float sign = direction == DriveMode::FORWARD ? 1.0f : -1.0f;
    check(std::abs(angleDelta(motor.getStatus().angle_e_rad, advanced) -
                   sign * FOC_TWO_PI * 20 * 3 / ActiveTiming::CURRENT_HZ) < 0.00001f,
          "real FOC uses current phase plus new increment, never a stale prepared phase");

    for (float speed : {0.0f, 3.0f, 100.0f}) {
        check(motor.prepareNativeTargetSpeed(numberCast<BldcController::Speed>(speed), prepared) &&
              motor.commitNativeTargetSpeed(prepared), "native requests retain zero and maximum clamping");
        check(motor.getStatus().target_speed_rps == (speed > 12 ? 12 : speed),
              "prepared target retains the configured maximum");
    }
    check(!motor.prepareNativeTargetSpeed(number_v<BldcController::Speed, -1>, prepared) &&
          !motor.commitNativeTargetSpeed(prepared) && motor.getStatus().target_speed_rps == 12.0f,
          "failed preparation invalidates an older command without changing the live target");
    motor.setTargetSpeed(std::numeric_limits<float>::quiet_NaN());
    motor.setTargetSpeed(std::numeric_limits<float>::infinity());
    check(motor.getStatus().target_speed_rps == 12.0f, "legacy float setter still ignores nonfinite inputs");
#ifndef ECU_FIXED_NUMERIC
    check(!motor.prepareNativeTargetSpeed(std::numeric_limits<float>::quiet_NaN(), prepared) &&
          !motor.commitNativeTargetSpeed(prepared), "native float NaN remains rejected");
    check(motor.prepareNativeTargetSpeed(std::numeric_limits<float>::infinity(), prepared) &&
          motor.commitNativeTargetSpeed(prepared) && motor.getStatus().target_speed_rps == 12.0f,
          "native positive infinity preserves the old finite-limit clamping semantics");
#endif
    motor.setDriveMode(DriveMode::NEUTRAL);
    motor.setNativeTargetSpeed(number_v<BldcController::Speed, 3>);
    check(!pwm.enabled && motor.getStatus().target_speed_rps == 0.0f,
          "legacy native setter still ignores requests in NEUTRAL");
}

static void testRate(uint8_t poles, float rps, DriveMode direction, unsigned seconds = 1) {
    Pwm pwm;
    Hall hall;
    Adc adc;
    BldcController motor(pwm, hall, params(poles), &adc);
    check(motor.initialize(), "open-loop test controller initializes");
    motor.start();
    motor.setControlMode(ControlMode::OPEN_LOOP);
    motor.setElectricMode(ElectricMode::VOLTAGE_MODE);
    motor.setDriveMode(direction);
    motor.pwmInterruptHandler();
    uint8_t last = motor.getNativeStatus().target_position;
    motor.setTargetSpeed(rps);
    const float accepted = motor.getStatus().target_speed_rps;
    const float expected_rate = accepted * poles * 3;
    const float interval = ActiveTiming::CURRENT_HZ / expected_rate;
    unsigned changes = 0, wrong_direction = 0, wrong_interval = 0, last_change = 0;
    const unsigned ticks = ActiveTiming::CURRENT_HZ * seconds;
    for (unsigned tick = 1; tick <= ticks; ++tick) {
        // No update() calls at all. Wall-clock jumps/wraps cannot change PWM-clocked phase.
        clock_us += tick % 2 ? UINT32_MAX - 50 : 1234567;
        motor.pwmInterruptHandler();
        const uint8_t step = motor.getNativeStatus().target_position;
        if (step != last) {
            const unsigned delta = (step + 6 - last) % 6;
            wrong_direction += delta != (direction == DriveMode::FORWARD ? 1u : 5u);
            if (changes) {
                const float elapsed = float(tick - last_change);
                wrong_interval += std::abs(elapsed - interval) > 1.1f;
            }
            ++changes;
            last_change = tick;
            last = step;
        }
    }
    check(std::abs(float(changes) - expected_rate * seconds) <= 1.0f,
          "mechanical RPS produces poles*3 commutations per revolution, without the old factor ten");
    check(wrong_direction == 0, "forward/reverse field visits every sector in the requested order");
    check(wrong_interval == 0, "commutation intervals follow the PWM clock within one tick");
    motor.update();
    check(std::abs(motor.getStatus().current_speed_rps) < 0.0001f,
          "a stationary physical rotor is not reported at the virtual field speed");
    motor.setTargetSpeed(0);
    motor.pwmInterruptHandler();
    const float held = motor.getStatus().angle_e_rad;
    for (unsigned i = 0; i < 1000; ++i) motor.pwmInterruptHandler();
    check(motor.getStatus().angle_e_rad == held, "zero RPS holds the field exactly");
    motor.setTargetSpeed(rps);
    motor.stop();
    motor.pwmInterruptHandler();
    const float stopped = motor.getStatus().angle_e_rad;
    for (unsigned i = 0; i < 1000; ++i) motor.pwmInterruptHandler();
    check(motor.getStatus().angle_e_rad == stopped, "stop freezes phase even if a speed request was present");
    motor.setDriveMode(DriveMode::NEUTRAL);
}

static void testFocContinuity(DriveMode direction) {
    Pwm pwm;
    Hall hall;
    Adc adc;
    auto configuration = params();
    configuration.algorithm = DriveAlgorithm::FOC;
    BldcController motor(pwm, hall, configuration, &adc);
    check(motor.initialize(), "FOC open loop initializes");
    motor.start();
    motor.setControlMode(ControlMode::OPEN_LOOP);
    motor.setElectricMode(ElectricMode::VOLTAGE_MODE);
    motor.setDriveMode(direction);
    motor.setDutyCycle(0.1f);
    motor.pwmInterruptHandler();
    float previous = motor.getStatus().angle_e_rad;
    float max_error = 0;
    const float sign = direction == DriveMode::FORWARD ? 1.0f : -1.0f;
    for (float speed : {0.5f, 3.0f, 0.0f, 12.0f}) {
        motor.setTargetSpeed(speed);
        const float expected_delta = sign * FOC_TWO_PI * 20 * speed / ActiveTiming::CURRENT_HZ;
        for (unsigned tick = 0; tick < 1000; ++tick) {
            motor.pwmInterruptHandler();
            const float angle = motor.getStatus().angle_e_rad;
            max_error = std::max(max_error, std::abs(angleDelta(angle, previous) - expected_delta));
            previous = angle;
        }
    }
    check(max_error < 0.00001f, "FOC gets a continuous phase every PWM tick, including speed changes and wrap");
    motor.setDriveMode(DriveMode::NEUTRAL);
    hall.position = 3;
    motor.setDriveMode(direction);
    motor.pwmInterruptHandler();
    const float seeded = motor.getStatus().angle_e_rad;
    check(std::abs(seeded - (3 * FOC_RAD_PER_STEP + configuration.foc.angle_offset_rad)) < 0.00001f,
          "rearming seeds the field from the present physical Hall angle");
    for (unsigned i = 0; i < 1000; ++i) motor.pwmInterruptHandler();
    check(motor.getStatus().angle_e_rad == seeded, "parking clears the cached speed increment before rearming");
    motor.setDriveMode(DriveMode::NEUTRAL);
}

static void testIndependentHall() {
    Pwm pwm;
    Hall hall;
    Adc adc;
    BldcController motor(pwm, hall, params(), &adc);
    motor.initialize();
    motor.start();
    motor.setControlMode(ControlMode::OPEN_LOOP);
    motor.setDriveMode(DriveMode::FORWARD);
    motor.setTargetSpeed(3.0f);
    unsigned changes = 0;
    uint8_t last = 1;
    for (unsigned tick = 0; tick < ActiveTiming::CURRENT_HZ; ++tick) {
        // Real rotor at ~0.5 RPS, field at 3 RPS. Hall must affect only observation.
        if (tick % 333 == 0) {
            hall.position = (hall.position + 1) % 6;
            motor.hallSensorInterruptHandler();
        }
        motor.pwmInterruptHandler();
        const uint8_t step = motor.getNativeStatus().target_position;
        if (step != last) { ++changes; last = step; }
        if (tick % 200 == 0) motor.update();
    }
    check(changes >= 359 && changes <= 361, "physical Hall events cannot steer the open-loop field");
    check(std::abs(motor.getStatus().current_speed_rps - 0.5f) < 0.08f,
          "open-loop speed telemetry measures the physical rotor");
    check(motor.getStatus().measured_position == hall.position && motor.getHallInfo().edges > 50,
          "Hall position and edge diagnostics remain live in open loop");
    const auto pll = motor.getPllInfo();
    motor.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    check(motor.getPllInfo().pll_integral == pll.pll_integral,
          "handoff preserves a real speed estimate rather than resetting it to zero");
    motor.setDriveMode(DriveMode::NEUTRAL);
}

static void testInvalidHall() {
    Pwm pwm;
    Hall hall;
    Adc adc;
    hall.position = 0xFF;
    BldcController motor(pwm, hall, params(), &adc);
    HallMonitorParams monitor;
    monitor.invalid_persist_time_us = 100;
    motor.setHallMonitorParams(monitor);
    motor.initialize();
    motor.start();
    motor.setControlMode(ControlMode::OPEN_LOOP);
    motor.setDriveMode(DriveMode::FORWARD);
    motor.setTargetSpeed(1.0f);
    for (unsigned tick = 0; tick < 1000; ++tick) motor.pwmInterruptHandler();
    check(pwm.enabled && motor.getDriveMode() == DriveMode::FORWARD,
          "open-loop commutation remains available without valid Hall feedback");
    motor.setControlMode(ControlMode::CLOSED_LOOP_TORQUE);
    for (unsigned tick = 0; tick < 10; ++tick) motor.pwmInterruptHandler();
    check(!pwm.enabled && motor.getDriveMode() == DriveMode::NEUTRAL,
          "Hall protection takes effect when physical feedback controls commutation");
}

int main() {
    testPreparedRate<FloatProfile, 16000, 0>();
    testPreparedRate<FloatProfile, 20000, 1>();
    testPreparedRate<FloatProfile, 32000, 2>();
    testPreparedRate<FixedProfile, 16000, 0>();
    testPreparedRate<FixedProfile, 20000, 1>();
    testPreparedRate<FixedProfile, 32000, 2>();
    testPreparedRateBounds();
    for (DriveMode direction : {DriveMode::FORWARD, DriveMode::REVERSE}) {
        testPreparedController(direction);
        for (float speed : {0.5f, 3.0f, 12.0f}) testRate(40, speed, direction);
        testRate(40, 1.0f / 240.0f, direction, 5); // The slow 0.5 commutations/s bench case.
        testRate(8, 200.0f, direction);
        testFocContinuity(direction);
    }
    testIndependentHall();
    testInvalidHall();
    OpenLoopPhase<ActiveNumericProfile, 20000> phase;
    check(!phase.setSpeed(number<ActiveNumericProfile::Speed>(-1), 40), "negative magnitude rejected");
    check(!phase.setSpeed(number<ActiveNumericProfile::Speed>(1), 3), "odd pole count rejected");
    std::printf("Open-loop PWM phase: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
