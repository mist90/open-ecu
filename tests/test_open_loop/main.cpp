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
    for (DriveMode direction : {DriveMode::FORWARD, DriveMode::REVERSE}) {
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
