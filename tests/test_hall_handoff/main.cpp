/** @file main.cpp
 * @brief Physical Hall must replace stale virtual/parked sectors before commutation.
 */
#include "bldc_controller.hpp"
#include <cstdio>
#include <initializer_list>

static uint32_t clock_us = 0;
uint32_t time_us() { return clock_us; }
extern "C" void disable_interrupts() {}
extern "C" void enable_interrupts() {}

using namespace libecu;
class Pwm final : public PwmInterface {
public:
    Pwm() { frequency_ = 20000; }
    bool initialize(uint32_t, uint16_t) override { return true; }
    void setChannelState(PwmChannel, PwmState) override {}
    void updateDutyCycle(float, float, float) override {}
    void enable(bool) override {}
};
class Hall final : public HallInterface {
public:
    uint8_t position = 4;
    bool initialize() override { return true; }
    uint8_t getPosition() override { return position; }
};
class Adc final : public AdcInterface {
public:
    Adc() {
        initialize(CurrentSensorCalibration{0.003f, 16.0f, 3.3f, 12, 1.65f, 1.65f, 1.65f},
                   VoltageSensorParameters{169000.0f, 18000.0f});
    }
    uint32_t getRawAdcValue(PwmChannel) override { return 2048; }
    uint32_t getRawAdcValue() override { return 2500; }
    uint32_t getRawTemperatureAdcValue() override { return 0; }
};

int main() {
    int checks = 0, failures = 0;
    auto check = [&](bool value, const char* text) {
        ++checks;
        if (!value) { ++failures; std::printf("FAIL: %s\n", text); }
    };
    for (ControlMode closed_mode : {ControlMode::CLOSED_LOOP_VELOCITY, ControlMode::CLOSED_LOOP_TORQUE}) {
        Pwm pwm;
        Hall hall;
        Adc adc;
        MotorControlParams params{};
        params.num_poles = 8;
        params.max_speed_rps = 200.0f;
        params.max_current = 6.0f;
        params.min_current = -6.0f;
        params.max_voltage = 36.0f;
        params.max_duty_cycle = 0.95f;
        params.max_temperature_c = 100.0f;
        params.useInverseCommTable = true;
        BldcController controller(pwm, hall, params, &adc);
        check(controller.initialize(), "controller initializes");
        controller.start();
        check(controller.getPllInfo().angle == 4.0f, "start seeds the actual Hall angle without a fictitious speed transient");
        controller.setElectricMode(ElectricMode::VOLTAGE_MODE);
        controller.setDriveMode(DriveMode::FORWARD);
        controller.setControlMode(ControlMode::OPEN_LOOP);
        controller.setTargetSpeed(20.0f);
        clock_us += 100000;
        for (unsigned tick = 0; tick < 100; ++tick) controller.pwmInterruptHandler();
        check(controller.getPllInfo().hall_state_raw == hall.position,
              "open-loop field does not overwrite physical Hall tracking");
        check(controller.getStatus().target_position != (hall.position + 4) % 6,
              "the virtual field advances independently on the PWM clock");

        // No sensor interrupt: the rotor has settled at sector 3 after the open-loop run.
        hall.position = 3;
        controller.setControlMode(closed_mode);
        check(controller.getPllInfo().hall_state_raw == 3 && controller.getPllInfo().angle == 3.0f,
              "leaving open loop immediately replaces the virtual position");
        controller.pwmInterruptHandler();
        check(controller.getStatus().measured_position == 3 && controller.getStatus().target_position == 1,
              "first closed-loop PWM cycle uses physical Hall and JK42BLS01 alignment");

        controller.setDriveMode(DriveMode::NEUTRAL);
        hall.position = 5;  // Rotor moved while parked, without a delivered edge.
        controller.setDriveMode(DriveMode::FORWARD);
        check(controller.getPllInfo().hall_state_raw == 5 && controller.getPllInfo().angle == 5.0f,
              "re-arming refreshes the physical position before enabling the bridge");
        controller.pwmInterruptHandler();
        check(controller.getStatus().target_position == 3, "re-armed commutation is based on the new sector");
        controller.setDriveMode(DriveMode::NEUTRAL);

        // Real edges continue to update the PLL while a rotor coasts in neutral.
        for (int sector = 0; sector < 18; ++sector) {
            hall.position = (hall.position + 1) % 6;
            controller.hallSensorInterruptHandler();
            for (int tick = 0; tick < 100; ++tick) controller.pwmInterruptHandler();
        }
        const auto before = controller.getPllInfo();
        check(before.pll_integral > 1.0f, "coasting Hall edges establish a real speed estimate");
        controller.setDriveMode(DriveMode::FORWARD);
        const auto after = controller.getPllInfo();
        check(after.angle == before.angle && after.pll_integral == before.pll_integral,
              "re-arming preserves an already current coasting estimate");
        controller.setDriveMode(DriveMode::NEUTRAL);
    }
    std::printf("Hall handoff: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
