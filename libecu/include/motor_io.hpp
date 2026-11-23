/** @file motor_io.hpp @brief The two sides of the PWM interrupt. */
#ifndef LIBECU_MOTOR_IO_HPP
#define LIBECU_MOTOR_IO_HPP

#include "ecu_config.hpp"
#include "motor_types.hpp"
#include "motor_pll.hpp"
#include "hall_monitor.hpp"
#include "open_loop_phase.hpp"

namespace libecu {
enum class ControlMode : uint8_t {
    OPEN_LOOP = 0, CLOSED_LOOP_VELOCITY = 1, CLOSED_LOOP_TORQUE = 2
};

using MotorField = OpenLoopPhase<ActiveNumericProfile, ActiveTiming::CURRENT_HZ>;

/** Commands and slow samples delivered by the foreground / SysTick.
 * Setters publish native values under a short CriticalSection. */
struct MotorInput {
    using P = ActiveNumericProfile;
    P::Speed target_speed_rps{};
    P::Current target_current{};
    MotorField::PreparedRate field_rate{};
    volatile DriveMode drive_mode = DriveMode::NEUTRAL;
    ControlMode control_mode = ControlMode::CLOSED_LOOP_VELOCITY;
    DriveAlgorithm algorithm = DriveAlgorithm::SIX_STEP;
    bool is_running = false;

    P::Speed current_speed_rps{};  // 100 Hz, filtered and signed relative to drive_mode.
    P::Temperature temperature_c = number_v<P::Temperature, -273>;
#if LIBECU_FOC_DUTY_TELEMETRY
    P::Duty foc_modulation{};      // Slow telemetry derived from a coherent dq/bus sample.
#endif
};

/** Storage shared by the electrical kernel and its standalone numerical adapter. */
struct MotorElectricalOutput {
    using P = ActiveNumericProfile;
    P::Current i_u{}, i_v{}, i_w{};
    P::Voltage bus_voltage{};
    P::Current measured_current{}, i_d{}, i_q{};
    P::Voltage v_d{}, v_q{};
    P::Angle angle_e_rad{};
    P::Duty duty_cycle{}, duty_u{}, duty_v{}, duty_w{};
    int8_t current_pi_saturation = 0;
    uint8_t target_position = 0xff;
};

/** Live results of the current interrupt. A preempting break only gates off and cancels. */
struct MotorOutput {
    using P = ActiveNumericProfile;
    MotorElectricalOutput electrical{};
    uint8_t measured_position = 0xff;
#if !defined(LIBECU_RAW_CURRENT_TELEMETRY) || !LIBECU_RAW_CURRENT_TELEMETRY
    P::Current measured_current_filtered{};
#endif
    MotorPLL pll;
    HallMonitor hall{};
    MotorField field{};

    explicit MotorOutput(float max_steps_s = 0, bool inverse = false) noexcept : pll(max_steps_s, inverse) {}

    P::Current telemetryCurrent() const noexcept {
#if defined(LIBECU_RAW_CURRENT_TELEMETRY) && LIBECU_RAW_CURRENT_TELEMETRY
        return electrical.measured_current;
#else
        return measured_current_filtered;
#endif
    }
};

/** One address for capture tools and a read-only interface for native consumers.
 * Read individual fields under the reader's lock, or after PWM completion in its IRQ. */
struct MotorIo {
    MotorInput in{};
    MotorOutput out;

    explicit MotorIo(float max_steps_s = 0, bool inverse = false) noexcept : out(max_steps_s, inverse) {}

    ActiveNumericProfile::Duty reportedDuty() const noexcept {
        if (in.drive_mode == DriveMode::NEUTRAL || !in.is_running) return {};
        if (in.algorithm != DriveAlgorithm::FOC) return out.electrical.duty_cycle;
#if LIBECU_FOC_DUTY_TELEMETRY
        return in.foc_modulation;
#else
        return {};
#endif
    }
};
} // namespace libecu
#endif
