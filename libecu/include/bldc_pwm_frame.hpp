/** @file bldc_pwm_frame.hpp @brief One qualified acquisition, one rotor tick, one PWM result. */
#ifndef LIBECU_BLDC_PWM_FRAME_HPP
#define LIBECU_BLDC_PWM_FRAME_HPP

#include "bldc_controller.hpp"

namespace libecu {
template<class Adc, class Pwm>
LIBECU_HOT_LOOP bool BldcController::pwmInterruptHandler(Adc& adc, Pwm& pwm, uint8_t hall) noexcept {
    const auto& in = io_.in;
    auto& out = io_.out;
    auto& e = out.electrical;
    // A BRK just before entry must still win. The board parks a latched bridge
    // before its next qualified tick; parked acquisition can then resume tracking.
    if (pwm_cancelled_) {
        if (in.drive_mode != DriveMode::NEUTRAL) return false;
        pwm_cancelled_ = false;
    }
    trackRotor(hall);
    if (pwm_cancelled_) return false; // A hardware break won while Hall/PLL was running.

    adc.readNativeCurrents(e.i_u, e.i_v, e.i_w);
    e.bus_voltage = adc.readNativeBusVoltage();
    recordLoopStage(LoopStage::Currents);
    const bool open = in.control_mode == ControlMode::OPEN_LOOP;
    if ((!open && out.hall.isFaulted() && in.drive_mode != DriveMode::NEUTRAL) ||
        e.bus_voltage > limits_.max_voltage) {
        parkFromCarrier();
    }
    recordLoopStage(LoopStage::Dispatch);
    if (pwm_cancelled_) return false;
    const auto angle = open ? out.field.angle() : out.pll.angle_steps;
    if (in.drive_mode == DriveMode::NEUTRAL) {
        e.duty_u = e.duty_v = e.duty_w = e.duty_cycle = Duty{};
        e.measured_current = e.i_d = e.i_q = Current{};
        e.v_d = e.v_q = Voltage{};
        e.current_pi_saturation = 0;
        e.angle_e_rad = multiplyTo<Profile::Angle>(angle, FocAlgorithm::Core::RAD_PER_STEP);
    } else if (in.algorithm == DriveAlgorithm::FOC) {
        const auto speed = !open ? out.pll.speed_steps_s :
                           in.is_running ? in.field_rate.signedSpeed(in.drive_mode) : Profile::PllSpeed{};
        foc_.updateCarrier(in, e, angle, speed, pwm);
    } else {
        six_step_.updateCarrier(in, e, angle);
    }
    recordLoopStage(LoopStage::Algorithm);
#if !defined(LIBECU_RAW_CURRENT_TELEMETRY) || !LIBECU_RAW_CURRENT_TELEMETRY
    if (pwm_cancelled_) return false;
    const auto alpha = limits_.current_alpha;
    out.measured_current_filtered = alpha > Fraction{} && alpha < number_v<Fraction, 1>
        ? CurrentFeedbackFilter<Profile>::advance(out.measured_current_filtered, e.measured_current, alpha)
        : e.measured_current;
#endif
    recordLoopStage(LoopStage::Status);
    return !pwm_cancelled_; // Includes a break during preload writes, filtering or the last timing marker.
}
} // namespace libecu
#endif
