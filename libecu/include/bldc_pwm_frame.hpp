/** @file bldc_pwm_frame.hpp @brief Complete one prepared control frame with statically bound or erased I/O. */
#ifndef LIBECU_BLDC_PWM_FRAME_HPP
#define LIBECU_BLDC_PWM_FRAME_HPP

#include "bldc_controller.hpp"

namespace libecu {
template<class Adc, class Pwm>
LIBECU_HOT_LOOP void BldcController::completePwmFrom(Adc& adc, Pwm& pwm) noexcept {
    if (!pwm_prepared_) return;
    pwm_prepared_ = false;
    auto& frame = prepared_pwm_frame_;
    auto& in = frame.input;
    adc.readNativeCurrents(in.i_u, in.i_v, in.i_w);
    in.bus_voltage = adc.readNativeBusVoltage();
    recordLoopStage(LoopStage::Currents);

    // Both live checks stay ahead of electrical control and all compare writes.
    if (prepared_hall_controls_drive_ && hall_monitor_.isFaulted() && dmode_ != DriveMode::NEUTRAL)
        setDriveMode(DriveMode::NEUTRAL);
    if (in.bus_voltage > limits_.max_voltage) setDriveMode(DriveMode::NEUTRAL);
    recordLoopStage(LoopStage::Dispatch);
    if (dmode_ == DriveMode::NEUTRAL || in.drive_mode == DriveMode::NEUTRAL) {
        frame.output = {};
        frame.output.angle_e_rad = multiplyTo<Profile::Angle>(in.angle_steps, FocAlgorithm::Core::RAD_PER_STEP);
    } else if (algorithm_ == DriveAlgorithm::FOC) {
        foc_.updateNative(frame, pwm);
    } else {
        six_step_.updateNative(frame);
    }
    recordLoopStage(LoopStage::Algorithm);
    publishPwmStatus();
}

template<class Adc, class Pwm>
LIBECU_HOT_LOOP void BldcController::completeOwnedCarrier(Adc& adc, Pwm& pwm) noexcept {
    if (!pwm_prepared_) return;
    pwm_prepared_ = false;
    auto& in = prepared_pwm_frame_.input;
    adc.readNativeCurrents(in.i_u, in.i_v, in.i_w);
    in.bus_voltage = adc.readNativeBusVoltage();
    recordLoopStage(LoopStage::Currents);
    if ((prepared_hall_controls_drive_ && hall_monitor_.isFaulted() && dmode_ != DriveMode::NEUTRAL)
        || in.bus_voltage > limits_.max_voltage) parkFromCarrier();
    recordLoopStage(LoopStage::Dispatch);

    if (dmode_ == DriveMode::NEUTRAL || in.drive_mode == DriveMode::NEUTRAL) {
        status_.duty_u = status_.duty_v = status_.duty_w = Duty{};
        status_.measured_current = status_.i_d = status_.i_q = Current{};
        status_.v_d = status_.v_q = Voltage{};
        status_.current_pi_saturation = 0;
        status_.angle_e_rad = multiplyTo<Profile::Angle>(in.angle_steps, FocAlgorithm::Core::RAD_PER_STEP);
        if (algorithm_ != DriveAlgorithm::FOC || !LIBECU_FOC_DUTY_TELEMETRY) status_.duty_cycle = Duty{};
    } else if (algorithm_ == DriveAlgorithm::FOC) {
        // Commands are read from status_, dq/duties are written straight back.
        // No temporary output or second publication. Only this IRQ produces them.
        foc_.updateOwnedCarrier(in, status_, pwm);
#if !LIBECU_FOC_DUTY_TELEMETRY
        status_.duty_cycle = Duty{};
#endif
    } else {
        six_step_.updateOwnedCarrier(in, status_);
    }
    recordLoopStage(LoopStage::Algorithm);
    finishOwnedStatus();
}
} // namespace libecu
#endif
