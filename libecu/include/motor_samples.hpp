/** @file motor_samples.hpp
 * @brief Typed electrical-loop measurements and results
 */
#ifndef LIBECU_MOTOR_SAMPLES_HPP
#define LIBECU_MOTOR_SAMPLES_HPP
#include "motor_types.hpp"
#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"
namespace libecu {
template<class P>
struct MotorAlgorithmInputT {
    typename P::Current i_u{}, i_v{}, i_w{};
    typename P::Voltage bus_voltage{};
    typename P::Angle angle_steps{};
    typename P::PllSpeed speed_steps_s{};
    typename P::Current target_current{};
    typename P::Duty duty_command{};
    uint8_t step = 0;
    DriveMode drive_mode = DriveMode::NEUTRAL;
    ElectricMode electric_mode = ElectricMode::VOLTAGE_MODE;
};
template<class P>
struct MotorAlgorithmOutputT {
    typename P::Duty duty_u{}, duty_v{}, duty_w{}, duty{};
    typename P::Current measured_current{}, i_d{}, i_q{};
    typename P::Voltage v_d{}, v_q{};
    typename P::Angle angle_e_rad{};
    int8_t saturation = 0;
};

/** @brief Explicit adaptation between a native sample and a compatibility interface. */
template<class Out, class In>
Out motorInputCast(const In& input) noexcept {
    Out out{};
    out.i_u = numberCast<decltype(out.i_u)>(input.i_u);
    out.i_v = numberCast<decltype(out.i_v)>(input.i_v);
    out.i_w = numberCast<decltype(out.i_w)>(input.i_w);
    out.bus_voltage = numberCast<decltype(out.bus_voltage)>(input.bus_voltage);
    out.angle_steps = numberCast<decltype(out.angle_steps)>(input.angle_steps);
    out.speed_steps_s = numberCast<decltype(out.speed_steps_s)>(input.speed_steps_s);
    out.target_current = numberCast<decltype(out.target_current)>(input.target_current);
    out.duty_command = numberCast<decltype(out.duty_command)>(input.duty_command);
    out.step = input.step;
    out.drive_mode = input.drive_mode;
    out.electric_mode = input.electric_mode;
    return out;
}

template<class Out, class In>
Out motorOutputCast(const In& input) noexcept {
    Out out{};
    out.duty_u = numberCast<decltype(out.duty_u)>(input.duty_u);
    out.duty_v = numberCast<decltype(out.duty_v)>(input.duty_v);
    out.duty_w = numberCast<decltype(out.duty_w)>(input.duty_w);
    out.duty = numberCast<decltype(out.duty)>(input.duty);
    out.measured_current = numberCast<decltype(out.measured_current)>(input.measured_current);
    out.i_d = numberCast<decltype(out.i_d)>(input.i_d);
    out.i_q = numberCast<decltype(out.i_q)>(input.i_q);
    out.v_d = numberCast<decltype(out.v_d)>(input.v_d);
    out.v_q = numberCast<decltype(out.v_q)>(input.v_q);
    out.angle_e_rad = numberCast<decltype(out.angle_e_rad)>(input.angle_e_rad);
    out.saturation = input.saturation;
    return out;
}
} // namespace libecu
#endif
