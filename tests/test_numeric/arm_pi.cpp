/** @file arm_pi.cpp
 * @brief Compile-only fixed PI probe for a Cortex-M0 target without an FPU
 */
#include "../../libecu/include/pi_controller.hpp"

using Types = libecu::FixedProfile::CurrentPiTypes;
static libecu::PiController<Types, 20000> controller;

extern "C" int32_t convert_input(float input) {
    return Types::Input::fromFloat(input).raw();
}

extern "C" void configure_pi(int32_t kp_raw, int32_t ki_raw, int32_t kb_raw) {
    libecu::PiParametersT<Types> params{};
    params.kp = Types::Gain::fromRaw(kp_raw);
    params.ki = Types::Gain::fromRaw(ki_raw);
    params.kb = Types::Gain::fromRaw(kb_raw);
    params.max_output = params.integral_max = Types::Output::fromRatio(95, 100);
    params.min_output = params.integral_min = Types::Output::fromRatio(-95, 100);
    controller.setParameters(params);
}

extern "C" int32_t update_pi(int32_t target_raw, int32_t measured_raw, int saturation) {
    return controller.update(Types::Input::fromRaw(target_raw),
                             Types::Input::fromRaw(measured_raw), saturation).raw();
}
