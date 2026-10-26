/** @file main.cpp
 * @brief Verify both retained shunts' ADC apertures over the F410 modulation range
 */
#include "board_config.hpp"
#include "ecu_config.hpp"
#include "foc_control_core.hpp"
#include <algorithm>
#include <cstdio>
#include <initializer_list>

using namespace libecu;
static int failures = 0;

template<class P>
static void checkSampling() {
    using Core = FocControlCore<P, ActiveTiming::CURRENT_HZ>;
    constexpr uint32_t period = ihm08::TIMER_CLOCK_HZ / (2 * ActiveTiming::CURRENT_HZ);
    constexpr int adc_clock_ticks = 4; // APB2/4 = 25 MHz; timer = 100 MHz
    constexpr int sample_ticks = 15 * adc_clock_ticks;
    constexpr int conversion_ticks = 12 * adc_clock_ticks;
    constexpr int sync_ticks = 3 * adc_clock_ticks + 1; // DS11144: 3 ADC clocks + 1 APB2 clock
    constexpr int settling_ticks = (ihm08::DEAD_TIME_NS + ihm08::CURRENT_SETTLING_NS) /
                                  (1000000000 / ihm08::TIMER_CLOCK_HZ);
    int minimum_margin = period;
    unsigned rejected_apertures = 0;
    Core core;
    typename Core::Params params;
    params.max_duty_cycle = numberCast<typename P::Duty>(ihm08::MAX_DUTY);
    params.low_side_clamped_pwm = ihm08::LOW_SIDE_CLAMPED_PWM;
    params.angle_offset_rad = typename P::Angle{};
    typename Core::Input input;
    input.electric_mode = ElectricMode::VOLTAGE_MODE;
    input.drive_mode = DriveMode::FORWARD;
    input.duty_command = number<typename P::Duty>(1);
    input.bus_voltage = number<typename P::Voltage>(31);

    for (float modulation : {0.0f, 0.2f, 0.6f, 0.8f, 0.9f, ihm08::MAX_MODULATION}) {
        params.max_modulation = numberCast<typename P::Duty>(modulation);
        core.setParams(params);
        core.reset();
        for (int step = 0; step < 3600; ++step) {
            input.angle_steps = number<typename P::Angle>(step, 600);
            const auto out = core.update(input);
            const typename P::Duty duties[] = {out.duty_u, out.duty_v, out.duty_w};
            // The same tie rule as current reconstruction, before timer quantization.
            const int discard = duties[0] >= duties[1] && duties[0] >= duties[2] ? 0 :
                                duties[1] >= duties[2] ? 1 : 2;
            for (int phase = 0; phase < 3; ++phase) {
                const int low_half = period - dutyToCompare(duties[phase], period);
                const int start = -int(ihm08::ADC_LEAD_TICKS) + phase * (sample_ticks + conversion_ticks);
                const int end = start + sample_ticks + sync_ticks;
                const int margin = std::min(start - (-low_half + settling_ticks), low_half - end);
                if (phase == discard) {
                    rejected_apertures += margin < 0;
                } else {
                    minimum_margin = std::min(minimum_margin, margin);
                    if (margin < 0) ++failures;
                }
            }
        }
    }
    // Verify this really exercises two-shunt reconstruction, not merely an all-low window.
    if (minimum_margin < 0 || rejected_apertures == 0) ++failures;
    // This checks each retained shunt's conduction and settling, not analog
    // cross-phase coupling; the latter still requires captures on the real bridge.
    std::printf("F410 sampling: retained-shunt margin %d ticks, %u rejected apertures\n",
                minimum_margin, rejected_apertures);
}

int main() {
    checkSampling<FloatProfile>();
    checkSampling<FixedProfile>();
    std::printf("%d failures\n", failures);
    return failures != 0;
}
