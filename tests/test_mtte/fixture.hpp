/** @file fixture.hpp @brief Explicit test hypotheses, not a calibration for a drive. */
#ifndef TEST_MTTE_FIXTURE_HPP
#define TEST_MTTE_FIXTURE_HPP

#include "motor_config.hpp"
#include "mtte_observer.hpp"
#include "pi_controller.hpp"

namespace fixture {

using libecu::DriveMode;
using libecu::MtteObserver;
static_assert(BLDC_NUM_POLES == 40 && BLDC_MAX_CURRENT == 45 && BLDC_MIN_CURRENT == -6,
              "The COMS3 comparison used the scooter motor and a 45 A drive");
// Keep the historical pair identical in every arm; it is not the motor's boot tuning.
constexpr libecu::PiParameters COMS3_PI{6, 24, BLDC_MAX_CURRENT, BLDC_MIN_CURRENT, BLDC_MAX_CURRENT, -4, 2};
constexpr float J = 0.00625f; // Inertia borrowed from another MOTOR_1 model, not a measured fixture value.
constexpr float KT = 1.5f * (BLDC_NUM_POLES / 2) * BLDC_KE_V_S_PER_RAD_E;
constexpr float R = 0.760f / MtteObserver::TWO_PI; // Nominal wheel circumference in metres, converted to radius.
constexpr float M = 80.0f; // Effective translating mass in this synthetic single-driven-wheel vehicle.
constexpr MtteObserver::Parameters PARAMS{
    J, KT, R, M, 0.95f, BLDC_PLL_KP, BLDC_PLL_KI,
    0.03f, 1.0f, 32, // common 30-ms LPF, rolling handoff, 320 ms coherent history
    0.08f, 1.0f, 0.5f // E17 ~ one small-signal observation delay; separately capped, optional bounded probe
};

inline MtteObserver::Frame sample(uint32_t k, float n, float iq, DriveMode mode = DriveMode::FORWARD) {
    return {mode, k, k, n, iq, true, true};
}

} // namespace fixture
#endif
