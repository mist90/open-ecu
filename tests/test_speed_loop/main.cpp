/**
 * @file main.cpp
 * @brief Torque-driven MOTOR_1 mechanical stability regression using the production PI and Hall PLL.
 *
 * Only the plant is simulated: current follows a bounded first-order response,
 * J*d(2*pi*speed)/dt = Kt*Iq - bearing drag - rolling resistance. Its integrated
 * angle generates quantized Hall sectors, which feed the real PLL. Nothing
 * prescribes the measured speed. See README.md for assumptions and acceptance
 * limits. The ramp and feedback LPF mirror BldcController's native arithmetic;
 * neither regulator is reimplemented here.
 */
#include "../../libecu/include/motor_config.hpp"
#include "../../libecu/include/motor_pll_core.hpp"
#include "../../libecu/include/pi_float_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <type_traits>

#ifndef MOTOR_1
#error "Build this regression with -DMOTOR_1, using the shared motor configuration"
#endif

namespace {

using namespace libecu;

constexpr float TWO_PI = 6.2831853071795864769f;
constexpr uint32_t SPEED_HZ = 100;
constexpr uint32_t STEPS_PER_REV = BLDC_NUM_POLES * 3;
constexpr float RADIUS_M = 0.127f;
constexpr float WHEEL_J = 0.00625f;
constexpr float NOMINAL_KT = 1.5f * (BLDC_NUM_POLES / 2) * BLDC_KE_V_S_PER_RAD_E;
constexpr float DRAG_COULOMB_A = 0.656f;
constexpr float DRAG_VISCOUS_A_PER_RPS = 0.0484f;
constexpr float GRAVITY = 9.80665f;
constexpr float INITIAL_TARGET = 6.0f;
constexpr float LOWER_TARGET = 3.0f;
// 0.2 RPS is ~33 ms of travel at the 6 RPS/s ramp, comparable to PLL + LPF
// lag. Near zero, a small percentage-of-step bound would mislabel a finite,
// decaying excursion as instability. This is an explicit absolute budget.
// Actual stability is gated independently by the settled-error/oscillation limits.
constexpr float LOW_SPEED_TRANSIENT_LIMIT = 0.20f;
constexpr unsigned PHASE_SECONDS = 120;
constexpr unsigned PHASE_TICKS = PHASE_SECONDS * SPEED_HZ;
constexpr unsigned TAIL_TICKS = 20 * SPEED_HZ;

static_assert(BLDC_NUM_POLES == 40, "The identified mechanical model is for the 40-pole F410");

struct PlantCase {
    const char* name;
    float wheel_j_scale;
    float mass_kg;
    float kt_scale;
    float drag_scale;
    float current_bandwidth_hz;
    float initial_target = INITIAL_TARGET;
    float following_target = LOWER_TARGET;
};

// Deterministic uncertainty corners, not a probability/confidence interval.
// Fast free-wheel corners stress delay margin; the slow road corner stresses damping.
constexpr PlantCase PLANTS[] = {
    {"free-nominal", 1.0f, 0.0f, 1.0f, 1.0f, 250.0f},
    {"free-light-slow-i", 0.75f, 0.0f, 1.2f, 0.75f, 125.0f},
    {"free-light-fast-i", 0.75f, 0.0f, 1.2f, 0.75f, 500.0f},
    {"free-heavy", 1.5f, 0.0f, 0.8f, 1.25f, 125.0f},
    {"free-low-speed", 1.0f, 0.0f, 1.0f, 1.0f, 250.0f, 0.3f, 1.0f},
    {"road-60kg", 1.0f, 60.0f, 1.2f, 0.75f, 500.0f},
    {"road-80kg", 1.0f, 80.0f, 1.0f, 1.0f, 250.0f},
    {"road-100kg-low-kt", 1.5f, 100.0f, 0.8f, 1.25f, 125.0f},
    {"road-100kg-high-kt", 0.75f, 100.0f, 1.2f, 0.75f, 500.0f},
};

// Mechanical integration is independent of the firmware's numeric profile.
// Compensated sums avoid a float plant artificially sticking when dv is less
// than one speed/angle LSB during a long, high-rate settling interval.
struct CompensatedSum {
    float value = 0.0f;
    float residual = 0.0f;

    void add(float increment) {
        const float corrected = increment - residual;
        const float next = value + corrected;
        residual = (next - value) - corrected;
        value = next;
    }
    void clear() { value = 0.0f; residual = 0.0f; }
};

class Mechanics {
public:
    Mechanics(const PlantCase& plant, unsigned rate_hz)
        : plant_(plant)
        , dt_(1.0f / rate_hz)
        , current_alpha_(-std::expm1(-TWO_PI * plant.current_bandwidth_hz * dt_))
        , average_current_factor_(current_alpha_ / (TWO_PI * plant.current_bandwidth_hz * dt_))
        , inertia_(WHEEL_J * plant.wheel_j_scale + plant.mass_kg * RADIUS_M * RADIUS_M) {
        phase_.value = 0.13f; // Start between Hall edges; sector centres are integers.
    }

    void tick(float request_a, bool second_phase) {
        const float bounded = std::clamp(request_a, BLDC_MIN_CURRENT, BLDC_MAX_CURRENT);
        // Exact first-order current endpoint and interval average (zero-order-held request).
        const float average_current = bounded + (current_a_ - bounded) * average_current_factor_;
        current_a_ += current_alpha_ * (bounded - current_a_);

        const float old_speed = speed_.value;
        const float c_rr = second_phase ? 0.020f : 0.008f;
        const float coulomb = NOMINAL_KT * DRAG_COULOMB_A * plant_.drag_scale +
                              c_rr * plant_.mass_kg * GRAVITY * RADIUS_M;
        const float viscous = NOMINAL_KT * DRAG_VISCOUS_A_PER_RPS * plant_.drag_scale;
        const float motor_torque = NOMINAL_KT * plant_.kt_scale * average_current;
        float net_torque = motor_torque - viscous * old_speed;
        if (old_speed > 0.0f) net_torque -= coulomb;
        else if (old_speed < 0.0f) net_torque += coulomb;
        else net_torque = std::copysign(std::max(0.0f, std::abs(net_torque) - coulomb), net_torque);

        // Midpoint viscous correction, then integrate actual mechanical acceleration.
        const float dv = dt_ * net_torque / (TWO_PI * inertia_ + 0.5f * dt_ * viscous);
        speed_.add(dv);
        if (old_speed * speed_.value < 0.0f) speed_.clear(); // Coulomb zero-crossing/stiction.
        phase_.add(0.5f * (old_speed + speed_.value) * STEPS_PER_REV * dt_);
        if (phase_.value >= 6.0f) phase_.value -= 6.0f;
        else if (phase_.value < 0.0f) phase_.value += 6.0f;
    }

    float speed() const { return speed_.value; }
    float current() const { return current_a_; }
    uint8_t hall() const {
        return static_cast<uint8_t>(static_cast<unsigned>(phase_.value + 0.5f) % 6);
    }

private:
    const PlantCase& plant_;
    float dt_;
    float current_alpha_;
    float average_current_factor_;
    float inertia_;
    float current_a_ = 0.0f;
    CompensatedSum speed_{};
    CompensatedSum phase_{};
};

struct Window {
    float minimum = 1e6f;
    float maximum = -1e6f;
    CompensatedSum sum{};
    unsigned count = 0;

    void add(float value) {
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
        sum.add(value);
        ++count;
    }
    float mean() const { return sum.value / count; }
    float peakToPeak() const { return maximum - minimum; }
};

struct PhaseResult {
    Window speed;
    Window tail_speed;
    Window tail_feedback;
    Window tail_current;
    Window first_12s_speed;
    float speed_at_12s = 0.0f;
    float feedback_error = 0.0f;
};

struct Result {
    PhaseResult phase[2];
    float max_current = 0.0f;
    float min_current = 0.0f;
    float max_integral = 0.0f;
    float min_integral = 0.0f;
    bool valid = true;
};

template<class Profile, unsigned CurrentHz>
Result simulate(const PlantCase& plant, float kp, float ki) {
    using Speed = typename Profile::Speed;
    using Fraction = typename Profile::Fraction;
    using Accumulator = typename Profile::Accumulator;
    using Pll = MotorPLLCore<Profile, CurrentHz>;
    static_assert(CurrentHz % SPEED_HZ == 0, "The speed tick must divide the current clock");

    Result result{};
    PiFloatInterface<typename Profile::SpeedPiTypes, SPEED_HZ> pi;
    PiParameters params{};
    params.kp = kp;
    params.ki = ki;
    params.min_output = BLDC_MIN_CURRENT;
    params.max_output = BLDC_MAX_CURRENT;
    // Shared main.cpp installs these limits and kb on all architectures.
    params.integral_min = -4.0f;
    params.integral_max = 12.0f;
    params.kb = 2.0f;
    if (!pi.setParameters(params)) {
        result.valid = false;
        return result;
    }

    Pll pll(constant<typename Pll::Speed>(BLDC_MAX_SPEED * STEPS_PER_REV));
    pll.setGains(constant<typename Pll::Gain>(BLDC_PLL_KP), constant<typename Pll::Gain>(BLDC_PLL_KI));
    pll.setUsePLL(true);
    Mechanics mechanics(plant, CurrentHz);
    pll.updateHall(mechanics.hall());
    pll.reset();
    Speed filtered_speed{};
    Accumulator limited_target{};
    const auto ramp_step = divideByInteger<Accumulator>(constant<Speed>(BLDC_MAX_ACCELERATION), SPEED_HZ);

    for (unsigned tick = 0; tick < 2 * PHASE_TICKS; ++tick) {
        const unsigned phase = tick / PHASE_TICKS;
        const float target = phase != 0 && plant.mass_kg == 0.0f ? plant.following_target : plant.initial_target;
        const Speed speed = divideByInteger<Speed>(pll.getSpeedStepsSec(), STEPS_PER_REV);
        filtered_speed += multiplyTo<Speed>(number_v<Fraction, 1, 2>, speed - filtered_speed);
        // target LPF is disabled (alpha=0), exactly as configured by shared main.cpp.
        const Accumulator difference = numberCast<Accumulator>(target) - limited_target;
        limited_target += numericClamp(difference, -ramp_step, ramp_step);
        const float request = numberCast<float>(pi.updateNative(numberCast<Speed>(limited_target), filtered_speed));

        result.max_current = std::max(result.max_current, request);
        result.min_current = std::min(result.min_current, request);
        result.max_integral = std::max(result.max_integral, pi.getIntegral());
        result.min_integral = std::min(result.min_integral, pi.getIntegral());
        for (unsigned sample = 0; sample < CurrentHz / SPEED_HZ; ++sample) {
            mechanics.tick(request, phase != 0);
            pll.updateHall(mechanics.hall());
            pll.updateTick();
        }

        const float physical_speed = mechanics.speed();
        if (!std::isfinite(physical_speed) || !std::isfinite(request) ||
            std::abs(physical_speed) > 20.0f || physical_speed < -0.1f) {
            result.valid = false;
            return result;
        }
        PhaseResult& summary = result.phase[phase];
        summary.speed.add(physical_speed);
        if (tick % PHASE_TICKS < 12 * SPEED_HZ) summary.first_12s_speed.add(physical_speed);
        if (tick % PHASE_TICKS == 12 * SPEED_HZ - 1) summary.speed_at_12s = physical_speed;
        if (tick % PHASE_TICKS >= PHASE_TICKS - TAIL_TICKS) {
            summary.tail_speed.add(physical_speed);
            summary.tail_feedback.add(numberCast<float>(filtered_speed));
            summary.tail_current.add(mechanics.current());
            summary.feedback_error = std::max(summary.feedback_error,
                                              std::abs(physical_speed - numberCast<float>(filtered_speed)));
        }
    }
    return result;
}

struct Summary {
    unsigned cases = 0;
    unsigned failed_cases = 0;
    unsigned failures = 0;
};

void require(bool condition, const char* message, Summary& summary) {
    if (!condition) {
        std::printf("    FAIL: %s\n", message);
        ++summary.failures;
    }
}

template<class Profile, unsigned CurrentHz>
void runCases(float kp, float ki, Summary& summary) {
    constexpr bool fixed = std::is_same<Profile, FixedProfile>::value;
    for (const PlantCase& plant : PLANTS) {
        ++summary.cases;
        const unsigned previous_failures = summary.failures;
        const Result result = simulate<Profile, CurrentHz>(plant, kp, ki);
        std::printf("%-11s %5u Hz %-21s", fixed ? "full-fixed" : "float", CurrentHz, plant.name);
        if (!result.valid) {
            std::printf(" invalid/divergent\n");
            require(false, "finite, representable, nondivergent closed-loop response", summary);
            ++summary.failed_cases;
            continue;
        }

        const bool road = plant.mass_kg > 0.0f;
        const bool low_speed = plant.initial_target < 1.0f;
        const bool step_down = plant.following_target < plant.initial_target;
        const float final_target = road ? plant.initial_target : plant.following_target;
        const float overshoot = std::max(0.0f, result.phase[0].speed.maximum - plant.initial_target);
        const float excursion = road || step_down
                              ? std::max(0.0f, final_target - result.phase[1].speed.minimum)
                              : std::max(0.0f, result.phase[1].speed.maximum - final_target);
        const float rebound = std::max(0.0f, result.phase[1].speed.maximum - plant.initial_target);
        std::printf(" rise+ %.3f, %s %.3f, tail %.4f/%.4f RPS, pp %.4f/%.4f",
                    overshoot, road ? "load-" : step_down ? "down-" : "up+", excursion,
                    result.phase[0].tail_speed.mean() - plant.initial_target,
                    result.phase[1].tail_speed.mean() - final_target,
                    result.phase[0].tail_speed.peakToPeak(), result.phase[1].tail_speed.peakToPeak());
        if (road) std::printf(", rebound %.3f", rebound);
        std::printf("\n");
        if (low_speed) {
            std::printf("    low-speed startup: first 12s mean %.3f RPS, at 12s %.3f RPS (target %.2f)\n",
                        result.phase[0].first_12s_speed.mean(), result.phase[0].speed_at_12s, plant.initial_target);
        }

        if (low_speed) {
            require(overshoot <= LOW_SPEED_TRANSIENT_LIMIT && excursion <= LOW_SPEED_TRANSIENT_LIMIT,
                    "finite low-speed transient <=0.20 RPS; decay/convergence checked independently", summary);
        } else {
            // 80 kg is the requested nominal road load; 60/100 kg are sensitivity
            // corners with a 15% transient budget, but identical ringing/tail gates.
            const bool uncertain_road_load = road && plant.mass_kg != 80.0f;
            const float startup_fraction = uncertain_road_load ? 0.15f : 0.10f;
            require(overshoot <= startup_fraction * plant.initial_target,
                    uncertain_road_load ? "uncertain-road startup overshoot <=15% of target"
                                        : "nominal/free startup overshoot <=10% of target", summary);
        }
        if (road) {
            require(excursion <= 0.20f * plant.initial_target, "rolling-load speed dip <=20% (1.20 RPS)", summary);
            require(rebound <= 0.25f * excursion + 0.02f,
                    "load recovery rebound <=25% of dip + 0.02 RPS (damping)", summary);
        } else if (!low_speed) {
            require(excursion <= 0.10f * std::abs(plant.initial_target - final_target),
                    "command-step over/undershoot <=10% of step", summary);
        }
        for (unsigned phase = 0; phase < 2; ++phase) {
            const PhaseResult& response = result.phase[phase];
            const float target = phase == 0 ? plant.initial_target : final_target;
            require(std::abs(response.tail_speed.mean() - target) <= 0.05f,
                    "tail mean mechanical speed error <=0.05 RPS", summary);
            require(std::max(std::abs(response.tail_speed.minimum - target),
                             std::abs(response.tail_speed.maximum - target)) <= 0.08f,
                    "tail worst mechanical speed error <=0.08 RPS", summary);
            require(response.tail_speed.peakToPeak() <= 0.08f,
                    "sustained mechanical oscillation <=0.08 RPS peak-to-peak", summary);
            require(std::abs(response.tail_feedback.mean() - target) <= 0.05f && response.feedback_error <= 0.08f,
                    "PLL/LPF feedback converges with the actual mechanical speed", summary);
            require(response.tail_current.peakToPeak() <= 1.0f,
                    "sustained torque-current oscillation <=1.0 A peak-to-peak", summary);
        }
        require(result.min_current >= BLDC_MIN_CURRENT && result.max_current <= BLDC_MAX_CURRENT,
                "current request stays inside configured limits", summary);
        require(result.min_integral >= -4.0f && result.max_integral <= 12.0f,
                "PI integral stays inside shared main.cpp limits", summary);
        if (summary.failures != previous_failures) ++summary.failed_cases;
    }
}

bool parseGain(const char* text, float& gain) {
    char* end = nullptr;
    gain = std::strtof(text, &end);
    return end != text && *end == '\0' && std::isfinite(gain) && gain > 0.0f;
}

} // namespace

int main(int argc, char** argv) {
    float kp = BLDC_SPEED_CURRENT_KP;
    float ki = BLDC_SPEED_CURRENT_KI;
    if (argc != 1 && (argc != 3 || !parseGain(argv[1], kp) || !parseGain(argv[2], ki))) {
        std::fprintf(stderr, "Usage: %s [kp_A_per_RPS ki_A_per_RPS_s] (both finite and positive)\n", argv[0]);
        return 2;
    }
    std::printf("MOTOR_1 speed-current PI: kp=%.6g A/RPS, ki=%.6g A/(RPS*s) [%s]\n",
                kp, ki, argc == 1 ? "shared motor_config.hpp" : "exploratory CLI override");
    std::printf("Plant estimates: Kt=%.5f Nm/A, Jwheel=%.5f, J80kg=%.5f+Jwheel kg*m^2; r=%.3f m.\n",
                NOMINAL_KT, WHEEL_J, 80.0f * RADIUS_M * RADIUS_M, RADIUS_M);
    std::printf("Drag=(0.656+0.0484*|RPS|) A equivalent; road Crr 0.008 -> 0.020 at %u s.\n", PHASE_SECONDS);
    std::printf("100 Hz PI, feedback LPF=0.5, target LPF=0, ramp=%.2f RPS/s; real Hall PLL %.0f/%.0f.\n",
                BLDC_MAX_ACCELERATION, BLDC_PLL_KP, BLDC_PLL_KI);
    std::printf("Bounded 125..500 Hz torque response; no voltage/commutation/tyre-slip model.\n");
    std::printf("Current limits [%.1f,%.1f] A; integral [-4,12] A, kb=2, trapezoidal PI.\n",
                BLDC_MIN_CURRENT, BLDC_MAX_CURRENT);
    std::printf("Free: 0->6->3 or 0->0.3->1 RPS; road: 0->6 then load step. Phase %u s, tail last 20 s.\n",
                PHASE_SECONDS);
    for (const PlantCase& plant : PLANTS) {
        const float inertia = WHEEL_J * plant.wheel_j_scale + plant.mass_kg * RADIUS_M * RADIUS_M;
        const float a = NOMINAL_KT * plant.kt_scale / (TWO_PI * inertia);
        std::printf("  %-21s J=%.6f kg*m^2, a=%.4f RPS/s/A, Kt x%.2f, drag x%.2f, I BW %.0f Hz\n",
                    plant.name, inertia, a, plant.kt_scale, plant.drag_scale, plant.current_bandwidth_hz);
    }
    Summary summary{};
    runCases<FloatProfile, 8000>(kp, ki, summary);
    runCases<FixedProfile, 8000>(kp, ki, summary);
    runCases<FloatProfile, 16000>(kp, ki, summary);
    runCases<FixedProfile, 16000>(kp, ki, summary);
    runCases<FloatProfile, 20000>(kp, ki, summary);
    runCases<FixedProfile, 20000>(kp, ki, summary);
    std::printf("%u mechanical closed-loop cases, %u failed cases (%u failed acceptance checks)\n",
                summary.cases, summary.failed_cases, summary.failures);
    return summary.failures == 0 ? 0 : 1;
}
