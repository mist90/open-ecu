/**
 * @file main.cpp
 * @brief Host-side harness for the BEMF zero-crossing observer.
 *
 * Models a trapezoidal BLDC turning at a fixed electrical speed and samples the
 * floating phase once per PWM cycle exactly the way the STM32 injected ADC
 * does.  The observer's synthetic event is what advances the bridge, as it does
 * in the firmware (through MotorPLL there, directly here).  The rotor is
 * kinematic - it turns at the commanded speed regardless of what the controller
 * does - so the metric is purely "how accurately does the observer place the
 * commutation relative to the true rotor angle".
 *
 * The two commutation-timing strategies are compared on identical noise
 * realisations: DELAY_30DEG (speed-derived countdown) and FLUX_INTEGRATE
 * (VESC-style volt-second integration).
 *
 * Not modelled: phase current and the R*I / L*di/dt terms, switching ringing
 * (noise here is white), current-dependent demagnetisation duration, coupling
 * from the chopping phases, divider mismatch, inter-phase sample skew, rotor
 * acceleration, and the PLL.  See docs/bemf_hybrid_algorithm.md section 11.
 *
 * Build/run: ./run.sh
 */

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>

#include "../../libecu/include/algorithms/bemf_observer.hpp"

using libecu::BemfAmplitude;
using libecu::BemfObserver;
using libecu::BemfObserverInput;
using libecu::BemfObserverParams;
using libecu::BemfTimingMode;

// ---------------------------------------------------------------------------
// Motor model
// ---------------------------------------------------------------------------

/// Ideal 120-degree trapezoid, +1 on [0,60), ramping down on [60,120), etc.
static float trap(float deg) {
    deg = std::fmod(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    if (deg <  60.0f) return  1.0f;
    if (deg < 120.0f) return  1.0f - (deg -  60.0f) / 30.0f;
    if (deg < 240.0f) return -1.0f;
    if (deg < 300.0f) return -1.0f + (deg - 240.0f) / 30.0f;
    return 1.0f;
}

enum class PhaseDrive { UP, DOWN, OFF };

struct CommStep { PhaseDrive u, v, w; };

static const CommStep kTable[6] = {
    {PhaseDrive::UP,   PhaseDrive::DOWN, PhaseDrive::OFF },  // 0 -> W floats
    {PhaseDrive::UP,   PhaseDrive::OFF,  PhaseDrive::DOWN},  // 1 -> V floats
    {PhaseDrive::OFF,  PhaseDrive::UP,   PhaseDrive::DOWN},  // 2 -> U floats
    {PhaseDrive::DOWN, PhaseDrive::UP,   PhaseDrive::OFF },  // 3 -> W floats
    {PhaseDrive::DOWN, PhaseDrive::OFF,  PhaseDrive::UP  },  // 4 -> V floats
    {PhaseDrive::OFF,  PhaseDrive::DOWN, PhaseDrive::UP  },  // 5 -> U floats
};

struct SimConfig {
    float pwm_freq       = 20000.0f;
    float steps_per_sec  = 1200.0f;   ///< electrical speed
    float vbus           = 24.0f;
    float bemf_peak      = 6.0f;      ///< |e| plateau in Volts at this speed
    float noise_sigma    = 0.0f;      ///< Gaussian noise on every phase sample (V)
    float spike_prob     = 0.0f;      ///< probability per sample of an EMI spike
    float spike_volts    = 4.0f;      ///< spike amplitude (V)
    float demag_deg      = 8.0f;      ///< electrical degrees of inductive discharge
    float seconds        = 1.0f;
    uint32_t seed        = 1;
    float deadband_v     = 0.0f;      ///< 0 = auto (1% of Vbus)
    uint8_t confirm      = 2;
};

struct Result {
    int    commutations   = 0;
    int    events         = 0;
    double mean_err_deg   = 0.0;
    double rms_err_deg    = 0.0;
    double max_abs_deg    = 0.0;
    int    lost_steps     = 0;   ///< steps that produced no synthetic event
    double amp_peak       = 0.0; ///< estimated |e| plateau (V)
    double amp_ke         = 0.0; ///< estimated BEMF constant (V*s/rad electrical)
};

static Result runSim(const SimConfig& cfg, BemfTimingMode mode) {
    const float dt = 1.0f / cfg.pwm_freq;
    const uint32_t ticks = static_cast<uint32_t>(cfg.seconds / dt);
    const float deg_per_tick = cfg.steps_per_sec * 60.0f * dt;

    std::mt19937 rng(cfg.seed);
    std::normal_distribution<float> gauss(0.0f, cfg.noise_sigma);
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);

    BemfObserver obs(cfg.pwm_freq);
    BemfObserverParams p = obs.getParameters();
    p.min_duty              = 0.05f;
    p.transition_speed_low  = 100.0f;
    p.transition_speed_high = 200.0f;
    p.blanking_cycles       = 2.0f;
    p.blanking_fraction     = 0.20f;
    p.zc_deadband_volts     = cfg.deadband_v;
    p.zc_confirm_samples    = cfg.confirm;
    p.auto_polarity         = false;
    p.auto_learn_limit      = false;  // no Hall edges here to learn from
    p.timing_mode           = mode;
    if (mode == BemfTimingMode::FLUX_INTEGRATE) {
        // Flux over the 30-degree arc = integral of a ramp reaching bemf_peak
        // at 30 degrees past the ZC, i.e. 0.5 * peak * (half step period).
        p.integrator_limit_vs = 0.5f * cfg.bemf_peak * (0.5f / cfg.steps_per_sec);
    }
    obs.setParameters(p);

    float   theta = 0.0f;                            // true electrical angle, deg
    uint8_t applied_step = 1;                        // rotor in sector 0 -> step 1
    float   demag_deg_left = 0.0f;
    float   deg_in_step = 0.0f;

    Result r;
    std::vector<double> errors;

    for (uint32_t k = 0; k < ticks; ++k) {
        theta += deg_per_tick;
        if (theta >= 360.0f) theta -= 360.0f;
        deg_in_step += deg_per_tick;

        // ---- phase voltages as the ADC sees them, mid-ON-time -------------
        const CommStep& st = kTable[applied_step];
        const float e[3] = {cfg.bemf_peak * trap(theta -   0.0f),
                            cfg.bemf_peak * trap(theta - 120.0f),
                            cfg.bemf_peak * trap(theta - 240.0f)};
        const PhaseDrive drv[3] = {st.u, st.v, st.w};

        float v[3];
        int floating = -1;
        for (int i = 0; i < 3; ++i) {
            if (drv[i] == PhaseDrive::UP)        v[i] = cfg.vbus;
            else if (drv[i] == PhaseDrive::DOWN) v[i] = 0.0f;
            else { v[i] = 0.5f * cfg.vbus + e[i]; floating = i; }
        }

        // Inductive discharge right after commutation pins the newly floating
        // phase to whichever rail its current freewheels into.
        demag_deg_left -= deg_per_tick;
        if (demag_deg_left > 0.0f && floating >= 0) {
            v[floating] = (e[floating] > 0.0f) ? 0.0f : cfg.vbus;
        }

        for (int i = 0; i < 3; ++i) {
            v[i] += gauss(rng);
            if (cfg.spike_prob > 0.0f && uni(rng) < cfg.spike_prob) {
                v[i] += (uni(rng) < 0.5f ? -1.0f : 1.0f) * cfg.spike_volts;
            }
            v[i] = std::max(0.0f, std::min(v[i], cfg.vbus));
        }

        // ---- observer ------------------------------------------------------
        bool fired = false;
        uint8_t synth = 0;
        if (floating >= 0) {
            BemfObserverInput in;
            in.v_float = v[floating];
            in.v_u = v[0]; in.v_v = v[1]; in.v_w = v[2];
            in.bus_voltage = cfg.vbus;
            in.step = applied_step;
            in.speed_steps_per_sec = cfg.steps_per_sec;
            fired = obs.update(in);
            synth = obs.getSyntheticHallStep();
        }

        // Watchdog: if the observer goes silent the bridge would stall, so
        // force the step on and count it as a miss.
        const bool forced = !fired && (deg_in_step > 90.0f);
        if (forced) {
            ++r.lost_steps;
            synth = applied_step;   // step C came from Hall C-1, so next Hall is C
        }

        if (fired || forced) {
            if (fired) {
                ++r.events;
                // The event names Hall position `synth`; ideally the rotor is
                // entering that sector at this instant.
                float err = theta - static_cast<float>(synth) * 60.0f;
                while (err >  180.0f) err -= 360.0f;
                while (err < -180.0f) err += 360.0f;
                errors.push_back(err);
                r.max_abs_deg = std::max(r.max_abs_deg, std::fabs(static_cast<double>(err)));
            }
            ++r.commutations;
            applied_step   = static_cast<uint8_t>((synth + 1) % 6);
            demag_deg_left = cfg.demag_deg;
            deg_in_step    = 0.0f;
            obs.onCommutation(applied_step);
        }
    }

    const BemfAmplitude a = obs.getAmplitude();
    r.amp_peak = a.peak_volts;
    r.amp_ke   = a.ke_v_s_per_rad;

    if (!errors.empty()) {
        double sum = 0.0;
        for (double e2 : errors) sum += e2;
        r.mean_err_deg = sum / errors.size();
        double sq = 0.0;
        for (double e2 : errors) sq += (e2 - r.mean_err_deg) * (e2 - r.mean_err_deg);
        r.rms_err_deg = std::sqrt(sq / errors.size());   // jitter about the mean
    }
    return r;
}

// ---------------------------------------------------------------------------

struct Case { const char* label; float speed; float bemf; float sigma; float spike; };

static const Case kCases[] = {
    {"clean, 1200 st/s",        1200.0f, 6.0f, 0.00f, 0.000f},
    {"clean, 2400 st/s",        2400.0f, 9.0f, 0.00f, 0.000f},
    {"weak BEMF (0.08*Vbus)",   1200.0f, 2.0f, 0.00f, 0.000f},
    {"noise 0.3 V rms",         1200.0f, 6.0f, 0.30f, 0.000f},
    {"noise 0.8 V rms",         1200.0f, 6.0f, 0.80f, 0.000f},
    {"noise 0.8 V + 1% spikes", 1200.0f, 6.0f, 0.80f, 0.010f},
    {"noise 0.8 V, 2400 st/s",  2400.0f, 9.0f, 0.80f, 0.000f},
};

static SimConfig configFor(const Case& c) {
    SimConfig cfg;
    cfg.steps_per_sec = c.speed;
    cfg.bemf_peak     = c.bemf;
    cfg.noise_sigma   = c.sigma;
    cfg.spike_prob    = c.spike;
    cfg.seconds       = 1.0f;
    cfg.seed          = 12345;
    return cfg;
}

int main() {
    printf("BEMF observer - trapezoidal model, 20 kHz PWM, 24 V bus\n");
    printf("Commutation angle error, electrical degrees (positive = late)\n\n");

    printf("  %-26s %-10s %8s %8s %8s %7s %7s\n",
           "case", "timing", "bias", "jitter", "max|e|", "steps", "missed");
    for (const Case& c : kCases) {
        const BemfTimingMode modes[2] = {BemfTimingMode::DELAY_30DEG,
                                         BemfTimingMode::FLUX_INTEGRATE};
        const char* names[2] = {"DELAY", "INTEGRATE"};
        for (int m = 0; m < 2; ++m) {
            Result r = runSim(configFor(c), modes[m]);
            printf("  %-26s %-10s %8.2f %8.2f %8.2f %7d %7d\n",
                   (m == 0) ? c.label : "", names[m], r.mean_err_deg,
                   r.rms_err_deg, r.max_abs_deg, r.commutations, r.lost_steps);
        }
    }
    printf("\n");

    // ---- BEMF amplitude estimator -----------------------------------------
    printf("--- BEMF amplitude estimate (least-squares fit of the ramp) ---\n");
    printf("  %-26s %9s %9s %8s %11s %11s\n",
           "case", "E true", "E est", "err %", "ke true", "ke est");
    for (const Case& c : kCases) {
        Result r = runSim(configFor(c), BemfTimingMode::FLUX_INTEGRATE);
        const double omega_e = 1.04719755 * c.speed;
        const double ke_true = c.bemf / omega_e;
        printf("  %-26s %9.3f %9.3f %8.2f %11.3e %11.3e\n",
               c.label, static_cast<double>(c.bemf), r.amp_peak,
               100.0 * (r.amp_peak - c.bemf) / c.bemf, ke_true, r.amp_ke);
    }
    printf("\n");

    // At 2400 steps/s there are only ~8 PWM samples per step, so the observer
    // has ~4 samples between the end of blanking and the zero-crossing.  The
    // deadband and the confirmation count trade noise rejection against having
    // enough samples left to confirm anything - sweep them.
    printf("--- tuning sweep: 2400 st/s, 0.8 V rms noise ---\n");
    printf("  %-9s %-8s %8s %8s %8s %7s\n",
           "deadband", "confirm", "bias", "jitter", "max|e|", "missed");
    const float deadbands[] = {0.24f, 0.8f, 1.6f, 2.4f};
    const uint8_t confirms[] = {1, 2};
    for (float db : deadbands) {
        for (uint8_t cf : confirms) {
            SimConfig cfg = configFor(kCases[6]);   // noise 0.8 V, 2400 st/s
            cfg.deadband_v = db;
            cfg.confirm    = cf;
            Result r = runSim(cfg, BemfTimingMode::FLUX_INTEGRATE);
            printf("  %-9.2f %-8u %8.2f %8.2f %8.2f %7d\n",
                   db, static_cast<unsigned>(cf), r.mean_err_deg,
                   r.rms_err_deg, r.max_abs_deg, r.lost_steps);
        }
    }
    printf("\n");
    return 0;
}
