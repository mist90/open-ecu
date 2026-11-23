/** @file main.cpp @brief Host speed and load steps with the STM32 control clocks and a simulated MOTOR_1.
 */
#include "bldc_controller.hpp"
#include "board.hpp"
#include "motor_config.hpp"
#include "../../STM32F410/Core/Inc/board_config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace libecu;
namespace {
constexpr unsigned Hz = ActiveTiming::CURRENT_HZ, slow = ActiveTiming::SPEED_HZ;
constexpr unsigned systick = ActiveTiming::SYSTICK_HZ, sectors = 3 * BLDC_NUM_POLES;
constexpr double turn = 6.283185307179586, dt = 1. / Hz, radius = .760 / turn;
constexpr double flux = BLDC_KE_V_S_PER_RAD_E, R = BLDC_R_PHASE_OHM, L = BLDC_L_PHASE_H;
constexpr double kt = 1.5 * (BLDC_NUM_POLES / 2) * flux, extra_nm = 3 * kt;

// Both profiles drive the same virtual bridge. Use the public IHM08M1 electrical scales;
// individual switching edges, dead time and the ADC sampling aperture are not simulated.
constexpr unsigned pwm_period = ihm08::TIMER_CLOCK_HZ / (2 * Hz);
constexpr float max_duty = .90f; // Exercise a 10% low-side window on every phase.
static_assert(ihm08::TIMER_CLOCK_HZ % (2 * Hz) == 0 && max_duty <= ihm08::MAX_DUTY);
constexpr float drive_limit_a = std::min(BLDC_MAX_CURRENT, BoardInfo{}.max_drive_current_a);
constexpr double phase_limit_a = 25; // Same independent phase budget as the public F410 port.
constexpr unsigned adc_bits = 12, adc_max = (1u << adc_bits) - 1, adc_zero = (adc_max + 1) / 2;
constexpr float zero_v = ihm08::ADC_REFERENCE * adc_zero / adc_max;
constexpr CurrentSensorCalibration sensor{ihm08::SHUNT_OHMS, ihm08::CURRENT_GAIN, ihm08::ADC_REFERENCE,
                                          adc_bits, zero_v, zero_v, zero_v};
constexpr VoltageSensorParameters bus_divider{ihm08::BUS_R_UP, ihm08::BUS_R_DOWN};
constexpr TemperatureSensorParameters ntc{};
constexpr double amps_per_count = double(sensor.adc_reference_voltage) /
                                  (adc_max * double(sensor.shunt_resistance_ohms) * sensor.opamp_gain);
constexpr unsigned bus_code = unsigned(.5 + double(ihm08::NOMINAL_BUS_VOLTAGE) / sensor.adc_reference_voltage *
                                       adc_max * bus_divider.r_down / (bus_divider.r_up + bus_divider.r_down));
static_assert(bus_code > 0 && bus_code < adc_max);
static_assert(slow == 100 && sectors == 120 && Hz % systick == 0 && systick % 500 == 0);
unsigned tick;
void need(bool ok, const char* message) {
    if (!ok) {
        std::fprintf(stderr, "tick %u: %s\n", tick, message);
        std::exit(1);
    }
}

struct Drive final : PwmInterface, HallInterface, AdcInterface {
    bool enabled = false;
    uint8_t sector = 0;
    double ia = 0, ib = 0, alpha_v = 0, beta_v = 0, peak_phase_a = 0;
    bool initialize(uint32_t, uint16_t) override { return true; }
    bool initialize() override { return true; }
    void enable(bool b) override { enabled = b; }
    void setChannelState(PwmChannel, PwmState) override {}
    template <class Duty>
    void applyDuty(Duty u, Duty v, Duty w) {
        const auto volts = [](Duty d) {
            constexpr auto limit = constant<Duty>(max_duty);
            need(d >= Duty{} && d <= limit, "PWM duty outside the fixture window");
            const auto counts = dutyToCompare(d, pwm_period);
            need(counts <= unsigned(pwm_period * max_duty), "PWM compare outside the fixture window");
            return double(ihm08::NOMINAL_BUS_VOLTAGE) * counts / pwm_period;
        };
        const double vu = volts(u), vv = volts(v), vw = volts(w);
        alpha_v = (2 * vu - vv - vw) / 3;
        beta_v = (vv - vw) / std::sqrt(3.);
    }
    void updateDutyCycle(float u, float v, float w) override { applyDuty(u, v, w); }
    void updateNativeDutyCycle(NativeDuty u, NativeDuty v, NativeDuty w) override { applyDuty(u, v, w); }
    uint8_t getPosition() override { return sector; }
    uint32_t getRawTemperatureAdcValue() override {
        return uint32_t(.5 + adc_max * ntc.r_pulldown_ohms / (ntc.r_pulldown_ohms + ntc.ntc_r25_ohms));
    }
    uint32_t getRawAdcValue() override { return bus_code; }
    uint32_t getRawAdcValue(PwmChannel ch) override {
        const double phases[] = {ia, -.5 * ia + std::sqrt(.75) * ib, -.5 * ia - std::sqrt(.75) * ib};
        const double phase = phases[static_cast<unsigned>(ch)];
        need(std::abs(phase) <= phase_limit_a, "model phase current exceeds fixture budget");
        peak_phase_a = std::max(peak_phase_a, std::abs(phase));
        // Positive motor current flows out of the inverter, opposite the low-side shunt convention.
        const double code = adc_zero - std::round(phase / amps_per_count);
        need(code >= 0 && code <= adc_max, "phase current outside ADC range");
        return uint32_t(code);
    }
};

// Use the public MOTOR_1 control calibration on the common virtual bridge.
MotorControlParams params(float kp, float ki) {
    MotorControlParams p{};
    p.num_poles = BLDC_NUM_POLES;
    p.max_speed_rps = BLDC_MAX_SPEED;
    p.useInverseCommTable = BLDC_INVERTION;
    p.max_current = drive_limit_a;
    p.min_current = BLDC_MIN_CURRENT;
    p.max_temperature_c = BLDC_MAX_TEMPERATURE;
    p.max_duty_cycle = max_duty;
    p.max_voltage = ihm08::MAX_BUS_VOLTAGE;
    p.algorithm = DriveAlgorithm::FOC;
    p.acceleration_rate = motor_config::soft_start_rps2;
    p.traction_acceleration_rps2 = motor_config::traction_acceleration_rps2;
    p.shape_velocity_error = motor_config::velocity_error_band_rps > 0;
    p.measured_speed_lpf_alpha = .5f;
    p.speed_pi = {
        kp, ki, p.max_current, p.min_current, std::min(p.max_current, motor_config::speed_integral_max_a),
        -4, 2};
    p.foc_current_regulator = motor_config::foc_current_pi;
    p.foc.max_duty_cycle = p.max_duty_cycle;
    p.foc.max_modulation = std::min(max_duty, ihm08::MAX_MODULATION);
    p.foc.low_side_clamped_pwm = ihm08::LOW_SIDE_CLAMPED_PWM;
    p.foc.angle_offset_rad = BLDC_FOC_ANGLE_OFFSET_DEG / 60.f * FOC_RAD_PER_STEP;
    p.foc.anti_windup_kb = 1;
    p.foc.current_filter_order = CurrentFilterOrder::FIRST_ORDER;
    p.foc.current_filter_cutoff_hz = ihm08::CURRENT_FEEDBACK_FILTER_HZ;
    return p;
}

struct Wheel {
    double n = 0, position, iq = 0;
    const double inertia, drag, response = -std::expm1(-dt * R / L);
    explicit Wheel(double kg, double initial)
        : position(initial), inertia(.00625 + kg * radius * radius),
          drag(kt * .656 + .008 * kg * 9.80665 * radius)
    {}
    int sector() const { return int(std::floor(position + .5)); }
    void step(Drive& io, double load) {
        // Fixed alpha/beta voltages from one carrier's PWM. Over this short interval approximate
        // the sinusoidal EMF at its midpoint and integrate the RL response and its mean exactly.
        const double omega = n * turn * (BLDC_NUM_POLES / 2);
        const double theta = (position - 1) * turn / 6 + omega * dt * .5;
        const double s = std::sin(theta), c = std::cos(theta);
        const double a_eq = (io.alpha_v + flux * omega * s) / R, b_eq = (io.beta_v - flux * omega * c) / R;
        const double avg = response * L / (dt * R);
        const double mean_a = a_eq + (io.ia - a_eq) * avg, mean_b = b_eq + (io.ib - b_eq) * avg;
        io.ia += (a_eq - io.ia) * response;
        io.ib += (b_eq - io.ib) * response;
        iq = -s * mean_a + c * mean_b;
        const double predicted = n + dt * kt * (iq - .0484 * n) / (turn * inertia);
        const double loss = dt * (drag + load) / (turn * inertia);
        const double next = predicted - std::clamp(predicted, -loss, loss);
        position += .5 * (n + next) * dt * sectors;
        n = next;
    }
};
enum Scene {
    Quiet,
    Pulse,
    From3,
    From5,
    HoldPulse,
    Sweep
};
const char* labels[] = {"quiet", "pulse", "3to4", "5to4", "hold-pulse", "sweep"};

void run(double kg, float kp, float ki, Scene scene, double initial, FILE* trace) {
    tick = 0;
    Drive io;
    Wheel wheel(kg, initial);
    need(io.AdcInterface::initialize(sensor, bus_divider, ntc), "ADC init");
    BldcController motor(io, io, params(kp, ki), &io);
    need(motor.setPllGains(BLDC_PLL_KP, BLDC_PLL_KI) && motor.initialize(), "controller init");
    motor.start();
    motor.setDriveMode(DriveMode::FORWARD);
    const auto& state = motor.io();
    unsigned divider = systick / slow - 1;
    const unsigned pre = scene == From3 || scene == From5 || scene == HoldPulse ? 140 * Hz : 0;
    const unsigned length = scene == Sweep ? 100 : 20;
    double last_target = -1;
    for (; tick < pre + length * Hz; ++tick) {
        constexpr double plateaus[] = {.5, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        const double target = scene == Sweep                 ? plateaus[tick / (10 * Hz)]
                              : tick < pre && scene == From3 ? 3.
                              : tick < pre && scene == From5 ? 5.
                                                             : 4.;
        const double load =
            (scene == Pulse || scene == HoldPulse) && tick >= pre + 10 * Hz && tick < pre + 12 * Hz ? extra_nm
                                                                                                    : 0.;
        if (target != last_target) {
            motor.setTargetSpeed(float(target));
            last_target = target;
        }
        if (tick % (Hz / systick) == 0) {
            const auto due = ActiveTiming::nextSysTick(divider);
            if (due == ControlTick::Service)
                motor.update();
            else if (due == ControlTick::SpeedPiOnly)
                motor.updateSpeedPi();
            const float request = numberCast<float>(state.in.target_current);
            need(request >= BLDC_MIN_CURRENT && request <= drive_limit_a, "current-command rail");
            if (trace && tick >= pre && tick % (Hz / 500) == 0)
                std::fprintf(trace, "%g,%.3g,%.3g,%s,%u,%.2f,%.6f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,"
                             "%d,%.7f,%.7f,%.7f\n",
                             kg, double(kp), double(ki), labels[scene], (tick - pre) / (Hz / 500) * 2, target,
                             load, wheel.n, numberCast<float>(state.in.velocity_feedback_rps),
                             numberCast<float>(state.in.current_speed_rps),
                             numberCast<float>(state.out.pll.speed_steps_s) / sectors,
                             numberCast<float>(state.in.target_current),
                             numberCast<float>(state.out.electrical.i_q), wheel.iq, io.peak_phase_a,
                             int(state.out.electrical.current_pi_saturation),
                             numberCast<float>(state.out.electrical.i_d), numberCast<float>(state.out.electrical.v_d),
                             numberCast<float>(state.out.electrical.v_q));
        }
        const int prev = wheel.sector();
        // The next frame samples the currents produced by the previously applied duties.
        wheel.step(io, load);
        const int next = wheel.sector();
        if (next != prev) {
            need(std::abs(next - prev) == 1, "missed Hall sector");
            io.sector = uint8_t((next % 6 + 6) % 6);
            motor.hallEdgeInterruptHandler();
        }
        need(motor.pwmInterruptHandler() && io.enabled && motor.getDriveMode() == DriveMode::FORWARD,
             "unexpected park");
        need(std::isfinite(wheel.n) && std::abs(wheel.n) <= BLDC_MAX_SPEED, "unbounded wheel");
    }
    motor.setDriveMode(DriveMode::NEUTRAL);
    need(!io.enabled && state.in.target_current == ActiveNumericProfile::Current{},
         "park did not clear torque");
}
} // namespace

uint32_t time_us() {
    return uint32_t(uint64_t(tick) * 1000000 / Hz);
}
extern "C" void disable_interrupts() {}
extern "C" void enable_interrupts() {}

int main(int argc, char** argv) {
    if (argc != 2 && argc != 4)
        return 2;
    const auto gain = [](const char* s) {
        char* end = nullptr;
        const float g = std::strtof(s, &end);
        need(end != s && *end == '\0' && std::isfinite(g) && g > 0, "invalid PI gain");
        return g;
    };
    const float kp = argc == 4 ? gain(argv[2]) : BLDC_SPEED_CURRENT_KP;
    const float ki = argc == 4 ? gain(argv[3]) : BLDC_SPEED_CURRENT_KI;
    FILE* trace = std::fopen(argv[1], "w");
    if (!trace)
        return 1;
    std::fputs("kg,kp,ki,scene,ms,target_rps,brake_nm,wheel_rps,feedback_rps,monitor_rps,pll_rps,request_A,"
               "measured_iq_A,torque_iq_A,peak_phase_A,voltage_saturation,measured_id_A,vd_V,vq_V\n", trace);
    for (double kg : {0., 80.})
        for (auto scene : {Quiet, Pulse, From3, From5, HoldPulse, Sweep})
            if (scene != Sweep || kg == 0.)
                run(kg, kp, ki, scene, .13, trace);
    need(std::fclose(trace) == 0, "trace write");
}
