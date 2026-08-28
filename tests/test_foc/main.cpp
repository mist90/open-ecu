/**
 * @file main.cpp
 * @brief Host tests for the FOC maths and the FOC control loop
 *
 * Two layers:
 *
 *  1. The pure functions in foc_math.hpp, checked against the standard library
 *     and against the algebraic identities they are supposed to satisfy.
 *  2. FocAlgorithm driving a simulated PMSM through a fake PwmInterface, with
 *     the loop closed the whole way round: the duties the algorithm writes are
 *     converted back to phase voltages exactly as the inverter would, fed to
 *     an RL + back-EMF model, and the resulting phase currents handed back in.
 *     That closes over the angle convention, the Park signs, the SVM scaling
 *     and the regulator gains at once - if any of them is wrong the current
 *     does not converge to the reference.
 *
 * Build and run: ./run.sh
 */

#include "../../libecu/include/foc_algorithm.hpp"
#include "../../libecu/include/foc_math.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace libecu;

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------

static int g_failures = 0;
static int g_checks = 0;

static void check(bool ok, const char* what)
{
    g_checks++;
    if (!ok) {
        g_failures++;
        std::printf("  FAIL  %s\n", what);
    }
}

static void checkNear(float got, float want, float tol, const char* what)
{
    g_checks++;
    if (!(std::fabs(got - want) <= tol)) {
        g_failures++;
        std::printf("  FAIL  %s: got %.6f want %.6f (tol %.6f)\n", what, got, want, tol);
    }
}

// ---------------------------------------------------------------------------
// Motor model, MOTOR_1 as identified on the bench
// ---------------------------------------------------------------------------

static const float R_PHASE = 0.658f;      // ohm
static const float L_PHASE = 0.0007f;     // H
static const float FLUX_WB = 1.2336e-2f;  // V.s/rad_e
static const float V_BUS   = 31.3f;       // V
static const float F_PWM   = 20000.0f;    // Hz

/**
 * @brief PMSM in the rotor frame, integrated with sub-stepping
 *
 * did/dt = (vd - R*id + w*L*iq) / L
 * diq/dt = (vq - R*iq - w*L*id - w*flux) / L
 */
struct MotorModel {
    float id = 0.0f;
    float iq = 0.0f;

    void step(float vd, float vq, float omega_e, float dt, int substeps = 20)
    {
        const float h = dt / static_cast<float>(substeps);
        for (int i = 0; i < substeps; i++) {
            const float did = (vd - R_PHASE * id + omega_e * L_PHASE * iq) / L_PHASE;
            const float diq = (vq - R_PHASE * iq - omega_e * L_PHASE * id - omega_e * FLUX_WB) / L_PHASE;
            id += did * h;
            iq += diq * h;
        }
    }
};

/// Records what the algorithm asked the bridge to do
class FakePwm : public PwmInterface {
public:
    FakePwm() { frequency_ = static_cast<uint32_t>(F_PWM); }

    bool initialize(uint32_t, uint16_t) override { return true; }
    void setChannelState(PwmChannel ch, PwmState st) override
    {
        state[static_cast<int>(ch)] = st;
        state_writes++;
    }
    void updateDutyCycle(float u, float v, float w) override
    {
        duty_u = u; duty_v = v; duty_w = w;
    }
    void enable(bool) override {}
    void apply() override { com_events++; }

    PwmState state[3] = {PwmState::OFF, PwmState::OFF, PwmState::OFF};
    float duty_u = 0.5f, duty_v = 0.5f, duty_w = 0.5f;
    int com_events = 0;
    int state_writes = 0;
};

// ---------------------------------------------------------------------------
// 1. sin/cos lookup table
// ---------------------------------------------------------------------------

static void testSinCos()
{
    std::printf("sinCos (256-entry table, linear interpolation)\n");

    float worst_sin = 0.0f, worst_cos = 0.0f, worst_unit = 0.0f;

    // Sweep several revolutions in both directions at a step that is not a
    // multiple of the table spacing, so the interpolation is exercised rather
    // than the exact entries.
    for (int i = -40000; i <= 40000; i++) {
        const float angle = static_cast<float>(i) * 0.000997f * 10.0f;
        float s, c;
        sinCos(angle, s, c);

        const float es = std::fabs(s - std::sin(static_cast<double>(angle)));
        const float ec = std::fabs(c - std::cos(static_cast<double>(angle)));
        const float eu = std::fabs(s * s + c * c - 1.0f);
        if (es > worst_sin)  worst_sin = es;
        if (ec > worst_cos)  worst_cos = ec;
        if (eu > worst_unit) worst_unit = eu;
    }

    std::printf("  worst |sin err| = %.3e, |cos err| = %.3e, |s^2+c^2-1| = %.3e\n",
                worst_sin, worst_cos, worst_unit);

    // The documented figure is 7.53e-5; allow a little slack for float rounding
    // in the comparison itself, but not enough to hide a broken table.
    check(worst_sin < 1.0e-4f, "sin error within the documented 7.53e-5 bound");
    check(worst_cos < 1.0e-4f, "cos error within the documented 7.53e-5 bound");
    check(worst_unit < 3.0e-4f, "(sin, cos) stays on the unit circle");

    // Exact table entries must be exact, not merely close
    float s, c;
    sinCos(0.0f, s, c);
    checkNear(s, 0.0f, 1e-7f, "sin(0)");
    checkNear(c, 1.0f, 1e-7f, "cos(0)");
    sinCos(FOC_PI * 0.5f, s, c);
    checkNear(s, 1.0f, 1e-4f, "sin(pi/2)");
    checkNear(c, 0.0f, 1e-4f, "cos(pi/2)");
    sinCos(-FOC_PI * 0.5f, s, c);
    checkNear(s, -1.0f, 1e-4f, "sin(-pi/2)");
    checkNear(c, 0.0f, 1e-4f, "cos(-pi/2)");
    sinCos(FOC_PI, s, c);
    checkNear(s, 0.0f, 1e-4f, "sin(pi)");
    checkNear(c, -1.0f, 1e-4f, "cos(pi)");
}

// ---------------------------------------------------------------------------
// 2. Clarke / Park
// ---------------------------------------------------------------------------

static void testTransforms()
{
    std::printf("Clarke / Park\n");

    // A balanced three-phase set of peak amplitude I, at electrical angle
    // theta, with the current vector leading the rotor d axis by load_angle.
    const float amplitude = 7.0f;

    for (int k = 0; k < 24; k++) {
        const float theta = static_cast<float>(k) * (FOC_TWO_PI / 24.0f);
        const float load_angle = 0.7f;       // arbitrary, constant
        const float phase = theta + load_angle;

        const float i_u = amplitude * std::cos(phase);
        const float i_v = amplitude * std::cos(phase - FOC_TWO_PI / 3.0f);
        const float i_w = amplitude * std::cos(phase + FOC_TWO_PI / 3.0f);

        const AlphaBeta ab = clarke(i_u, i_v, i_w);
        checkNear(std::sqrt(ab.alpha * ab.alpha + ab.beta * ab.beta), amplitude, 1e-3f,
                  "Clarke is amplitude-invariant");

        float s, c;
        sinCos(theta, s, c);
        const DqVector dq = park(ab, s, c);

        // In the rotor frame the vector is stationary: d = I*cos(load_angle),
        // q = I*sin(load_angle), independent of theta.
        checkNear(dq.d, amplitude * std::cos(load_angle), 3e-3f, "Id is constant over a revolution");
        checkNear(dq.q, amplitude * std::sin(load_angle), 3e-3f, "Iq is constant over a revolution");

        // Round trip
        const AlphaBeta back = invPark(dq, s, c);
        checkNear(back.alpha, ab.alpha, 3e-3f, "invPark(park(x)).alpha == x.alpha");
        checkNear(back.beta,  ab.beta,  3e-3f, "invPark(park(x)).beta == x.beta");
    }

    // Pure q-axis current at theta = 0 must be pure beta: the q axis leads d
    // by 90 degrees, and d is alpha at theta = 0.
    float s, c;
    sinCos(0.0f, s, c);
    const AlphaBeta q_only = invPark(DqVector{0.0f, 1.0f}, s, c);
    checkNear(q_only.alpha, 0.0f, 1e-5f, "q axis at theta=0 has no alpha component");
    checkNear(q_only.beta,  1.0f, 1e-5f, "q axis at theta=0 is +beta");
}

// ---------------------------------------------------------------------------
// 3. SVPWM
// ---------------------------------------------------------------------------

/**
 * @brief Rotor-frame currents to what the three low-side shunts would report
 *
 * The shunts sit in the low-side legs and read current leaving the phase as
 * positive, which is the negative of the motor convention the transforms use.
 * Verified on the bench: six-step driving forward at 3.0 RPS against a
 * +0.525 A target reads +0.538 A on the DOWN phase - the phase current leaves
 * through by definition.
 *
 * The tests feed the algorithm what the hardware actually produces, so that
 * getting this convention wrong fails here rather than on a live motor. It is
 * not a cosmetic detail: an inverted measurement makes the current loop
 * positive-feedback, and it looks exactly like a 180-degree angle error in
 * the telemetry without being one.
 */
static void dqToShuntCurrents(float id, float iq, float sin_t, float cos_t,
                              float& i_u, float& i_v, float& i_w)
{
    const AlphaBeta ab = invPark(DqVector{id, iq}, sin_t, cos_t);
    // Motor convention (into the phase) ...
    const float u =  ab.alpha;
    const float v = -0.5f * ab.alpha + FOC_SQRT3_2 * ab.beta;
    const float w = -0.5f * ab.alpha - FOC_SQRT3_2 * ab.beta;
    // ... negated into the shunt convention the ADC layer reports.
    i_u = -u;
    i_v = -v;
    i_w = -w;
}

/// Differential (line-to-neutral, common mode removed) voltage from duties
static AlphaBeta phaseVoltageFromDuties(float du, float dv, float dw, float v_bus)
{
    const float vu = (du - 0.5f) * v_bus;
    const float vv = (dv - 0.5f) * v_bus;
    const float vw = (dw - 0.5f) * v_bus;
    // Same 2/3 projection as the current Clarke - it discards common mode,
    // which is exactly what the motor's floating star point does.
    AlphaBeta ab;
    ab.alpha = (2.0f * vu - vv - vw) * (1.0f / 3.0f);
    ab.beta  = (vv - vw) * FOC_INV_SQRT3;
    return ab;
}

static void testSvpwm()
{
    std::printf("SVPWM\n");

    const float v_limit = svpwmVoltageLimit(V_BUS);
    checkNear(v_limit, V_BUS / std::sqrt(3.0f), 1e-3f, "linear limit is Vbus/sqrt(3)");

    // 1.1547x the sinusoidal-PWM limit of Vbus/2 - the bus utilisation gain
    // that distinguishes space-vector from sine modulation.
    checkNear(v_limit / (V_BUS * 0.5f), 1.15470f, 1e-4f, "SVM buys 15.5 % over sine PWM");

    float min_duty = 1.0f, max_duty = 0.0f;
    float worst_reconstruction = 0.0f;
    float max_common_mode = 0.0f;

    // Sweep a full revolution at the linear limit: the hardest case.
    for (int k = 0; k < 720; k++) {
        const float theta = static_cast<float>(k) * (FOC_TWO_PI / 720.0f);
        const float va = v_limit * std::cos(theta);
        const float vb = v_limit * std::sin(theta);

        const PhaseDuties d = svpwm(va, vb, V_BUS);

        if (d.u < min_duty) min_duty = d.u;
        if (d.v < min_duty) min_duty = d.v;
        if (d.w < min_duty) min_duty = d.w;
        if (d.u > max_duty) max_duty = d.u;
        if (d.v > max_duty) max_duty = d.v;
        if (d.w > max_duty) max_duty = d.w;

        // The whole point of zero-sequence injection: it must change the
        // common mode and leave the differential voltage - the only part the
        // motor sees - exactly as requested.
        const AlphaBeta got = phaseVoltageFromDuties(d.u, d.v, d.w, V_BUS);
        const float err = std::sqrt((got.alpha - va) * (got.alpha - va) +
                                    (got.beta - vb) * (got.beta - vb));
        if (err > worst_reconstruction) worst_reconstruction = err;

        const float common = (d.u + d.v + d.w) / 3.0f - 0.5f;
        if (std::fabs(common) > max_common_mode) max_common_mode = std::fabs(common);
    }

    std::printf("  duty span at the linear limit: [%.4f, %.4f]\n", min_duty, max_duty);
    std::printf("  worst voltage reconstruction error: %.3e V\n", worst_reconstruction);
    std::printf("  peak injected common mode: %.4f of the period\n", max_common_mode);

    check(worst_reconstruction < 1e-3f, "duties reproduce the requested (alpha,beta) voltage");
    check(min_duty > -1e-4f && max_duty < 1.0f + 1e-4f,
          "duties stay inside [0,1] right up to the linear limit");
    // At exactly the inscribed-circle limit the duties must *touch* both rails
    check(min_duty < 1e-3f && max_duty > 1.0f - 1e-3f,
          "the linear limit is tight - duties reach both rails");
    // The injected term is the triangular third harmonic, peak 1/(4*sqrt(3))
    // of the period at the limit; a sinusoidal modulator would inject nothing.
    check(max_common_mode > 0.1f, "zero-sequence injection is present (this is SVM, not sine PWM)");

    // A zero request must give the null vector, all three phases at 50 %
    const PhaseDuties zero = svpwm(0.0f, 0.0f, V_BUS);
    checkNear(zero.u, 0.5f, 1e-6f, "zero vector: duty U = 0.5");
    checkNear(zero.v, 0.5f, 1e-6f, "zero vector: duty V = 0.5");
    checkNear(zero.w, 0.5f, 1e-6f, "zero vector: duty W = 0.5");

    // A dead bus must not produce infinities
    const PhaseDuties dead = svpwm(5.0f, 5.0f, 0.0f);
    check(std::isfinite(dead.u) && std::isfinite(dead.v) && std::isfinite(dead.w),
          "zero bus voltage does not produce NaN/Inf duties");
}

// ---------------------------------------------------------------------------
// 4. Closed-loop FOC against the motor model
// ---------------------------------------------------------------------------

struct LoopResult {
    float i_d;
    float i_q;
    float reported_current;
    float max_duty;
    float min_duty;
};

/**
 * @brief Run the FOC algorithm against the motor model for a given time
 *
 * @param iq_command Torque command handed to the algorithm (A, magnitude)
 * @param drive_mode FORWARD or REVERSE
 * @param speed_hz_e Electrical frequency of rotation (Hz)
 * @param seconds Simulated time
 * @param corrupt_worst_shunt Replace the highest-duty phase's current with
 *        nonsense, to prove the reconstruction is actually load-bearing
 */
static LoopResult runLoop(float iq_command, DriveMode drive_mode, float speed_hz_e,
                          float seconds, bool corrupt_worst_shunt = false)
{
    FakePwm pwm;
    FocAlgorithm foc(pwm, static_cast<uint32_t>(F_PWM));

    FocParams p;
    p.max_modulation = 0.95f;
    p.max_duty_cycle = 0.95f;
    p.angle_offset_rad = 0.0f;   // the model and the algorithm share one angle
    p.id_target = 0.0f;
    p.use_decoupling = false;
    p.l_phase_h = L_PHASE;
    p.flux_linkage_wb = FLUX_WB;
    p.anti_windup_kb = 1.0f;
    foc.setParams(p);

    // Gains from the model, exactly as main.cpp computes them
    const float w_c = FOC_TWO_PI * 500.0f;
    foc.setCurrentPiGains(L_PHASE * w_c, R_PHASE * w_c);
    foc.onEnter();

    MotorModel motor;
    const float dt = 1.0f / F_PWM;
    const float omega_e = FOC_TWO_PI * speed_hz_e;
    float theta = 0.0f;

    LoopResult result{};
    result.max_duty = 0.0f;
    result.min_duty = 1.0f;

    const int steps = static_cast<int>(seconds * F_PWM);
    for (int n = 0; n < steps; n++) {
        // Rotor frame -> phase currents, the way the shunts would see them
        float s, c;
        sinCos(theta, s, c);
        float i_u, i_v, i_w;
        dqToShuntCurrents(motor.id, motor.iq, s, c, i_u, i_v, i_w);

        if (corrupt_worst_shunt) {
            // Whichever phase had the highest duty last cycle is the one whose
            // low-side window was shortest, and it is the one the algorithm is
            // supposed to be throwing away.
            if (pwm.duty_u >= pwm.duty_v && pwm.duty_u >= pwm.duty_w)      i_u = 99.0f;
            else if (pwm.duty_v >= pwm.duty_w)                             i_v = 99.0f;
            else                                                           i_w = 99.0f;
        }

        MotorAlgorithmInput in{};
        in.i_u = i_u;
        in.i_v = i_v;
        in.i_w = i_w;
        in.bus_voltage = V_BUS;
        in.angle_steps = theta / FOC_RAD_PER_STEP;
        in.speed_steps_s = omega_e / FOC_RAD_PER_STEP;
        in.target_current = iq_command;
        in.duty_command = 0.0f;
        in.step = 0;
        in.drive_mode = drive_mode;
        in.electric_mode = ElectricMode::CURRENT_MODE;

        const MotorAlgorithmOutput out = foc.update(in);
        result.reported_current = out.measured_current;

        // Duties -> the differential voltage the motor actually sees
        const AlphaBeta v_ab = phaseVoltageFromDuties(pwm.duty_u, pwm.duty_v, pwm.duty_w, V_BUS);
        const DqVector v_dq = park(v_ab, s, c);

        motor.step(v_dq.d, v_dq.q, omega_e, dt);

        theta += omega_e * dt;
        if (theta > FOC_TWO_PI) theta -= FOC_TWO_PI;

        // Ignore the opening transient when recording the duty envelope
        if (n > steps / 2) {
            const float du[3] = {pwm.duty_u, pwm.duty_v, pwm.duty_w};
            for (int j = 0; j < 3; j++) {
                if (du[j] > result.max_duty) result.max_duty = du[j];
                if (du[j] < result.min_duty) result.min_duty = du[j];
            }
        }
    }

    result.i_d = motor.id;
    result.i_q = motor.iq;

    // The bridge must have been configured exactly once, and never commutated
    check(pwm.com_events == 1, "FOC raises exactly one COM event, at onEnter()");
    check(pwm.state[0] == PwmState::UP && pwm.state[1] == PwmState::UP &&
          pwm.state[2] == PwmState::UP, "FOC leaves all three phases modulating");
    check(pwm.state_writes == 3, "FOC writes the channel states once and never again");

    return result;
}

/**
 * @brief The shunt sign convention, pinned as a one-shot check
 *
 * The closed-loop tests catch an inverted measurement too - they go unstable -
 * but they catch it as "everything is broken", which is a slow thing to
 * diagnose. This one says which end is wrong.
 */
static void testCurrentSignConvention()
{
    std::printf("Current sign convention\n");

    FakePwm pwm;
    FocAlgorithm foc(pwm, static_cast<uint32_t>(F_PWM));

    FocParams p;
    p.max_modulation = 0.95f;
    p.max_duty_cycle = 0.95f;
    p.angle_offset_rad = 0.0f;
    p.id_target = 0.0f;
    p.use_decoupling = false;
    p.l_phase_h = L_PHASE;
    p.flux_linkage_wb = FLUX_WB;
    p.anti_windup_kb = 1.0f;
    foc.setParams(p);
    foc.setCurrentPiGains(0.0f, 0.0f);   // report only, no regulator action
    foc.onEnter();

    // A pure +q current of 4 A at a handful of rotor angles, presented the way
    // the shunts would present it, must be reported back as Iq = +4, Id = 0.
    for (int k = 0; k < 12; k++) {
        const float theta = static_cast<float>(k) * (FOC_TWO_PI / 12.0f);
        float s, c;
        sinCos(theta, s, c);

        MotorAlgorithmInput in{};
        dqToShuntCurrents(0.0f, 4.0f, s, c, in.i_u, in.i_v, in.i_w);
        in.bus_voltage = V_BUS;
        in.angle_steps = theta / FOC_RAD_PER_STEP;
        in.speed_steps_s = 0.0f;
        in.target_current = 0.0f;
        in.drive_mode = DriveMode::FORWARD;
        in.electric_mode = ElectricMode::CURRENT_MODE;

        const MotorAlgorithmOutput out = foc.update(in);
        checkNear(out.i_q, 4.0f, 0.02f, "a +q current reads back as +Iq");
        checkNear(out.i_d, 0.0f, 0.02f, "a +q current has no d component");
    }

    std::printf("  +4 A on q, presented in the shunt convention, reads back as +Iq at all angles\n");
}

static void testClosedLoop()
{
    std::printf("Closed-loop FOC against the MOTOR_1 model\n");

    // Standstill, DC currents in the rotor frame
    {
        LoopResult r = runLoop(6.0f, DriveMode::FORWARD, 0.0f, 0.05f);
        std::printf("  standstill, Iq* = 6.0 A -> Id = %+.4f, Iq = %+.4f\n", r.i_d, r.i_q);
        checkNear(r.i_q, 6.0f, 0.05f, "Iq tracks its reference at standstill");
        checkNear(r.i_d, 0.0f, 0.05f, "Id is regulated to zero at standstill");
        checkNear(r.reported_current, 6.0f, 0.1f, "reported current is Iq");
    }

    // Rotating: 60 electrical Hz is 3 mechanical RPS on a 20-pole-pair motor
    {
        LoopResult r = runLoop(6.0f, DriveMode::FORWARD, 60.0f, 0.05f);
        std::printf("  60 Hz_e, Iq* = 6.0 A -> Id = %+.4f, Iq = %+.4f, duty [%.3f, %.3f]\n",
                    r.i_d, r.i_q, r.min_duty, r.max_duty);
        checkNear(r.i_q, 6.0f, 0.10f, "Iq tracks its reference while rotating");
        checkNear(r.i_d, 0.0f, 0.10f, "Id is regulated to zero while rotating");
        check(r.min_duty > 0.0f && r.max_duty < 1.0f, "duties stay inside the switchable range");
    }

    // Reverse: the same command magnitude must produce the opposite torque,
    // which in the rotor frame is negative Iq at the same angle.
    {
        LoopResult r = runLoop(6.0f, DriveMode::REVERSE, -60.0f, 0.05f);
        std::printf("  reverse, Iq* = 6.0 A -> Id = %+.4f, Iq = %+.4f, reported %+.4f\n",
                    r.i_d, r.i_q, r.reported_current);
        checkNear(r.i_q, -6.0f, 0.10f, "REVERSE inverts the q-axis current");
        checkNear(r.reported_current, 6.0f, 0.15f,
                  "reported current is referred to the commanded direction");
    }

    // Zero command must hold zero current even with back-EMF pushing
    {
        LoopResult r = runLoop(0.0f, DriveMode::FORWARD, 60.0f, 0.05f);
        std::printf("  60 Hz_e, Iq* = 0 -> Id = %+.4f, Iq = %+.4f\n", r.i_d, r.i_q);
        checkNear(r.i_q, 0.0f, 0.05f, "zero command holds zero current against back-EMF");
    }

    // The worst-shunt rejection is load-bearing: destroy the highest-duty
    // phase's sample every cycle and the loop must be unaffected, because that
    // phase is reconstructed from the other two.
    {
        LoopResult r = runLoop(6.0f, DriveMode::FORWARD, 60.0f, 0.05f, true);
        std::printf("  worst shunt corrupted -> Id = %+.4f, Iq = %+.4f\n", r.i_d, r.i_q);
        checkNear(r.i_q, 6.0f, 0.10f, "highest-duty shunt is discarded, not used");
        checkNear(r.i_d, 0.0f, 0.10f, "reconstruction keeps the d axis clean");
    }

    // Beyond what the bus can deliver: the loop must saturate gracefully, stay
    // finite, and keep the duties switchable rather than winding up.
    {
        LoopResult r = runLoop(200.0f, DriveMode::FORWARD, 60.0f, 0.05f);
        std::printf("  over-demand, Iq* = 200 A -> Id = %+.4f, Iq = %+.4f, duty [%.3f, %.3f]\n",
                    r.i_d, r.i_q, r.min_duty, r.max_duty);
        check(std::isfinite(r.i_q) && std::isfinite(r.i_d), "over-demand stays finite");
        check(r.min_duty >= 0.0f && r.max_duty <= 1.0f, "over-demand duties stay switchable");
        check(r.i_q > 0.0f, "over-demand still pushes current the right way");
    }
}

/**
 * @brief Drive the loop hard into the bus limit, then drop the demand
 *
 * The failure this catches is a back-calculation term that looks right but is
 * scaled by dt: at 50 us it comes out three orders of magnitude smaller than
 * the ki*err*dt it is meant to oppose, so the integrator winds up freely and
 * the drive stays pinned at full voltage long after the command is removed.
 * Steady-state tests all pass with that bug present - only removing the demand
 * exposes it.
 */
static void testWindupRecovery()
{
    std::printf("Anti-windup recovery\n");

    FakePwm pwm;
    FocAlgorithm foc(pwm, static_cast<uint32_t>(F_PWM));

    FocParams p;
    p.max_modulation = 0.95f;
    p.max_duty_cycle = 0.95f;
    p.angle_offset_rad = 0.0f;
    p.id_target = 0.0f;
    p.use_decoupling = false;
    p.l_phase_h = L_PHASE;
    p.flux_linkage_wb = FLUX_WB;
    p.anti_windup_kb = 1.0f;
    foc.setParams(p);

    const float w_c = FOC_TWO_PI * 500.0f;
    foc.setCurrentPiGains(L_PHASE * w_c, R_PHASE * w_c);
    foc.onEnter();

    MotorModel motor;
    const float dt = 1.0f / F_PWM;
    const float omega_e = FOC_TWO_PI * 60.0f;
    float theta = 0.0f;

    // Phase 1: 100 ms of an impossible demand, pinning the bus.
    // Phase 2: command drops to 1 A. Measure how long Iq takes to come back.
    const int phase1 = static_cast<int>(0.100f * F_PWM);
    const int phase2 = static_cast<int>(0.050f * F_PWM);
    int settle_step = -1;

    for (int n = 0; n < phase1 + phase2; n++) {
        float s, c;
        sinCos(theta, s, c);

        MotorAlgorithmInput in{};
        dqToShuntCurrents(motor.id, motor.iq, s, c, in.i_u, in.i_v, in.i_w);
        in.bus_voltage = V_BUS;
        in.angle_steps = theta / FOC_RAD_PER_STEP;
        in.speed_steps_s = omega_e / FOC_RAD_PER_STEP;
        in.target_current = (n < phase1) ? 500.0f : 1.0f;
        in.duty_command = 0.0f;
        in.step = 0;
        in.drive_mode = DriveMode::FORWARD;
        in.electric_mode = ElectricMode::CURRENT_MODE;

        foc.update(in);

        const AlphaBeta v_ab = phaseVoltageFromDuties(pwm.duty_u, pwm.duty_v, pwm.duty_w, V_BUS);
        const DqVector v_dq = park(v_ab, s, c);
        motor.step(v_dq.d, v_dq.q, omega_e, dt);

        theta += omega_e * dt;
        if (theta > FOC_TWO_PI) theta -= FOC_TWO_PI;

        if (n >= phase1 && settle_step < 0 && std::fabs(motor.iq - 1.0f) < 0.1f) {
            settle_step = n - phase1;
        }
    }

    const float settle_ms = (settle_step < 0) ? -1.0f
                          : static_cast<float>(settle_step) * dt * 1000.0f;
    std::printf("  after 100 ms pinned at the bus limit, Iq* 500 A -> 1 A:\n");
    std::printf("  recovered in %.2f ms, final Iq = %+.4f A\n", settle_ms, motor.iq);

    check(settle_step >= 0, "the loop recovers from a wound-up integrator at all");
    // The closed-loop time constant at 500 Hz is ~0.32 ms. Anything past a few
    // milliseconds means the integrator is unwinding, not the plant responding.
    check(settle_step >= 0 && settle_ms < 5.0f, "recovery is plant-limited, not windup-limited");
    checkNear(motor.iq, 1.0f, 0.05f, "settles on the new reference");
}

// ---------------------------------------------------------------------------

int main()
{
    std::printf("=== FOC tests ===\n\n");

    testSinCos();
    std::printf("\n");
    testTransforms();
    std::printf("\n");
    testSvpwm();
    std::printf("\n");
    testCurrentSignConvention();
    std::printf("\n");
    testClosedLoop();
    std::printf("\n");
    testWindupRecovery();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
