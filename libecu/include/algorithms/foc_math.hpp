/**
 * @file foc_math.hpp
 * @brief Clarke/Park transforms, space-vector modulation and a fast sin/cos
 *
 * Everything here is a pure function of its arguments - no state, no hardware,
 * no allocation - so it is testable on the host (tests/test_foc) and cheap
 * enough to run inside the 20 kHz current-loop ISR.
 *
 * Conventions used throughout:
 *
 * - Phase axes U, V, W sit at 0, 120 and 240 electrical degrees.
 * - The Clarke transform is the **amplitude-invariant** (2/3) form, so for a
 *   balanced sinusoidal set of peak I the resulting |(alpha,beta)| is I, and
 *   therefore Iq in the rotor frame is the *peak phase current*, in amperes.
 *   The alternative power-invariant (sqrt(2/3)) form would put a 1.225 factor
 *   between the current command and the actual phase current; every gain in
 *   this project is expressed against real amperes, so the 2/3 form is the one
 *   to use.
 * - The d axis is aligned with the rotor flux, q leads d by 90 electrical
 *   degrees, and positive Iq produces positive (forward) torque.
 * - Voltages are in volts, not per-unit duty. That keeps the current-loop gains
 *   independent of the bus voltage and makes the modulation limit a physical
 *   quantity rather than a fudge factor.
 */

#ifndef LIBECU_FOC_MATH_HPP
#define LIBECU_FOC_MATH_HPP

#include <cstdint>

namespace libecu {

constexpr float FOC_PI        = 3.14159265358979f;
constexpr float FOC_TWO_PI    = 6.28318530717959f;
constexpr float FOC_SQRT3     = 1.73205080756888f;
constexpr float FOC_SQRT3_2   = 0.86602540378444f;
constexpr float FOC_INV_SQRT3 = 0.57735026918963f;

/// One electrical revolution expressed in the PLL's "step" units
constexpr float FOC_STEPS_PER_EREV = 6.0f;
/// Radians per PLL step (60 electrical degrees)
constexpr float FOC_RAD_PER_STEP = FOC_PI / 3.0f;

/**
 * @brief Vector in the stationary two-phase (alpha, beta) frame
 */
struct AlphaBeta {
    float alpha;
    float beta;
};

/**
 * @brief Vector in the rotor (d, q) frame
 */
struct DqVector {
    float d;
    float q;
};

/**
 * @brief Per-phase duty cycles, 0.0 to 1.0, high-side on-time fraction
 */
struct PhaseDuties {
    float u;
    float v;
    float w;
};

/**
 * @name Sine lookup table
 *
 * One electrical revolution in FOC_SIN_TABLE_SIZE steps, linearly
 * interpolated. Cosine comes from the same table read a quarter turn along,
 * so both axes cost two loads and a lerp - no library call, no range
 * reduction, no polynomial, and the same number of cycles on every input,
 * which matters when the budget is a 20 kHz ISR that already spends 60-77
 * percent of its period in a `-O0` build (see docs/FOC_HANDOFF.md).
 *
 * 256 entries of float is 1 KB of flash, not RAM - the table is `constexpr`,
 * so it lives in .rodata and is read in place.
 *
 * Accuracy: linear interpolation between 256 samples is worst-case 7.53e-5
 * absolute in exact arithmetic, and 7.9e-5 end to end in float once the index
 * scaling is included - measured, not estimated; tests/test_foc sweeps 80 000
 * angles against the standard library and prints the figure. The Hall PLL
 * contributes about 1.3 electrical degrees of
 * angle error, which is 2.3e-2 on a unit sine, so the table is roughly 300x
 * finer than the angle it is being handed. Enlarging it buys nothing until
 * the rotor position comes from something better than six Hall sectors.
 */
///@{
constexpr uint32_t FOC_SIN_TABLE_BITS = 8;
constexpr uint32_t FOC_SIN_TABLE_SIZE = 1u << FOC_SIN_TABLE_BITS;   ///< 256
constexpr uint32_t FOC_SIN_TABLE_MASK = FOC_SIN_TABLE_SIZE - 1u;
/// Quarter turn, in table entries - the sin-to-cos offset
constexpr uint32_t FOC_SIN_TABLE_QUARTER = FOC_SIN_TABLE_SIZE / 4u;
/// Table entries per radian
constexpr float FOC_SIN_TABLE_SCALE = static_cast<float>(FOC_SIN_TABLE_SIZE) / FOC_TWO_PI;

/// sin(2*pi*i/256) for i in [0, 256)
inline constexpr float FOC_SIN_TABLE[FOC_SIN_TABLE_SIZE] = {
     0.000000000f,  0.024541229f,  0.049067674f,  0.073564564f,
     0.098017140f,  0.122410675f,  0.146730474f,  0.170961889f,
     0.195090322f,  0.219101240f,  0.242980180f,  0.266712757f,
     0.290284677f,  0.313681740f,  0.336889853f,  0.359895037f,
     0.382683432f,  0.405241314f,  0.427555093f,  0.449611330f,
     0.471396737f,  0.492898192f,  0.514102744f,  0.534997620f,
     0.555570233f,  0.575808191f,  0.595699304f,  0.615231591f,
     0.634393284f,  0.653172843f,  0.671558955f,  0.689540545f,
     0.707106781f,  0.724247083f,  0.740951125f,  0.757208847f,
     0.773010453f,  0.788346428f,  0.803207531f,  0.817584813f,
     0.831469612f,  0.844853565f,  0.857728610f,  0.870086991f,
     0.881921264f,  0.893224301f,  0.903989293f,  0.914209756f,
     0.923879533f,  0.932992799f,  0.941544065f,  0.949528181f,
     0.956940336f,  0.963776066f,  0.970031253f,  0.975702130f,
     0.980785280f,  0.985277642f,  0.989176510f,  0.992479535f,
     0.995184727f,  0.997290457f,  0.998795456f,  0.999698819f,
     1.000000000f,  0.999698819f,  0.998795456f,  0.997290457f,
     0.995184727f,  0.992479535f,  0.989176510f,  0.985277642f,
     0.980785280f,  0.975702130f,  0.970031253f,  0.963776066f,
     0.956940336f,  0.949528181f,  0.941544065f,  0.932992799f,
     0.923879533f,  0.914209756f,  0.903989293f,  0.893224301f,
     0.881921264f,  0.870086991f,  0.857728610f,  0.844853565f,
     0.831469612f,  0.817584813f,  0.803207531f,  0.788346428f,
     0.773010453f,  0.757208847f,  0.740951125f,  0.724247083f,
     0.707106781f,  0.689540545f,  0.671558955f,  0.653172843f,
     0.634393284f,  0.615231591f,  0.595699304f,  0.575808191f,
     0.555570233f,  0.534997620f,  0.514102744f,  0.492898192f,
     0.471396737f,  0.449611330f,  0.427555093f,  0.405241314f,
     0.382683432f,  0.359895037f,  0.336889853f,  0.313681740f,
     0.290284677f,  0.266712757f,  0.242980180f,  0.219101240f,
     0.195090322f,  0.170961889f,  0.146730474f,  0.122410675f,
     0.098017140f,  0.073564564f,  0.049067674f,  0.024541229f,
     0.000000000f, -0.024541229f, -0.049067674f, -0.073564564f,
    -0.098017140f, -0.122410675f, -0.146730474f, -0.170961889f,
    -0.195090322f, -0.219101240f, -0.242980180f, -0.266712757f,
    -0.290284677f, -0.313681740f, -0.336889853f, -0.359895037f,
    -0.382683432f, -0.405241314f, -0.427555093f, -0.449611330f,
    -0.471396737f, -0.492898192f, -0.514102744f, -0.534997620f,
    -0.555570233f, -0.575808191f, -0.595699304f, -0.615231591f,
    -0.634393284f, -0.653172843f, -0.671558955f, -0.689540545f,
    -0.707106781f, -0.724247083f, -0.740951125f, -0.757208847f,
    -0.773010453f, -0.788346428f, -0.803207531f, -0.817584813f,
    -0.831469612f, -0.844853565f, -0.857728610f, -0.870086991f,
    -0.881921264f, -0.893224301f, -0.903989293f, -0.914209756f,
    -0.923879533f, -0.932992799f, -0.941544065f, -0.949528181f,
    -0.956940336f, -0.963776066f, -0.970031253f, -0.975702130f,
    -0.980785280f, -0.985277642f, -0.989176510f, -0.992479535f,
    -0.995184727f, -0.997290457f, -0.998795456f, -0.999698819f,
    -1.000000000f, -0.999698819f, -0.998795456f, -0.997290457f,
    -0.995184727f, -0.992479535f, -0.989176510f, -0.985277642f,
    -0.980785280f, -0.975702130f, -0.970031253f, -0.963776066f,
    -0.956940336f, -0.949528181f, -0.941544065f, -0.932992799f,
    -0.923879533f, -0.914209756f, -0.903989293f, -0.893224301f,
    -0.881921264f, -0.870086991f, -0.857728610f, -0.844853565f,
    -0.831469612f, -0.817584813f, -0.803207531f, -0.788346428f,
    -0.773010453f, -0.757208847f, -0.740951125f, -0.724247083f,
    -0.707106781f, -0.689540545f, -0.671558955f, -0.653172843f,
    -0.634393284f, -0.615231591f, -0.595699304f, -0.575808191f,
    -0.555570233f, -0.534997620f, -0.514102744f, -0.492898192f,
    -0.471396737f, -0.449611330f, -0.427555093f, -0.405241314f,
    -0.382683432f, -0.359895037f, -0.336889853f, -0.313681740f,
    -0.290284677f, -0.266712757f, -0.242980180f, -0.219101240f,
    -0.195090322f, -0.170961889f, -0.146730474f, -0.122410675f,
    -0.098017140f, -0.073564564f, -0.049067674f, -0.024541229f,
};
///@}

/**
 * @brief sin and cos of the same angle, from the lookup table
 *
 * Both results are interpolated with the *same* fractional position, so the
 * pair stays consistent: the (sin, cos) point drifts along the unit circle
 * rather than off it, and the Park / inverse-Park pair remains very nearly a
 * rotation. That matters more for field orientation than the absolute
 * accuracy of either value on its own.
 *
 * @param angle Angle in radians. Any value within +-32000 rad is handled;
 *              beyond that the floor trick below stops being exact, which is
 *              far outside anything an electrical angle reaches.
 * @param sin_out Output: sin(angle)
 * @param cos_out Output: cos(angle)
 */
inline void sinCos(float angle, float& sin_out, float& cos_out) noexcept
{
    const float pos = angle * FOC_SIN_TABLE_SCALE;

    // floor(pos) without a branch or a libm call: adding the bias makes the
    // argument non-negative, where truncation and floor agree, and subtracting
    // it again undoes the shift exactly (2^15 is exact in float, and so is
    // every integer below 2^24).
    const int32_t index_floor = static_cast<int32_t>(pos + 32768.0f) - 32768;
    const float frac = pos - static_cast<float>(index_floor);

    const uint32_t is0 = static_cast<uint32_t>(index_floor) & FOC_SIN_TABLE_MASK;
    const uint32_t is1 = (is0 + 1u) & FOC_SIN_TABLE_MASK;
    const uint32_t ic0 = (is0 + FOC_SIN_TABLE_QUARTER) & FOC_SIN_TABLE_MASK;
    const uint32_t ic1 = (ic0 + 1u) & FOC_SIN_TABLE_MASK;

    const float s0 = FOC_SIN_TABLE[is0];
    const float c0 = FOC_SIN_TABLE[ic0];
    sin_out = s0 + frac * (FOC_SIN_TABLE[is1] - s0);
    cos_out = c0 + frac * (FOC_SIN_TABLE[ic1] - c0);
}

/**
 * @brief Clarke transform: three phase currents to the stationary frame
 *
 * Uses the general 2/3 form rather than the `alpha = i_u, beta = (i_u + 2*i_v)/sqrt(3)`
 * shortcut. The shortcut assumes i_u + i_v + i_w == 0 exactly and silently
 * folds any common-mode error into beta; the general form spreads it across
 * both axes, where the current loops reject it.
 *
 * @param i_u,i_v,i_w Phase currents (A), signed, motor-positive into the phase
 */
inline AlphaBeta clarke(float i_u, float i_v, float i_w) noexcept
{
    AlphaBeta ab;
    ab.alpha = (2.0f * i_u - i_v - i_w) * (1.0f / 3.0f);
    ab.beta  = (i_v - i_w) * FOC_INV_SQRT3;
    return ab;
}

/**
 * @brief Park transform: stationary frame to rotor frame
 * @param ab Vector in the (alpha, beta) frame
 * @param sin_theta,cos_theta Sine and cosine of the electrical rotor angle
 */
inline DqVector park(const AlphaBeta& ab, float sin_theta, float cos_theta) noexcept
{
    DqVector dq;
    dq.d =  ab.alpha * cos_theta + ab.beta * sin_theta;
    dq.q = -ab.alpha * sin_theta + ab.beta * cos_theta;
    return dq;
}

/**
 * @brief Inverse Park transform: rotor frame to stationary frame
 * @param dq Vector in the (d, q) frame
 * @param sin_theta,cos_theta Sine and cosine of the electrical rotor angle
 */
inline AlphaBeta invPark(const DqVector& dq, float sin_theta, float cos_theta) noexcept
{
    AlphaBeta ab;
    ab.alpha = dq.d * cos_theta - dq.q * sin_theta;
    ab.beta  = dq.d * sin_theta + dq.q * cos_theta;
    return ab;
}

/**
 * @brief Largest line-to-neutral voltage amplitude SVM can synthesise linearly
 *
 * The hexagon inscribed circle sits at Vbus/sqrt(3), which is where the
 * 15.5 % bus-utilisation advantage of space-vector over sinusoidal modulation
 * (Vbus/2) comes from. Above it the modulator overmodulates and the output
 * stops being a faithful sinusoid.
 */
inline float svpwmVoltageLimit(float v_bus) noexcept
{
    return v_bus * FOC_INV_SQRT3;
}

/**
 * @brief Space-vector modulation: (alpha, beta) voltage request to duty cycles
 *
 * ## This is SVPWM, written in its min/max form
 *
 * The textbook construction picks the sector the reference vector is in,
 * solves for the two adjacent active-vector on-times T1 and T2, and splits the
 * remaining time equally between the two null vectors V0 (000) and V7 (111) to
 * get the symmetric seven-segment pattern. That symmetric null split is the
 * only degree of freedom in the whole scheme, and it is exactly equivalent to
 * adding the common-mode (zero-sequence) term
 *
 *     v_com = (max(va,vb,vc) + min(va,vb,vc)) / 2
 *
 * to all three phase references: centring the three duties about 0.5 places
 * the null time symmetrically at both ends of the period, which *is* the
 * seven-segment pattern. The gate waveforms are identical, phase for phase and
 * edge for edge - what differs is only how they were arrived at. The min/max
 * form has no sector search, no branches on the vector angle, no trig beyond
 * the inverse Park that already happened, and costs about a dozen flops.
 *
 * The signature that this is space-vector and not sinusoidal PWM is the
 * bus utilisation: the injected common-mode term is a triangular third
 * harmonic, and it is what buys the extra 2/sqrt(3) = 1.155 of linear range
 * (see svpwmVoltageLimit()).
 *
 * Because the modulator is centre-aligned, all three low-side switches are on
 * together around the counter peak, which is where the ADC is triggered - so
 * the three shunts all carry phase current at the sampling instant. That
 * window shrinks as the duty of a phase approaches 1; see FocAlgorithm for how
 * the shortest window is dealt with.
 *
 * @param v_alpha,v_beta Requested stator voltage in the stationary frame (V)
 * @param v_bus Measured DC bus voltage (V)
 * @return Per-phase high-side duty cycles, nominally within [0, 1]. Duties are
 *         NOT clamped here - overmodulation is the caller's decision to make
 *         and to detect.
 */
inline PhaseDuties svpwm(float v_alpha, float v_beta, float v_bus) noexcept
{
    // Inverse Clarke: line-to-neutral voltage requested of each phase
    const float va = v_alpha;
    const float vb = -0.5f * v_alpha + FOC_SQRT3_2 * v_beta;
    const float vc = -0.5f * v_alpha - FOC_SQRT3_2 * v_beta;

    float v_max = va;
    float v_min = va;
    if (vb > v_max) v_max = vb; else if (vb < v_min) v_min = vb;
    if (vc > v_max) v_max = vc; else if (vc < v_min) v_min = vc;

    // Zero-sequence injection - see the note above on why this is SVM
    const float v_com = 0.5f * (v_max + v_min);

    // A dead bus would divide by zero; report 50 % (all phases equal, zero
    // differential voltage) rather than producing infinities.
    const float inv_v_bus = (v_bus > 1.0f) ? (1.0f / v_bus) : 0.0f;

    PhaseDuties d;
    d.u = 0.5f + (va - v_com) * inv_v_bus;
    d.v = 0.5f + (vb - v_com) * inv_v_bus;
    d.w = 0.5f + (vc - v_com) * inv_v_bus;
    return d;
}

} // namespace libecu

#endif // LIBECU_FOC_MATH_HPP
