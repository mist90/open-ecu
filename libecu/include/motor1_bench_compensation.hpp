/** @file motor1_bench_compensation.hpp
 * @brief Bounded sixth-order current trim for the unloaded MOTOR_1 vibration bench.
 */
#ifndef LIBECU_MOTOR1_BENCH_COMPENSATION_HPP
#define LIBECU_MOTOR1_BENCH_COMPENSATION_HPP

#include "motor_samples.hpp"
#include "control_optimization.hpp"
#include <algorithm>

namespace libecu {

/** @brief Speed-dependent coefficients prepared at the 100-Hz tick, before masking IRQs. */
struct Motor1BenchPoint {
    using S = FixedProfile::PllSpeed;
    using W = FixedProfile::Fraction;
    using C = FixedProfile::Current;
    W d = number_v<W, 1>, q = number_v<W, 1>;
    int16_t angle_gain = 0;
    uint16_t ripple_gain = 0;

    /** @brief Build all gains from the signed electrical speed (120 steps per wheel revolution). */
    LIBECU_SLOW_PATH static Motor1BenchPoint fromSpeed(S velocity) noexcept {
        static_assert(S::FRACTION_BITS == 14 && W::FRACTION_BITS == 30);
        constexpr uint32_t unit = S::SCALE;
        const uint32_t bits = uint32_t(velocity.raw());
        const uint32_t v = velocity.raw() < 0 ? 0u - bits : bits;
        Motor1BenchPoint p;
        if (v < 31u * unit) {
            p.d = W::fromRaw(int32_t((32u * unit - std::max(v, 16u * unit)) << 12));
        } else if (v < 79u * unit) {
            p.d = W::fromRaw(int32_t((std::max(v, 64u * unit) - 63u * unit) << 12));
        }
        p.q = p.d;
        if (v > 79u * unit && v < 107u * unit) {
            const uint32_t side = std::min(v - 79u * unit, 107u * unit - v);
            p.q = W::fromRaw(int32_t(W::SCALE + (side << 11)));
        } else if (v > 232u * unit) {
            p.q = W::fromRaw(int32_t((264u * unit - std::min(v, 262u * unit)) << 11));
        }
        if (v > 68u * unit && v < 100u * unit) {
            const uint32_t side = std::min(v - 68u * unit, 100u * unit - v);
            p.q = multiplyTo<W>(p.q, W::fromRaw(int32_t(W::SCALE - (side << 11))));
        }
        if (v > 16u * unit && v < 80u * unit) {
            const auto fade = std::min<uint32_t>(256, std::min((v - 16u * unit) >> 10, (80u * unit - v) >> 10));
            p.angle_gain = int16_t(int32_t(fade * 3 / 4) * (velocity.raw() < 0 ? -1 : 1));
        }
        if (velocity.raw() > int32_t(60u * unit) && v < 116u * unit)
            p.ripple_gain = uint16_t(std::min<uint32_t>(256, std::min((v - 60u * unit) >> 10,
                                                                      (116u * unit - v) >> 10)));
#if defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH
        if (v > 476u * unit && v < 812u * unit) {
            const auto side = std::min(v-476u*unit,812u*unit-v);
            p.d = W::fromRaw(int32_t(W::SCALE)-int32_t((3u*std::min<uint32_t>(256,side>>12))<<20));
        }
#endif
        return p;
    }

    /** @brief Scale a PI error without scaling the stored voltage integral. */
    static C weight(C error, W gain) noexcept {
        if (gain == number_v<W, 1>) return error;
        // Keep above-unity saturation without a narrowing overflow; nominal
        // feedback through unity needs no wide saturation branch.
        if (gain > number_v<W, 1>) {
            return error + C::fromRaw(int32_t(fixed_detail::roundedProduct<30>(
                gain.raw() - int32_t(W::SCALE), error.raw())));
        }
        return C::fromRaw(int32_t(fixed_detail::roundedProduct<30>(gain.raw(), error.raw())));
    }
};

/**
 * @brief Reduce the predictable phase ripple of a PLL tracking Hall-sector centres.
 * The observer still advances and reports its original phase/speed. Only the
 * angle used in Park and inverse Park changes. With constant rotor speed,
 * the sector-centre error is a sawtooth; its angular integral is quadratic.
 */
inline FixedProfile::Angle motor1BenchParkAngle(FixedProfile::Angle angle, int16_t weight) noexcept {
    using A = FixedProfile::Angle;
    static_assert(A::FRACTION_BITS == 24);
    if (weight == 0) return angle;
    const int32_t r = (int32_t((uint32_t(angle.raw()) + (1u << 23)) & 0xffffffu) - (1 << 23)) >> 12;
    // r is in [-0.5,0.5) in Q12; r²/2 - 1/24 has zero mean over a sector.
    const int32_t shape = r * r / 2 - (1 << 24) / 24;
    return angle + A::fromRaw(shape * weight / 256); // -1.875 .. +3.75 electrical degrees forward
}

/**
 * @brief Additive q-current demand for the measured low-speed 6fe disturbance.
 * One Hall step is one cycle of the disturbance. The parabolic sine uses
 * integer math; -1/8 cycle is -45 degrees of the harmonic, not the rotor.
 * @param in Native Hall/drive snapshot for this current-control tick.
 * @param request Signed q-current request before compensation.
 * @param speed_fade 0..256 gain from the lower-rate speed snapshot.
 * @return Signed trim, at most 1/16 A, zero outside the tested motor quadrant.
 */
inline FixedProfile::Current motor1BenchRipple(
    const MotorAlgorithmInputT<FixedProfile>& in, FixedProfile::Current request, uint16_t speed_fade) noexcept {
    using C = FixedProfile::Current;
    using S = FixedProfile::PllSpeed;
    using A = FixedProfile::Angle;
    static_assert(C::FRACTION_BITS == 20 && S::FRACTION_BITS == 14 && A::FRACTION_BITS == 24);
    if (speed_fade == 0 || in.drive_mode != DriveMode::FORWARD || in.speed_steps_s.raw() <= 0 ||
        request.raw() <= 0 || request.raw() >= 3 * int32_t(C::SCALE)) return {};

    // Q8 envelope: full over 38..50 RPM. Vanishes at 30/58 RPM, at
    // zero torque, and above 3 A. No trim on reverse or regenerative demand.
    const int32_t demand_fade = std::min(request.raw() >> 9, (3 * int32_t(C::SCALE) - request.raw()) >> 11);
    const int32_t fade = std::min<int32_t>(speed_fade, demand_fade);
    const uint32_t phase = uint32_t(in.angle_steps.raw()) - (1u << 21);
    const int32_t x = (int32_t((phase + (1u << 23)) & 0xffffffu) - (1 << 23)) >> 11;
    const int32_t wave = x * (4096 - (x < 0 ? -x : x)) / 16; // Q18, |wave| <= 2^18.
    return C::fromRaw(wave * fade / 1024); // Q20 current: peak 1/16 A.
}

#if defined(LIBECU_MOTOR1_SPAN_BENCH) && LIBECU_MOTOR1_SPAN_BENCH
/** Prepared additive demand for the next millisecond. One Q20 current and one Q12 voltage. */
struct Motor1BenchTrims {
    FixedProfile::Current current{};
    FixedProfile::VoltageRequest voltage{};
    /** @brief Compute the next millisecond's held commands from the native state snapshot. */
    static Motor1BenchTrims prepare(FixedProfile::Angle angle, FixedProfile::PllSpeed velocity,
                                    FixedProfile::Current amps) noexcept {
        using C = FixedProfile::Current;
        using V = FixedProfile::VoltageRequest;
        constexpr int32_t u = FixedProfile::PllSpeed::SCALE;
        const int32_t speed = velocity.raw();
        Motor1BenchTrims trim;
        if (speed<=40*u || speed>=116*u || amps.raw()<=0 || amps.raw()>=3*int32_t(C::SCALE)) return trim;
        // Half a held sample in Q24 Hall steps; cancel the mean 1 kHz ZOH lag.
        const auto ph=uint32_t(angle.raw())+uint32_t((int64_t(speed)*64+62)/125);
        const int32_t load=std::min<int32_t>(256,std::min(amps.raw()>>9,(3*int32_t(C::SCALE)-amps.raw())>>11));
        auto wave=[](uint32_t phase){
            const int32_t x=(int32_t((phase+(1u<<23))&0xffffffu)-(1<<23))>>11;
            return x*(4096-(x<0?-x:x))/16;
        };
        if (speed>60*u) {
            const int32_t gain=std::min(load,std::min((speed-60*u)>>10,(116*u-speed)>>10));
            trim.current=C::fromRaw(wave(ph-(1u<<21))*gain/1024);
        }
        if (speed<76*u) {
            const int32_t gain=std::min(load,std::min((speed-40*u)>>10,(76*u-speed)>>10));
            const int32_t n=wave(3u*ph)*gain;
            trim.voltage=V::fromRaw((n+(1<<18)-(n<0))>>19);
        }
        return trim;
    }
};
#endif
} // namespace libecu
#endif
