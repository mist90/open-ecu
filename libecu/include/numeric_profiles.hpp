/**
 * @file numeric_profiles.hpp
 * @brief Numerical formats for physical values and regulator state
 */
#ifndef LIBECU_NUMERIC_PROFILES_HPP
#define LIBECU_NUMERIC_PROFILES_HPP

#include "fixed_point.hpp"

namespace libecu {

using FloatType = float;

/** @brief A regulator's input, output, gain and accumulator representations. */
template<class InputT, class OutputT, class GainT, class StepGainT, class AccumulatorT, class WideOutputT>
struct PiLoopTypes {
    using Input = InputT;
    using Output = OutputT;
    using Gain = GainT;
    using StepGain = StepGainT;
    using Accumulator = AccumulatorT;
    using WideOutput = WideOutputT;
};

/** @brief Native single-precision numerical profile. */
struct FloatProfile {
    using Current = FloatType;
    using Voltage = FloatType;
    using Angle = FloatType;
    using Speed = FloatType;
    using Duty = FloatType;
    using Fraction = FloatType;
    using Gain = FloatType;
    using Accumulator = FloatType;
    using PiTypes = PiLoopTypes<float, float, float, float, float, float>;
    using SpeedPiTypes = PiTypes;
    using CurrentPiTypes = PiTypes;
};

/** @brief Fixed-point formats; gains and wide requests have separate headroom. */
struct FixedProfile {
    using Current = FixedPointType<20>;
    using Voltage = FixedPointType<18>;
    using Angle = FixedPointType<24>;
    using Speed = FixedPointType<16>;
    using Duty = FixedPointType<30>;
    using Fraction = FixedPointType<30>;
    using Gain = FixedPointType<16>;
    using SpeedGain = FixedPointType<24>;
    using Accumulator = FixedPointType<32, int64_t>;
    using SpeedPiTypes = PiLoopTypes<Speed, Current, SpeedGain, FixedPointType<28>,
                                    Accumulator, FixedPointType<12>>;
    using CurrentPiTypes = PiLoopTypes<Current, Duty, Gain, FixedPointType<28>,
                                      Accumulator, FixedPointType<12>>;
};

} // namespace libecu
#endif // LIBECU_NUMERIC_PROFILES_HPP
