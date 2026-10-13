/**
 * @file numeric_profiles.hpp
 * @brief Numerical formats for physical values and regulator state
 */
#ifndef LIBECU_NUMERIC_PROFILES_HPP
#define LIBECU_NUMERIC_PROFILES_HPP

#include "fixed_point.hpp"

namespace libecu {

using FloatType = float;
class FloatMath;
class FixedMath;

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
    using Math = FloatMath;
    using Current = FloatType;
    using Voltage = FloatType;
    using VoltageRequest = FloatType;
    using Inductance = FloatType;
    using Flux = FloatType;
    using Angle = FloatType;
    using Speed = FloatType;
    using Duty = FloatType;
    using Fraction = FloatType;
    using Gain = FloatType;
    using Accumulator = FloatType;
    using PllSpeed = FloatType;
    using PllGain = FloatType;
    using PllStepGain = FloatType;
    using PllAccumulator = FloatType;
    using PhaseAccumulator = FloatType;
    using FocStepGain = FloatType;
    using Count = FloatType;
    using CountAccumulator = FloatType;
    using CalibrationScale = FloatType;
    using AdcCount = FloatType;
    using ShuntResistance = FloatType;
    using Temperature = FloatType;
    using Logarithm = FloatType;
    using PiTypes = PiLoopTypes<float, float, float, float, float, float>;
    using SpeedPiTypes = PiTypes;
    using CurrentPiTypes = PiTypes;
};

/** @brief Fixed-point formats; gains and wide requests have separate headroom. */
struct FixedProfile {
    using Math = FixedMath;
    using Current = FixedPointType<20>;
    using Voltage = FixedPointType<18>;
    using VoltageRequest = FixedPointType<12>;
    using Inductance = FixedPointType<30>;
    using Flux = FixedPointType<30>;
    using Angle = FixedPointType<24>;
    using Speed = FixedPointType<16>;
    using Duty = FixedPointType<30>;
    using Fraction = FixedPointType<30>;
    using Gain = FixedPointType<16>;
    using SpeedGain = FixedPointType<24>;
    using Accumulator = FixedPointType<32, int64_t>;
    using PllSpeed = FixedPointType<14>;
    using PllGain = FixedPointType<14>;
    using PllStepGain = FixedPointType<24>;
    using PllAccumulator = FixedPointType<24, int64_t>;
    using PhaseAccumulator = FixedPointType<32, int64_t>;
    using FocStepGain = FixedPointType<24>;
    using Count = FixedPointType<16>;
    using CountAccumulator = FixedPointType<32, int64_t>;
    using CalibrationScale = FixedPointType<30>;
    using AdcCount = FixedPointType<12>;
    using ShuntResistance = FixedPointType<30>;
    using Temperature = FixedPointType<16>;
    using Logarithm = FixedPointType<24>;
    using SpeedPiTypes = PiLoopTypes<Speed, Current, SpeedGain, FixedPointType<28>,
                                    Accumulator, FixedPointType<12>>;
    using CurrentPiTypes = PiLoopTypes<Current, Duty, Gain, FixedPointType<28>,
                                      Accumulator, FixedPointType<12>>;
};

} // namespace libecu
#endif // LIBECU_NUMERIC_PROFILES_HPP
