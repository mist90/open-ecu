/** @file adc_conversion_core.hpp
 * @brief Typed ADC conversion with integer zero calibration and NTC mathematics
 */
#ifndef LIBECU_ADC_CONVERSION_CORE_HPP
#define LIBECU_ADC_CONVERSION_CORE_HPP

#include "profile_math.hpp"

namespace libecu {

template<class Profile>
struct AdcCalibrationT {
    typename Profile::ShuntResistance shunt{};
    uint16_t opamp_gain = 16;
    typename Profile::Voltage reference{};
    uint8_t resolution_bits = 12;
    uint16_t injected_sum = 1;
    uint32_t divider_up_ohms = 169000;
    uint32_t divider_down_ohms = 18000;
    uint32_t ntc_pulldown_ohms = 4700;
    uint32_t ntc_r25_ohms = 10000;
    uint32_t ntc_beta_k = 3435;
};

/** @brief ADC physical-value conversion; hardware supplies raw integer samples. */
template<class Profile>
class AdcConversionCore {
public:
    using Current = typename Profile::Current;
    using Voltage = typename Profile::Voltage;
    using Duty = typename Profile::Duty;
    using Temperature = typename Profile::Temperature;
    using Count = typename Profile::AdcCount;
    using Scale = typename Profile::CalibrationScale;
    using Shunt = typename Profile::ShuntResistance;
    using Gain = typename Profile::Gain;
    using Math = typename Profile::Math;
    using Calibration = AdcCalibrationT<Profile>;
    inline static constexpr Temperature INVALID_TEMPERATURE = number<Temperature>(-273);

    bool configure(const Calibration& calibration) noexcept {
        if (calibration.resolution_bits < 2 || calibration.resolution_bits > 16 || !calibration.injected_sum ||
            !calibration.opamp_gain || calibration.shunt <= Shunt{} || calibration.reference <= Voltage{} ||
            !calibration.divider_down_ohms) return false;
        const uint32_t adc_max = (1u << calibration.resolution_bits) - 1u;
        const uint32_t injected_max = adc_max * calibration.injected_sum;
        if constexpr (!std::is_same<Count, float>::value) {
            if (uint64_t(injected_max) * Count::SCALE > uint64_t(INT32_MAX)) return false;
            if (int64_t(calibration.shunt.raw()) * calibration.opamp_gain > INT32_MAX) return false;
        }
        const Shunt amplified_shunt = scaleInteger<Shunt>(calibration.shunt, calibration.opamp_gain);
        const Gain amperes_per_volt = divideTo<Gain>(number<Gain>(1), amplified_shunt);
        const Scale adc_scale = divideByInteger<Scale>(calibration.reference, injected_max);
        const Gain divider = number<Gain>(uint64_t(calibration.divider_up_ohms) + calibration.divider_down_ohms,
                                         calibration.divider_down_ohms);
        const Scale current_scale = multiplyTo<Scale>(amperes_per_volt, adc_scale);
        const Scale voltage_scale = multiplyTo<Scale>(divider, adc_scale);
        if (!(current_scale > Scale{}) || !(voltage_scale > Scale{}) ||
            !hasHeadroom(amperes_per_volt) || !hasHeadroom(divider) || !hasHeadroom(adc_scale) ||
            !hasHeadroom(current_scale) || !hasHeadroom(voltage_scale) ||
            !hasHeadroom(multiplyTo<Current>(number<Count>(injected_max), current_scale)) ||
            !hasHeadroom(multiplyTo<Voltage>(number<Count>(injected_max), voltage_scale))) return false;
        current_scale_ = current_scale;
        voltage_scale_ = voltage_scale;
        reference_scale_ = adc_scale;
        calibration_ = calibration;
        adc_max_ = adc_max;
        injected_max_ = injected_max;
        return true;
    }

    /** @brief Store a zero point from an integer sum of calibration samples. */
    void setZero(unsigned phase, uint32_t sum, uint32_t samples) noexcept {
        if (phase < 3 && samples) offsets_[phase] = number<Count>(sum, samples);
    }
    /** @brief Compatibility input for a previously calibrated physical offset. */
    void setZeroVoltage(unsigned phase, Voltage voltage) noexcept {
        if (phase >= 3) return;
        if constexpr (std::is_same<Voltage, float>::value) {
            offsets_[phase] = voltage * injected_max_ / calibration_.reference;
        } else {
            offsets_[phase] = Count::fromRatio(int64_t(voltage.raw()) * injected_max_, calibration_.reference.raw());
        }
    }
    Voltage zeroVoltage(unsigned phase) const noexcept {
        return phase < 3 ? multiplyTo<Voltage>(offsets_[phase], reference_scale_) : Voltage{};
    }
    Current current(uint32_t raw, unsigned phase) const noexcept {
        if (phase >= 3) return Current{};
        return multiplyTo<Current>(number<Count>(raw) - offsets_[phase], current_scale_);
    }
    Voltage busVoltage(uint32_t raw) const noexcept {
        return multiplyTo<Voltage>(number<Count>(raw), voltage_scale_);
    }
    Duty throttle(uint32_t raw) const noexcept {
        if (raw > adc_max_) raw = adc_max_;
        return number<Duty>(raw, adc_max_);
    }
    Temperature temperature(uint32_t raw) const noexcept {
        if (!raw || !calibration_.ntc_pulldown_ohms || !calibration_.ntc_r25_ohms || !calibration_.ntc_beta_k)
            return INVALID_TEMPERATURE;
        if (raw >= adc_max_) raw = adc_max_ - 1;
        const uint64_t numerator = uint64_t(calibration_.ntc_pulldown_ohms) * (adc_max_ - raw);
        const uint64_t denominator = uint64_t(calibration_.ntc_r25_ohms) * raw;
        using Inverse = typename Profile::Fraction;
        const auto log_ratio = Math::logRatio(numerator, denominator);
        const Inverse inverse_temperature = number<Inverse>(100, 29815)
                                         + divideByInteger<Inverse>(log_ratio, calibration_.ntc_beta_k);
        if (inverse_temperature <= Inverse{}) return INVALID_TEMPERATURE;
        return divideTo<Temperature>(number<Temperature>(1), inverse_temperature)
             - number<Temperature>(27315, 100);
    }

private:
    template<class T>
    static bool hasHeadroom(T value) noexcept {
        if constexpr (std::is_same<T, float>::value) return std::isfinite(value);
        else return value.raw() != std::numeric_limits<typename T::StorageType>::max() &&
                    value.raw() != std::numeric_limits<typename T::StorageType>::min();
    }
    Calibration calibration_{};
    uint32_t adc_max_ = 4095;
    uint32_t injected_max_ = 4095;
    Count offsets_[3]{};
    Scale current_scale_{};
    Scale voltage_scale_{};
    Scale reference_scale_{};
};

} // namespace libecu
#endif // LIBECU_ADC_CONVERSION_CORE_HPP
