/** @file adc_interface.hpp
 * @brief Raw ADC hardware interface with a profile-owned physical conversion core
 */
#ifndef LIBECU_ADC_INTERFACE_HPP
#define LIBECU_ADC_INTERFACE_HPP

#include "pwm_interface.hpp"
#include "ecu_config.hpp"
#include "adc_conversion_core.hpp"
#include <cmath>
#include <cstdint>

uint32_t time_us();

namespace libecu {

constexpr float TEMPERATURE_INVALID_C = -273.0f;

/** @brief Physical calibration at the application/configuration boundary. */
struct CurrentSensorCalibration {
    float shunt_resistance_ohms;
    float opamp_gain;
    float adc_reference_voltage;
    uint32_t adc_resolution_bits;
    float offset_voltage_u = 0.0f;
    float offset_voltage_v = 0.0f;
    float offset_voltage_w = 0.0f;
    uint16_t injected_sample_sum = 1;
};
struct VoltageSensorParameters { float r_up; float r_down; };

/** @brief NTC from Vref to the signal, with a fixed pulldown to ground. */
struct TemperatureSensorParameters {
    float r_pulldown_ohms = 4700.0f;
    float ntc_r25_ohms = 10000.0f;
    float ntc_beta_k = 3435.0f;
};

/**
 * @brief Raw hardware plus typed current/voltage/temperature conversion
 *
 * Injected current and bus readings share the configured sum scale; regular
 * NTC/throttle readings retain the physical ADC resolution. The float-facing
 * methods adapt existing callers; readNative* exposes the conversion core's
 * types directly to a profile-parametric control loop.
 */
class AdcInterface {
public:
    using Core = AdcConversionCore<ActiveNumericProfile>;
    using NativeCurrent = Core::Current;
    using NativeVoltage = Core::Voltage;
    virtual ~AdcInterface() = default;

    /** @brief Install calibration transactionally; hardware initialization is separate. */
    bool initialize(const CurrentSensorCalibration& calibration, const VoltageSensorParameters& divider,
                    const TemperatureSensorParameters& temperature = TemperatureSensorParameters{}) noexcept {
        if (calibration.adc_resolution_bits < 2 || calibration.adc_resolution_bits > 16) return false;
        Core::Calibration typed{};
        if (!tryNumber(calibration.shunt_resistance_ohms, typed.shunt) ||
            !tryNumber(calibration.adc_reference_voltage, typed.reference) ||
            !tryNumber(calibration.opamp_gain, typed.opamp_gain) ||
            !unsignedParameter(divider.r_up, typed.divider_up_ohms) ||
            !unsignedParameter(divider.r_down, typed.divider_down_ohms)) return false;
        typed.resolution_bits = static_cast<uint8_t>(calibration.adc_resolution_bits);
        typed.injected_sum = calibration.injected_sample_sum;
        typed.ntc_pulldown_ohms = typed.ntc_r25_ohms = typed.ntc_beta_k = 0;
        const bool disabled_ntc = temperature.r_pulldown_ohms <= 0.0f ||
                                  temperature.ntc_r25_ohms <= 0.0f || temperature.ntc_beta_k <= 0.0f;
        if (!disabled_ntc &&
            (!unsignedParameter(temperature.r_pulldown_ohms, typed.ntc_pulldown_ohms) ||
             !unsignedParameter(temperature.ntc_r25_ohms, typed.ntc_r25_ohms) ||
             !unsignedParameter(temperature.ntc_beta_k, typed.ntc_beta_k))) return false;
        Core next;
        if (!next.configure(typed)) return false;
        const float offsets[3] = {calibration.offset_voltage_u, calibration.offset_voltage_v,
                                  calibration.offset_voltage_w};
        for (unsigned phase = 0; phase < 3; ++phase) {
            NativeVoltage offset{};
            if (!tryNumber(offsets[phase], offset)) return false;
            if (offset < NativeVoltage{} || offset > typed.reference) return false;
            next.setZeroVoltage(phase, offset);
        }
        core_ = next;
        calibration_ = calibration;
        temp_params_ = temperature;
        initialized_ = true;
        return true;
    }

    /** @return Injected phase sum, including injected_sample_sum scaling. */
    virtual uint32_t getRawAdcValue(PwmChannel channel) = 0;
    /** @return Injected DC bus sum at the same scale as phase readings. */
    virtual uint32_t getRawAdcValue() = 0;
    /** @return Regular NTC ADC reading; zero on a failed/open reading. */
    virtual uint32_t getRawTemperatureAdcValue() = 0;

    /** @return Signed phase current, positive out of the phase into its low-side shunt. */
    float convertAdcToCurrent(uint32_t raw, PwmChannel channel) noexcept {
        return numberCast<float>(core_.current(raw, static_cast<unsigned>(channel)));
    }
    /** @return DC bus voltage in volts. */
    float convertAdcToVoltage(uint32_t raw) noexcept { return numberCast<float>(core_.busVoltage(raw)); }
    /**
     * @return NTC temperature, or TEMPERATURE_INVALID_C on open/invalid input
     * @note This divider cannot distinguish an open NTC from a very cold one.
     *       The sentinel is diagnostic and does not trigger the thermal cutout.
     */
    float convertAdcToTemperature(uint32_t raw) const noexcept {
        return numberCast<float>(core_.temperature(raw));
    }
    /** @return Normalized regular-channel throttle reading. */
    float convertAdcToThrottle(uint32_t raw) const noexcept { return numberCast<float>(core_.throttle(raw)); }
    Core::Duty convertNativeThrottle(uint32_t raw) const noexcept { return core_.throttle(raw); }

    NativeVoltage readNativeBusVoltage() noexcept { return core_.busVoltage(getRawAdcValue()); }
    Core::Temperature readNativeTemperature() noexcept { return core_.temperature(getRawTemperatureAdcValue()); }
    void readNativeCurrents(NativeCurrent& u, NativeCurrent& v, NativeCurrent& w) noexcept {
        u = core_.current(getRawAdcValue(PwmChannel::PHASE_U), 0);
        v = core_.current(getRawAdcValue(PwmChannel::PHASE_V), 1);
        w = core_.current(getRawAdcValue(PwmChannel::PHASE_W), 2);
    }
    float readBusVoltage() noexcept { return numberCast<float>(readNativeBusVoltage()); }
    float readTemperature() noexcept { return convertAdcToTemperature(getRawTemperatureAdcValue()); }
    float readPhaseCurrent(PwmChannel channel) noexcept {
        return convertAdcToCurrent(getRawAdcValue(channel), channel);
    }
    void readAllCurrents(float& u, float& v, float& w) noexcept {
        NativeCurrent a{}, b{}, c{};
        readNativeCurrents(a, b, c);
        u = numberCast<float>(a); v = numberCast<float>(b); w = numberCast<float>(c);
    }

    /** @brief Calibrate all three offsets from integer ADC sums while the bridge is inactive. */
    bool calibrateZeroOffset() noexcept {
        if (!initialized_) return false;
        constexpr uint32_t samples = 100;
        uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
        for (uint32_t sample = 0; sample < samples; ++sample) {
            const uint32_t start = time_us();
            while (time_us() - start < 100) {}
            sum_u += getRawAdcValue(PwmChannel::PHASE_U);
            sum_v += getRawAdcValue(PwmChannel::PHASE_V);
            sum_w += getRawAdcValue(PwmChannel::PHASE_W);
        }
        core_.setZero(0, sum_u, samples);
        core_.setZero(1, sum_v, samples);
        core_.setZero(2, sum_w, samples);
        calibration_.offset_voltage_u = numberCast<float>(core_.zeroVoltage(0));
        calibration_.offset_voltage_v = numberCast<float>(core_.zeroVoltage(1));
        calibration_.offset_voltage_w = numberCast<float>(core_.zeroVoltage(2));
        return true;
    }

    const CurrentSensorCalibration& getCalibration() const noexcept { return calibration_; }
    const TemperatureSensorParameters& getTemperatureParams() const noexcept { return temp_params_; }

private:
    static bool unsignedParameter(float value, uint32_t& result) noexcept {
        if (!std::isfinite(value) || value < 0.0f || value >= 4294967296.0f) return false;
        const uint32_t integer = static_cast<uint32_t>(value);
        if (float(integer) != value) return false;
        result = integer;
        return true;
    }
    Core core_;
    CurrentSensorCalibration calibration_{};
    TemperatureSensorParameters temp_params_{};
    bool initialized_ = false;
};

} // namespace libecu
#endif // LIBECU_ADC_INTERFACE_HPP
