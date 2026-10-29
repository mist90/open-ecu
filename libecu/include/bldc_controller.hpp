/**
 * @file bldc_controller.hpp
 * @brief High-level BLDC motor controller integrating all components
 */

#ifndef LIBECU_BLDC_CONTROLLER_HPP
#define LIBECU_BLDC_CONTROLLER_HPP

#include <cstddef>
#include <cstdint>
#include <array>

#include "pwm_interface.hpp"
#include "hall_interface.hpp"
#include "adc_interface.hpp"
#include "motor_algorithm.hpp"
#include "six_step_algorithm.hpp"
#include "foc_algorithm.hpp"
#include "pi_float_interface.hpp"
#include "ecu_config.hpp"
#include "motor_pll.hpp"
#include "open_loop_phase.hpp"
#include "current_loop_tuning.hpp"
#include "hall_monitor.hpp"

namespace libecu {

/**
 * @brief Motor control mode (mechanical/commutation strategy)
 */
enum class ControlMode : uint8_t {
    OPEN_LOOP = 0,              ///< Open loop control (timing-based, no sensors)
    CLOSED_LOOP_VELOCITY = 1,   ///< Closed loop velocity control with PI + Hall sensors
    CLOSED_LOOP_TORQUE = 2      ///< Closed loop torque control (fixed duty/current + Hall sensors)
};

// ElectricMode and DriveAlgorithm live in algorithms/motor_algorithm.hpp -
// the algorithms themselves branch on them, and they would otherwise have to
// include this header to see them.

/**
 * @brief Motor control parameters
 */
struct MotorControlParams {
    /**
     * @brief Number of motor poles
     *
     * There are `num_poles / 2` electrical revolutions per mechanical one and
     * 6 commutation steps per electrical revolution, so the conversion the
     * whole controller uses is `steps_per_mech_rev = num_poles * 3`. MOTOR_1
     * has 40 poles, hence 120 steps per mechanical revolution, hence
     * RPS = steps_per_second / 120.
     *
     * Moved here from CommutationController, which no longer exists.
     */
    uint8_t num_poles;
    float max_duty_cycle;     ///< Maximum duty cycle (0.0 to 1.0)
    float max_current;        ///< Maximum motor current (A)
    float min_current;        ///< Minimum negative current (A)
    float max_voltage;        ///< Maximum bus voltage (V), in case of overvoltage (it means battery damage and recuperation) ECU goes to NEUTRAL mode.

    /**
     * @brief Thermal cut-out, degrees Celsius
     *
     * Above this the drive goes to NEUTRAL, the same way over-voltage does.
     * The trip does not latch and there is no hysteresis: while the sensor
     * still reads hot every 100 Hz tick re-asserts NEUTRAL, so re-arming with
     * AT+DMODE only takes once it has actually cooled below the limit.
     *
     * Checked against AdcInterface::readTemperature(), which returns
     * TEMPERATURE_INVALID_C when the NTC reads open - that never trips, so a
     * disconnected sensor removes the protection silently. See
     * convertAdcToTemperature() for why the divider cannot tell "open" from
     * "cold". 0 or negative falls back to 100 C.
     */
    float max_temperature_c;
    float max_speed_rps;      ///< Maximum speed in RPS
    float acceleration_rate;  ///< Acceleration rate (RPS/s), 0 = disabled
    float target_speed_lpf_alpha; ///< LPF alpha for target speed (0.0-1.0), 0 = disabled, 1.0 = no filtering
    float measured_speed_lpf_alpha; ///< LPF alpha for target speed (0.0-1.0), 0 = disabled, 1.0 = no filtering
    /**
     * LPF alpha for the *reported* measured current, applied per PWM cycle.
     *
     * The raw current is one ADC sample per PWM cycle of a chopped waveform, so
     * reading it at the 100 Hz telemetry rate aliases badly - consecutive +TM
     * lines can differ by an amp with nothing actually changing. This filter
     * only affects what is reported; the control loop and the +OSC capture keep
     * using the raw value.
     *
     * 0.005 at 20 kHz is a 10 ms time constant, matched to the telemetry period.
     * 0 or 1 disables it.
     */
    float measured_current_lpf_alpha;

    /**
     * @brief PWM cycles to blank the current measurement for after a commutation
     *
     * For the first cycles after the phases change, the outgoing phase is still
     * freewheeling through the body diodes, so the shunt of the nominal DOWN
     * phase is not carrying the phase current. Those samples are wrong by up to
     * an amp, and handing them to the current PI turns a measurement artefact
     * into real duty modulation - which is torque ripple the motor can be heard
     * and felt making. During the blanking window the duty is held and the PI
     * is not updated, so the integrator cannot wind up on the bad samples.
     *
     * 0 disables blanking.
     */
    uint8_t current_blanking_cycles;
    bool useInverseCommTable; ///< Use inverse six-step commutation table
    PiParameters pi_voltage_mode; ///< Velocity PI parameters for voltage mode (outputs duty cycle)
    PiParameters pi_current_mode; ///< Velocity PI parameters for current mode (outputs current)
    PiParameters pi_current_regulator; ///< Current PI parameters for current mode (outputs duty cycle)

    /**
     * @brief Which electrical algorithm the drive starts in
     *
     * Orthogonal to ElectricMode - see the table in motor_algorithm.hpp.
     * SIX_STEP is the default because it is the mode with years of bench time
     * behind it; FOC is selected at runtime with AT+ALGO.
     */
    DriveAlgorithm algorithm;

    /// @brief FOC modulation limits, angle offset and feed-forward configuration
    FocParams foc;

    /**
     * @brief d/q current regulator gains for FOC (output in volts)
     *
     * Only kp and ki are used. These are NOT the same numbers as
     * pi_current_regulator: that one outputs duty and sees two phases in
     * series, this one outputs volts and sees one. Use tuneFocCurrentPi().
     */
    PiParameters foc_current_regulator;
};

/**
 * @brief Motor status information
 */
template<class P>
struct MotorStatusT {
    /**
     * @brief Measured motor speed (RPS), signed relative to the commanded direction
     *
     * Positive when the rotor turns the way DriveMode asks, negative when it
     * turns against it - so in normal running it reads the same as it always
     * did. The sign is what lets the velocity loop tell "too fast" from
     * "going the wrong way"; a magnitude cannot, and feeding it one made the
     * drive run away when the shaft was turned backwards. See the long note in
     * BldcController::update().
     */
    typename P::Speed current_speed_rps;
    typename P::Speed target_speed_rps;   ///< Target motor speed (RPS)
    typename P::Duty duty_cycle;         ///< Current duty cycle
    typename P::Current target_current;     ///< Target motor current (A)
    typename P::Current measured_current;   ///< Raw per-PWM-cycle sample (A)
    typename P::Current measured_current_filtered; ///< Telemetry LPF (A)
    typename P::Voltage bus_voltage;         ///< Measured bus voltage (V)
    /**
     * @brief Measured NTC temperature (deg C), sampled at the 100 Hz loop rate
     *
     * TEMPERATURE_INVALID_C (-273) means the sensor read open or the ADC
     * conversion did not complete - not a cold motor.
     */
    typename P::Temperature temperature_c;
    typename P::Angle pll_angle;          ///< Rotor angle from PLL (steps, 0-6)
    int8_t current_pi_saturation; ///< +1/-1 if duty is clamped high/low, else 0 (for speed PI anti-windup)
    uint8_t target_position;   ///< Driven motor position
    uint8_t measured_position;   ///< Measured motor position
    bool is_running;          ///< Motor running status
    ControlMode control_mode;   ///< Current control mode (mechanical)
    ElectricMode electric_mode; ///< Current electric mode (electrical)

    /// @name FOC state, zero while six-step is selected
    ///@{
    DriveAlgorithm algorithm; ///< Electrical algorithm currently running
    typename P::Current i_d;                ///< Measured d-axis current (A)
    typename P::Current i_q;                ///< Measured q-axis current (A)
    typename P::Voltage v_d;                ///< Applied d-axis voltage (V)
    typename P::Voltage v_q;                ///< Applied q-axis voltage (V)
    typename P::Angle angle_e_rad;        ///< Electrical angle used by the algorithm (rad)
    typename P::Duty duty_u;             ///< Phase U duty actually applied
    typename P::Duty duty_v;             ///< Phase V duty actually applied
    typename P::Duty duty_w;             ///< Phase W duty actually applied
    ///@}
};

using MotorStatus = MotorStatusT<FloatProfile>;
using NativeMotorStatus = MotorStatusT<ActiveNumericProfile>;

/** @brief Convert a completed status snapshot at a diagnostics boundary. */
template<class OutProfile, class InProfile>
MotorStatusT<OutProfile> motorStatusCast(const MotorStatusT<InProfile>& in) noexcept {
    MotorStatusT<OutProfile> out{};
    out.current_speed_rps = numberCast<typename OutProfile::Speed>(in.current_speed_rps);
    out.target_speed_rps = numberCast<typename OutProfile::Speed>(in.target_speed_rps);
    out.duty_cycle = numberCast<typename OutProfile::Duty>(in.duty_cycle);
    out.target_current = numberCast<typename OutProfile::Current>(in.target_current);
    out.measured_current = numberCast<typename OutProfile::Current>(in.measured_current);
    out.measured_current_filtered = numberCast<typename OutProfile::Current>(in.measured_current_filtered);
    out.bus_voltage = numberCast<typename OutProfile::Voltage>(in.bus_voltage);
    out.temperature_c = numberCast<typename OutProfile::Temperature>(in.temperature_c);
    out.pll_angle = numberCast<typename OutProfile::Angle>(in.pll_angle);
    out.current_pi_saturation = in.current_pi_saturation;
    out.target_position = in.target_position;
    out.measured_position = in.measured_position;
    out.is_running = in.is_running;
    out.control_mode = in.control_mode;
    out.electric_mode = in.electric_mode;
    out.algorithm = in.algorithm;
    out.i_d = numberCast<typename OutProfile::Current>(in.i_d);
    out.i_q = numberCast<typename OutProfile::Current>(in.i_q);
    out.v_d = numberCast<typename OutProfile::Voltage>(in.v_d);
    out.v_q = numberCast<typename OutProfile::Voltage>(in.v_q);
    out.angle_e_rad = numberCast<typename OutProfile::Angle>(in.angle_e_rad);
    out.duty_u = numberCast<typename OutProfile::Duty>(in.duty_u);
    out.duty_v = numberCast<typename OutProfile::Duty>(in.duty_v);
    out.duty_w = numberCast<typename OutProfile::Duty>(in.duty_w);
    return out;
}

/**
 * @brief High-level BLDC motor controller
 */
class BldcController {
public:
    using Profile = ActiveNumericProfile;
    using Current = Profile::Current;
    using Voltage = Profile::Voltage;
    using Speed = Profile::Speed;
    using Duty = Profile::Duty;
    using Fraction = Profile::Fraction;
    using Temperature = Profile::Temperature;
    using Accumulator = Profile::Accumulator;
    /**
     * @brief Constructor
     * @param pwm_interface PWM interface
     * @param hall_interface Hall sensor interface
     * @param params Motor control parameters (includes PI parameters for both modes)
     * @param adc_interface ADC interface for current sensing (optional, nullptr for voltage mode only)
     */
    BldcController(
        PwmInterface& pwm_interface,
        HallInterface& hall_interface,
        const MotorControlParams& params,
        AdcInterface* adc_interface = nullptr
    ) noexcept;

    /**
     * @brief Initialize motor controller
     * @return true if initialization successful
     */
    bool initialize() noexcept;

    /**
     * @brief Update motor control (call at control frequency)
     */
    void update() noexcept;

    /**
     * @brief Set mechanical-RPS demand for velocity control or the open-loop field
     * @param speed_rps Target speed in RPS
     */
    void setTargetSpeed(float speed_rps) noexcept;
    /** @brief Set a native throttle/speed-loop request without float conversion. */
    void setNativeTargetSpeed(Speed speed_rps) noexcept;

    /**
     * @brief Set duty cycle for open-loop control
     * @param duty_cycle Duty cycle (0.0 to 1.0)
     *                   0.0 = 0V output (no torque)
     *                   1.0 = maximum voltage output (full torque)
     */
    void setDutyCycle(float duty_cycle) noexcept;

    /**
     * @brief Set target current for torque control
     * @param current_a Target current in Amperes (min_current to max_current)
     *                  Clamped to [min_current, max_current] range
     */
    void setCurrent(float current_a) noexcept;

    /**
     * @brief Set control mode (mechanical/commutation strategy)
     * @param mode Control mode
     */
    void setControlMode(ControlMode mode) noexcept;

    /**
     * @brief Set electric mode (electrical control strategy)
     * @param mode Electric mode
     * @return false if the target mode's tuning cannot be represented; the old mode remains active
     */
    bool setElectricMode(ElectricMode mode) noexcept;

    /**
     * @brief Select the electrical algorithm (six-step or FOC)
     *
     * Switching hands the bridge over: the outgoing algorithm's state is
     * cleared and the incoming one's onEnter() runs, which is what puts the
     * inverter into the switching configuration that algorithm assumes. Safe
     * to call while running, but it is a discontinuity in the applied voltage
     * - do it at standstill or in NEUTRAL unless you are deliberately testing
     * the transition.
     *
     * @param algorithm SIX_STEP or FOC
     */
    void setAlgorithm(DriveAlgorithm algorithm) noexcept;

    /// @brief Currently selected electrical algorithm
    DriveAlgorithm getAlgorithm() const noexcept;

    /**
     * @brief Set the FOC d/q current regulator gains
     * @param kp Proportional gain (V/A)
     * @param ki Integral gain (V/A/s)
     */
    bool setFocCurrentPi(float kp, float ki) noexcept;

    /// @brief Get the FOC d/q current regulator gains
    void getFocCurrentPi(float& kp, float& ki) const noexcept;

    /**
     * @brief Set the FOC electrical angle offset
     *
     * The offset between the PLL's Hall-derived step angle and the rotor d
     * axis. Default is -60 electrical degrees; see the derivation in
     * foc_algorithm.hpp. Expect to trim this on the bench.
     *
     * @param degrees_e Offset in electrical degrees
     */
    bool setFocAngleOffsetDeg(float degrees_e) noexcept;

    /// @brief Get the FOC electrical angle offset, in electrical degrees
    float getFocAngleOffsetDeg() const noexcept;

    /**
     * @brief Set motor drive mode
     * @param mode Rotation mode
     */
    void setDriveMode(DriveMode mode) noexcept;

    /**
     * @brief Set the target-speed slew rate limit
     *
     * Applies to the velocity command only - it is the rate at which the
     * *setpoint* is allowed to move, not a torque limit, so it does nothing in
     * TORQUE or OPEN_LOOP mode. Defaults to BLDC_MAX_ACCELERATION.
     *
     * Worth keeping below what the PLL can follow: the steady-state tracking
     * error during a ramp is alpha/ki steps, and past 1.5 steps the commutation
     * lead goes negative. See the tuning notes under AT+PLLID.
     *
     * @param rate_rps2 Rate in RPS per second, 0 disables the limiter
     */
    void setAccelerationRate(float rate_rps2) noexcept;

    /// @brief Get the target-speed slew rate limit (RPS/s), 0 = disabled
    float getAccelerationRate() const noexcept;

    /**
     * @brief Set speed PI controller gains (kp, ki)
     * @param kp Proportional gain
     * @param ki Integral gain
     *
     * Preserves existing output limits.
     * @return false if the selected numerical profile cannot represent the tuning
     */
    bool setSpeedPi(float kp, float ki) noexcept;

    /**
     * @brief Set current PI controller gains (kp, ki)
     * @param kp Proportional gain
     * @param ki Integral gain
     *
     * Preserves existing output limits.
     * @return false if the selected numerical profile cannot represent the tuning
     */
    bool setCurrentPi(float kp, float ki) noexcept;

    /**
     * @brief Get PLL internal state snapshot (for telemetry)
     * @return PllInfo struct snapshot, taken atomically under CriticalSection
     */
    MotorPLL::PllInfo getPllInfo() const noexcept;

    /**
     * @brief Hall sensor health snapshot
     *
     * The monitor latches; clear it with clearHallFault() once the wiring has
     * been dealt with.
     */
    HallMonitor::Info getHallInfo() const noexcept;

    /// @brief Atomic Hall fault only, without telemetry conversion or floating-point work
    HallFault getHallFault() const noexcept;

    /// @brief Release a latched Hall fault
    void clearHallFault() noexcept;

    /// @brief Configure the Hall health monitor
    bool setHallMonitorParams(const HallMonitorParams& params) noexcept;

    /**
     * @brief Set PLL base PI gains (kp_base, ki_base)
     * @param kp Base proportional gain
     * @param ki Base integral gain
     */
    bool setPllGains(float kp, float ki) noexcept;

    /**
     * @brief Get current drive mode
     * @return Current drive mode
     */
    DriveMode getDriveMode() const noexcept;

    /**
     * @brief Get speed PI controller gains
     * @param kp Output: proportional gain
     * @param ki Output: integral gain
     */
    void getSpeedPiGains(float& kp, float& ki) const noexcept;

    /**
     * @brief Get current PI controller gains
     * @param kp Output: proportional gain
     * @param ki Output: integral gain
     */
    void getCurrentPiGains(float& kp, float& ki) const noexcept;

    /**
     * @brief Get PLL base PI gains
     * @param kp Output: base proportional gain
     * @param ki Output: base integral gain
     */
    void getPllBaseGains(float& kp, float& ki) const noexcept;

    /**
     * @brief Start motor
     */
    void start() noexcept;

    /**
     * @brief Stop motor
     */
    void stop() noexcept;

    /**
     * @brief Get motor status
     * @return Current motor status
     */
    MotorStatus getStatus() const noexcept;

    /** @brief Native snapshot for the PWM callback; caller owns synchronization. */
    const NativeMotorStatus& getNativeStatus() const noexcept { return status_; }

    /**
     * @brief Get motor control parameters (configured limits)
     * @return Reference to the parameters the controller was configured with
     */
    const MotorControlParams& getParams() const noexcept { return params_; }

    void hallSensorInterruptHandler() noexcept;

    /**
     * @brief PWM interrupt handler for high-frequency current control loop (20kHz)
     * Call this from TIM1 update interrupt when current control mode is active.
     * This runs the inner current control loop at PWM frequency.
     */
    void pwmInterruptHandler() noexcept;



private:
    /// @brief The algorithm currently selected, never null
    MotorAlgorithm& algorithm() noexcept;
    const MotorAlgorithm& algorithm() const noexcept;

    // Component references
    PwmInterface& pwm_interface_;
    HallInterface& hall_interface_;
    AdcInterface* adc_interface_;

    // Owned components
    HallMonitor hall_monitor_;
    MotorPLL motor_pll_;
    OpenLoopPhase<Profile, ActiveTiming::CURRENT_HZ> open_loop_;
    PiFloatInterface<ActivePiProfile::SpeedPiTypes, ActiveTiming::SPEED_HZ> pi_speed_controller_;

    // Electrical algorithms. Both are constructed and kept alive; only the one
    // selected by algorithm_ is ticked. Keeping the unselected one around
    // costs a few hundred bytes and makes switching back an O(1) operation
    // with no allocation, which matters because A/B-ing FOC against six-step
    // on the same run is the whole point of having both.
    SixStepAlgorithm six_step_;
    FocAlgorithm foc_;
    DriveAlgorithm algorithm_;

    // Configuration
    MotorControlParams params_;
    struct NativeLimits {
        Duty max_duty{};
        Current max_current{}, min_current{};
        Voltage max_voltage{};
        Temperature max_temperature{};
        Speed max_speed{};
        Fraction target_alpha{}, speed_alpha{}, current_alpha{};
    } limits_;
    bool parameters_valid_ = false;
    bool acceleration_enabled_ = false;

    // State variables
    NativeMotorStatus status_{};
    volatile DriveMode dmode_;
    bool initialized_;

    // Deferred Hall update (debounce via PWM ISR re-read)
    volatile bool hall_update_pending_;      ///< Set by Hall ISR, processed by PWM ISR

    /**
     * @brief Voltage-mode duty request, 0.0 to 1.0
     *
     * Kept separate from MotorStatus::duty_cycle, which is what the algorithm
     * *reports*. The two agree in six-step voltage mode, but FOC reports a
     * modulation index rather than the command it was given, so a single field
     * serving as both would feed the report back in as the next command.
     */
    Duty duty_command_;

    // Target speed filtering state (LPF → slew rate limiter cascade)
    Speed filtered_target_speed_;            ///< LPF-filtered target speed
    Speed filtered_measured_speed_;
    Current filtered_measured_current_;       ///< LPF-filtered current, telemetry only
    Accumulator limited_target_speed_;        ///< Retains sub-LSB ramp increments
    Accumulator acceleration_step_;           ///< Maximum target-speed change per control tick



    /**
     * @brief Hold the velocity loop while the drive is parked in NEUTRAL
     *
     * Keeps the speed PI and the slew limiter from integrating against a
     * setpoint the disabled bridge cannot follow, and leaves both in the state
     * a bumpless re-engagement needs. See the note at the call site.
     */
    void holdVelocityLoop() noexcept;

    /// @brief Refresh physical Hall after a mode change/parking; caller holds CriticalSection.
    void synchronizeHallPosition(bool discard_tracking = false) noexcept;

    /**
     * @brief Apply acceleration/deceleration limits to target speed
     *
     * Implements a slew rate limiter that restricts how fast target_speed can change.
     * Stores previous limited value internally - does NOT use measured speed.
     *
     * @param target_speed Desired target speed
     * @return Rate-limited target speed
     */
    Speed applyAccelerationLimit(Speed target_speed) noexcept;

};

} // namespace libecu

#endif // LIBECU_BLDC_CONTROLLER_HPP
