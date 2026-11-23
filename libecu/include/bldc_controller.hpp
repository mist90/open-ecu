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
#include "motor_io.hpp"
#include "current_loop_tuning.hpp"
#include "hall_monitor.hpp"
#include "traction_acceleration.hpp"

namespace libecu {
class CriticalSection;

/**
 * @brief Motor control parameters
 */
struct MotorControlParams {
    /**
     * @brief Number of motor poles
     *
     * There are `num_poles / 2` electrical revolutions per mechanical one and
     * 6 commutation steps per electrical revolution, so the conversion the
     * whole controller uses is `steps_per_mech_rev = num_poles * 3`.
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
     * The raw current is one current-frame value per PWM cycle of a chopped waveform, so
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
    PiParameters speed_pi; ///< Velocity PI, outputs current in amperes
    PiParameters pi_current_regulator; ///< Six-step current PI, outputs duty

    /**
     * @brief Which electrical algorithm the drive starts in
     *
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
    float traction_acceleration_rps2 = 0.0f; ///< Mechanical feedback ceiling; 0 leaves speed PI unchanged.
};

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

    /** @brief Controller-bound rate command; default construction is invalid. */
    class PreparedTargetSpeed {
    public:
        PreparedTargetSpeed() noexcept = default;
    private:
        friend class BldcController;
        const BldcController* owner_ = nullptr;
        Speed limited_speed_{};
        MotorField::PreparedRate rate_{};
    };

    /**
     * @brief Constructor
     * @param pwm_interface PWM interface
     * @param hall_interface Hall sensor interface
     * @param params Motor and regulator parameters
     * @param adc_interface Current acquisition; without it the generic PWM handler does no work
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
     * @brief Run the slow service tick: sample speed, update the ramp and limits, then take one speed PI step.
     */
    void update() noexcept;
    /** @brief Take one intermediate speed PI step. Same SysTick owner as update(); never call both on one tick.
     * Uses the last slow sample and ceilings; without a valid velocity command there is no work. */
    void updateSpeedPi() noexcept;

    /**
     * @brief Set mechanical-RPS demand for velocity control or the open-loop field
     * @param speed_rps Target speed in RPS
     */
    void setTargetSpeed(float speed_rps) noexcept;
    /** @brief Set a native throttle/speed-loop request without float conversion. */
    void setNativeTargetSpeed(Speed speed_rps) noexcept;

    /**
     * @brief Clamp and convert a demand without masking interrupts or reading live drive state
     *
     * Motor poles and speed limit are immutable, so a successful command may be
     * cached across gain/mode changes. Preparation neither enables nor changes the drive.
     * @param speed_rps Native mechanical RPS
     * @param prepared Controller-bound output; invalidated on failure
     * @return true if the demand has a representable DDS rate
     */
    bool prepareNativeTargetSpeed(Speed speed_rps, PreparedTargetSpeed& prepared) const noexcept;

    /**
     * @brief Atomically publish a prepared rate and target, preserving the current phase
     * @param prepared Command produced by this controller
     * @return false for an invalid/foreign command or a drive currently in NEUTRAL
     */
    bool commitNativeTargetSpeed(const PreparedTargetSpeed& prepared) noexcept;
    /**
     * @brief Publish inside an existing safety transaction without nesting the mask
     * @param guard Live guard owned by the caller for the entire call
     */
    bool commitNativeTargetSpeed(const PreparedTargetSpeed& prepared, const CriticalSection& guard) noexcept;

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
    /** @brief Next active FOC's discarded phase, or -1 for other modes.
     *  @note Read under current-ISR ownership, before pwmInterruptHandler().
     */
    int currentReconstructedPhase() const noexcept {
        return io_.in.algorithm == DriveAlgorithm::FOC && io_.in.drive_mode != DriveMode::NEUTRAL
            ? int(foc_.reconstructedPhase()) : -1;
    }
    /** @brief Set this current frame's FOC reconstruction choice without changing its raw samples.
     *  @return false outside active FOC or for a phase outside U/V/W (0/1/2).
     *  @note Caller qualifies both retained phases and owns the serialized current ISR.
     *        Call immediately before pwmInterruptHandler();
     *        if the frame is aborted, park/reset the drive.
     */
    bool setCurrentReconstructedPhase(unsigned phase) noexcept {
        return io_.in.algorithm == DriveAlgorithm::FOC && io_.in.drive_mode != DriveMode::NEUTRAL &&
               foc_.setNextReconstructedPhase(phase);
    }

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
     * Foreground setter for the velocity loop. Preserves the last
     * command at the last sampled error, error history, and output/integral
     * limits. Normal integration continues on the next speed tick.
     * @return false without changing the tuning if it is unrepresentable or a
     * live bumpless transfer is unavailable (saturation, integral limit or
     * changed operating point); the caller may retry on a later tick.
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
    // One volatile scalar load is atomic on supported Cortex-M targets. Keep
    // this in the caller's optimization domain, including size-optimized code.
    DriveMode getDriveMode() const noexcept { return io_.in.drive_mode; }

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

    /** @brief Live native I/O; read after PWM completion or under a reader's CriticalSection. */
    const MotorIo& io() const noexcept { return io_; }

    /**
     * @brief Get motor control parameters (configured limits)
     * @return Reference to the parameters the controller was configured with
     */
    const MotorControlParams& getParams() const noexcept { return params_; }

    void hallSensorInterruptHandler() noexcept;

    /**
     * @brief Process one qualified current frame at ActiveTiming::CURRENT_HZ.
     * The port calls this once per PWM period after capture and protection,
     * regardless of the acquisition mechanism or number of samples in the frame.
     * Currents and the separately cached bus voltage must be ready to read.
     */
    bool pwmInterruptHandler() noexcept;
#if defined(LIBECU_PWM_BUS_LOOKAHEAD) && LIBECU_PWM_BUS_LOOKAHEAD
    /** @brief Prepare exact normalization for an available auxiliary DMA result outside the PWM IRQ. */
    void prepareFocBus(typename Profile::Voltage voltage) noexcept { foc_.prepareBus(voltage); }
#endif
    /** @brief Same single PWM tick with statically bound drivers. An exclusive
     * carrier can commit directly to its preloads and pass the decoded Hall
     * sector sampled at entry (0xff is invalid). */
    static constexpr uint8_t HALL_NOT_SAMPLED = 0xfe;
    template<class Adc, class Pwm>
    bool pwmInterruptHandler(Adc& adc, Pwm& pwm, uint8_t hall = HALL_NOT_SAMPLED) noexcept;
    /** A protection IRQ can cancel this tick without touching rotor or command state. */
    void cancelPwmTick() noexcept { pwm_cancelled_ = true; }
    void parkFromCarrier() noexcept;

private:
    /// @brief The algorithm currently selected, never null
    MotorAlgorithm& algorithm() noexcept;

    // Component references
    PwmInterface& pwm_interface_;
    HallInterface& hall_interface_;
    AdcInterface* adc_interface_;

    // Owned components
    MotorIo io_;
    volatile bool pwm_cancelled_ = false; // Only BRK can preempt the carrier.
    using SpeedPi = PiFloatInterface<ActivePiProfile::SpeedPiTypes, ActiveTiming::SPEED_PI_HZ>;
    SpeedPi pi_speed_controller_;
    struct {
        SpeedPi::Output integral_ceiling{}, traction_ceiling{};
    } speed_pi_limits_; // Recomputed once per measured acceleration sample.
    volatile bool speed_pi_ready_ = false; // A park in the higher-priority current ISR invalidates it.

    // Electrical algorithms. Both are constructed and kept alive; only the one
    // selected by in.algorithm is ticked. Keeping the unselected one around
    // costs a few hundred bytes and makes switching back an O(1) operation
    // with no allocation, which matters because A/B-ing FOC against six-step
    // on the same run is the whole point of having both.
    SixStepAlgorithm six_step_;
    FocAlgorithm foc_;

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
    // Deferred Hall update (re-read by the active current-rate handler)
    volatile bool hall_update_pending_ = false; ///< Set by Hall ISR, consumed by the PWM tick

    // Target speed filtering state (LPF → slew rate limiter cascade)
    Speed filtered_target_speed_{};          ///< LPF-filtered target speed
    Accumulator limited_target_speed_{};     ///< Retains sub-LSB ramp increments
    Accumulator acceleration_step_{};        ///< Maximum target-speed change per control tick
    TractionAcceleration<Profile> traction_acceleration_{};
    Speed traction_acceleration_step_{};

    /**
     * @brief Hold the velocity loop while the drive is parked in NEUTRAL
     *
     * Keeps the speed PI and the slew limiter from integrating against a
     * setpoint the disabled bridge cannot follow, and leaves both in the state
     * a bumpless re-engagement needs. See the note at the call site.
     */
    void holdVelocityLoop() noexcept;
    void publishSpeedPiOutput(SpeedPi::Output output) noexcept;

    /// @brief Refresh physical Hall after a mode change/parking; caller holds CriticalSection.
    void synchronizeHallPosition(bool discard_tracking = false) noexcept;

    /// @brief Age Hall health and consume a deferred reading; caller owns the current-rate ISR.
    void observeHall(bool guard_active, uint8_t position = HALL_NOT_SAMPLED) noexcept;
    void trackRotor(uint8_t position) noexcept;
    void clearParkedCommands() noexcept;

    /**
     * @brief Apply acceleration/deceleration limits to target speed
     *
     * Implements a slew rate limiter that restricts how fast target_speed can change.
     * Stores previous limited value internally - does NOT use measured speed.
     *
     * @param target_speed Desired target speed
     */
    void applyAccelerationLimit(Speed target_speed) noexcept;

};

} // namespace libecu

#include "bldc_pwm_frame.hpp"

#endif // LIBECU_BLDC_CONTROLLER_HPP
