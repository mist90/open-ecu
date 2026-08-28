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
#include "pid_controller.hpp"
#include "motor_pll.hpp"
#include "current_loop_tuning.hpp"
#include "hall_monitor.hpp"

namespace libecu {

/**
 * @brief Motor control mode (mechanical/commutation strategy)
 */
enum class ControlMode : uint8_t {
    OPEN_LOOP = 0,              ///< Open loop control (timing-based, no sensors)
    CLOSED_LOOP_VELOCITY = 1,   ///< Closed loop velocity control with PID + Hall sensors
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
    uint32_t control_frequency; ///< Control loop frequency (Hz)
    bool useInverseCommTable; ///< Use inverse six-step commutation table
    PidParameters pid_voltage_mode; ///< Velocity PID parameters for voltage mode (outputs duty cycle)
    PidParameters pid_current_mode; ///< Velocity PID parameters for current mode (outputs current)
    PidParameters pid_current_regulator; ///< Current PID parameters for current mode (outputs duty cycle)

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
     * pid_current_regulator: that one outputs duty and sees two phases in
     * series, this one outputs volts and sees one. Use tuneFocCurrentPi().
     */
    PidParameters foc_current_regulator;
};

/**
 * @brief Motor status information
 */
struct MotorStatus {
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
    float current_speed_rps;
    float target_speed_rps;   ///< Target motor speed (RPS)
    float duty_cycle;         ///< Current duty cycle
    float target_current;     ///< Target motor current (A)
    float measured_current;   ///< Measured motor current (A), raw per-PWM-cycle sample
    float measured_current_filtered; ///< Measured current after the telemetry LPF (A)
    float bus_voltage;         ///< Measured bus voltage (V)
    /**
     * @brief Measured NTC temperature (deg C), sampled at the 100 Hz loop rate
     *
     * TEMPERATURE_INVALID_C (-273) means the sensor read open or the ADC
     * conversion did not complete - not a cold motor.
     */
    float temperature_c;
    float pll_angle;          ///< Rotor angle from PLL (degrees, 0-360)
    int8_t current_pid_saturation; ///< +1/-1 if duty is clamped high/low, else 0 (for speed PID anti-windup)
    uint8_t target_position;   ///< Driven motor position
    uint8_t measured_position;   ///< Measured motor position
    bool is_running;          ///< Motor running status
    ControlMode control_mode;   ///< Current control mode (mechanical)
    ElectricMode electric_mode; ///< Current electric mode (electrical)

    /// @name FOC state, zero while six-step is selected
    ///@{
    DriveAlgorithm algorithm; ///< Electrical algorithm currently running
    float i_d;                ///< Measured d-axis current (A)
    float i_q;                ///< Measured q-axis current (A)
    float v_d;                ///< Applied d-axis voltage (V)
    float v_q;                ///< Applied q-axis voltage (V)
    float angle_e_rad;        ///< Electrical angle used by the algorithm (rad)
    float duty_u;             ///< Phase U duty actually applied
    float duty_v;             ///< Phase V duty actually applied
    float duty_w;             ///< Phase W duty actually applied
    ///@}
};

/**
 * @brief High-level BLDC motor controller
 */
class BldcController {
public:
    /**
     * @brief Constructor
     * @param pwm_interface PWM interface
     * @param hall_interface Hall sensor interface
     * @param params Motor control parameters (includes PID parameters for both modes)
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
     * @brief Set target speed for closed-loop control
     * @param speed_rps Target speed in RPS
     */
    void setTargetSpeed(float speed_rps) noexcept;

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
     */
    void setElectricMode(ElectricMode mode) noexcept;

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
    void setFocCurrentPi(float kp, float ki) noexcept;

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
    void setFocAngleOffsetDeg(float degrees_e) noexcept;

    /// @brief Get the FOC electrical angle offset, in electrical degrees
    float getFocAngleOffsetDeg() const noexcept;

    /**
     * @brief Set motor drive mode
     * @param mode Rotation mode
     */
    void setDriveMode(DriveMode mode) noexcept;

    /**
     * @brief Set speed PID controller gains (kp, ki, kd)
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     *
     * Preserves existing output limits and sample time.
     */
    void setSpeedPid(float kp, float ki, float kd) noexcept;

    /**
     * @brief Set current PID controller gains (kp, ki, kd)
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     *
     * Preserves existing output limits and sample time.
     */
    void setCurrentPid(float kp, float ki, float kd) noexcept;

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

    /// @brief Release a latched Hall fault
    void clearHallFault() noexcept;

    /// @brief Configure the Hall health monitor
    void setHallMonitorParams(const HallMonitorParams& params) noexcept;

    /**
     * @brief Set PLL base PI gains (kp_base, ki_base)
     * @param kp Base proportional gain
     * @param ki Base integral gain
     */
    void setPllGains(float kp, float ki) noexcept;

    /**
     * @brief Get current drive mode
     * @return Current drive mode
     */
    DriveMode getDriveMode() const noexcept;

    /**
     * @brief Get speed PID controller gains
     * @param kp Output: proportional gain
     * @param ki Output: integral gain
     * @param kd Output: derivative gain
     */
    void getSpeedPidGains(float& kp, float& ki, float& kd) const noexcept;

    /**
     * @brief Get current PID controller gains
     * @param kp Output: proportional gain
     * @param ki Output: integral gain
     * @param kd Output: derivative gain
     */
    void getCurrentPidGains(float& kp, float& ki, float& kd) const noexcept;

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
    PidController pid_speed_controller_;     // Speed controller (outer loop)

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

    // State variables
    MotorStatus status_;
    volatile DriveMode dmode_;
    bool initialized_;

    // Deferred Hall update (debounce via PWM ISR re-read)
    volatile bool hall_update_pending_;      ///< Set by Hall ISR, processed by PWM ISR

    // Open-loop timing control
    uint8_t open_loop_step_;                 ///< Current step in open-loop mode (0-5)
    uint32_t open_loop_last_step_time_us_;   ///< Timestamp of last step change

    // Control loop timing
    uint32_t last_pid_update_time_us_;       ///< Timestamp of last successful PID update

    /**
     * @brief Voltage-mode duty request, 0.0 to 1.0
     *
     * Kept separate from MotorStatus::duty_cycle, which is what the algorithm
     * *reports*. The two agree in six-step voltage mode, but FOC reports a
     * modulation index rather than the command it was given, so a single field
     * serving as both would feed the report back in as the next command.
     */
    float duty_command_;

    // Target speed filtering state (LPF → slew rate limiter cascade)
    float filtered_target_speed_;            ///< LPF-filtered target speed
    float filtered_measured_speed_;
    float filtered_measured_current_;           ///< LPF-filtered measured current, telemetry only
    float limited_target_speed_;             ///< Rate-limited target speed (after LPF)



    /**
     * @brief Apply acceleration/deceleration limits to target speed
     *
     * Implements a slew rate limiter that restricts how fast target_speed can change.
     * Stores previous limited value internally - does NOT use measured speed.
     *
     * @param target_speed Desired target speed
     * @param dt Time step in seconds
     * @return Rate-limited target speed
     */
    float applyAccelerationLimit(float target_speed, float dt) noexcept;

    /**
     * @brief Calculate step interval from target speed for open-loop control
     * @param speed_rps Target speed in RPS
     * @return Step interval in microseconds
     */
    uint32_t calculateOpenLoopStepInterval(float speed_rps) noexcept;
};

} // namespace libecu

#endif // LIBECU_BLDC_CONTROLLER_HPP
