/**
 * @file main.cpp
 * @brief ECU application entry point
 *
 * Platform independent by construction: every hardware detail is reached
 * through libecu::Board (board.hpp) and every motor detail through
 * motor_config.hpp. Adding a target means adding a port that implements
 * libecu::createBoard(), not editing this file.
 */

#include "../include/board.hpp"
#include "../include/bldc_controller.hpp"
#include "../include/at_command_processor.hpp"
#include "../include/critical_section.hpp"
#include "../include/current_loop_tuning.hpp"
#include "../include/motor_config.hpp"

#include <cstdio>

//#define THROTTLE_BRAKE_CONTROL

namespace {

#ifdef THROTTLE_BRAKE_CONTROL
/**
 * @brief Closed-throttle dead-band, as a fraction of full scale
 *
 * Below this the throttle counts as released and the drive parks. It has to
 * clear the potentiometer's own noise - a few LSB of a 12-bit conversion,
 * around 0.001 - without swallowing usable travel at the bottom of the sweep.
 * 1 % is a starting point, not a measurement: check it on the bench against
 * the real pot before relying on it, since a dead-band that is too small makes
 * the drive chatter between NEUTRAL and FORWARD at rest.
 */
constexpr float THROTTLE_DEADBAND = 0.01f;
#endif

/**
 * @brief Assemble the motor control parameters
 *
 * Motor properties come from motor_config.hpp; loop rates and the DC link
 * voltage come from the board, because those are the numbers that change when
 * the same motor is driven by different hardware.
 */
libecu::MotorControlParams makeMotorParams(const libecu::BoardInfo& board_info) noexcept
{
    libecu::MotorControlParams motor_params;
    motor_params.num_poles = BLDC_NUM_POLES;
    motor_params.max_duty_cycle = 0.95f;
    motor_params.max_current = BLDC_MAX_CURRENT;
    motor_params.min_current = BLDC_MIN_CURRENT;
    motor_params.max_voltage = BLDC_MAX_VOLTAGE;
    motor_params.max_temperature_c = BLDC_MAX_TEMPERATURE;
    motor_params.max_speed_rps = BLDC_MAX_SPEED;
    motor_params.acceleration_rate = BLDC_MAX_ACCELERATION;  // RPS/s
    motor_params.target_speed_lpf_alpha = 0.0f;  // LPF smoothing for noisy potentiometer input
    motor_params.measured_speed_lpf_alpha = 0.5f; // LPF smoothing for noisy velocity measurement
    motor_params.measured_current_lpf_alpha = 0.005f; // 10 ms at 20 kHz; telemetry only
    // Demagnetisation blanking: the current reading settles to its noise floor
    // about nine PWM cycles after a commutation on this motor. Six covers the
    // bulk of the transient without holding the duty for much of a short sector.
    motor_params.current_blanking_cycles = 6;
    motor_params.control_frequency = board_info.control_frequency_hz;
    motor_params.pid_voltage_mode = {0.1f, 0.2f}; // Speed PID for VOLTAGE_MODE (outputs duty cycle 0.0-1.0)
    motor_params.pid_voltage_mode.integral_max = 0.8f;
    motor_params.pid_voltage_mode.integral_min = 0.0f;
    motor_params.pid_voltage_mode.kb = 2.0f;

    motor_params.pid_current_mode = {0.5f, 2.0f}; // Speed PID for CURRENT_MODE (outputs current, A)
    motor_params.pid_current_mode.integral_max = 12.0f;
    motor_params.pid_current_mode.integral_min = -4.0f;
    motor_params.pid_current_mode.kb = 2.0f;

    // Current-loop PI from the electrical model: kp = 2*L*w_c/Vbus,
    // ki = 2*R*w_c/Vbus, which places the PI zero on the plant pole (Ti = L/R)
    // and puts the crossover at BLDC_ILOOP_BW_HZ. Falls back to the previous
    // hand-tuned gains if the motor has not been identified.
    {
        libecu::CurrentLoopModel iloop;
        iloop.l_phase_h    = BLDC_L_PHASE_H;
        iloop.r_phase_ohm  = BLDC_R_PHASE_OHM;
        iloop.bus_voltage  = board_info.nominal_bus_voltage;
        iloop.bandwidth_hz = BLDC_ILOOP_BW_HZ;
        libecu::PidParameters ip = libecu::tuneCurrentPi(iloop);
        if (ip.kp <= 0.0f) {
            ip.kp = 0.1f;   // un-identified motor: previous hand-tuned values
            ip.ki = 50.0f;
        }
        motor_params.pid_current_regulator = ip;
    }
    motor_params.pid_current_regulator.min_output = 0.0f;
    motor_params.pid_current_regulator.max_output = 1.0f;
    motor_params.pid_current_regulator.integral_max = 1.0f;
    motor_params.pid_current_regulator.integral_min = 0.0f;
    motor_params.pid_current_regulator.kb = 5.0f;
    motor_params.useInverseCommTable = BLDC_INVERTION;

    // ---- FOC ---------------------------------------------------------------
    // FOC is the default. Measured against six-step at 6.00 RPS on this motor
    // (docs/FOC_HANDOFF.md section 9.5): current ripple 0.0308 A vs 0.2398 A -
    // 7.8x smoother - holding the same speed on about 17 % less RMS phase
    // current, i.e. roughly a third less copper loss, with Id regulated to
    // -2 mA. Six-step is still built and one command away (AT+ALGO=0), which is
    // what makes an A/B on the same run possible.
    //
    // Caveat worth knowing at boot: FOC has only been run FORWARD, at or below
    // 8 RPS, under about 1 A. High modulation index - where the low-side
    // sampling window shrinks and the worst-phase shunt rejection starts
    // earning its keep - is still unexercised.
    motor_params.algorithm = libecu::DriveAlgorithm::FOC;

    motor_params.foc.max_modulation = 0.95f;
    motor_params.foc.max_duty_cycle = motor_params.max_duty_cycle;
    // -60 electrical degrees, derived from the six-step table geometry and the
    // PLL's +1-step field offset (see foc_algorithm.hpp). A derivation, not a
    // measurement - trim with AT+FANG on the bench before trusting the torque.
    motor_params.foc.angle_offset_rad = -libecu::FOC_RAD_PER_STEP;
    motor_params.foc.id_target = 0.0f;   // surface-magnet rotor: no reluctance to exploit
    // Cross-coupling and back-EMF feed-forward stays off for bring-up. A wrong
    // feed-forward is harder to diagnose than none; enable it once the plain
    // loops are known good.
    motor_params.foc.use_decoupling = false;
    motor_params.foc.l_phase_h = BLDC_L_PHASE_H;
    // ke is in V.s/rad_e, which is exactly the flux linkage the q-axis
    // feed-forward needs.
    motor_params.foc.flux_linkage_wb = BLDC_KE_V_S_PER_RAD_E;
    motor_params.foc.anti_windup_kb = 1.0f;

    // d/q current regulators. Same motor model and same target bandwidth as the
    // six-step loop, but the gains differ because the FOC regulators see one
    // phase rather than two in series and output volts rather than duty.
    {
        libecu::CurrentLoopModel floop;
        floop.l_phase_h    = BLDC_L_PHASE_H;
        floop.r_phase_ohm  = BLDC_R_PHASE_OHM;
        floop.bus_voltage  = board_info.nominal_bus_voltage;
        floop.bandwidth_hz = BLDC_ILOOP_BW_HZ;
        motor_params.foc_current_regulator = libecu::tuneFocCurrentPi(floop);
    }

    return motor_params;
}

/**
 * @brief Hall health thresholds
 *
 * Sized from a measurement on this hardware (AT+HSTATUS with the detectors
 * disabled, swept 2-9 RPS):
 *
 *   speed    illegal/s   max standing time   step period
 *   6 RPS         0            0 us            1389 us
 *   8 RPS        44            0 us            1042 us
 *   9 RPS      3532           50 us             926 us
 *
 * Illegal codes are common on intact wiring here - Hall bounce, strongly
 * speed dependent - but always transient, because a bouncing line keeps
 * raising EXTI and the next reading is valid. A stuck line has nothing left
 * to raise an edge, so its illegal code stands for a whole step. Counting
 * cannot separate those (benign is 20x the fault rate); duration can.
 */
libecu::HallMonitorParams makeHallMonitorParams() noexcept
{
    libecu::HallMonitorParams hall_params;
    hall_params.decay_time_s           = 0.5f;
    hall_params.invalid_threshold      = 0.0f;    // counting is useless here
    hall_params.invalid_debounce       = 1;
    hall_params.invalid_persist_time_s = 0.0003f; // 6x the benign max, 3x below the fault
    // Fraction, not a count: the count scales with edge rate. A loaded run at
    // 6 RPS / 2 A measured a healthy fraction of ~0.06 while an absolute
    // threshold of 20 tripped on it (surplus 21 of 351 edges).
    hall_params.erratic_fraction       = 0.35f;
    hall_params.erratic_min_edges      = 10.0f;
    hall_params.require_drive_active   = true;
    return hall_params;
}

} // namespace

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    libecu::Board& board = libecu::createBoard();

    if (!board.initialize()) {
        board.fail();
    }

    libecu::MotorControlParams motor_params = makeMotorParams(board.info());

    static libecu::BldcController motor_controller(
        board.pwm(), board.hall(), motor_params, board.adc());

    motor_controller.setHallMonitorParams(makeHallMonitorParams());

    if (!motor_controller.initialize()) {
        board.fail();
    }

    libecu::AtCommandProcessor& at_processor = board.createConsole(motor_controller);

    if (!board.startCurrentLoop(motor_controller, at_processor)) {
        board.fail();
    }

    motor_controller.start();

    if (!board.startControlLoop()) {
        board.fail();
    }

    printf("ECU started\n");
    while (1) {
        // motor control loop
        if (board.takeControlTick()) {
            // Update motor controller status
            libecu::MotorStatus status;
            {
                libecu::CriticalSection cs;
                status = motor_controller.getStatus();
            }
#ifdef THROTTLE_BRAKE_CONTROL
            // Reads the throttle sampled during the last control tick, not the
            // hardware: on this board the potentiometer shares an ADC group
            // with the NTC, so the conversion has to happen in the same
            // context as the temperature read. See Board::readThrottle() and
            // the re-entrancy note on Stm32Adc::readRegularChannel().
            //
            // Taken once per tick, normalised, and scaled below into the
            // speed demand.
            const float throttle = board.readThrottle(1.0f);

            // The throttle is a velocity demand, so the controller is held in
            // closed-loop velocity for as long as this input owns the drive -
            // including back out of any mode set over AT. setControlMode()
            // ignores a no-op itself; testing the snapshot we already hold
            // saves even the critical section.
            if (status.control_mode != libecu::ControlMode::CLOSED_LOOP_VELOCITY) {
                motor_controller.setControlMode(libecu::ControlMode::CLOSED_LOOP_VELOCITY);
            }

            // A closed throttle parks the drive rather than holding a zero
            // setpoint with the bridge live; anything past the dead-band
            // engages it forward.
            //
            // Edge-triggered against the controller's own mode, because
            // setDriveMode() is not idempotent: the non-NEUTRAL path runs
            // algorithm().onEnter(), which under FOC re-arms all three phases
            // at 50 % duty. Calling it every tick would keep resetting the
            // loop it is supposed to be driving.
            const libecu::DriveMode wanted = (throttle <= THROTTLE_DEADBAND)
                                                 ? libecu::DriveMode::NEUTRAL
                                                 : libecu::DriveMode::FORWARD;
            if (motor_controller.getDriveMode() != wanted) {
                motor_controller.setDriveMode(wanted);
            }

            if (wanted != libecu::DriveMode::NEUTRAL) {
                // The brake zeroes the demand but deliberately does not park the
                // drive: with the bridge still live, a zero setpoint is what lets
                // the speed loop hold the motor down rather than let it coast.
                const float demand = board.brakeEngaged() ? 0.0f : throttle - THROTTLE_DEADBAND;

                motor_controller.setTargetSpeed(demand * motor_params.max_speed_rps);
            } else {
                motor_controller.setTargetSpeed(0.0f);
            }
#endif

            if (at_processor.isTelemetryEnabled()) {
                at_processor.sendTelemetry(status);
            }
            if (at_processor.isPllTelemetryEnabled()) {
                libecu::MotorPLL::PllInfo pll_info = motor_controller.getPllInfo();
                at_processor.sendPllTelemetry(pll_info);
            }
            if (at_processor.isHallTelemetryEnabled()) {
                at_processor.sendHallTelemetry(motor_controller.getHallInfo());
            }
        }

        at_processor.process();
        at_processor.processOscOutput();
    }
}
