/**
 * @file board.hpp
 * @brief Abstract factory for the hardware a BLDC ECU runs on
 *
 * This is the only seam between the ECU (libecu/src/main.cpp, which is the
 * same source on every target) and a particular piece of hardware. A port
 * consists of one translation unit that:
 *
 *   1. derives from Board and builds the concrete PwmInterface,
 *      HallInterface, AdcInterface and AtCommandProcessor,
 *   2. implements createBoard() to return the single instance of it,
 *   3. implements the free functions the library expects from the platform:
 *      time_us() (adc_interface.hpp) and disable_interrupts() /
 *      enable_interrupts() (critical_section.hpp).
 *
 * Nothing above this header may include a vendor HAL, and nothing below it
 * may know which motor is attached.
 */

#ifndef LIBECU_BOARD_HPP
#define LIBECU_BOARD_HPP

#include <cstdint>

#include "adc_interface.hpp"
#include "hall_interface.hpp"
#include "pwm_interface.hpp"

namespace libecu {

class AtCommandProcessor;
class BldcController;

/**
 * @brief Hardware facts the ECU needs but cannot derive
 *
 * Everything here is a property of the board - the inverter, the supply, the
 * timers - rather than of the motor bolted to it. Motor properties live in
 * motor_config.hpp.
 */
struct BoardInfo {
    uint32_t pwm_frequency_hz;      ///< Inverter switching / current-loop rate (Hz)
    uint32_t control_frequency_hz;  ///< Periodic (speed) control loop rate (Hz)
    float    nominal_bus_voltage;   ///< Nominal DC link voltage (V), used for PI tuning
    uint16_t requested_dead_time_ns;///< Dead time asked of the PWM driver (ns)
    uint16_t actual_dead_time_ns;   ///< Dead time the hardware actually programmed (ns)
};

/**
 * @brief Abstract hardware factory and lifecycle for one ECU board
 */
class Board {
public:
    virtual ~Board() = default;

    /**
     * @brief Bring up the whole board
     *
     * Clocks, GPIO, timers, the serial console, the PWM/Hall/ADC drivers and
     * the zero-current offset calibration. On return the peripherals are live
     * but no control interrupt is armed yet.
     *
     * @return true on success
     */
    virtual bool initialize() noexcept = 0;

    /// @brief Hardware description, valid after initialize()
    virtual const BoardInfo& info() const noexcept = 0;

    virtual PwmInterface&  pwm() noexcept = 0;
    virtual HallInterface& hall() noexcept = 0;

    /// @brief Current/voltage sensing, or nullptr on a board without it
    virtual AdcInterface* adc() noexcept = 0;

    /**
     * @brief Build the console the AT command protocol talks over
     * @param controller Controller the commands act on
     * @return Reference to the board-owned processor
     */
    virtual AtCommandProcessor& createConsole(BldcController& controller) noexcept = 0;

    /**
     * @brief Route the hardware interrupts and arm the current loop
     *
     * After this the commutation and current-control ISRs run at
     * BoardInfo::pwm_frequency_hz against the given controller, and every PWM
     * cycle is offered to the console for oscilloscope capture.
     *
     * @return true on success
     */
    virtual bool startCurrentLoop(BldcController& controller,
                                  AtCommandProcessor& console) noexcept = 0;

    /**
     * @brief Arm the periodic control tick at BoardInfo::control_frequency_hz
     *
     * Separate from startCurrentLoop() because the ECU starts the controller
     * between the two: the current loop must already be feeding it, and the
     * speed loop must not run until it is running.
     *
     * @return true on success
     */
    virtual bool startControlLoop() noexcept = 0;

    /**
     * @brief Consume one pending control tick
     * @return true if a control period elapsed since the last call
     */
    virtual bool takeControlTick() noexcept = 0;

    /**
     * @brief Unrecoverable hardware failure - does not return
     */
    [[noreturn]] virtual void fail() noexcept = 0;

    /**
     * @brief Manual throttle input, if the board has one
     *
     * Returns the most recent sample rather than converting at call time. On
     * boards where the throttle shares an ADC group with a sensor read from
     * the control tick, the two conversions must not interleave, so sampling
     * belongs in that context. Callers therefore get a value at most one
     * control period old and must not assume this touches hardware.
     *
     * @param max_value Value corresponding to a fully open throttle
     * @return Demand in [0, max_value]; 0 when unsupported
     */
    virtual float readThrottle(float max_value) noexcept {
        (void)max_value;
        return 0.0f;
    }

    /**
     * @brief Manual brake input, if the board has one
     * @return true while the brake is engaged; false when unsupported
     */
    virtual bool brakeEngaged() noexcept { return false; }
};

/**
 * @brief Obtain the board this firmware was built for
 *
 * Implemented by exactly one translation unit per target. The instance has
 * static storage duration and outlives main().
 */
Board& createBoard() noexcept;

} // namespace libecu

#endif // LIBECU_BOARD_HPP
