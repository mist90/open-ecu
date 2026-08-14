/**
 * @file hall_monitor.hpp
 * @brief Hall sensor health monitor for 3-sensor 120-degree BLDC feedback
 *
 * Replaces the time-since-last-edge rule that used to live in MotorPLL, which
 * could not see the failure that matters most: a *single* broken Hall line.
 * Any edge on any of the three lines reset that timer, so with one line dead
 * the other two kept it alive indefinitely.  Worse, when commutation goes wrong
 * the rotor oscillates across a sector boundary and produces a burst of genuine
 * edges, which fed the timer even harder.
 *
 * Detection here is based on what a broken line actually does to the code
 * sequence, not on the absence of activity.
 *
 * Why an invalid code is guaranteed
 * ---------------------------------
 * With 120-degree spacing the healthy sequence is a Gray code - exactly one bit
 * changes per transition:
 *
 *      001 -> 011 -> 010 -> 110 -> 100 -> 101 ->
 *
 * Force any single line low and the observed sequence becomes
 *
 *      000   010   010   110   100   100
 *       ^illegal    ^dup            ^dup
 *
 * and forcing it high produces 111 instead.  So a stuck line *necessarily*
 * yields one illegal code per electrical revolution, plus repeated states.  A
 * disconnected loom sits at the pull level on all three lines, which is 000 or
 * 111 continuously.  Illegal codes cannot occur in health: a Gray transition
 * only ever passes through the two adjacent valid states, and even two edges
 * inside one sampling window land on a state two positions away - still legal.
 *
 * Bounce versus breakdown
 * -----------------------
 * Short bounce happens on healthy hardware and must not trip a fault.  Two
 * different discriminators are needed, because counting alone does not work:
 * measured on real hardware, healthy running at 9 RPS produced 3532 illegal
 * readings per second, twenty times more than a genuinely stuck line would.
 *
 * What separates them is *duration*.  A bouncing line keeps raising interrupts,
 * so an illegal reading is followed by a valid one within a PWM period.  A
 * stuck line has nothing left to raise an edge, so its illegal code stands for
 * a whole commutation step - measured at 50 us versus 926 us, a clean 20x gap.
 *
 * For the erratic detector, where counting does work, each accumulator leaks:
 * a burst adds a fixed amount and decays, while a sustained fault settles at
 * `rate * decay_time_s`.
 *
 * Bounce itself is detected without needing to know the rotation direction:
 * every healthy edge advances the position by one step in a consistent
 * direction, so over any window `edges == |net advance|`.  Chatter breaks that
 * equality - many edges, no net progress - and the surplus is the fault signal.
 *
 * There is deliberately no "no edges for too long" rule.  A disconnected loom
 * already shows as an illegal code, and a rotor that has simply stopped is a
 * stall, not a Hall fault; conflating the two is what made the old rule assert
 * on a parked motor.
 */

#ifndef LIBECU_HALL_MONITOR_HPP
#define LIBECU_HALL_MONITOR_HPP

#include <cstdint>

namespace libecu {

/// Position value used by HallInterface::getPosition() for an illegal code
static constexpr uint8_t HALL_POSITION_INVALID = 0xFF;

/**
 * @brief Why the monitor latched a fault
 */
enum class HallFault : uint8_t {
    NONE = 0,
    INVALID_CODE,     ///< Illegal 000/111, either sustained or recurring
    ERRATIC_SEQUENCE, ///< Sustained edges that make no net progress
};

/**
 * @brief Configuration for HallMonitor
 *
 * Both thresholds are in "events", and each accumulator decays with
 * `decay_time_s`.  A steady fault at `r` events/sec settles the accumulator at
 * `r * decay_time_s`, while an isolated burst of `n` events peaks at about `n`.
 * So a threshold `T` means: trip on a burst larger than `T` events, or on a
 * sustained rate above `T / decay_time_s`.
 */
struct HallMonitorParams {
    float   decay_time_s;        ///< Leak time constant for every accumulator (s)

    /**
     * Accumulated illegal-code events that latch a fault; 0 disables.
     *
     * Disabled by default, because on measured hardware it cannot
     * discriminate. Healthy running at 9 RPS produced 3532 illegal readings
     * per second - transient ones, from Hall bounce - which settles this
     * accumulator near 1766. A genuinely stuck line puts only one illegal code
     * per electrical revolution, about 180/s at that speed, settling near 90.
     * The benign rate is twenty times the fault signature, so no threshold
     * separates them. Duration does; see invalid_persist_time_s.
     *
     * Worth enabling only on hardware where illegal codes are genuinely rare.
     */
    float   invalid_threshold;
    /**
     * Consecutive illegal *readings* that must arrive before one is scored.
     *
     * Readings are already deferred ~50 us past the Hall edge and therefore
     * settled, so 1 is the sensible default. It must stay at 1 to catch a
     * single stuck line: that produces exactly one illegal code per electrical
     * revolution, always bracketed by valid ones, so requiring two in a row
     * would never fire. Noise immunity comes from the leaky accumulator, not
     * from this.
     */
    uint8_t invalid_debounce;

    /**
     * Seconds the latest reading may stay illegal before faulting on its own.
     *
     * Recurrence and persistence are different failures and the accumulator
     * only catches the first: a disconnected loom produces one edge as the
     * lines settle to the pull level and then nothing ever again, so it is a
     * single event no matter how long it lasts. Ageing the *last* reading
     * catches it without needing any further edges. A healthy transition
     * cannot leave an illegal code standing for a millisecond.
     */
    float invalid_persist_time_s;
    /**
     * Measured basis for the 300 us default, on MOTOR_1 at 31 V:
     *
     *   speed    illegal/s   max standing time   step period
     *   6 RPS         0            0 us            1389 us
     *   8 RPS        44            0 us            1042 us
     *   9 RPS      3532           50 us             926 us
     *
     * Benign bounce is transient - an illegal reading is followed by a valid
     * one within a PWM period, because the bouncing line keeps raising EXTI.
     * A stuck line has nothing left to raise an edge, so its illegal code
     * stands for a full step. 300 us sits six times above the benign maximum
     * and three times below the fault signature at the top of the speed range.
     */

    float   erratic_threshold;   ///< Accumulated non-advancing edges that latch a fault

    bool    require_drive_active;///< Only accumulate while the bridge is driving
};

/**
 * @brief Hall sensor health monitor
 *
 * Usage, from the PWM ISR:
 *  - Call tick() every cycle. This is only a time base - it decays the
 *    accumulators and ages a standing illegal code. It reads no hardware.
 *  - Call onPosition() with the deferred, debounced Hall reading, i.e. inside
 *    the same `hall_update_pending_` block that feeds MotorPLL, so the monitor
 *    and the PLL see exactly the same sample from exactly the same read.
 *  - Check isFaulted() / faultCause(); the fault latches until clearFault()
 *
 * Sampling the Hall lines at arbitrary PWM instants instead would risk
 * catching them mid-transition, where the three sensors have not all switched
 * yet, and manufacturing illegal codes that were never really there.
 */
class HallMonitor {
public:
    /**
     * @brief Constructor
     * @param pwm_frequency Rate at which update() will be called (Hz)
     */
    explicit HallMonitor(float pwm_frequency) noexcept;

    void setParameters(const HallMonitorParams& params) noexcept;
    const HallMonitorParams& getParameters() const noexcept { return params_; }

    /**
     * @brief Advance the time base; call every PWM cycle
     * @param drive_active Bridge enabled and commanded to produce torque
     */
    void tick(bool drive_active) noexcept;

    /**
     * @brief Feed one freshly deferred Hall reading
     * @param position     Decoded position 0-5, or HALL_POSITION_INVALID
     * @param drive_active Bridge enabled and commanded to produce torque
     */
    void onPosition(uint8_t position, bool drive_active) noexcept;

    /// @return true while a fault is latched
    bool isFaulted() const noexcept { return fault_ != HallFault::NONE; }

    /// @return Which detector latched, or HallFault::NONE
    HallFault faultCause() const noexcept { return fault_; }

    /**
     * @brief Clear a latched fault and reset the accumulators
     *
     * The fault latches deliberately: the old design re-asserted every PWM
     * cycle and the controller answered each one, which made a persistent
     * condition look like commands being ignored rather than a fault.
     */
    void clearFault() noexcept;

    /// @brief Clear all state including the last position
    void reset() noexcept;

    /**
     * @brief Telemetry snapshot
     */
    struct Info {
        float     invalid_score;   ///< Leaky accumulator for illegal codes
        float     erratic_score;   ///< Leaky accumulator for non-advancing edges
        float     edge_accum;      ///< Leaky edge count
        float     advance_accum;   ///< Leaky *signed* net advance, in steps
        uint32_t  invalid_events;  ///< Illegal-code runs counted since reset
        uint8_t   invalid_run;     ///< Consecutive illegal readings
        float     invalid_time_s;  ///< How long the latest reading has been illegal
        uint32_t  edges;           ///< Transitions counted since reset
        uint8_t   last_position;   ///< Last valid position seen
        HallFault fault;
    };

    Info getInfo() const noexcept;

private:
    /// Recompute the erratic score and latch any threshold crossing
    void evaluate() noexcept;

    float              dt_;
    HallMonitorParams  params_;

    float    invalid_score_;
    float    erratic_score_;
    float    edge_accum_;
    float    advance_accum_;

    uint8_t  last_position_;
    bool     have_last_;
    uint8_t  invalid_run_;      ///< Consecutive illegal readings
    float    invalid_time_;     ///< Seconds the latest reading has been illegal
    bool     last_invalid_;     ///< Latest reading was an illegal code
    bool     invalid_counted_;  ///< This run of illegal samples already scored

    uint32_t invalid_events_;
    uint32_t edges_;
    HallFault fault_;
};

} // namespace libecu

#endif // LIBECU_HALL_MONITOR_HPP
