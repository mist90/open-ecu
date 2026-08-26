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
 * Bounce is detected as **direction reversals per edge**, and still without
 * needing to know the commanded direction: healthy rotation keeps turning the
 * same way, so consecutive edges share a sign and reversals are rare, while
 * chatter alternates A->B->A->B and reverses on *every* edge.
 *
 * This was originally written as `edges - |net advance|` over the decay window,
 * on the reasoning that healthy motion gives `edges == |net advance|`.  That is
 * true only while the direction is constant, and it produced a false fault the
 * first time somebody turned the shaft back and forth by hand: the forward and
 * backward edges cancel in the *signed* accumulator while the edge count keeps
 * climbing, so a perfectly genuine reversal reads as 100 % chatter.  Measured
 * from a bench capture - about 15 edges one way, then the other - it tripped
 * ERRATIC_SEQUENCE and dropped the drive to NEUTRAL.
 *
 * Counting reversals instead costs a genuine change of direction exactly one
 * event, which decays away, while leaving both the healthy baseline and the
 * chatter signature where they were: a dither excursion A->B->A contributes two
 * reversals, the same two events the old surplus counted.
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
    ERRATIC_SEQUENCE, ///< Sustained edges that keep reversing direction
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

    /**
     * Fraction of edges that reverse direction, 0..1; 0 disables.
     *
     * A *count* cannot be used here, because the benign level scales with the
     * edge rate and therefore with speed. Measured on real hardware at 6 RPS
     * under 2 A of load, healthy running sat at 14-21 non-advancing edges
     * against an edge accumulator of ~345 - about 6% - and an absolute
     * threshold of 20 tripped on it. The same 6% at 9 RPS would be 27.
     *
     * The ratio is what stays put: real rotor dither at a sector boundary is a
     * few percent of edges, while genuine chatter approaches 100% because every
     * edge undoes the one before it.
     *
     * Note this is reversals, not net progress - see the header comment. A
     * sustained change of rotation direction, such as somebody turning the
     * shaft by hand, costs one event rather than invalidating the whole window.
     */
    float   erratic_fraction;

    /**
     * Minimum accumulated edges before the fraction is judged.
     *
     * A ratio computed from two or three edges is meaningless. Below roughly
     * 20 steps/sec the accumulator never reaches this, which leaves the
     * detector inactive at crawling speed - acceptable, since chatter there
     * neither damages anything nor misleads the PLL much.
     */
    float   erratic_min_edges;

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
        float     erratic_score;   ///< Direction reversals as a fraction of all edges (0..1)
        float     edge_accum;      ///< Leaky edge count
        float     reversal_accum;  ///< Leaky count of edges that reversed direction
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
    float    reversal_accum_;

    /// Sign of the previous edge's step delta, 0 before the first edge. Only
    /// a *change* of sign counts as a reversal, so the first edge after a
    /// reset - and the single edge that turns a hand-rotation around - is free.
    int8_t   last_delta_sign_;

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
