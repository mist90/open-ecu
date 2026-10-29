/**
 * @file command_processor.hpp
 * @brief Protocol-independent command processing and foreground/ISR lifecycle
 */

#ifndef LIBECU_COMMAND_PROCESSOR_HPP
#define LIBECU_COMMAND_PROCESSOR_HPP

#include <cstddef>
#include <cstdint>

#include "ecu_config.hpp"

namespace libecu {

class BldcController;
template<class P> struct MotorStatusT;
using NativeMotorStatus = MotorStatusT<ActiveNumericProfile>;

/**
 * @brief Byte-oriented protocol selected by a board's console factory
 *
 * A transport derives from the selected processor and implements read()/write().
 * The ECU uses only this interface; framing, telemetry and capture storage belong
 * to the protocol. Instances and their controllers have static storage duration.
 */
class CommandProcessor {
public:
    /**
     * @brief Bind commands to the board's controller
     * @param controller Controller which outlives this processor
     */
    explicit CommandProcessor(BldcController* controller) noexcept : controller_(controller) {}

    virtual ~CommandProcessor() noexcept = default;

    /**
     * @brief Read one byte without waiting for input
     * @return Byte in [0, 255], or -1 if no data is available
     */
    virtual int32_t read() = 0;

    /**
     * @brief Write bytes from foreground context, including embedded zero bytes
     * @param data Bytes to transmit; no string terminator is required
     * @param length Number of bytes
     */
    virtual void write(const char* data, std::size_t length) = 0;

    /**
     * @brief Consume available input on every main-loop iteration
     *
     * Must return without waiting for additional input. Protocols may also poll
     * deadlines here so link watchdogs need not wait for a control tick.
     */
    virtual void process() noexcept = 0;

    /// @brief Called once after the controller and both loops start; silent by default
    virtual void onStarted() noexcept {}

    /// @brief Foreground slow tick; obtain only the controller snapshots the protocol needs
    virtual void onControlTick() noexcept {}

    /// @brief Drain deferred output once per main-loop iteration; no output by default
    virtual void processOutput() noexcept {}

    /**
     * @brief Opt in to a native status snapshot after the current-loop ISR
     * @return false by default, avoiding a snapshot and capture work entirely
     *
     * Called from the ISR. Both this query and captureCurrentSample() must be
     * bounded, non-blocking, and must not transmit or format output.
     */
    virtual bool wantsCurrentSample() const noexcept { return false; }

    /**
     * @brief Consume an ISR snapshot when wantsCurrentSample() is true
     * @param status Current-loop status in the selected numeric profile
     */
    virtual void captureCurrentSample(const NativeMotorStatus& status) noexcept { (void)status; }

protected:
    /// @brief Controller shared with the application; no protocol state lives in the base
    BldcController* controller_;
};

} // namespace libecu

#endif // LIBECU_COMMAND_PROCESSOR_HPP
