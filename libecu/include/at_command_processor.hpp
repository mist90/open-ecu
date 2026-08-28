/**
 * @file at_command_processor.hpp
 * @brief AT command processor for motor controller communication
 */

#ifndef LIBECU_AT_COMMAND_PROCESSOR_HPP
#define LIBECU_AT_COMMAND_PROCESSOR_HPP

#include <cstddef>
#include <cstdint>

#include "motor_pll.hpp"
#include "hall_monitor.hpp"

namespace libecu {

// Forward declarations
class BldcController;
struct MotorStatus;

/**
 * @brief AT command processor for serial communication with motor controller
 */
class AtCommandProcessor {
public:
    /**
     * @brief Parse state for command processing
     */
    enum class ParseState : uint8_t {
        Idle,           ///< Waiting for command start
        Receiving,      ///< Receiving command data
        CrcParsing,     ///< Parsing CRC checksum
        CrcAccumulating,///< Accumulating CRC hex digits
        Execute         ///< Ready to execute command
    };

    /**
     * @brief Constructor
     * @param controller Pointer to BLDC controller instance
     */
    explicit AtCommandProcessor(BldcController* controller) noexcept;

    /**
     * @brief Virtual destructor
     */
    virtual ~AtCommandProcessor() noexcept = default;

    /**
     * @brief Read a character from the communication interface
     * @return Character value or -1 if no data available
     */
    virtual int32_t read() = 0;

    /**
     * @brief Write data to the communication interface
     * @param str String to write
     * @param len Length of string
     */
    virtual void write(const char* str, std::size_t len) = 0;

    /**
     * @brief Process incoming commands (non-blocking)
     * Should be called in main loop
     */
    void process() noexcept;

    /**
     * @brief Send telemetry data
     * @param status Motor status information
     */
    void sendTelemetry(const MotorStatus& status) noexcept;

    /**
     * @brief Enable or disable telemetry output
     * @param enabled True to enable telemetry
     */
    void setTelemetryEnabled(bool enabled) noexcept;

    /**
     * @brief Check if telemetry is enabled
     * @return True if telemetry is enabled
     */
    bool isTelemetryEnabled() const noexcept;

    /**
     * @brief Start oscilloscope streaming
     */
    void startOscilloscope() noexcept;

    /**
     * @brief Stop oscilloscope streaming
     */
    void stopOscilloscope() noexcept;

    /**
     * @brief Capture an oscilloscope sample
     *
     * Called from the PWM ISR, so it must stay cheap: the conversion to the
     * packed representation happens here rather than at output time, but it is
     * only a handful of multiplies.
     *
     * @param status Motor status snapshot for this PWM cycle
     */
    void captureOscSample(const MotorStatus& status) noexcept;

    /**
     * @brief Process oscilloscope output (send one sample per call)
     */
    void processOscOutput() noexcept;

    /**
     * @brief Send PLL telemetry line (+PLL:)
     * @param info PLL internal state snapshot (angle_per_second, pll_integral, angle, hall_state_raw, is_sync)
     */
    void sendPllTelemetry(const MotorPLL::PllInfo& info) noexcept;

    /**
     * @brief Enable or disable PLL telemetry output
     * @param enabled True to enable +PLL output
     */
    void setPllTelemetryEnabled(bool enabled) noexcept;

    /**
     * @brief Check if PLL telemetry is enabled
     * @return True if +PLL output is enabled
     */
    bool isPllTelemetryEnabled() const noexcept;

    /**
     * @brief Send Hall health telemetry line (+HSTATUS:)
     *
     * Unbuffered, one line per call.  Every field is either a leaky
     * accumulator or a running total, so a 100 Hz poll sees the same picture
     * the monitor does - unlike a raw Hall sample, which at this rate would be
     * an arbitrary point between commutations.
     *
     * @param info Hall monitor snapshot
     */
    void sendHallTelemetry(const HallMonitor::Info& info) noexcept;

    /**
     * @brief Enable or disable Hall health telemetry output
     * @param enabled True to enable +HSTATUS output
     */
    void setHallTelemetryEnabled(bool enabled) noexcept;

    /**
     * @brief Check if Hall health telemetry is enabled
     * @return True if +HSTATUS output is enabled
     */
    bool isHallTelemetryEnabled() const noexcept;

    // Configuration constants
    static constexpr std::size_t MAX_COMMAND_LENGTH = 64;
    static constexpr std::size_t OSC_BUFFER_SIZE = 1024;

private:
    /**
     * @brief Process a complete command
     */
    void processCommand() noexcept;

    /**
     * @brief Send OK response
     */
    void sendOk() noexcept;

    /**
     * @brief Send error response
     */
    void sendError() noexcept;

    /**
     * @brief Send a response with prefix and value
     * @param prefix Response prefix
     * @param value Response value
     */
    void sendResponse(const char* prefix, const char* value) noexcept;

    /**
     * @brief Send a float response: +CMD:value\r\n
     */
    void sendFloatResponse(const char* prefix, float value) noexcept;

    /**
     * @brief Send an integer response: +CMD:value\r\n
     */
    void sendIntResponse(const char* prefix, int value) noexcept;

    /**
     * @brief Convert nibble to hex character
     * @param nibble Nibble value (0-15)
     * @return Hex character ('0'-'F')
     */
    char hexChar(uint8_t nibble) const noexcept;

    /**
     * @brief Convert hex character to nibble
     * @param c Hex character ('0'-'9', 'A'-'F')
     * @return Nibble value (0-15) or 0xFF if invalid
     */
    uint8_t hexValue(char c) const noexcept;

    /**
     * @brief Check if character is a valid hex character
     * @param c Character to check
     * @return True if character is valid hex
     */
    bool isHexChar(char c) const noexcept;

    // Member variables
    BldcController* controller_;
    ParseState state_;
    char command_buffer_[MAX_COMMAND_LENGTH];
    std::size_t cmd_index_;
    bool telemetry_enabled_;
    bool osc_streaming_;
    bool pll_telemetry_enabled_;
    bool hall_telemetry_enabled_;

    // Oscilloscope single buffer with two phases
    enum class OscPhase : uint8_t {
        Accumulating,  ///< ISR fills buffer, output blocked
        Outputting     ///< Output drains buffer, capture blocked
    };

    /**
     * @brief One 20 kHz oscilloscope sample
     *
     * Packed into fixed point rather than floats, because the buffer is
     * OSC_BUFFER_SIZE deep and the MCU has 32 KB of RAM in total. Carrying
     * Id/Iq/angle as floats alongside the original two currents would have
     * taken the buffer from 12 KB to 24 KB; as milliamps and a 16-bit angle it
     * stays at 12 KB while carrying three more signals.
     *
     * int16 milliamps spans +-32.7 A, which comfortably covers the +-34 A the
     * shunt/PGA chain can measure at all, so the packing loses nothing the
     * hardware could have told us.
     */
    struct OscSample {
        int16_t target_current_ma;   ///< Target current (mA)
        int16_t measured_current_ma; ///< Measured current (mA)
        int16_t i_d_ma;              ///< d-axis current (mA), FOC only
        int16_t i_q_ma;              ///< q-axis current (mA), FOC only
        uint16_t angle_e;            ///< Electrical angle, full scale = one electrical rev
        uint8_t duty_cycle;          ///< Duty cycle * 100 (0..100)
        uint8_t position;            ///< Commutation position (0-5)
    };

    OscSample osc_buffer_[OSC_BUFFER_SIZE];
    std::size_t osc_write_index_;
    std::size_t osc_read_index_;
    volatile OscPhase osc_phase_;
    std::int32_t osc_sample_counter_;
    uint8_t crc_index_;  ///< Index for accumulating CRC hex digits

    /**
     * Partially accumulated received CRC.
     *
     * The four hex digits of a frame's CRC do not necessarily arrive within a
     * single process() call - the loop drains whatever the UART has buffered
     * and returns, which at 2 Mbaud is regularly in the middle of them.  So the
     * accumulator has to be state, not a local.
     */
    uint16_t received_crc_;
};

} // namespace libecu

#endif // LIBECU_AT_COMMAND_PROCESSOR_HPP
