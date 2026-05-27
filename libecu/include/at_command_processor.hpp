/**
 * @file at_command_processor.hpp
 * @brief AT command processor for motor controller communication
 */

#ifndef LIBECU_AT_COMMAND_PROCESSOR_HPP
#define LIBECU_AT_COMMAND_PROCESSOR_HPP

#include <cstddef>
#include <cstdint>

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
     * @param duty_cycle Current duty cycle
     * @param target_current Target current
     * @param measured_current Measured current
     * @param position Rotor position
     */
    void captureOscSample(float duty_cycle, float target_current, float measured_current, uint8_t position) noexcept;

    /**
     * @brief Process oscilloscope output (send one sample per call)
     */
    void processOscOutput() noexcept;

    /**
     * @brief Swap oscilloscope buffers (for ISR use)
     */
    void swapOscBuffers() noexcept;

    // Configuration constants
    static constexpr std::size_t MAX_COMMAND_LENGTH = 64;
    static constexpr std::size_t OSC_BUFFER_SIZE = 256;
    static constexpr std::size_t OSC_NUM_BUFFERS = 2;

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

    // Locally-tracked state (mirrors BldcController, avoids private access)
    uint8_t tracked_drive_mode_;  // 0=FORWARD, 1=REVERSE, 2=NEUTRAL
    float tracked_pid_kp_;
    float tracked_pid_ki_;
    float tracked_pid_kd_;

    // Oscilloscope double buffering
    struct OscSample {
        float duty_cycle;
        float target_current;
        float measured_current;
        uint8_t position;
    };

    OscSample osc_buffers_[OSC_NUM_BUFFERS][OSC_BUFFER_SIZE];
    volatile std::size_t osc_write_buffer_;
    std::size_t osc_write_index_;
    std::size_t osc_read_index_;
    volatile bool osc_buffers_swapped_;
    std::int32_t osc_sample_counter_;
    uint8_t crc_index_;  ///< Index for accumulating CRC hex digits
};

} // namespace libecu

#endif // LIBECU_AT_COMMAND_PROCESSOR_HPP