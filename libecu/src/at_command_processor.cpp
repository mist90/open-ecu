#include "../include/at_command_processor.hpp"
#include "../include/bldc_controller.hpp"
#include "../include/platform/crc16.hpp"
#include "../include/platform/critical_section.hpp"
#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace libecu {

AtCommandProcessor::AtCommandProcessor(BldcController* controller) noexcept
    : controller_(controller),
      state_(ParseState::Idle),
      command_buffer_{},
      cmd_index_(0),
      telemetry_enabled_(true),
      osc_streaming_(false),
      pll_telemetry_enabled_(true),
      tracked_drive_mode_(2),
      tracked_spid_kp_(0.01f),
      tracked_spid_ki_(0.1f),
      tracked_spid_kd_(0.0f),
      tracked_cpid_kp_(0.01f),
      tracked_cpid_ki_(0.1f),
      tracked_cpid_kd_(0.0f),
      tracked_pll_kp_(200.0f),
      tracked_pll_ki_(10000.0f),
      osc_write_index_(0),
      osc_read_index_(0),
      osc_phase_(OscPhase::Accumulating),
      osc_sample_counter_(0),
      crc_index_(0) {
}

void AtCommandProcessor::process() noexcept {
    while (true) {
        int32_t ch = read();
        if (ch < 0) {
            return;
        }

        char c = static_cast<char>(ch);

        switch (state_) {
        case ParseState::Idle:
            if (c == '\r') {
                break;
            }
            if (c == '\n') {
                break;
            }
            cmd_index_ = 0;
            command_buffer_[cmd_index_++] = c;
            state_ = ParseState::Receiving;
            break;

        case ParseState::Receiving:
            if (c == '\r') {
                break;
            }
            if (c == '\n') {
                command_buffer_[cmd_index_] = '\0';
                state_ = ParseState::CrcParsing;
                break;
            }
            if (c == '*') {
                command_buffer_[cmd_index_] = '\0';
                crc_index_ = 0;
                state_ = ParseState::CrcAccumulating;
                break;
            }
            if (cmd_index_ < MAX_COMMAND_LENGTH - 1) {
                command_buffer_[cmd_index_++] = c;
            } else {
                cmd_index_ = 0;
                state_ = ParseState::Idle;
                sendError();
            }
            break;

        case ParseState::CrcParsing:
            command_buffer_[cmd_index_] = '\0';
            for (std::size_t i = 0; i < cmd_index_; ++i) {
                if (command_buffer_[i] == '*') {
                    command_buffer_[i] = '\0';
                    crc_index_ = 0;
                    state_ = ParseState::CrcAccumulating;
                    break;
                }
            }
            if (state_ == ParseState::CrcParsing) {
                sendError();
                cmd_index_ = 0;
                state_ = ParseState::Idle;
            }
            break;

        case ParseState::CrcAccumulating: {
            if (!isHexChar(c)) {
                sendError();
                cmd_index_ = 0;
                state_ = ParseState::Idle;
                break;
            }

            char crc_buf[5];
            crc_buf[crc_index_] = c;
            crc_index_++;

            if (crc_index_ < 4) {
                break;
            }

            crc_buf[4] = '\0';

            uint16_t received_crc = 0;
            for (int i = 0; i < 4; ++i) {
                received_crc = (received_crc << 4) | hexValue(crc_buf[i]);
            }

            std::size_t cmd_len = std::strlen(command_buffer_);
            uint16_t computed_crc = crc16_compute(
                reinterpret_cast<const uint8_t*>(command_buffer_), cmd_len);

            if (computed_crc != received_crc) {
                sendError();
                cmd_index_ = 0;
                state_ = ParseState::Idle;
                break;
            }

            cmd_index_ = 0;
            processCommand();
            state_ = ParseState::Idle;
            break;
        }

        case ParseState::Execute:
            state_ = ParseState::Idle;
            break;
        }
    }
}

char AtCommandProcessor::hexChar(uint8_t nibble) const noexcept {
    return nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
}

uint8_t AtCommandProcessor::hexValue(char c) const noexcept {
    if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
    return 0xFF;
}

bool AtCommandProcessor::isHexChar(char c) const noexcept {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

void AtCommandProcessor::sendOk() noexcept {
    write("OK\r\n", 4);
}

void AtCommandProcessor::sendError() noexcept {
    write("ERROR\r\n", 7);
}

void AtCommandProcessor::sendResponse(const char* prefix, const char* value) noexcept {
    write("+", 1);
    write(prefix, std::strlen(prefix));
    write(":", 1);
    write(value, std::strlen(value));
    write("\r\n", 2);
}

namespace {

/** Command IDs for AT command dispatch */
enum class CommandId : uint8_t {
    Unknown,
    Spd, Cur, Dut, Mode, EMode, DMode, Spid, Cpid, PllId, Pll, Ver, Status, Tm, Osc, Maxvals
};

CommandId matchCommand(const char* cmd) noexcept {
    if (std::strncmp(cmd, "MAXVALS", 7) == 0) return CommandId::Maxvals;
    if (std::strncmp(cmd, "DMODE", 5) == 0) return CommandId::DMode;
    if (std::strncmp(cmd, "EMODE", 5) == 0) return CommandId::EMode;
    if (std::strncmp(cmd, "STATUS", 6) == 0) return CommandId::Status;
    if (std::strncmp(cmd, "SPID", 4) == 0) return CommandId::Spid;
    if (std::strncmp(cmd, "CPID", 4) == 0) return CommandId::Cpid;
    if (std::strncmp(cmd, "PLLID", 5) == 0) return CommandId::PllId;
    if (std::strncmp(cmd, "SPD", 3) == 0) return CommandId::Spd;
    if (std::strncmp(cmd, "CUR", 3) == 0) return CommandId::Cur;
    if (std::strncmp(cmd, "DUT", 3) == 0) return CommandId::Dut;
    if (std::strncmp(cmd, "MODE", 4) == 0) return CommandId::Mode;
    if (std::strncmp(cmd, "VER", 3) == 0) return CommandId::Ver;
    if (std::strncmp(cmd, "TM", 2) == 0) return CommandId::Tm;
    if (std::strncmp(cmd, "OSC", 3) == 0) return CommandId::Osc;
    if (std::strncmp(cmd, "PLL", 3) == 0) return CommandId::Pll;
    return CommandId::Unknown;
}

bool isQuery(const char* cmd) noexcept {
    std::size_t len = std::strlen(cmd);
    return len > 0 && cmd[len - 1] == '?';
}

const char* getValuePtr(const char* cmd) noexcept {
    for (std::size_t i = 0; cmd[i]; i++) {
        if (cmd[i] == '=') return cmd + i + 1;
    }
    return nullptr;
}

float parseFloatParam(const char* str) noexcept {
    if (!str) return 0.0f;
    return std::strtof(str, nullptr);
}

int parseIntParam(const char* str) noexcept {
    if (!str) return 0;
    return static_cast<int>(std::strtol(str, nullptr, 10));
}

} // namespace

void AtCommandProcessor::sendFloatResponse(const char* prefix, float value) noexcept {
    char buf[64];
    int len = std::snprintf(buf, sizeof(buf), "+%s:%.2f\r\n", prefix, value);
    if (len > 0) write(buf, static_cast<std::size_t>(len));
}

void AtCommandProcessor::sendIntResponse(const char* prefix, int value) noexcept {
    char buf[32];
    int len = std::snprintf(buf, sizeof(buf), "+%s:%d\r\n", prefix, value);
    if (len > 0) write(buf, static_cast<std::size_t>(len));
}

void AtCommandProcessor::processCommand() noexcept {
    if (std::strncmp(command_buffer_, "AT+", 3) != 0) {
        sendError();
        return;
    }

    const char* cmd = command_buffer_ + 3;
    CommandId id = matchCommand(cmd);

    if (id == CommandId::Unknown) {
        sendError();
        return;
    }

    bool query = isQuery(cmd);
    const char* valuePtr = nullptr;
    if (!query) {
        valuePtr = getValuePtr(cmd);
    }

    // Guard: commands that need controller_
    bool needsController = (id == CommandId::Spd || id == CommandId::Cur ||
                            id == CommandId::Dut || id == CommandId::Mode ||
                            id == CommandId::EMode || id == CommandId::DMode ||
                            id == CommandId::Spid || id == CommandId::Cpid ||
                            id == CommandId::PllId ||
                            id == CommandId::Status);
    if (needsController && controller_ == nullptr) {
        sendError();
        return;
    }

    switch (id) {
    case CommandId::Spd: {
        if (query) {
            MotorStatus status = controller_->getStatus();
            sendFloatResponse("SPD", status.current_speed_rps);
        } else {
            float val = parseFloatParam(valuePtr);
            if (val < 0.0f || val > 200.0f) {
                sendError();
                return;
            }
            controller_->setTargetSpeed(val);
            sendOk();
        }
        break;
    }

    case CommandId::Cur: {
        if (query) {
            MotorStatus status = controller_->getStatus();
            sendFloatResponse("CUR", status.target_current);
        } else {
            float val = parseFloatParam(valuePtr);
            if (val < -6.0f || val > 6.0f) {
                sendError();
                return;
            }
            controller_->setCurrent(val);
            sendOk();
        }
        break;
    }

    case CommandId::Dut: {
        if (query) {
            MotorStatus status = controller_->getStatus();
            sendFloatResponse("DUT", status.duty_cycle);
        } else {
            float val = parseFloatParam(valuePtr);
            if (val < 0.0f || val > 1.0f) {
                sendError();
                return;
            }
            controller_->setDutyCycle(val);
            sendOk();
        }
        break;
    }

    case CommandId::Mode: {
        if (query) {
            MotorStatus status = controller_->getStatus();
            sendIntResponse("MODE", static_cast<int>(status.control_mode));
        } else {
            int val = parseIntParam(valuePtr);
            if (val < 0 || val > 2) {
                sendError();
                return;
            }
            controller_->setControlMode(static_cast<ControlMode>(val));
            sendOk();
        }
        break;
    }

    case CommandId::EMode: {
        if (query) {
            MotorStatus status = controller_->getStatus();
            sendIntResponse("EMODE", static_cast<int>(status.electric_mode));
        } else {
            int val = parseIntParam(valuePtr);
            if (val < 0 || val > 1) {
                sendError();
                return;
            }
            controller_->setElectricMode(static_cast<ElectricMode>(val));
            sendOk();
        }
        break;
    }

    case CommandId::DMode: {
        if (query) {
            sendIntResponse("DMODE", static_cast<int>(tracked_drive_mode_));
        } else {
            int val = parseIntParam(valuePtr);
            if (val < 0 || val > 2) {
                sendError();
                return;
            }
            controller_->setDriveMode(static_cast<DriveMode>(val));
            tracked_drive_mode_ = static_cast<uint8_t>(val);
            sendOk();
        }
        break;
    }

    case CommandId::Spid: {
        if (query) {
            char buf[64];
            int len = std::snprintf(buf, sizeof(buf), "+SPID:%.3f,%.3f,%.3f\r\n",
                tracked_spid_kp_, tracked_spid_ki_, tracked_spid_kd_);
            if (len > 0) write(buf, static_cast<std::size_t>(len));
            sendOk();
        } else {
            if (!valuePtr) { sendError(); return; }
            float kp = std::strtof(valuePtr, nullptr);
            float ki = 0.0f, kd = 0.0f;
            const char* comma = std::strchr(valuePtr, ',');
            if (comma) {
                ki = std::strtof(comma + 1, nullptr);
                const char* comma2 = std::strchr(comma + 1, ',');
                if (comma2) kd = std::strtof(comma2 + 1, nullptr);
            }
            tracked_spid_kp_ = kp;
            tracked_spid_ki_ = ki;
            tracked_spid_kd_ = kd;
            controller_->setSpeedPid(kp, ki, kd);
            sendOk();
        }
        break;
    }

    case CommandId::Cpid: {
        if (query) {
            char buf[64];
            int len = std::snprintf(buf, sizeof(buf), "+CPID:%.3f,%.3f,%.3f\r\n",
                tracked_cpid_kp_, tracked_cpid_ki_, tracked_cpid_kd_);
            if (len > 0) write(buf, static_cast<std::size_t>(len));
            sendOk();
        } else {
            if (!valuePtr) { sendError(); return; }
            float kp = std::strtof(valuePtr, nullptr);
            float ki = 0.0f, kd = 0.0f;
            const char* comma = std::strchr(valuePtr, ',');
            if (comma) {
                ki = std::strtof(comma + 1, nullptr);
                const char* comma2 = std::strchr(comma + 1, ',');
                if (comma2) kd = std::strtof(comma2 + 1, nullptr);
            }
            tracked_cpid_kp_ = kp;
            tracked_cpid_ki_ = ki;
            tracked_cpid_kd_ = kd;
            controller_->setCurrentPid(kp, ki, kd);
            sendOk();
        }
        break;
    }

    case CommandId::PllId: {
        if (query) {
            char buf[64];
            int len = std::snprintf(buf, sizeof(buf), "+PLLID:%.3f,%.3f\r\n",
                tracked_pll_kp_, tracked_pll_ki_);
            if (len > 0) write(buf, static_cast<std::size_t>(len));
            sendOk();
        } else {
            if (!valuePtr) { sendError(); return; }
            float kp = std::strtof(valuePtr, nullptr);
            float ki = 0.0f;
            const char* comma = std::strchr(valuePtr, ',');
            if (comma) {
                ki = std::strtof(comma + 1, nullptr);
            }
            tracked_pll_kp_ = kp;
            tracked_pll_ki_ = ki;
            controller_->setPllGains(kp, ki);
            sendOk();
        }
        break;
    }

    case CommandId::Ver: {
        sendResponse("VER", "1.0.0");
        break;
    }

    case CommandId::Maxvals: {
        sendResponse("MAXVALS", "200.0,-6.0,6.0,36.0,0.95");
        break;
    }

    case CommandId::Status: {
        MotorStatus status = controller_->getStatus();
        char buf[128];
        int len = std::snprintf(buf, sizeof(buf),
            "+STATUS:%d,%d,%.2f,%.2f,%.2f,%.2f\r\n",
            static_cast<int>(status.control_mode),
            static_cast<int>(status.electric_mode),
            status.current_speed_rps,
            status.target_current,
            status.duty_cycle,
            status.bus_voltage);
        if (len > 0) write(buf, static_cast<std::size_t>(len));
        break;
    }

    case CommandId::Tm: {
        int val = parseIntParam(valuePtr);
        setTelemetryEnabled(val == 1);
        sendOk();
        break;
    }

    case CommandId::Osc: {
        if (query) {
            sendIntResponse("OSC", osc_streaming_ ? 1 : 0);
        } else {
            int val = parseIntParam(valuePtr);
            if (val == 1) {
                startOscilloscope();
            } else {
                stopOscilloscope();
            }
            sendOk();
        }
        break;
    }

    case CommandId::Pll: {
        if (query) {
            sendIntResponse("PLL", pll_telemetry_enabled_ ? 1 : 0);
        } else {
            int val = parseIntParam(valuePtr);
            setPllTelemetryEnabled(val == 1);
            sendOk();
        }
        break;
    }

    default:
        sendError();
        break;
    }
}

void AtCommandProcessor::sendTelemetry(const MotorStatus& status) noexcept {
    if (!telemetry_enabled_) {
        return;
    }

    char buf[128];
    int len = std::snprintf(buf, sizeof(buf), "+TM:%u;%u;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.1f\n",
            static_cast<unsigned>(status.measured_position),
            static_cast<unsigned>(status.target_position),
            status.target_speed_rps,
            status.current_speed_rps,
            status.duty_cycle,
            status.target_current,
            status.measured_current,
            status.bus_voltage,
            status.pll_angle);

    if (len > 0 && static_cast<std::size_t>(len) < sizeof(buf)) {
        write(buf, static_cast<std::size_t>(len));
    }
}

void AtCommandProcessor::sendPllTelemetry(const MotorPLL::PllInfo& info) noexcept {
    if (!pll_telemetry_enabled_) {
        return;
    }

    char buf[64];
    int len = std::snprintf(buf, sizeof(buf), "+PLL:%.3f;%.3f;%.4f;%.2f;%.2f\n",
            info.angle_per_second,
            info.pll_integral,
            info.time_since_last_hall,
            info.kp,
            info.ki);

    if (len > 0 && static_cast<std::size_t>(len) < sizeof(buf)) {
        write(buf, static_cast<std::size_t>(len));
    }
}

void AtCommandProcessor::setPllTelemetryEnabled(bool enabled) noexcept {
    pll_telemetry_enabled_ = enabled;
}

bool AtCommandProcessor::isPllTelemetryEnabled() const noexcept {
    return pll_telemetry_enabled_;
}

void AtCommandProcessor::setTelemetryEnabled(bool enabled) noexcept {
    telemetry_enabled_ = enabled;
}

bool AtCommandProcessor::isTelemetryEnabled() const noexcept {
    return telemetry_enabled_;
}

void AtCommandProcessor::startOscilloscope() noexcept {
    CriticalSection cs;
    osc_streaming_ = true;
    osc_phase_ = OscPhase::Accumulating;
    osc_write_index_ = 0;
    osc_read_index_ = 0;
    osc_sample_counter_ = 0;
}

void AtCommandProcessor::stopOscilloscope() noexcept {
    osc_streaming_ = false;
    osc_phase_ = OscPhase::Accumulating;
}

void AtCommandProcessor::captureOscSample(float duty_cycle, float target_current, float measured_current, uint8_t position) noexcept {
    if (osc_phase_ != OscPhase::Accumulating)
        return;

    if (osc_write_index_ >= OSC_BUFFER_SIZE)
        return;

    osc_buffer_[osc_write_index_].duty_cycle = duty_cycle;
    osc_buffer_[osc_write_index_].target_current = target_current;
    osc_buffer_[osc_write_index_].measured_current = measured_current;
    osc_buffer_[osc_write_index_].position = position;

    osc_write_index_++;

    if (osc_write_index_ >= OSC_BUFFER_SIZE) {
        osc_phase_ = OscPhase::Outputting;
    }
}

void AtCommandProcessor::processOscOutput() noexcept {
    if (!osc_streaming_)
        return;

    if (osc_phase_ != OscPhase::Outputting)
        return;

    const OscSample& sample = osc_buffer_[osc_read_index_];

    int32_t meas_cur_int = static_cast<int32_t>(sample.measured_current * 1000.0f);
    int32_t tgt_cur_int = static_cast<int32_t>(sample.target_current * 1000.0f);
    int32_t duty_int = static_cast<int32_t>(sample.duty_cycle * 1000.0f);

    char out_buf[64];
    int len = std::snprintf(out_buf, sizeof(out_buf), "+OSC:%d,%ld,%ld,%ld,%u\r\n",
        static_cast<int>(osc_sample_counter_),
        static_cast<long>(meas_cur_int),
        static_cast<long>(tgt_cur_int),
        static_cast<long>(duty_int),
        static_cast<unsigned>(sample.position));

    if (len > 0) {
        write(out_buf, static_cast<std::size_t>(len));
    }

    osc_read_index_++;
    osc_sample_counter_++;

    if (osc_read_index_ >= OSC_BUFFER_SIZE) {
        write("+OSC:\r\n", 7);

        CriticalSection cs;
        osc_read_index_ = 0;
        osc_write_index_ = 0;
        osc_sample_counter_ = 0;
        osc_phase_ = OscPhase::Accumulating;
    }
}

} // namespace libecu
