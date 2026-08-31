#include "../include/at_command_processor.hpp"
#include "../include/bldc_controller.hpp"
#include "../include/crc16.hpp"
#include "../include/critical_section.hpp"
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
      pll_telemetry_enabled_(false),
      hall_telemetry_enabled_(false),
      osc_write_index_(0),
      osc_read_index_(0),
      osc_phase_(OscPhase::Accumulating),
      osc_sample_counter_(0),
      crc_index_(0),
      received_crc_(0) {
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

            if (crc_index_ == 0) {
                received_crc_ = 0;
            }
            received_crc_ = static_cast<uint16_t>((received_crc_ << 4) | hexValue(c));
            crc_index_++;

            if (crc_index_ < 4) {
                break;
            }

            const uint16_t received_crc = received_crc_;
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
    Spd, Cur, Dut, Mode, EMode, DMode, Spid, Cpid, PllId, Pll, HStatus, HClear, Ver, Status, Tm, Osc, Maxvals,
    Algo, Fpid, Fang, Acc
};

CommandId matchCommand(const char* cmd) noexcept {
    if (std::strncmp(cmd, "MAXVALS", 7) == 0) return CommandId::Maxvals;
    if (std::strncmp(cmd, "ACC", 3) == 0) return CommandId::Acc;
    if (std::strncmp(cmd, "ALGO", 4) == 0) return CommandId::Algo;
    if (std::strncmp(cmd, "FPID", 4) == 0) return CommandId::Fpid;
    if (std::strncmp(cmd, "FANG", 4) == 0) return CommandId::Fang;
    if (std::strncmp(cmd, "DMODE", 5) == 0) return CommandId::DMode;
    if (std::strncmp(cmd, "EMODE", 5) == 0) return CommandId::EMode;
    if (std::strncmp(cmd, "HSTATUS", 7) == 0) return CommandId::HStatus;
    if (std::strncmp(cmd, "HCLEAR", 6) == 0) return CommandId::HClear;
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
                            id == CommandId::PllId || id == CommandId::Acc ||
                            id == CommandId::Status || id == CommandId::Maxvals);
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

    case CommandId::Acc: {
        // Slew rate limit on the *velocity setpoint*, RPS/s. Defaults to
        // BLDC_MAX_ACCELERATION; 0 disables the limiter, which makes AT+SPD a
        // step command.
        if (query) {
            sendFloatResponse("ACC", controller_->getAccelerationRate());
        } else {
            float val = parseFloatParam(valuePtr);
            if (val < 0.0f || val > 1000.0f) {
                sendError();
                return;
            }
            controller_->setAccelerationRate(val);
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

    case CommandId::Algo: {
        if (query) {
            sendIntResponse("ALGO", static_cast<int>(controller_->getAlgorithm()));
        } else {
            int val = parseIntParam(valuePtr);
            if (val < 0 || val > 1) {
                sendError();
                return;
            }
            controller_->setAlgorithm(static_cast<DriveAlgorithm>(val));
            sendOk();
        }
        break;
    }

    case CommandId::Fpid: {
        // FOC d/q current regulator, in volts per amp - not interchangeable
        // with CPID, which outputs duty. See tuneFocCurrentPi().
        if (query) {
            float kp, ki;
            controller_->getFocCurrentPi(kp, ki);
            char buf[64];
            int len = std::snprintf(buf, sizeof(buf), "+FPID:%.4f,%.4f\r\n", kp, ki);
            if (len > 0) write(buf, static_cast<std::size_t>(len));
            sendOk();
        } else {
            if (!valuePtr) { sendError(); return; }
            float kp = std::strtof(valuePtr, nullptr);
            float ki = 0.0f;
            const char* comma = std::strchr(valuePtr, ',');
            if (comma) ki = std::strtof(comma + 1, nullptr);
            controller_->setFocCurrentPi(kp, ki);
            sendOk();
        }
        break;
    }

    case CommandId::Fang: {
        // Electrical angle offset between the PLL step angle and the rotor d
        // axis, in electrical degrees. Expect to trim this on the bench; see
        // foc_algorithm.hpp for where the -60 default comes from.
        if (query) {
            sendFloatResponse("FANG", controller_->getFocAngleOffsetDeg());
        } else {
            float val = parseFloatParam(valuePtr);
            if (val < -360.0f || val > 360.0f) {
                sendError();
                return;
            }
            controller_->setFocAngleOffsetDeg(val);
            sendOk();
        }
        break;
    }

    case CommandId::DMode: {
        if (query) {
            sendIntResponse("DMODE", static_cast<int>(controller_->getDriveMode()));
        } else {
            int val = parseIntParam(valuePtr);
            if (val < 0 || val > 2) {
                sendError();
                return;
            }
            controller_->setDriveMode(static_cast<DriveMode>(val));
            sendOk();
        }
        break;
    }

    case CommandId::Spid: {
        if (query) {
            float kp, ki, kd;
            controller_->getSpeedPidGains(kp, ki, kd);
            char buf[64];
            int len = std::snprintf(buf, sizeof(buf), "+SPID:%.3f,%.3f,%.3f\r\n",
                kp, ki, kd);
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
            controller_->setSpeedPid(kp, ki, kd);
            sendOk();
        }
        break;
    }

    case CommandId::Cpid: {
        if (query) {
            float kp, ki, kd;
            controller_->getCurrentPidGains(kp, ki, kd);
            char buf[64];
            int len = std::snprintf(buf, sizeof(buf), "+CPID:%.3f,%.3f,%.3f\r\n",
                kp, ki, kd);
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
            controller_->setCurrentPid(kp, ki, kd);
            sendOk();
        }
        break;
    }

    case CommandId::PllId: {
        if (query) {
            float kp, ki;
            controller_->getPllBaseGains(kp, ki);
            char buf[64];
            int len = std::snprintf(buf, sizeof(buf), "+PLLID:%.3f,%.3f\r\n",
                kp, ki);
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
        const MotorControlParams& params = controller_->getParams();
        char buf[96];
        int len = std::snprintf(buf, sizeof(buf),
            "+MAXVALS:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            params.max_speed_rps,
            params.min_current,
            params.max_current,
            params.max_voltage,
            params.max_duty_cycle,
            params.max_temperature_c);
        if (len > 0) write(buf, static_cast<std::size_t>(len));
        break;
    }

    case CommandId::Status: {
        MotorStatus status = controller_->getStatus();
        // Algorithm and the d/q pair are appended rather than woven in, so a
        // consumer that only knows the original six fields still parses. This
        // is a one-shot query, not a stream - +TM is deliberately left alone,
        // because at 100 Hz it aliases everything FOC does anyway and +OSC is
        // the view that can actually see it.
        char buf[160];
        int len = std::snprintf(buf, sizeof(buf),
            "+STATUS:%d,%d,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.1f\r\n",
            static_cast<int>(status.control_mode),
            static_cast<int>(status.electric_mode),
            status.current_speed_rps,
            status.target_current,
            status.duty_cycle,
            status.bus_voltage,
            static_cast<int>(status.algorithm),
            status.i_d,
            status.i_q,
            status.angle_e_rad * (180.0f / FOC_PI));
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

    case CommandId::HStatus: {
        if (query) {
            sendIntResponse("HSTATUS", hall_telemetry_enabled_ ? 1 : 0);
        } else {
            int val = parseIntParam(valuePtr);
            setHallTelemetryEnabled(val == 1);
            sendOk();
        }
        break;
    }

    case CommandId::HClear: {
        if (controller_) {
            controller_->clearHallFault();
            sendOk();
        } else {
            sendError();
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
    // Temperature is appended, not woven in, so a host that only knows the
    // original nine fields still parses the line.
    int len = std::snprintf(buf, sizeof(buf), "+TM:%u;%u;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.1f;%.1f\n",
            static_cast<unsigned>(status.measured_position),
            static_cast<unsigned>(status.target_position),
            status.target_speed_rps,
            status.current_speed_rps,
            status.duty_cycle,
            status.target_current,
            status.measured_current_filtered,
            status.bus_voltage,
            status.pll_angle,
            status.temperature_c);

    if (len > 0 && static_cast<std::size_t>(len) < sizeof(buf)) {
        write(buf, static_cast<std::size_t>(len));
    }
}

void AtCommandProcessor::sendPllTelemetry(const MotorPLL::PllInfo& info) noexcept {
    if (!pll_telemetry_enabled_) {
        return;
    }

    char buf[64];
    int len = std::snprintf(buf, sizeof(buf), "+PLL:%.3f;%.3f;%.3f;%u;%d\n",
            info.angle_per_second,
            info.pll_integral,
            info.angle,
            static_cast<unsigned>(info.hall_state_raw),
            static_cast<int>(info.is_sync));

    if (len > 0 && static_cast<std::size_t>(len) < sizeof(buf)) {
        write(buf, static_cast<std::size_t>(len));
    }
}

void AtCommandProcessor::sendHallTelemetry(const HallMonitor::Info& info) noexcept {
    if (!hall_telemetry_enabled_) {
        return;
    }

    char buf[96];
    int len = std::snprintf(buf, sizeof(buf), "+HSTATUS:%d;%.2f;%.2f;%.2f;%lu;%lu;%u;%.0f\n",
            static_cast<int>(info.fault),
            info.invalid_score,
            info.erratic_score,
            info.edge_accum,
            static_cast<unsigned long>(info.invalid_events),
            static_cast<unsigned long>(info.edges),
            static_cast<unsigned>(info.last_position),
            info.invalid_time_s * 1e6f);

    if (len > 0 && static_cast<std::size_t>(len) < sizeof(buf)) {
        write(buf, static_cast<std::size_t>(len));
    }
}

void AtCommandProcessor::setHallTelemetryEnabled(bool enabled) noexcept {
    hall_telemetry_enabled_ = enabled;
}

bool AtCommandProcessor::isHallTelemetryEnabled() const noexcept {
    return hall_telemetry_enabled_;
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

namespace {

/// Amperes to the int16 milliamp field, saturating rather than wrapping
inline int16_t oscMilliamps(float amps) noexcept {
    float ma = amps * 1000.0f;
    if (ma > 32767.0f)  ma =  32767.0f;
    if (ma < -32768.0f) ma = -32768.0f;
    return static_cast<int16_t>(ma);
}

} // namespace

void AtCommandProcessor::captureOscSample(const MotorStatus& status) noexcept {
    if (osc_phase_ != OscPhase::Accumulating)
        return;

    if (osc_write_index_ >= OSC_BUFFER_SIZE)
        return;

    OscSample& sample = osc_buffer_[osc_write_index_];
    sample.duty_cycle = static_cast<uint8_t>(status.duty_cycle * 100.0f);
    sample.target_current_ma = oscMilliamps(status.target_current);
    sample.measured_current_ma = oscMilliamps(status.measured_current);
    sample.i_d_ma = oscMilliamps(status.i_d);
    sample.i_q_ma = oscMilliamps(status.i_q);

    // Electrical angle folded to one revolution and scaled to 16 bits, so the
    // consumer divides by 65536 to get revolutions without ever seeing a wrap
    // discontinuity that is not real.
    float rev = status.angle_e_rad * (1.0f / FOC_TWO_PI);
    rev -= static_cast<float>(static_cast<int32_t>(rev));
    if (rev < 0.0f) rev += 1.0f;
    sample.angle_e = static_cast<uint16_t>(rev * 65536.0f);

    sample.position = status.measured_position;

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

    // The first five fields are unchanged from before FOC existed, so an older
    // consumer that splits on commas and takes what it knows still works; the
    // three FOC fields are appended. Angle is in electrical degrees.
    const int angle_deg_e = static_cast<int>(
        (static_cast<float>(sample.angle_e) * (360.0f / 65536.0f)));

    char out_buf[96];
    int len = std::snprintf(out_buf, sizeof(out_buf), "+OSC:%d,%d,%d,%u,%u,%d,%d,%d\r\n",
        static_cast<int>(osc_sample_counter_),
        static_cast<int>(sample.measured_current_ma),
        static_cast<int>(sample.target_current_ma),
        static_cast<unsigned>(sample.duty_cycle),
        static_cast<unsigned>(sample.position),
        static_cast<int>(sample.i_d_ma),
        static_cast<int>(sample.i_q_ma),
        angle_deg_e);

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
