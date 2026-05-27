#ifndef UART_AT_BRIDGE_HPP
#define UART_AT_BRIDGE_HPP

#include "../../../libecu/include/at_command_processor.hpp"
#include "../Inc/main.h"

extern UART_HandleTypeDef huart2;

extern "C" {
    int __io_getchar(void);
}

namespace libecu {

class UartAtBridge : public AtCommandProcessor {
public:
    explicit UartAtBridge(BldcController* controller) noexcept
        : AtCommandProcessor(controller) {}

    int32_t read() override {
        return static_cast<int32_t>(__io_getchar());
    }

    void write(const char* str, std::size_t len) override {
        // Use HAL_UART_Transmit for bulk write (more efficient than per-byte)
        HAL_UART_Transmit(&huart2,
            reinterpret_cast<const uint8_t*>(str),
            static_cast<uint16_t>(len),
            HAL_MAX_DELAY);
    }
};

} // namespace libecu

#endif // UART_AT_BRIDGE_HPP