#ifndef UART_AT_BRIDGE_HPP
#define UART_AT_BRIDGE_HPP

#include "../../../libecu/include/at_command_processor.hpp"
#include "../Inc/main.h"

extern UART_HandleTypeDef huart2;

namespace libecu {

class UartAtBridge : public AtCommandProcessor {
public:
    explicit UartAtBridge(BldcController* controller) noexcept
        : AtCommandProcessor(controller) {}

    int32_t read() override {
        uint8_t byte;
        // Using same pattern as __io_getchar in main.cpp
        // HAL_UART_Receive with 0 timeout = non-blocking
        // Returns -1 if no data available
        if (HAL_UART_Receive(&huart2, &byte, 1, 0) == HAL_OK) {
            return static_cast<int32_t>(byte);
        }
        return -1;
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