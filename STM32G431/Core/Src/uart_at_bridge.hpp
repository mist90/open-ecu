#ifndef UART_AT_BRIDGE_HPP
#define UART_AT_BRIDGE_HPP

#include "../../../libecu/include/at_command_processor.hpp"
#include "../Inc/main.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern uint8_t dma_rx_buffer[256];
extern volatile uint16_t dma_rx_index;

// Forward declarations for interrupt control functions
extern "C" void disable_interrupts();
extern "C" void enable_interrupts();

namespace libecu {

class UartAtBridge : public AtCommandProcessor {
public:
    explicit UartAtBridge(BldcController* controller) noexcept
        : AtCommandProcessor(controller) {}

    int32_t read() override {
        int32_t ch = -1;

        disable_interrupts();

        uint16_t cndtr = hdma_usart2_rx.Instance->CNDTR;
        uint16_t write_pos = 256 - cndtr;

        if (dma_rx_index != write_pos) {
            ch = static_cast<int32_t>(dma_rx_buffer[dma_rx_index]);
            dma_rx_index = (dma_rx_index + 1) % 256;
        }

        enable_interrupts();

        return ch;
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