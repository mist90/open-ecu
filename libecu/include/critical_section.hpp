/**
 * @file critical_section.hpp
 * @brief Platform-independent critical section interface for interrupt control
 */

#ifndef LIBECU_CRITICAL_SECTION_HPP
#define LIBECU_CRITICAL_SECTION_HPP

#include <cstdint>

// Forward declarations of platform-specific interrupt control functions
// These must be implemented by the application layer
extern "C" void disable_interrupts();
extern "C" void enable_interrupts();
#if defined(LIBECU_CRITICAL_SECTION_TOKENS) && LIBECU_CRITICAL_SECTION_TOKENS
// Optional platform contract: each token restores its own entry mask. Tokens
// and legacy disable/enable pairs must be released in reverse acquisition order.
extern "C" uint32_t save_interrupt_state();
extern "C" void restore_interrupt_state(uint32_t state);
#endif

namespace libecu {

/**
 * @brief RAII-style critical section guard
 * Automatically disables interrupts on construction and restores on destruction
 */
class CriticalSection {
public:
#if defined(LIBECU_CRITICAL_SECTION_TOKENS) && LIBECU_CRITICAL_SECTION_TOKENS
    CriticalSection() noexcept : state_(save_interrupt_state()) {}

    ~CriticalSection() noexcept {
        restore_interrupt_state(state_);
    }
#else
    CriticalSection() noexcept {
        disable_interrupts();
    }

    ~CriticalSection() noexcept {
        enable_interrupts();
    }
#endif

    // Non-copyable, non-movable
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
    CriticalSection(CriticalSection&&) = delete;
    CriticalSection& operator=(CriticalSection&&) = delete;

#if defined(LIBECU_CRITICAL_SECTION_TOKENS) && LIBECU_CRITICAL_SECTION_TOKENS
private:
    const uint32_t state_;
#endif
};

} // namespace libecu

#endif // LIBECU_CRITICAL_SECTION_HPP
