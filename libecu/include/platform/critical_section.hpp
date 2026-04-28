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

namespace libecu {

/**
 * @brief RAII-style critical section guard
 * Automatically disables interrupts on construction and restores on destruction
 */
class CriticalSection {
public:
    CriticalSection() {
        disable_interrupts();
    }

    ~CriticalSection() {
        enable_interrupts();
    }

    // Non-copyable, non-movable
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
    CriticalSection(CriticalSection&&) = delete;
    CriticalSection& operator=(CriticalSection&&) = delete;
};

} // namespace libecu

#endif // LIBECU_CRITICAL_SECTION_HPP
