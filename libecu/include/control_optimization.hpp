/** @file control_optimization.hpp
 * @brief Speed optimization for bounded numerical kernels in a size-optimized image
 */
#ifndef LIBECU_CONTROL_OPTIMIZATION_HPP
#define LIBECU_CONTROL_OPTIMIZATION_HPP

// Native telemetry can send dq voltage components and let the host calculate
// the scalar magnitude. Per-phase PWM duties and every control value remain
// available. Zero selects no scalar FOC duty telemetry (reported as zero).
#ifndef LIBECU_FOC_DUTY_TELEMETRY
#define LIBECU_FOC_DUTY_TELEMETRY 1
#endif

// Port opt-in: the link script must collect .RamFunc.* and .data.* in
// executable SRAM, with a flash load image copied before any interrupts.
// Keep the annotated entry out of line so LTO cannot silently put its body
// back into its flash caller. Use ordinary (non-COMDAT) definitions for LUTs.
#if defined(__GNUC__) && defined(LIBECU_PWM_RAM) && LIBECU_PWM_RAM
#define LIBECU_RAM_CODE(name) __attribute__((section(".RamFunc." name), noinline))
#define LIBECU_RAM_DATA(name) __attribute__((section(".data.libecu." name)))
#define LIBECU_RAM_INLINE __attribute__((always_inline))
#else
#define LIBECU_RAM_CODE(name)
#define LIBECU_RAM_DATA(name)
#define LIBECU_RAM_INLINE
#endif

// In tight SRAM layouts reserve the deadline path for code needed in the PWM frame.
#if defined(LIBECU_PWM_DEADLINE_ONLY) && LIBECU_PWM_DEADLINE_ONLY
#define LIBECU_DEADLINE_CODE(name) LIBECU_RAM_CODE(name)
#define LIBECU_DEADLINE_INLINE __attribute__((always_inline))
#else
#define LIBECU_DEADLINE_CODE(name)
#define LIBECU_DEADLINE_INLINE
#endif

// GCC -Os outlines many saturating arithmetic operations. Keep the image at
// -Os, but let the two largest 20 kHz kernels simplify/in-line at -O2. Debug
// builds retain -O0; other compilers use their selected optimization policy.
// A measured target may define LIBECU_HOT_OPTIMIZE_O3 for these same kernels;
// foreground/configuration code retains its size policy.
#if defined(__GNUC__) && !defined(__clang__) && defined(__OPTIMIZE__)
#if defined(LIBECU_HOT_OPTIMIZE_O3)
#define LIBECU_HOT_LOOP [[gnu::optimize("O3")]]
#else
#define LIBECU_HOT_LOOP [[gnu::optimize("O2")]]
#endif
// Configuration and slow-loop work need not inflate the protected firmware image.
#define LIBECU_SLOW_PATH [[gnu::optimize("Os")]]
#else
#define LIBECU_HOT_LOOP
#define LIBECU_SLOW_PATH
#endif

#endif // LIBECU_CONTROL_OPTIMIZATION_HPP
