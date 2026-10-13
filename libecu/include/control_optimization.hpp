/** @file control_optimization.hpp
 * @brief Speed optimization for bounded numerical kernels in a size-optimized image
 */
#ifndef LIBECU_CONTROL_OPTIMIZATION_HPP
#define LIBECU_CONTROL_OPTIMIZATION_HPP

// GCC -Os outlines many saturating arithmetic operations. Keep the image at
// -Os, but let the two largest 20 kHz kernels simplify/in-line at -O2. Debug
// builds retain -O0; other compilers use their selected optimization policy.
#if defined(__GNUC__) && !defined(__clang__) && defined(__OPTIMIZE__)
#define LIBECU_HOT_LOOP [[gnu::optimize("O2")]]
#else
#define LIBECU_HOT_LOOP
#endif

#endif // LIBECU_CONTROL_OPTIMIZATION_HPP
