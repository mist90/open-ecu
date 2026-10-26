/** @file console_libc.cpp
 * @brief Heapless adapters for the common ECU's printf and strtof API
 *
 * Selected only by this target's linker --wrap options. Newlib-nano's float
 * printf/strtof allocate internally; these implementations keep all work on
 * the stack and allow _sbrk to reject every request.
 */
#define NANOPRINTF_IMPLEMENTATION
#include "nanoprintf.h"
#include "fast_float/fast_float.h"
#include <cerrno>
#include <cstring>

extern "C" int board_console_write(const char* data, int length);
namespace {
void consoleCharacter(int character, void*) {
    const char byte = static_cast<char>(character);
    board_console_write(&byte, 1);
}
}

extern "C" int __wrap_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    const int result = npf_vpprintf(consoleCharacter, nullptr, format, args);
    va_end(args);
    return result;
}
extern "C" int __wrap_snprintf(char* output, size_t size, const char* format, ...) {
    va_list args;
    va_start(args, format);
    const int result = npf_vsnprintf(output, size, format, args);
    va_end(args);
    return result;
}
extern "C" int __wrap_puts(const char* text) {
    const int result = board_console_write(text, static_cast<int>(std::strlen(text)));
    return result < 0 ? result : board_console_write("\n", 1);
}
extern "C" int __wrap_putchar(int character) {
    const char byte = static_cast<char>(character);
    return board_console_write(&byte, 1) < 0 ? -1 : character;
}
extern "C" float __wrap_strtof(const char* text, char** end) {
    float value = 0.0f;
    const auto result = fast_float::from_chars(text, text + std::strlen(text), value,
        fast_float::chars_format::general | fast_float::chars_format::skip_white_space |
        fast_float::chars_format::allow_leading_plus);
    if (end) *end = const_cast<char*>(result.ec == std::errc::invalid_argument ? text : result.ptr);
    if (result.ec == std::errc::result_out_of_range) errno = ERANGE;
    return value;
}
