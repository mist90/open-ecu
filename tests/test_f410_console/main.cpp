/** @file main.cpp
 * @brief F410 heapless AT-console numeric formatting/parsing contracts
 */
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <initializer_list>

extern "C" int __wrap_snprintf(char*, size_t, const char*, ...);
extern "C" float __wrap_strtof(const char*, char**);
extern "C" int __wrap_printf(const char*, ...);
extern "C" int __wrap_puts(const char*);
static char uart[256];
static unsigned written = 0;
extern "C" int board_console_write(const char* text, int length) {
    if (length < 0 || written + unsigned(length) >= sizeof(uart)) return -1;
    std::memcpy(uart + written, text, length);
    written += length;
    uart[written] = 0;
    return length;
}
int main() {
    int failures = 0;
    auto check = [&](bool ok, const char* message) {
        if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
    };
    char line[128];
    int length = __wrap_snprintf(line, sizeof(line), "+FPID:%.4f,%.4f\r\n", 1.125f, -0.25f);
    check(std::strcmp(line, "+FPID:1.1250,-0.2500\r\n") == 0 && length == int(std::strlen(line)),
          "float PI reply and returned length match the AT protocol");
    __wrap_snprintf(line, sizeof(line), "+HSTATUS:%d;%.2f;%lu;%u", -1, 3.5f, 4294967295UL, 20U);
    check(std::strcmp(line, "+HSTATUS:-1;3.50;4294967295;20") == 0, "32-bit unsigned counters are preserved");
    char short_line[4];
    check(__wrap_snprintf(short_line, sizeof(short_line), "%s", "abcdef") == 6 &&
          std::strcmp(short_line, "abc") == 0, "bounded formatting reports full length and terminates");
    for (const char* text : {" +1.25,4", "-0.0045", "3e2", "1e-7", "12oops", "1e+", "nan", "-inf", "nonsense"}) {
        char *reference_end = nullptr, *end = nullptr;
        const float expected = std::strtof(text, &reference_end);
        const float value = __wrap_strtof(text, &end);
        check((value == expected || (std::isnan(value) && std::isnan(expected))) && end == reference_end,
              "decimal/scientific parsing and end pointer match strtof");
    }
    errno = 0;
    char* end = nullptr;
    check(std::isinf(__wrap_strtof("1e99", &end)) && errno == ERANGE && *end == 0,
          "overflow remains detectable by command validation");
    errno = 0;
    check(__wrap_strtof("1e-99", &end) == 0.0f && errno == ERANGE && *end == 0,
          "underflow has defined value and end pointer");
    __wrap_printf("PWM %u, %.3f V", 20000U, 1.125f);
    __wrap_puts(" ready");
    check(std::strcmp(uart, "PWM 20000, 1.125 V ready\n") == 0, "printf/puts reach the board transport");
    std::printf("F410 console: %d failures\n", failures);
    return failures != 0;
}
