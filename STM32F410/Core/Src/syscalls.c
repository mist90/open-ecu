/** @file syscalls.c
 * @brief Newlib console transport; the firmware has no heap
 */
#include "main.h"
#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>

/* USER CODE BEGIN Syscalls */
int _write(int fd, char* data, int length) { (void)fd; return board_console_write(data, length); }
int _read(int fd, char* data, int length) {
    (void)fd;
    int count = 0;
    while (count < length) {
        const int value = board_console_read();
        if (value < 0) break;
        data[count++] = (char)value;
    }
    return count;
}
int _close(int fd) { (void)fd; return -1; }
int _lseek(int fd, int offset, int whence) { (void)fd; (void)offset; (void)whence; return 0; }
int _fstat(int fd, struct stat* st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _getpid(void) { return 1; }
int _kill(int pid, int signal) { (void)pid; (void)signal; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; Error_Handler(); while (1) {} }

/* Float formatting/parsing is replaced by heapless adapters in console_libc.cpp. */
void* _sbrk(ptrdiff_t increment) {
    (void)increment;
    errno = ENOMEM;
    return (void*)-1;
}
/* USER CODE END Syscalls */
