#!/bin/bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    -I../../libecu/include -o test main.cpp
./test
if command -v arm-none-eabi-g++ >/dev/null; then
    tmp=$(mktemp -d "${TMPDIR:-/tmp/opencode}/ecu-cores.XXXXXX")
    trap 'rm -rf "$tmp"' EXIT
    arm-none-eabi-g++ -std=c++17 -Os -mcpu=cortex-m0 -mthumb -mfloat-abi=soft \
        -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Wno-psabi -I../../libecu/include \
        -c arm_cores.cpp -o "$tmp/cores.o"
    arm-none-eabi-nm -u "$tmp/cores.o" > "$tmp/undefined.txt"
    if grep -E '__aeabi_([fd](add|sub|mul|div|neg|cmp|2)|(i|ui|l|ul)2[fd])| (sin|cos|sqrt|log|fmod|round)f?$' "$tmp/undefined.txt"; then
        echo "Unexpected floating-point helper in the fixed kernels" >&2
        exit 1
    fi
    echo "Cortex-M0: PLL, Hall, FOC, ADC/NTC and PWM kernels compile without floating-point helpers"
fi
