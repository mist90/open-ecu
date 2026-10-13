#!/bin/bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined \
    -fno-sanitize-recover=all -o test main.cpp
./test

if command -v arm-none-eabi-g++ >/dev/null; then
    tmp=$(mktemp -d "${TMPDIR:-/tmp/opencode}/ecu-numeric.XXXXXX")
    trap 'rm -rf "$tmp"' EXIT
    arm-none-eabi-g++ -std=c++17 -Os -mcpu=cortex-m0 -mthumb -mfloat-abi=soft \
        -fno-exceptions -fno-rtti -Wall -Wextra -Werror -c arm_pi.cpp -o "$tmp/pi.o"
    arm-none-eabi-nm -u "$tmp/pi.o" > "$tmp/undefined.txt"
    if grep -E '__aeabi_([fd](add|sub|mul|div|neg|cmp|2)|[iu]l?2[fd])' "$tmp/undefined.txt"; then
        echo "Unexpected floating-point helper in fixed PI" >&2
        exit 1
    fi
    echo "Cortex-M0 fixed PI: compiled without floating-point helpers"
fi
