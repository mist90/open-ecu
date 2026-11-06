#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
tmp=$(mktemp -d "${TMPDIR:-/tmp/opencode}/ecu-voltage-limit.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
    -I../../libecu/include main.cpp -o "$tmp/test"
"$tmp/test"
if command -v arm-none-eabi-g++ >/dev/null; then
    arm-none-eabi-g++ -std=c++17 -O2 -flto -mcpu=cortex-m3 -mthumb -mfloat-abi=soft \
        -fno-exceptions -fno-rtti -fno-unwind-tables -Wall -Wextra -Werror -Wno-psabi \
        -I../../libecu/include -nostdlib -Wl,-Ttext=0x1000,-e,prepare_limit \
        arm.cpp -lgcc -o "$tmp/limit.elf"
    python=${LIBECU_TEST_ARM_PYTHON:-python3}
    if "$python" -c 'import unicorn, elftools' >/dev/null 2>&1; then
        "$python" arm.py "$tmp/limit.elf"
    else
        echo "Cortex-M3 limiter: compiled; machine-code oracle needs Python unicorn and pyelftools"
    fi
fi
