#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_root=${BUILD_DIR:-${TMPDIR:-/tmp/opencode}}
mkdir -p "$build_root"
work=$(mktemp -d "$build_root/f410-adc.XXXXXX")
trap 'rm -rf "$work"' EXIT
drivers=$(realpath ../../STM32F410/Drivers)
if [[ ! -d "$drivers" ]]; then
    tar -xzf ../../STM32F410/Drivers.tar.gz -C "$work"
    drivers="$work/Drivers"
fi
for profile in float fixed; do
    flags=()
    if [[ $profile == fixed ]]; then flags+=(-DECU_FIXED_NUMERIC); fi
    # ST's unused inline LL helpers narrow pointers to uint32_t. Permit those
    # declarations on a 64-bit host; the mapped register addresses fit in 32 bits.
    g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -fpermissive \
        -fsanitize=undefined -fno-sanitize-recover=undefined -ffunction-sections -fdata-sections \
        -DSTM32F410Rx -DUSE_HAL_DRIVER "${flags[@]}" \
        -I../../STM32F410/Core/Inc -I../../libecu/include -I../../libecu/hal/stm32f4 \
        -isystem "$drivers/STM32F4xx_HAL_Driver/Inc" \
        -isystem "$drivers/CMSIS/Device/ST/STM32F4xx/Include" -isystem "$drivers/CMSIS/Include" \
        main.cpp ../../libecu/hal/stm32f4/stm32_adc.cpp \
        -Wl,--gc-sections -o "$work/test-$profile"
    "$work/test-$profile"
done
