#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
work=$(mktemp -d "${TMPDIR:-/tmp}/f410-adc-x2.XXXXXX")
trap 'rm -rf "$work"' EXIT
drivers=$(realpath ../../STM32F410/Drivers)
if [[ ! -d "$drivers" ]]; then
    tar -xzf ../../STM32F410/Drivers.tar.gz -C "$work"
    drivers="$work/Drivers"
fi
for profile in float fixed; do
    flags=()
    if [[ $profile == fixed ]]; then flags+=(-DECU_FIXED_NUMERIC); fi
    for x2 in 0 1; do
        feature=()
        # x2=1 deliberately uses the header default rather than an explicit define.
        if [[ $x2 == 0 ]]; then feature+=(-DSTM32F4_ADC_SOFTWARE_X2=0); fi
        # ST's unused inline LL helpers narrow pointers to uint32_t. Permit those
        # declarations on a 64-bit host; the mapped register addresses fit in 32 bits.
        g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -fpermissive \
            -fsanitize=undefined -fno-sanitize-recover=undefined -ffunction-sections -fdata-sections \
            -DSTM32F410Rx -DUSE_HAL_DRIVER "${feature[@]}" "${flags[@]}" \
            -I../../STM32F410/Core/Inc -I../../libecu/include -I../../libecu/hal/stm32f4 \
            -isystem "$drivers/STM32F4xx_HAL_Driver/Inc" \
            -isystem "$drivers/CMSIS/Device/ST/STM32F4xx/Include" -isystem "$drivers/CMSIS/Include" \
            main.cpp ../../libecu/hal/stm32f4/stm32_adc.cpp \
            -Wl,--gc-sections -o "$work/test-$profile-$x2"
        "$work/test-$profile-$x2"
    done
done
