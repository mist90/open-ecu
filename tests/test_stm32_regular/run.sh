#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_root=${BUILD_DIR:-${TMPDIR:-/tmp/opencode}}
mkdir -p "$build_root"
work=$(mktemp -d "$build_root/stm32-regular.XXXXXX")
trap 'rm -rf "$work"' EXIT
read -r -a platforms <<< "${PLATFORMS:-STM32F410 STM32G431}"
for platform in "${platforms[@]}"; do
    if [[ $platform == STM32F410 ]]; then
        family=stm32f4; vendor=STM32F4xx; model=STM32F410Rx; sums=(0 1)
    else
        family=stm32g4; vendor=STM32G4xx; model=STM32G431xx; sums=(0 1)
    fi
    drivers=$(realpath "../../$platform/Drivers")
    if [[ ! -d $drivers ]]; then
        mkdir -p "$work/$platform"
        tar -xzf "../../$platform/Drivers.tar.gz" -C "$work/$platform"
        drivers="$work/$platform/Drivers"
    fi
    for profile in float fixed; do
        flags=()
        if [[ $profile == fixed ]]; then flags+=(-DECU_FIXED_NUMERIC); fi
        for x2 in "${sums[@]}"; do
            binary="$work/test-$platform-$profile-$x2"
            g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fpermissive \
                -fsanitize=undefined -fno-sanitize-recover=undefined -ffunction-sections -fdata-sections \
                -D"$model" -DUSE_HAL_DRIVER -DSTM32F4_ADC_SOFTWARE_X2="$x2" -DSTM32G4_ADC_X2="$x2" "${flags[@]}" \
                -I"../../$platform/Core/Inc" -I../../libecu/include -I"../../libecu/hal/$family" \
                -isystem "$drivers/${vendor}_HAL_Driver/Inc" \
                -isystem "$drivers/CMSIS/Device/ST/$vendor/Include" -isystem "$drivers/CMSIS/Include" \
                main.cpp "../../libecu/hal/$family/stm32_adc.cpp" \
                -Wl,--gc-sections -o "$binary"
            timeout 10 "$binary"
        done
    done
done
