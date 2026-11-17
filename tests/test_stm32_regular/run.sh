#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_root=${BUILD_DIR:-${TMPDIR:-/tmp/opencode}}
mkdir -p "$build_root"
work=$(mktemp -d "$build_root/stm32-regular.XXXXXX")
trap 'rm -rf "$work"' EXIT
for platform in STM32F410 STM32G431; do
    if [[ $platform == STM32F410 ]]; then
        family=stm32f4; vendor=STM32F4xx; model=STM32F410Rx
    else
        family=stm32g4; vendor=STM32G4xx; model=STM32G431xx
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
        binary="$work/test-$platform-$profile"
        g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fpermissive \
            -fsanitize=undefined -fno-sanitize-recover=undefined -ffunction-sections -fdata-sections \
            -D"$model" -DUSE_HAL_DRIVER "${flags[@]}" \
            -include publication_hook.hpp \
            -I"../../$platform/Core/Inc" -I../../libecu/include -I"../../libecu/hal/$family" \
            -isystem "$drivers/${vendor}_HAL_Driver/Inc" \
            -isystem "$drivers/CMSIS/Device/ST/$vendor/Include" -isystem "$drivers/CMSIS/Include" \
            main.cpp "../../libecu/hal/$family/stm32_adc.cpp" \
            -Wl,--gc-sections -o "$binary"
        timeout 10 "$binary"
    done
done
