#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_root=${BUILD_DIR:-${TMPDIR:-/tmp/opencode}}
mkdir -p "$build_root"
work=$(mktemp -d "$build_root/parameter-publication.XXXXXX")
trap 'rm -rf "$work"' EXIT
for profile in float fixed fixed-narrow-step; do
    flags=()
    clocks=(8000 16000 20000)
    sources=(../../libecu/src/{bldc_controller,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp)
    if [[ $profile != float ]]; then flags+=(-DECU_FIXED_NUMERIC); fi
    if [[ $profile == fixed-narrow-step ]]; then
        flags+=(-DTEST_NARROW_PLL_STEP)
        clocks=(20000)
        sources=(../../libecu/src/{hall_monitor,motor_pll}.cpp)
    fi
    for hz in "${clocks[@]}"; do
        g++ -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
            -fsanitize=undefined -fno-sanitize-recover=all -ffunction-sections -fdata-sections \
            -DTEST_CURRENT_HZ="$hz" "${flags[@]}" -I. -I../../libecu/include \
            main.cpp "${sources[@]}" \
            -Wl,--gc-sections -o "$work/test-$profile-$hz"
        "$work/test-$profile-$hz"
    done
done
