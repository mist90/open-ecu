#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/command-processor.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT
for profile in float fixed; do
    flags=()
    if [[ $profile == fixed ]]; then flags+=(-DECU_FIXED_NUMERIC); fi
    for manual in off on; do
        manual_flags=()
        if [[ $manual == on ]]; then manual_flags+=(-DTHROTTLE_BRAKE_CONTROL); fi
        # Deliberately link the real firmware main without AT or CRC sources.
        g++ -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
            -fsanitize=undefined -fno-sanitize-recover=all "${flags[@]}" "${manual_flags[@]}" \
            -I../../STM32G431/Core/Inc -I../../libecu/include -o "$build_dir/test" main.cpp \
            ../../libecu/src/{main,bldc_controller,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp
        echo "Protocol lifecycle: profile=$profile manual=$manual"
        "$build_dir/test"
    done
done
