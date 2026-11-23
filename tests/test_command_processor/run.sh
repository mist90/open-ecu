#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/command-processor.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT
for profile in float fixed; do
    flags=()
    if [[ $profile == fixed ]]; then flags+=(-DECU_FIXED_NUMERIC); fi
    for motor in jk42 scooter scooter-free scooter-loaded; do
        motor_flags=()
        if [[ $motor != jk42 ]]; then motor_flags+=(-DMOTOR_1); fi
        if [[ $motor == scooter-free ]]; then motor_flags+=(-DMOTOR_1_LOADED=0); fi
        if [[ $motor == scooter-loaded ]]; then motor_flags+=(-DMOTOR_1_LOADED=1); fi
        for manual in off on; do
            manual_flags=()
            if [[ $manual == on ]]; then manual_flags+=(-DTHROTTLE_BRAKE_CONTROL); fi
            # Deliberately link the real firmware main without AT or CRC sources.
            g++ -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
                -fsanitize=undefined -fno-sanitize-recover=all "${flags[@]}" "${motor_flags[@]}" "${manual_flags[@]}" \
                -I../../STM32G431/Core/Inc -I../../libecu/include -o "$build_dir/test" main.cpp \
                ../../libecu/src/{main,bldc_controller,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp
            for cutoff in 0 1250 2000; do
                echo "Protocol lifecycle and boot defaults: profile=$profile motor=$motor manual=$manual filter=$cutoff"
                TEST_BOARD_CURRENT_FILTER_HZ=$cutoff "$build_dir/test"
            done
            # Distinguish the board rail (including a value below the 12 A rolling
            # base) from the motor rail when the inverter has current to spare.
            for drive_limit in 8.5 45 60; do
                echo "Drive current cap: profile=$profile motor=$motor manual=$manual board=$drive_limit A"
                TEST_BOARD_DRIVE_LIMIT_A=$drive_limit "$build_dir/test"
            done
            if [[ $motor != jk42 ]]; then
                echo "PI-only motor rejects hybrid board: profile=$profile motor=$motor manual=$manual"
                TEST_BOARD_HYBRID_IQ=1 TEST_BOARD_CURRENT_FILTER_HZ=1000 "$build_dir/test"
            fi
        done
    done
done
