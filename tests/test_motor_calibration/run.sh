#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
build=$(mktemp -d "${TMPDIR:-/tmp}/motor-calibration.XXXXXX")
trap 'rm -rf "$build"' EXIT
for motor in jk42 scooter; do
    motor_flags=()
    if [[ $motor == scooter ]]; then motor_flags+=(-DMOTOR_1); fi
    for numeric in float fixed; do
        flags=("${motor_flags[@]}")
        if [[ $numeric == fixed ]]; then flags+=(-DECU_FIXED_NUMERIC); fi
        echo "Native FOC: $motor / $numeric"
        "${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
            "${flags[@]}" -DTEST_CURRENT_HZ=16000 -I../test_parameter_publication -I../../libecu/include \
            main.cpp ../../libecu/src/foc_algorithm.cpp -o "$build/test"
        "$build/test"
    done
done
