#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
flags=(-std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all
       -DLIBECU_MOTOR1_SMOOTH_BENCH=1 -I"$here/../../libecu/include")
for probe in test_smooth_iq test_held_motor1 test_motor1_order test_motor1_bus_cache; do
    profile=()
    if [[ $probe != test_smooth_iq ]]; then profile=(-DLIBECU_MOTOR1_SPAN_BENCH=1); fi
    "${CXX:-g++}" "${flags[@]}" "${profile[@]}" "$here/$probe.cpp" -o "$build/$probe"
    "$build/$probe"
    echo "PASS: $probe"
done
