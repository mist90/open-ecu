#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
out=${1:-$(mktemp -d "${TMPDIR:-/tmp}/open-ecu-mtte.XXXXXX")}
mkdir -p "$out"
flags=(-std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti
       -fsanitize=undefined -fno-sanitize-recover=all
        -DMOTOR_1 -DLIBECU_MOTOR1_T60_LOAD=1 -DLIBECU_MOTOR1_T60_45A=1 -I../../libecu/include
        -DLIBECU_T60_SPEED_KP=6.0f) # Hold the original COMS3 A/B tuning fixed.
for program in test_observer model; do
    "${CXX:-g++}" "${flags[@]}" "$program.cpp" -o "$out/$program"
done
"$out/test_observer"
"$out/model" "$out"
echo "Model traces: $out"
