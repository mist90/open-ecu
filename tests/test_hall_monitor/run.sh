#!/bin/bash
set -e
cd "$(dirname "$0")"
flags=()
if [[ ${1:-float} == fixed ]]; then flags=(-DECU_FIXED_NUMERIC); fi
g++ -std=c++17 -O2 -Wall -Wextra -I../../STM32G431/Core/Inc -I../../libecu/include \
    "${flags[@]}" -o test main.cpp ../../libecu/src/hall_monitor.cpp
./test
