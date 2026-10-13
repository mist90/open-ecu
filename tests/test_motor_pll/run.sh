#!/bin/bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -I../../STM32G431/Core/Inc -I../../libecu/include \
    -o test main.cpp ../../libecu/src/motor_pll.cpp
./test
if [[ ${1:-} == --plot ]]; then
    ./plot_results.py
fi
