#!/bin/bash
set -e
cd "$(dirname "$0")"
flags=()
if [[ ${1:-float} == fixed ]]; then
    flags=(-DECU_FIXED_PI)
elif [[ ${1:-float} != float ]]; then
    echo "Usage: $0 [float|fixed]" >&2
    exit 1
fi
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    "${flags[@]}" \
    -I../../STM32G431/Core/Inc -I../../libecu/include -o test main.cpp \
    ../../libecu/src/{at_command_processor,bldc_controller,crc16,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp
./test
