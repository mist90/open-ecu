#!/bin/bash
set -e
cd "$(dirname "$0")"
flags=()
if [[ ${1:-float} == fixed ]]; then flags=(-DECU_FIXED_PI); fi
sources=(../../libecu/src/{bldc_controller,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp)
if [[ -f ../../libecu/src/pid_controller.cpp ]]; then sources+=(../../libecu/src/pid_controller.cpp); fi
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    -I../../libecu/include -I../../STM32G431/Core/Inc "${flags[@]}" -o test main.cpp "${sources[@]}"
./test
