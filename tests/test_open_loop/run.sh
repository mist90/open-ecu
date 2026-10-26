#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
flags=()
case ${1:-float} in
    float) ;;
    fixed) flags=(-DECU_FIXED_NUMERIC) ;;
    *) echo "Usage: $0 [float|fixed]" >&2; exit 1 ;;
esac
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    -I../../libecu/include -I../../STM32G431/Core/Inc "${flags[@]}" -o test main.cpp \
    ../../libecu/src/{bldc_controller,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp
./test
