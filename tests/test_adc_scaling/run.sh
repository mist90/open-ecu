#!/bin/bash
set -e
cd "$(dirname "$0")"
flags=()
if [[ ${1:-float} == fixed ]]; then flags=(-DECU_FIXED_NUMERIC); fi
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined \
    -fno-sanitize-recover=all -I../../libecu/include -I../../STM32G431/Core/Inc \
    "${flags[@]}" -o test main.cpp
./test
