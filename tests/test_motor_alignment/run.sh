#!/bin/bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    -I../../libecu/include -I../../STM32G431/Core/Inc -o test main.cpp ../../libecu/src/motor_pll.cpp
./test
