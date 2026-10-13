#!/bin/bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    -I../../STM32G431/Core/Inc -I../../libecu/include -o test main.cpp \
    ../../libecu/src/{at_command_processor,bldc_controller,crc16,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp
./test
