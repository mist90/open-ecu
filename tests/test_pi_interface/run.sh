#!/bin/bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined \
    -fno-sanitize-recover=all -DMOTOR_1 -I../../libecu/include -o test main.cpp
./test
