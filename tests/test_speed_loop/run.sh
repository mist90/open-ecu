#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p .build
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
    -fsanitize=undefined -fno-sanitize-recover=all -DMOTOR_1 \
    -I../../libecu/include main.cpp -o .build/test_speed_loop
exec .build/test_speed_loop "$@"
