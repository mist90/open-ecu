#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined -fno-sanitize-recover=undefined \
    -I../../libecu/include main.cpp -o test
./test
