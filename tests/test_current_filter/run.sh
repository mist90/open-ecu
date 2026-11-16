#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined -fno-sanitize-recover=undefined \
    -I../../libecu/include main.cpp -o test
./test
for cutoff in 0 100 1000 2000 6000 7900; do
    g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined -fno-sanitize-recover=undefined \
        -I../../libecu/include -DLIBECU_CURRENT_FILTER_FIXED_HZ="$cutoff" \
        -DLIBECU_CURRENT_FILTER_FIXED_RATE=16000 fixed_config.cpp -o test
    ./test
done
