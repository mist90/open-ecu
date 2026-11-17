#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_dir="${BUILD_DIR:-/tmp/opencode/openecu-hybrid-iq/host}"
mkdir -p "$build_dir"
for profile in float fixed; do
    flags=()
    if [[ "$profile" == fixed ]]; then flags=(-DECU_FIXED_NUMERIC); fi
    g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=undefined \
        -I../../STM32G431/Core/Inc -I../../libecu/include "${flags[@]}" \
        main.cpp ../../libecu/src/foc_algorithm.cpp -o "$build_dir/test-$profile"
    "$build_dir/test-$profile"
done
