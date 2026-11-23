#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
dir=$(mktemp -d "${TMPDIR:-/tmp}/speed-loop.XXXXXX")
trap 'rm -rf "$dir"' EXIT
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
    -fsanitize=undefined -fno-sanitize-recover=all -DMOTOR_1 \
    -I../../libecu/include main.cpp -o "$dir/test_speed_loop"
"$dir/test_speed_loop" "$@"
