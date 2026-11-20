#!/bin/bash
set -eu
cd "$(dirname "$0")"
dir=$(mktemp -d)
trap 'rm -rf "$dir"' EXIT
for profile in standard t60-45a; do
    flags=()
    if [[ $profile == t60-45a ]]; then flags=(-DLIBECU_MOTOR1_T60_LOAD=1 -DLIBECU_MOTOR1_T60_45A=1); fi
    g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined \
        -fno-sanitize-recover=all -DMOTOR_1 "${flags[@]}" -I../../libecu/include -o "$dir/$profile" main.cpp
    "$dir/$profile"
done
