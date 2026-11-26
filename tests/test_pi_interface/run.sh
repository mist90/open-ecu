#!/bin/bash
set -eu
cd "$(dirname "$0")"
dir=$(mktemp -d)
trap 'rm -rf "$dir"' EXIT
for motor in jk42 scooter; do
    flags=()
    if [[ $motor == scooter ]]; then flags+=(-DMOTOR_1); fi
    g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined \
        -fno-sanitize-recover=all "${flags[@]}" -I../../libecu/include -o "$dir/$motor" main.cpp
    "$dir/$motor"
done
