#!/bin/bash
set -eu
cd "$(dirname "$0")"
dir=$(mktemp -d)
trap 'rm -rf "$dir"' EXIT
for motor in jk42 scooter scooter-loaded; do
    flags=()
    if [[ $motor != jk42 ]]; then flags+=(-DMOTOR_1); fi
    if [[ $motor == scooter-loaded ]]; then flags+=(-DMOTOR_1_LOADED=1); fi
    g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined \
        -fno-sanitize-recover=all "${flags[@]}" -I../../libecu/include -o "$dir/$motor" main.cpp
    "$dir/$motor"
done
