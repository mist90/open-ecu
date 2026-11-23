#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
out=${1:-$(mktemp -d "${TMPDIR:-/tmp}/open-ecu-mtte.XXXXXX")}
mkdir -p "$out"
flags=(-std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti
       -fsanitize=undefined -fno-sanitize-recover=all -DMOTOR_1 -I../../libecu/include)
for program in test_observer model; do
    "${CXX:-g++}" "${flags[@]}" "$program.cpp" -o "$out/$program"
done
"$out/test_observer"
"$out/model" "$out"
echo "Model traces: $out"
