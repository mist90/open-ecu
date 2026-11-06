#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
tmp=$(mktemp -d "${TMPDIR:-/tmp/opencode}/ecu-pll-diagnostics.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    -DLIBECU_PLL_DIAGNOSTICS=1 -I../../libecu/include diagnostics.cpp -o "$tmp/diagnostics"
"$tmp/diagnostics"
