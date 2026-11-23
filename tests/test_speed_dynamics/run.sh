#!/bin/sh
set -eu
here=$(CDPATH= cd -P "$(dirname "$0")" && pwd)
lib=${LIBECU_DIR:-"$here/../../libecu"}
port=${DYNAMICS_PORT:-STM32F410}
case "$port" in
    STM32F410|STM32G431) ;;
    *) echo "DYNAMICS_PORT must be STM32F410 or STM32G431" >&2; exit 2;;
esac
case "${NUMERIC_PROFILE:-float}" in
    float) numeric=;;
    fixed) numeric=-DECU_FIXED_NUMERIC;;
    *) echo "NUMERIC_PROFILE must be float or fixed" >&2; exit 2;;
esac
case "$#" in
    0|1|3) ;;
    *) echo "Usage: $0 [trace.csv [Kp Ki]]" >&2; exit 2;;
esac
tmp=$(mktemp -d "${TMPDIR:-/tmp}/speed-dynamics.XXXXXX")
trap 'rm -rf "$tmp"' 0
trap 'exit 1' HUP INT TERM
export LC_ALL=C
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
    -fsanitize=undefined -fno-sanitize-recover=all -DMOTOR_1 $numeric \
    ${SPEED_PI_HZ:+"-DLIBECU_SPEED_PI_HZ=$SPEED_PI_HZ"} \
    -I"$here/../../$port/Core/Inc" -I"$lib/include" "$here/main.cpp" "$lib/src/bldc_controller.cpp" \
    "$lib/src/foc_algorithm.cpp" "$lib/src/six_step_algorithm.cpp" "$lib/src/motor_pll.cpp" \
    "$lib/src/hall_monitor.cpp" -o "$tmp/sim"
trace=${1:-$tmp/trace.csv}
if [ "$#" -gt 0 ]; then shift; fi
"$tmp/sim" "$trace" "$@"
if [ "$#" -eq 0 ]; then
    python3 -E "$here/metrics.py" "$trace" --check  # Keep assertions active even with PYTHONOPTIMIZE set.
else
    python3 "$here/metrics.py" "$trace"
fi
