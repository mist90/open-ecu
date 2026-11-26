#!/bin/sh
set -eu
if [ "$#" -gt 1 ]; then echo "Usage: $0 [trace-directory]" >&2; exit 2; fi
here=$(CDPATH= cd -P "$(dirname "$0")" && pwd)
lib="$here/../../libecu"
export LC_ALL=C
tmp=$(mktemp -d "${TMPDIR:-/tmp}/speed-pi-rate.XXXXXX")
trap 'rm -rf "$tmp"' 0
trap 'exit 1' HUP INT TERM
if [ "$#" -eq 1 ]; then mkdir -p "$1"; fi
echo 'pi_hz,load,kp,ki,launch_wheel200_rps2,launch_feedback200_rps2,wheel200_rps2,feedback200_rps2,hall2x200_rps2,wheel10_rps2,feedback10_rps2,rise10_90_s,t90_s,overshoot_rpm,tail_mean_rpm,tail_pp_rpm'
for hz in 100 500; do
    "${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
        -fsanitize=undefined -fno-sanitize-recover=all \
        -DMOTOR_1 -DECU_FIXED_NUMERIC -DTEST_SPEED_PI_HZ="$hz" -I"$here" -I"$lib/include" \
        "$here/main.cpp" "$lib/src/bldc_controller.cpp" "$lib/src/foc_algorithm.cpp" \
        "$lib/src/six_step_algorithm.cpp" "$lib/src/hall_monitor.cpp" "$lib/src/motor_pll.cpp" -o "$tmp/ab"
    if [ "$#" -eq 1 ]; then "$tmp/ab" "$1/trace-$hz.csv"; else "$tmp/ab"; fi
done
