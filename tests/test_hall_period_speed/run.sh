#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
tmp=$(mktemp -d "${TMPDIR:-/tmp}/hall-period.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
profiles=("$@")
if (($# == 0)); then profiles=(float fixed rollback stock); fi
for profile in "${profiles[@]}"; do
    case "$profile" in
        float) flags=(-DMOTOR_1 -DTEST_CURRENT_HZ=20000) ;;
        fixed) flags=(-DMOTOR_1 -DECU_FIXED_NUMERIC -DTEST_CURRENT_HZ=16000) ;;
        rollback) flags=(-DMOTOR_1 -DECU_FIXED_NUMERIC -DTEST_CURRENT_HZ=16000 -DLIBECU_HALL_PERIOD_SPEED=0) ;;
        stock) flags=(-DECU_FIXED_NUMERIC -DTEST_CURRENT_HZ=20000) ;;
        *) echo "Usage: $0 [float|fixed|rollback|stock]..." >&2; exit 2 ;;
    esac
    "${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror -fno-rtti -fno-exceptions \
        -fsanitize=undefined -fno-sanitize-recover=all -DLIBECU_SPEED_PI_HZ=500 "${flags[@]}" \
        -I../test_parameter_publication -I../../libecu/include main.cpp \
        ../../libecu/src/{bldc_controller,motor_pll,hall_monitor,six_step_algorithm,foc_algorithm}.cpp -o "$tmp/test"
    "$tmp/test"
done
