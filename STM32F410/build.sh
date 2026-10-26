#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_type=Release
numeric=float
throttle=OFF
clean=false
verbose=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -r|--release) build_type=Release; shift ;;
        -d|--debug) build_type=Debug; shift ;;
        -c|--clean) clean=true; shift ;;
        -v|--verbose) verbose=(--verbose); shift ;;
        --throttle) throttle=ON; shift ;;
        --numeric-profile) numeric="${2:?Specify float or fixed}"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--release|--debug] [--clean] [--verbose] [--throttle]"
            echo "          [--numeric-profile float|fixed]"
            echo "NUCLEO-F410RB + IHM08M1; MOTOR_1; Release (-O2 + hard FPU), AT control by default."
            echo "Debug uses -Og. --throttle enables the shield pot and B1 brake."
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done
build_dir=build-release
[[ $build_type == Debug ]] && build_dir=build
if $clean; then rm -rf "$build_dir"; fi
cmake -S . -B "$build_dir" -DCMAKE_BUILD_TYPE="$build_type" \
    -DNUMERIC_PROFILE="$numeric" -DENABLE_THROTTLE_BRAKE_CONTROL="$throttle" \
    -DENABLE_PWM_TIMING_BENCH=OFF
cmake --build "$build_dir" --parallel "$(nproc)" "${verbose[@]}"
