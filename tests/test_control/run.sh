#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
dir=$(mktemp -d "${TMPDIR:-/tmp}/ecu-control.XXXXXX")
trap 'rm -rf "$dir"' EXIT
flags=()
motors=(jk42 scooter)
config_dir=../../STM32G431/Core/Inc
if [[ ${1:-float} == scooter-fixed ]]; then
    # 16 kHz current, 100 Hz supervision and the port's independently selected PI clock.
    config_dir=../test_parameter_publication
    motors=(scooter)
    flags=(-DTEST_CURRENT_HZ=16000 -DLIBECU_SPEED_PI_HZ="${SPEED_PI_HZ:-500}" -DECU_FIXED_NUMERIC)
elif [[ ${1:-float} == numeric-fixed ]]; then
    flags=(-DECU_FIXED_NUMERIC)
elif [[ ${1:-float} == fixed ]]; then
    flags=(-DECU_FIXED_PI)
elif [[ ${1:-float} != float ]]; then
    echo "Usage: $0 [float|fixed|numeric-fixed|scooter-fixed]" >&2
    exit 1
fi
if [[ -n ${SPEED_PI_HZ:-} && $config_dir != ../test_parameter_publication ]]; then
    flags+=(-DLIBECU_SPEED_PI_HZ="$SPEED_PI_HZ")
fi
for motor in "${motors[@]}"; do
    motor_flags=()
    if [[ $motor == scooter ]]; then motor_flags+=(-DMOTOR_1); fi
    echo "Control integration: $motor / ${1:-float}"
    g++ -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
        -fsanitize=undefined -fno-sanitize-recover=all \
        -DLIBECU_OSC_BUFFER_SIZE="${LIBECU_OSC_BUFFER_SIZE:-1024}" \
        -DLIBECU_FOC_DUTY_TELEMETRY="${LIBECU_FOC_DUTY_TELEMETRY:-1}" \
        "${flags[@]}" "${motor_flags[@]}" \
        -I"$config_dir" -I../../libecu/include -o "$dir/test" main.cpp \
        ../../libecu/src/{at_command_processor,bldc_controller,crc16,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp
    "$dir/test"
done
