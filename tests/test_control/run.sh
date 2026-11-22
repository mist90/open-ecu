#!/bin/bash
set -e
cd "$(dirname "$0")"
flags=()
config_dir=../../STM32G431/Core/Inc
if [[ ${1:-float} == t60-fixed || ${1:-float} == t60-feedback ]]; then
    # Use the host timing fixture: only the 16 kHz current and 100 Hz speed clocks matter here.
    config_dir=../test_parameter_publication
    flags=(-DTEST_CURRENT_HZ=16000 -DMOTOR_1 -DECU_FIXED_NUMERIC -DLIBECU_MOTOR1_T60_LOAD=1 -DLIBECU_MOTOR1_T60_45A=1)
    if [[ $1 == t60-feedback ]]; then flags+=(-DLIBECU_TRACTION_ACCEL_RPS2=2); fi
elif [[ ${1:-float} == numeric-fixed ]]; then
    flags=(-DECU_FIXED_NUMERIC)
elif [[ ${1:-float} == fixed ]]; then
    flags=(-DECU_FIXED_PI)
elif [[ ${1:-float} != float ]]; then
    echo "Usage: $0 [float|fixed|numeric-fixed|t60-fixed|t60-feedback]" >&2
    exit 1
fi
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
    -fsanitize=undefined -fno-sanitize-recover=all \
    -DLIBECU_OSC_BUFFER_SIZE="${LIBECU_OSC_BUFFER_SIZE:-1024}" \
    -DLIBECU_FOC_DUTY_TELEMETRY="${LIBECU_FOC_DUTY_TELEMETRY:-1}" \
    "${flags[@]}" \
    -I"$config_dir" -I../../libecu/include -o test main.cpp \
    ../../libecu/src/{at_command_processor,bldc_controller,crc16,foc_algorithm,hall_monitor,motor_pll,six_step_algorithm}.cpp
./test
