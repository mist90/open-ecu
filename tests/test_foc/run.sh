#!/bin/bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -o test main.cpp ../../libecu/src/foc_algorithm.cpp
./test
