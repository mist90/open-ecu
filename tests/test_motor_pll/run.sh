#!/bin/bash

g++ -o test main.cpp ../../libecu/src/motor_pll.cpp
./test
./plot_results.py
