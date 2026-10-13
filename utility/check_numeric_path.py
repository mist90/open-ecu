#!/usr/bin/env python3
"""Audit the fixed control paths and concrete STM32 virtual targets in a linked ELF."""

import argparse
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf")
    args = parser.parse_args()
    assembly = subprocess.check_output(["arm-none-eabi-objdump", "-d", "-C", args.elf], text=True)
    functions = {}
    current = None
    for line in assembly.splitlines():
        header = re.match(r"^([0-9a-f]+) <(.*)>:$", line)
        if header:
            current = int(header[1], 16)
            functions[current] = {"name": header[2], "body": []}
        elif current is not None:
            functions[current]["body"].append(line)
    roots = (
        "HAL_SYSTICK_Callback",
        "libecu::(anonymous namespace)::Stm32Board::readNativeThrottle(",
        "libecu::BldcController::setNativeTargetSpeed(",
        "libecu::BldcController::pwmInterruptHandler(", "libecu::BldcController::update(",
        "libecu::AtCommandProcessor::captureNativeOscSample(",
        "libecu::Stm32Pwm::updateNativeDutyCycle(", "libecu::Stm32Pwm::setChannelState(",
        "libecu::Stm32Pwm::enable(", "libecu::Stm32Pwm::apply(",
        "libecu::Stm32Adc::getRawAdcValue(", "libecu::Stm32Adc::getRawTemperatureAdcValue(",
        "libecu::Stm32TimHallSensor::getPosition(", "libecu::FocAlgorithm::reset(",
        "libecu::FocAlgorithm::onEnter(", "libecu::SixStepAlgorithm::reset(",
        "libecu::SixStepAlgorithm::onEnter(",
    )
    for root in roots:
        if not any(fn["name"].startswith(root) for fn in functions.values()):
            raise RuntimeError("Missing audit root: " + root)
    if not any("captureNativeOscSample(libecu::MotorStatusT<libecu::FixedProfile>" in fn["name"]
               for fn in functions.values()):
        raise RuntimeError("Expected a NUMERIC_PROFILE=fixed ELF")
    pending = [addr for addr, fn in functions.items() if fn["name"].startswith(roots)]
    visited = set()
    failures = []
    helpers = re.compile(r"__aeabi_([fd](add|sub|mul|div|neg|cmp|2)|(i|ui|l|ul)2[fd])|^(sin|cos|sqrt|log|fmod|round)f?$")
    fp_instruction = re.compile(r"^v(add|sub|mul|div|sqrt|abs|neg|cmp|cmpe|cvt|fma|fms|fnma|fnms|mla|mls|nmul)\.")
    while pending:
        address = pending.pop()
        if address in visited:
            continue
        visited.add(address)
        fn = functions[address]
        if helpers.search(fn["name"]):
            failures.append(fn["name"])
        for line in fn["body"]:
            instruction = re.match(r"\s*[0-9a-f]+:\s+(?:[0-9a-f]{4,8}\s+)+([^\s]+)\s*(.*)", line)
            if not instruction:
                continue
            mnemonic, operands = instruction.groups()
            if fp_instruction.match(mnemonic):
                failures.append(fn["name"] + ": " + line.strip())
            target = re.match(r"([0-9a-f]+) <", operands)
            if mnemonic.startswith("b") and target and int(target[1], 16) in functions:
                pending.append(int(target[1], 16))
    if failures:
        raise RuntimeError("Floating arithmetic in native path:\n" + "\n".join(failures))
    print(f"Fixed native paths: {len(visited)} reachable functions, no FP arithmetic/helpers")
    print("Indirect calls covered by the explicit STM32 ADC/Hall/PWM and algorithm roots")


if __name__ == "__main__":
    main()
