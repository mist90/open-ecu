#!/usr/bin/env python3
"""Run compiled Thumb-2 probes against Python's arbitrary-precision oracle."""
import random
import math
import sys

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.mem_map(0, 1024 * 1024)
machine.mem_map(0x20000000, 0x10000)
with open(sys.argv[1], "rb") as file:
    elf = ELFFile(file)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            machine.mem_write(segment["p_vaddr"], segment.data())
    symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}


def run(name, a, b, wide=False):
    machine.reg_write(UC_ARM_REG_R0, a & 0xFFFFFFFF)
    machine.reg_write(UC_ARM_REG_R1, b & 0xFFFFFFFF)
    machine.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
    machine.reg_write(UC_ARM_REG_LR, 0xF0001)
    machine.emu_start(symbols[name] | 1, 0xF0000, count=10000)
    assert machine.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, "did not return within the instruction budget")
    raw = machine.reg_read(UC_ARM_REG_R0)
    bits = 32
    if wide:
        raw |= machine.reg_read(UC_ARM_REG_R1) << 32
        bits = 64
    return raw if raw < 1 << (bits - 1) else raw - (1 << bits)


def saturate(value, bits=32):
    return max(-(1 << (bits - 1)), min((1 << (bits - 1)) - 1, value))


def rounded(value, divisor):
    return (1 if value >= 0 else -1) * ((abs(value) + divisor // 2) // divisor)


edges = [-(1 << 31), -(1 << 31) + 1, -(1 << 30), -2, -1, 0, 1, 2,
         (1 << 30), (1 << 31) - 2, (1 << 31) - 1]
random_source = random.Random(0x853740)
cases = [(a, b) for a in edges for b in edges]
cases += [(random_source.randrange(-(1 << 31), 1 << 31), random_source.randrange(-(1 << 31), 1 << 31))
          for _ in range(20000)]
checks = 0
for a, b in cases:
    for name, expected, wide in [
        ("raw_product", a * b, True),
        ("fraction_product", saturate(rounded(a * b, 1 << 30)), False),
        ("saturated_product", saturate(rounded(saturate(a - b) * saturate(a + b), 1 << 12)), False),
        ("widening_product", saturate(a * b * 16, 64), True),
    ]:
        actual = run(name, a, b, wide)
        assert actual == expected, (name, a, b, actual, expected)
        checks += 1
    divisor = (a & 0x7FFFFFFF) or 1
    expected = saturate(rounded(1 << 48, divisor))
    actual = run("reciprocal", divisor, 0)
    assert actual == expected, ("reciprocal", divisor, actual, expected)
    checks += 1
    full = ((a & 0xFFFFFFFF) << 32) | (b & 0xFFFFFFFF)
    for value in (full, full & ((1 << 62) - 1)):
        actual = run("integer_root", value & 0xFFFFFFFF, value >> 32) & 0xFFFFFFFF
        assert actual == math.isqrt(value), ("integer_root", value, actual, math.isqrt(value))
        checks += 1
print(f"Thumb-2 machine code: {checks} full-range arbitrary-precision oracle cases passed")
