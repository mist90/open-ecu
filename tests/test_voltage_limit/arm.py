#!/usr/bin/env python3
"""Execute Thumb-2 limiter code against an independent half-plane oracle."""
import json
import math
import random
import struct
import subprocess
import sys

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.mem_map(0, 1024 * 1024)
machine.mem_map(0x20000000, 0x10000)
with open(sys.argv[1], "rb") as stream:
    elf = ELFFile(stream)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            machine.mem_write(segment["p_vaddr"], segment.data())
    symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
disassembly = subprocess.check_output(["arm-none-eabi-objdump", "-d", sys.argv[1]], text=True)
assert not any(name in disassembly for name in ("sqrt", "Sqrt", "div", "__aeabi_f", "__aeabi_d")), \
    "Root, divider or soft floating arithmetic in the compiled limiter"
count = 0


def instruction(_machine, _address, _size, _user):
    global count
    count += 1


def call(name, a, b=0):
    global count
    count = 0
    machine.reg_write(UC_ARM_REG_R0, a & 0xffffffff)
    machine.reg_write(UC_ARM_REG_R1, b & 0xffffffff)
    machine.reg_write(UC_ARM_REG_R2, 0x20008000)
    machine.reg_write(UC_ARM_REG_SP, 0x2000fff0)
    machine.reg_write(UC_ARM_REG_LR, 0xf0001)
    machine.emu_start(symbols[name] | 1, 0xf0000, count=10000)
    assert machine.reg_read(UC_ARM_REG_PC) == 0xf0000, (name, "instruction budget exceeded")
    return machine.reg_read(UC_ARM_REG_R0)


coefficients = [(math.floor((1 << 28) / math.cos(k * math.pi / 16)),
                 math.ceil((1 << 28) * math.tan(k * math.pi / 16))) for k in range(1, 8)]


def room(d, limit):
    d = min(abs(d), limit)
    return min([limit] + [(limit * a - d * b) // (1 << 28) for a, b in coefficients])


hook = machine.hook_add(UC_HOOK_CODE, instruction)
rng = random.Random(0x32f051)
limits = list(range(65)) + [1 << 18, 20 << 18, (1 << 31) - 1]
limits += [rng.randrange(1 << 31) for _ in range(1000)]
checks = 0
for limit in limits:
    call("prepare_limit", limit)
    probes = [-1 << 31, -limit, 0, limit, (1 << 31) - 1]
    for k in range(7):
        vertex = int(limit * math.sin((k + 0.5) * math.pi / 16) / math.cos(math.pi / 32))
        probes.extend(vertex + offset for offset in (-2, -1, 0, 1, 2))
    for d in probes:
        actual = call("query_limit", d)
        assert actual == room(d, limit), (limit, d, actual, room(d, limit))
        checks += 1
    for _ in range(8):
        d, q = rng.randrange(-1 << 31, 1 << 31), rng.randrange(-1 << 31, 1 << 31)
        expected_d = max(-limit, min(limit, d * 64))
        headroom = room(expected_d, limit)
        expected_q = max(-headroom, min(headroom, q * 64))
        expected = (expected_d, expected_q, int(expected_d != d * 64 or expected_q != q * 64))
        call("apply_limit", d, q)
        assert struct.unpack("<3i", machine.mem_read(0x20008000, 12)) == expected
        checks += 1

# Instruction counts describe executed code, not Cortex-M3 cycles/Flash timing.
counts = {"prepare": [], "query": [], "apply": []}
for limit in (1, 20 << 18, (1 << 31) - 1):
    call("prepare_limit", limit)
    counts["prepare"].append(count)
    for i in range(129):
        d = limit * i // 128
        call("query_limit", d)
        counts["query"].append(count)
        call("apply_limit", d >> 6, 100 << 12)
        counts["apply"].append(count)
machine.hook_del(hook)
assert all(min(values) > 0 for values in counts.values()), "Instruction hook did not cover translated code"
print(json.dumps({"checks": checks, "instructions": {k: [min(v), max(v)] for k, v in counts.items()}}, indent=2))
