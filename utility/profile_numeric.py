#!/usr/bin/env python3
"""Read bounded DWT core/callback measurements from an ENABLE_BENCH_PROFILING ELF."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--sample", type=int, default=0,
                        help="Arm this many PWM callbacks in the current drive state; zero only reads")
    args = parser.parse_args()
    if not 0 <= args.sample <= 20000:
        parser.error("sample must be 0..20000 (bounded uint32 cycle total)")
    symbols = subprocess.check_output(["arm-none-eabi-nm", args.elf], text=True)
    names = ("numeric_bench_results", "pwm_bench_profile", "SystemCoreClock", "pwm_irq_bench_profile")
    addresses = {name: int(re.search(r"(?m)^([0-9a-f]+) \w " + name + r"$", symbols)[1], 16)
                 for name in names}
    command = ["openocd", "-f", "interface/stlink.cfg", "-f", "target/stm32g4x.cfg", "-c", "init"]
    if args.sample:
        address = addresses["pwm_bench_profile"]
        for offset in (0, 4, 8, 12, 16):
            command += ["-c", f"mww 0x{address + offset:x} 0"]
        command += ["-c", f"mww 0x{address:x} {args.sample}", "-c", "sleep 1600"]
    for name, count in zip(names, (20, 5, 1, 5)):
        command += ["-c", f"mdw 0x{addresses[name]:x} {count}"]
    command += ["-c", "shutdown"]
    result = subprocess.run(command, capture_output=True, text=True, timeout=15, check=True)
    raw = result.stdout + result.stderr
    words = {}
    for address, values in re.findall(r"(?m)^0x([0-9a-f]+): ([0-9a-f ]+)$", raw):
        words.update({int(address, 16) + 4 * n: int(value, 16)
                      for n, value in enumerate(values.split())})
    def read(name, count):
        return [words[addresses[name] + 4 * n] for n in range(count)]
    clock = read("SystemCoreClock", 1)[0]
    report = {"elf_sha256": hashlib.sha256(Path(args.elf).read_bytes()).hexdigest(),
              "clock_hz": clock, "cores": {}}
    data = read("numeric_bench_results", 20)
    for index, name in enumerate(("pll", "hall", "adc", "foc_normal", "foc_saturated")):
        samples, total, maximum, checksum = data[index * 4:index * 4 + 4]
        report["cores"][name] = {"samples": samples, "mean_us": total / samples / clock * 1e6,
                                 "max_us": maximum / clock * 1e6, "checksum": checksum}
    remaining, samples, total, maximum, overruns = read("pwm_bench_profile", 5)
    report["pwm"] = {"remaining": remaining, "samples": samples, "total_cycles": total,
                     "max_cycles": maximum, "overruns": overruns,
                     "mean_us": total / samples / clock * 1e6 if samples else None,
                     "max_us": maximum / clock * 1e6}
    if remaining:
        raise RuntimeError("PWM capture has not completed: " + str(report["pwm"]))
    remaining, samples, total, maximum, overruns = read("pwm_irq_bench_profile", 5)
    report["irq"] = {"samples": samples, "total_cycles": total, "max_cycles": maximum,
                     "overruns": overruns, "mean_us": total / samples / clock * 1e6 if samples else None,
                     "max_us": maximum / clock * 1e6}
    if remaining or samples != report["pwm"]["samples"]:
        raise RuntimeError("IRQ and callback captures disagree: " + json.dumps(report))
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
