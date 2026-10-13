#!/usr/bin/env python3
"""Compare current PI gains with synchronized OSC captures in six-step open loop."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from bench_motor import Bench


def analyze(records, low, high, frequency):
    bursts = []
    current = []
    for row in records:
        line = row["line"]
        if line == "+OSC:":
            if len(current) == 1024 and all(sample[0] == index for index, sample in enumerate(current)):
                bursts.append(current)
            current = []
        elif line.startswith("+OSC:"):
            try:
                sample = [int(value) for value in line[5:].split(",")]
            except ValueError:
                continue
            if len(sample) == 8:
                current.append(sample)

    edges = []
    errors = []
    for burst in bursts:
        filtered = [statistics.mean(sample[1] for sample in burst[max(0, i-4):i+1])
                    for i in range(len(burst))]
        age = 0
        for i, sample in enumerate(burst):
            age += 1
            if i and sample[2] != burst[i-1][2]:
                age = 0
                old = burst[i-1][2]
                new = sample[2]
                if abs(new-old) < 0.5 * (high-low) * 1000:
                    continue
                end = min(len(burst), i + int(0.02 * frequency))
                band = max(20.0, 0.1 * abs(new-old))
                hold = int(0.001 * frequency)
                settled = None
                for j in range(i, end-hold):
                    if all(abs(value-new) <= band for value in filtered[j:j+hold]):
                        settled = (j-i) * 1000.0 / frequency
                        break
                up = new > old
                window = filtered[i:end]
                excursion = max(window)-new if up else new-min(window)
                edges.append({"up": up, "settle_ms": settled,
                              "overshoot_ma": max(0.0, excursion), "target_ma": new})
            if age >= int(0.005 * frequency):
                errors.append(sample[1]-sample[2])
    return {
        "bursts": len(bursts), "steps": edges,
        "steady_error_rms_ma": (statistics.mean(error*error for error in errors)**0.5) if errors else None,
        "steady_error_mean_ma": statistics.mean(errors) if errors else None,
        "steady_abs_error_p95_ma": sorted(abs(error) for error in errors)[int(len(errors)*0.95)] if errors else None,
        "up_settle_ms": [row["settle_ms"] for row in edges if row["up"]],
        "down_settle_ms": [row["settle_ms"] for row in edges if not row["up"]],
        "worst_overshoot_ma": max((row["overshoot_ma"] for row in edges), default=None),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gains", nargs="+", required=True, help="KP,KI pairs")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--low", type=float, default=0.15)
    parser.add_argument("--high", type=float, default=0.4)
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    if not (math.isfinite(args.low) and math.isfinite(args.high) and 0 <= args.low < args.high and args.steps > 0):
        parser.error("Require finite 0 <= low < high currents and a positive step count")
    args.hall_swd = None
    args.profile_address = None
    args.max_current = 1.5
    args.max_speed = 30
    reports = []
    for gains in args.gains:
        bench = Bench(args)
        report = {"gains": gains}
        try:
            for command in ("AT+DMODE=2", "AT+SPD=0", "AT+TM=0", "AT+OSC=0", "AT+PLL=0",
                            "AT+HSTATUS=0", "AT+HCLEAR", "AT+ALGO=0", "AT+MODE=0", "AT+EMODE=1",
                            "AT+CPID=" + gains, "AT+TM=1", "AT+HSTATUS=1"):
                bench.command(command)
            bench.armed = True
            bench.command("AT+DMODE=0")
            bench.command("AT+SPD=20")
            bench.command(f"AT+CUR={args.low}")
            bench.collect(1.0)
            bench.tag = "steps:" + gains
            bench.command("AT+OSC=1")
            sent = 0
            completed = False
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                lines = bench.read()
                if "+OSC:" not in lines:
                    continue
                if sent == args.steps:
                    completed = True
                    break
                # Firmware begins capturing the next 51.2 ms block as soon as
                # it transmits the terminator. Put the command edge inside it.
                value = args.high if sent % 2 == 0 else args.low
                bench.command(f"AT+CUR={value}")
                sent += 1
            if not completed:
                raise RuntimeError("Not enough OSC blocks for all requested current steps")
            report["analysis"] = analyze(bench.records, args.low, args.high, 20000)
            print(gains, json.dumps(report["analysis"]), flush=True)
        except (Exception, KeyboardInterrupt) as error:
            report["error"] = str(error) or "Interrupted"
            print("STOP:", report["error"], flush=True)
        finally:
            try:
                report["parked"] = bench.park()
            except Exception as error:
                report["park_error"] = str(error)
            bench.port.close()
            report["commands"] = bench.commands
            report["records"] = bench.records
            reports.append(report)
            path = Path(args.out)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(reports, indent=2), encoding="utf-8")
        if "error" in report or "park_error" in report:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
