#!/usr/bin/env python3
"""Bounded, logged motor probes with live telemetry guards and neutral cleanup.

Example: python3 utility/bench_motor.py --poles 8 --mode speed --values 2 5 10
         --out /tmp/opencode/jk42-speed.json
"""

import argparse
import datetime
import json
import math
import re
from pathlib import Path
import socket
import statistics
import subprocess
import time

import serial
from ecu import BAUD, frame


class Bench:
    def __init__(self, args):
        self.args = args
        self.port = serial.Serial(args.port, BAUD, timeout=0.01, exclusive=True)
        self.buffer = b""
        self.started = time.monotonic()
        self.last_tm = self.started
        self.armed = False
        self.guard = True
        self.tag = "setup"
        self.records = []
        self.commands = []
        self.swd = None
        self.swd_output = ""
        try:
            self.hall_socket = socket.create_connection(("127.0.0.1", args.hall_swd), timeout=1) if args.hall_swd else None
        except Exception:
            self.port.close()
            raise
        self.next_hall = self.started
        self.port.reset_input_buffer()

    def read(self):
        if self.hall_socket is not None and time.monotonic() >= self.next_hall:
            self.hall_socket.sendall(b"capture {mdw 0x48000410 1}\x1a")
            reply = b""
            while not reply.endswith(b"\x1a"):
                chunk = self.hall_socket.recv(4096)
                if not chunk:
                    raise RuntimeError("SWD connection closed")
                reply += chunk
            match = re.search(rb"0x48000410:\s*([0-9a-fA-F]+)", reply)
            if not match:
                raise RuntimeError("Unexpected GPIO read: " + repr(reply))
            code = (int(match[1], 16) >> 6) & 7
            position = (255, 0, 2, 1, 4, 5, 3, 255)[code]
            now = time.monotonic()
            self.records.append({"t": now - self.started, "tag": self.tag,
                                 "line": "+GPIO:" + str(code), "hall": position})
            self.next_hall = now + 0.003
        self.buffer += self.port.read(self.port.in_waiting or 1)
        *lines, self.buffer = self.buffer.split(b"\n")
        result = []
        for raw in lines:
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            now = time.monotonic()
            row = {"t": now - self.started, "tag": self.tag, "line": line}
            result.append(line)
            self.records.append(row)
            if line.startswith(("+TM:", "+PLL:", "+HSTATUS:")):
                try:
                    values = [float(value) for value in line.split(":", 1)[1].split(";")]
                except ValueError:
                    row["parse_error"] = True
                    continue
                expected = 10 if line.startswith("+TM:") else 5 if line.startswith("+PLL:") else 8
                if len(values) != expected:
                    row["parse_error"] = True
                    continue
                row["values"] = values
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError("Non-finite telemetry: " + line)
                if line.startswith("+TM:"):
                    self.last_tm = now
                    if self.guard and self.armed:
                        if abs(values[6]) > self.args.max_current:
                            raise RuntimeError(f"Current guard: {values[6]:.3f} A")
                        if abs(values[3]) > self.args.max_speed:
                            raise RuntimeError(f"Speed guard: {values[3]:.3f} RPS")
                        if not 10.0 <= values[7] <= 30.0:
                            raise RuntimeError(f"Bus-voltage guard: {values[7]:.3f} V")
                        if len(values) >= 10 and values[9] > 65.0:
                            raise RuntimeError(f"Temperature guard: {values[9]:.1f} C")
                elif line.startswith("+HSTATUS:") and self.guard and self.armed and values[0]:
                    raise RuntimeError("Hall fault: " + line)
        if self.guard and self.armed and time.monotonic() - self.last_tm > 0.5:
            raise RuntimeError("Telemetry timeout while the bridge is enabled")
        return result

    def command(self, command, prefix=None):
        self.commands.append({"t": time.monotonic() - self.started, "command": command})
        self.port.write(frame(command))
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            for line in self.read():
                if line == "ERROR":
                    raise RuntimeError(f"Rejected command: {command}")
                if prefix and line.startswith(prefix):
                    return line[len(prefix):].strip()
                if not prefix and line == "OK":
                    return "OK"
        raise RuntimeError(f"Command timeout: {command}")

    def collect(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.read()

    def start_profile(self):
        if not self.args.profile_address:
            return
        address = int(self.args.profile_address, 0)
        commands = ["init", f"mww 0x{address:x} 0"]
        commands += [f"mww 0x{address + offset:x} 0" for offset in (4, 8, 12, 16)]
        commands += [f"mww 0x{address:x} 20000", "shutdown"]
        self.swd = subprocess.Popen(
            ["openocd", "-f", "interface/stlink.cfg", "-f", "target/stm32g4x.cfg",
             "-c", "; ".join(commands)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True)

    def finish_profile(self):
        if self.swd is None:
            return
        deadline = time.monotonic() + 10
        while self.swd.poll() is None:
            if time.monotonic() >= deadline:
                self.swd.terminate()
                raise RuntimeError("SWD profiling setup timed out")
            self.read()
        self.swd_output = self.swd.communicate(timeout=10)[0]
        if self.swd.returncode:
            raise RuntimeError("SWD profiling setup failed: " + self.swd_output)
        self.swd = None

    def park(self):
        self.guard = False
        # Send both frames even if the first acknowledgement is lost.
        self.port.write(frame("AT+DMODE=2") + frame("AT+SPD=0"))
        self.armed = False
        self.collect(0.1)
        for command in ("AT+OSC=0", "AT+TM=0", "AT+PLL=0", "AT+HSTATUS=0"):
            self.command(command)
        mode = self.command("AT+DMODE?", "+DMODE:")
        speed = self.command("AT+SPD?", "+SPD:")
        current = self.command("AT+CUR?", "+CUR:")
        duty = self.command("AT+DUT?", "+DUT:")
        # SPD? reports measured speed: a disconnected bridge may still be coasting.
        if mode != "2" or float(current) != 0.0 or float(duty) != 0.0:
            raise RuntimeError(f"Unexpected parked state: DMODE={mode}, CUR={current}, DUT={duty}")
        return {"dmode": mode, "speed_command": 0, "speed_measured": speed,
                "current_command": current, "duty": duty}


def summarize(records, tag, since, poles):
    rows = [row for row in records if row["tag"] == tag and row["t"] >= since]
    tm = [row["values"] for row in rows if row["line"].startswith("+TM:") and "values" in row]
    pll = [row["values"] for row in rows if row["line"].startswith("+PLL:") and "values" in row]
    hall = [row for row in rows if row["line"].startswith("+HSTATUS:") and "values" in row]
    if len(tm) < 10:
        raise RuntimeError("Too few telemetry samples")
    result = {"samples": len(tm)}
    for name, column in (("speed_rps", 3), ("duty", 4), ("target_a", 5), ("current_a", 6), ("bus_v", 7)):
        values = [row[column] for row in tm]
        result[name] = statistics.mean(values)
        result[name + "_sd"] = statistics.pstdev(values)
        result[name + "_min"] = min(values)
        result[name + "_max"] = max(values)
    if pll:
        result["pll_signed_rps"] = statistics.mean(row[1] for row in pll) / (poles * 3)
        result["pll_sync_fraction"] = statistics.mean(row[4] for row in pll)
    if len(hall) >= 10:
        mean_t = statistics.mean(row["t"] for row in hall)
        mean_edges = statistics.mean(row["values"][5] for row in hall)
        numerator = sum((row["t"] - mean_t) * (row["values"][5] - mean_edges) for row in hall)
        denominator = sum((row["t"] - mean_t) ** 2 for row in hall)
        result["hall_edge_rps"] = numerator / denominator / (poles * 3)
        result["hall_erratic_fraction"] = statistics.mean(row["values"][2] for row in hall)
        result["hall_faults"] = sum(bool(row["values"][0]) for row in hall)
    gpio = [row for row in rows if "hall" in row and row["hall"] < 6]
    if len(gpio) > 10:
        points = []
        position = 0
        previous = gpio[0]["hall"]
        skipped = 0
        forward = reverse = 0
        for row in gpio:
            delta = (row["hall"] - previous + 3) % 6 - 3
            skipped += abs(delta) > 1
            forward += delta > 0
            reverse += delta < 0
            position += delta
            previous = row["hall"]
            points.append((row["t"], position))
        mean_t = statistics.mean(row[0] for row in points)
        mean_position = statistics.mean(row[1] for row in points)
        slope = sum((t - mean_t) * (pos - mean_position) for t, pos in points)
        variance = sum((t - mean_t) ** 2 for t, _ in points)
        result["gpio_signed_rps"] = slope / variance / (poles * 3)
        result["gpio_samples"] = len(gpio)
        result["gpio_skipped_steps"] = skipped
        result["gpio_forward_edges"] = forward
        result["gpio_reverse_edges"] = reverse
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--poles", type=int, required=True)
    parser.add_argument("--algo", type=int, choices=(0, 1), default=0)
    parser.add_argument("--mode", choices=("voltage", "current", "speed", "speed-voltage",
                                           "open-voltage", "open-current"), default="speed")
    parser.add_argument("--open-speed", type=float, default=20.0,
                        help="SPD command in open-loop mode (check actual commutation frequency)")
    parser.add_argument("--osc", action="store_true", help="Record the raw 20 kHz OSC bursts as well")
    parser.add_argument("--direction", type=int, choices=(0, 1), default=0)
    parser.add_argument("--values", type=float, nargs="+", required=True)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--max-current", type=float, default=2.0)
    parser.add_argument("--max-speed", type=float, default=40.0)
    parser.add_argument("--set", action="append", default=[], help="Additional setting, e.g. PLLID=100,5000")
    parser.add_argument("--profile-address", help="Address of pwm_bench_profile in the bench ELF")
    parser.add_argument("--hall-swd", type=int, help="OpenOCD Tcl port for independent GPIO Hall sampling")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    if args.poles <= 0 or args.poles % 2 or args.seconds <= 0 or args.settle < 0:
        parser.error("Require an even positive pole count and positive measurement duration")
    bench = Bench(args)
    report = {"date": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "config": vars(args), "results": []}
    try:
        for command in ("AT+DMODE=2", "AT+SPD=0", "AT+OSC=0", "AT+TM=0", "AT+PLL=0", "AT+HSTATUS=0",
                        "AT+HCLEAR", f"AT+ALGO={args.algo}",
                        f"AT+EMODE={0 if args.mode in ('voltage', 'speed-voltage', 'open-voltage') else 1}",
                        f"AT+MODE={0 if args.mode.startswith('open') else 1 if args.mode.startswith('speed') else 2}",
                        "AT+ACC=5"):
            bench.command(command)
        for setting in args.set:
            bench.command("AT+" + setting)
        report["gains"] = {name: bench.command("AT+" + name + "?", "+" + name + ":")
                           for name in ("SPID", "CPID", "FPID", "PLLID", "FANG")}
        for command in ("AT+TM=1", "AT+PLL=1", "AT+HSTATUS=1"):
            bench.command(command)
        bench.collect(0.1)
        bench.armed = True
        bench.command(f"AT+DMODE={args.direction}")
        if args.mode.startswith("open"):
            bench.command(f"AT+SPD={args.open_speed:.5f}")
        if args.osc:
            bench.command("AT+OSC=1")
        setting = ("SPD" if args.mode.startswith("speed") else
                   "DUT" if args.mode in ("voltage", "open-voltage") else "CUR")
        for index, value in enumerate(args.values):
            bench.tag = f"{setting}={value}"
            bench.command(f"AT+{setting}={value:.5f}")
            bench.collect(args.settle)
            since = time.monotonic() - bench.started
            if index == len(args.values) - 1:
                bench.start_profile()
            bench.collect(args.seconds)
            bench.finish_profile()
            result = summarize(bench.records, bench.tag, since, args.poles)
            result["command"] = value
            report["results"].append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = str(error) or "Interrupted"
        print("STOP:", report["error"], flush=True)
    finally:
        try:
            report["parked"] = bench.park()
            print("Parked:", report["parked"], flush=True)
        except Exception as error:
            report["park_error"] = str(error)
            print("PARK ERROR:", error, flush=True)
        try:
            bench.finish_profile()
        except Exception as error:
            report["profile_error"] = str(error)
        finally:
            bench.port.close()
            if bench.hall_socket is not None:
                bench.hall_socket.close()
        report["commands"] = bench.commands
        report["records"] = bench.records
        report["swd_setup"] = bench.swd_output
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return int(any(key in report for key in ("error", "park_error", "profile_error")))


if __name__ == "__main__":
    raise SystemExit(main())
