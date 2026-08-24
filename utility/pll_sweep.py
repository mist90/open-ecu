#!/usr/bin/env python3
"""Sweep speeds in VELOCITY mode and score PLL lock quality at each one.

The PLL's own speed estimate cannot judge the PLL, so the reference here is
HallMonitor's cumulative `edges` counter from +HSTATUS: every Hall transition
is one "step", so d(edges)/dt is the true electrical speed in steps/s,
measured completely independently of the PLL.

    ./pll_sweep.py 0.5 1 1.5 2 3 5 8
    ./pll_sweep.py 1.5 --gains 40,900 --out /tmp/run.json
"""

import argparse
import json
import sys
import time

from ecu import Ecu

STEPS_PER_REV = 120.0   # MOTOR_1: 20 pole pairs * 6 steps
ACCEL_RPS2 = 2.0        # BLDC_MAX_ACCELERATION


def collect(ecu, seconds):
    """Read telemetry for `seconds`, returning (pll_rows, hall_rows)."""
    pll, hall = [], []
    buf = b""
    end = time.time() + seconds
    while time.time() < end:
        buf += ecu.s.read(65536)
        now = time.time()
        *lines, buf = buf.split(b"\n")
        for raw in lines:
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("+PLL:"):
                p = line[5:].split(";")
                if len(p) == 5:
                    try:
                        pll.append((now, float(p[0]), float(p[1]), float(p[2]),
                                    int(p[3]), int(p[4])))
                    except ValueError:
                        pass
            elif line.startswith("+HSTATUS:"):
                p = line[9:].split(";")
                if len(p) == 8:
                    try:
                        # fault, edges, erratic_score
                        hall.append((now, int(p[0]), int(p[5]), float(p[2])))
                    except ValueError:
                        pass
    return pll, hall


def score(pll, hall, rps):
    if len(pll) < 20 or len(hall) < 20:
        return None
    # True electrical speed from the Hall edge counter (PLL-independent).
    #
    # Fit a line through (arrival time, edges) rather than dividing endpoint by
    # endpoint.  Right after the input buffer is flushed the ECU's queued
    # telemetry arrives in a burst, so the first samples carry arrival
    # timestamps that are far too close together; a two-point rate over a short
    # window then over-reads by several percent.  Least squares over the whole
    # window, with the first second dropped, is immune to that.
    hall = [r for r in hall if r[0] >= hall[0][0] + 1.0] or hall
    n_h = len(hall)
    tm = sum(r[0] for r in hall) / n_h
    em = sum(r[2] for r in hall) / n_h
    sxx = sum((r[0] - tm) ** 2 for r in hall)
    sxy = sum((r[0] - tm) * (r[2] - em) for r in hall)
    raw_steps_s = sxy / sxx if sxx > 0 else float("nan")
    # `edges` counts every transition, including non-advancing chatter, so it
    # over-reads the real rotation rate by the erratic fraction.
    erratic = sum(r[3] for r in hall) / n_h
    true_steps_s = raw_steps_s * (1.0 - erratic)
    faults = sum(1 for r in hall if r[1])
    pll = [r for r in pll if r[0] >= pll[0][0] + 1.0] or pll

    # In REVERSE the Hall sequence descends and the PLL speed goes negative,
    # while `edges` still counts transitions upwards - compare magnitudes.
    aps = [abs(r[1]) for r in pll]
    integ = [abs(r[2]) for r in pll]
    sync = [r[5] for r in pll]
    n = len(aps)
    mean = sum(aps) / n
    var = sum((a - mean) ** 2 for a in aps) / n
    sd = var ** 0.5
    imean = sum(integ) / n
    isd = (sum((a - imean) ** 2 for a in integ) / n) ** 0.5
    return {
        "raw_steps_s": raw_steps_s,
        "erratic": erratic,
        "integ_sd": isd,
        "integ_bias_pct": 100.0 * (imean - true_steps_s) / true_steps_s if true_steps_s else float("nan"),
        "rps_cmd": rps,
        "true_steps_s": true_steps_s,
        "true_rps": true_steps_s / STEPS_PER_REV,
        "pll_steps_s_mean": mean,
        "pll_steps_s_sd": sd,
        "pll_steps_s_min": min(aps),
        "pll_steps_s_max": max(aps),
        "integ_mean": sum(integ) / n,
        "sync_frac": sum(sync) / n,
        "bias_pct": 100.0 * (mean - true_steps_s) / true_steps_s if true_steps_s else float("nan"),
        "hall_fault": faults,
        "samples": n,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("speeds", nargs="+", type=float, help="target speeds in RPS")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--settle", type=float, default=2.5)
    ap.add_argument("--gains", default=None, metavar="KP,KI",
                    help="apply AT+PLLID=KP,KI before the sweep")
    ap.add_argument("--out", default=None, help="write results as JSON")
    ap.add_argument("--dmode", type=int, default=0, help="0=FORWARD, 1=REVERSE")
    args = ap.parse_args()

    ecu = Ecu(args.port)
    dmode_was = ecu.query("AT+DMODE?", "+DMODE:") or "2"
    results = []

    try:
        ecu.cmd("AT+OSC=0")
        ecu.cmd("AT+TM=0")
        ecu.cmd("AT+PLL=0")
        ecu.cmd("AT+HSTATUS=0")
        ecu.cmd("AT+HCLEAR")
        if args.gains:
            ecu.cmd(f"AT+PLLID={args.gains}")
        gains = ecu.query("AT+PLLID?", "+PLLID:")
        print(f"PLL gains: {gains}")

        ecu.cmd("AT+MODE=1")     # CLOSED_LOOP_VELOCITY
        for attempt in range(3):
            ecu.cmd(f"AT+DMODE={args.dmode}")
            if (ecu.query("AT+DMODE?", "+DMODE:") or "").strip() == str(args.dmode):
                break
        else:
            print("!! could not leave NEUTRAL")
            return 1

        prev = 0.0
        for rps in args.speeds:
            ecu.cmd(f"AT+SPD={rps:.2f}")
            ramp = abs(rps - prev) / ACCEL_RPS2 + args.settle
            prev = rps
            time.sleep(ramp)
            ecu.cmd("AT+PLL=1", settle=0.0)
            ecu.cmd("AT+HSTATUS=1", settle=0.0)
            ecu.s.reset_input_buffer()
            time.sleep(0.3)
            ecu.s.reset_input_buffer()
            pll, hall = collect(ecu, args.seconds)
            ecu.cmd("AT+PLL=0", settle=0.05)
            ecu.cmd("AT+HSTATUS=0", settle=0.05)
            ecu.s.reset_input_buffer()
            r = score(pll, hall, rps)
            if r is None:
                print(f"{rps:5.2f} RPS: no telemetry ({len(pll)} pll, {len(hall)} hall)")
                continue
            r["gains"] = gains
            results.append(r)
            print(f"{rps:5.2f} cmd | true {r['true_rps']:6.3f} RPS "
                  f"({r['true_steps_s']:7.1f} st/s, chatter {100*r['erratic']:4.1f}%) "
                  f"| PIout {r['pll_steps_s_mean']:7.1f} sd {r['pll_steps_s_sd']:5.1f} "
                  f"| SPEED {r['integ_bias_pct']:+6.1f}% sd {r['integ_sd']:5.1f} "
                  f"| sync {100*r['sync_frac']:5.1f}% | hf {r['hall_fault']}")
    finally:
        ecu.cmd("AT+PLL=0")
        ecu.cmd("AT+HSTATUS=0")
        ecu.cmd("AT+SPD=0")
        time.sleep(max(args.speeds) / ACCEL_RPS2 + 1.5)
        ecu.cmd(f"AT+DMODE={dmode_was}")
        ecu.cmd("AT+TM=0")
        ecu.close()

    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=1)
        print(f"-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
