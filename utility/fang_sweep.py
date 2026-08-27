#!/usr/bin/env python3
"""Find the FOC electrical angle offset (AT+FANG) by sweeping it on the bench.

The offset between the PLL's Hall-derived step angle and the rotor d axis is
*derived* in the firmware (-60 electrical degrees, from the six-step table
geometry) but has never been measured.  If it is wrong it is wrong by a whole
multiple of 60 degrees, and the symptom is little or no torque.

Method: current-mode FOC (AT+ALGO=1, AT+EMODE=1) held at a fixed speed by the
velocity loop, measuring the **current it takes** to sustain that speed.  The
mechanical load is the motor's own friction and windage, which at a fixed speed
is fixed, so the current required is inversely proportional to torque per amp
and is *minimised* at the correct offset.

Do not use speed at a fixed current instead.  The motor on this bench is
unloaded, so a fixed current command just runs it up against its no-load
ceiling (~9 RPS, set by back-EMF against the bus) where the current loop is
voltage-saturated; the speed then reads 8.4-8.7 RPS across a 120-degree span of
offsets and discriminates nothing.  That was measured.  A speed low enough to
leave the loop headroom turns the same experiment into a sharp minimum.

    ./fang_sweep.py                           # +-90 deg around the derived value
    ./fang_sweep.py --range -110 -50 --step 5 # refine around the coarse winner

SAFETY - why current mode rather than voltage mode:

Voltage-mode FOC has no current limit at all.  At duty d the applied q-axis
voltage is d * 0.95 * Vbus/sqrt(3), and if the angle is wrong enough that the
rotor stalls or hunts there is nothing to stop the current; this was measured,
not guessed - a first attempt at this sweep in voltage mode hit -10.5 A at duty
0.08 on the very first point and had to abort.

In current mode the d/q regulators drive the measured (Id, Iq) to (0, Iq*), so
the current magnitude is the command regardless of how wrong the angle is.  The
one exception is an error near 180 degrees, where the feedback sign inverts and
the loop becomes positive feedback - which is why --coarse spans +-90 degrees
around the derived value rather than the whole circle, and why the offsets are
visited outward from the derived value rather than in numerical order.  A
current guard is still in place as a backstop.
"""

import argparse
import json
import sys
import time

from ecu import Ecu

STEPS_PER_REV = 120.0    # MOTOR_1: 20 pole pairs * 6 steps
CURRENT_CAP = 4.0        # the sweep never commands more than this
DERIVED_FANG = -60.0     # what the firmware derives; the sweep centres on it
ABORT_CURRENT_A = 6.0    # bail out if the drive pulls more than this


def collect(ecu, seconds):
    """Read +PLL and +TM for `seconds`, returning (pll_rows, tm_rows)."""
    pll, tm = [], []
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
                        # angle_per_second; pll_integral; angle; hall_raw; is_sync
                        pll.append((now, float(p[1]), int(p[4])))
                    except ValueError:
                        pass
            elif line.startswith("+TM:"):
                p = line[4:].split(";")
                if len(p) >= 9:  # temperature was appended as field 9
                    try:
                        # pos;target_pos;tgt_rps;cur_rps;duty;tgt_cur;meas_cur;vbus;angle
                        tm.append((now, float(p[6]), float(p[7]), float(p[5]),
                                   float(p[3])))
                    except ValueError:
                        pass
    return pll, tm


def measure(ecu, seconds):
    """Steady-state signed speed (steps/s) and mean current (A)."""
    ecu.cmd("AT+PLL=1", settle=0.0)
    ecu.cmd("AT+TM=1", settle=0.0)
    ecu.s.reset_input_buffer()
    time.sleep(0.3)
    ecu.s.reset_input_buffer()
    pll, tm = collect(ecu, seconds)
    ecu.cmd("AT+PLL=0", settle=0.05)
    ecu.cmd("AT+TM=0", settle=0.05)
    ecu.s.reset_input_buffer()

    if not pll or not tm:
        return None

    # Drop the first second: right after a buffer flush the ECU's queued
    # telemetry arrives in a burst, and those samples are not evenly spaced in
    # real time.  Same reasoning as pll_sweep.py.
    pll = [r for r in pll if r[0] >= pll[0][0] + 1.0] or pll
    tm = [r for r in tm if r[0] >= tm[0][0] + 1.0] or tm

    speed = sum(r[1] for r in pll) / len(pll)          # signed, steps/s
    sync = sum(r[2] for r in pll) / len(pll)
    current = sum(r[1] for r in tm) / len(tm)          # A, filtered, measured
    vbus = sum(r[2] for r in tm) / len(tm)
    target = sum(r[3] for r in tm) / len(tm)           # A, speed-loop demand
    rps_tm = sum(r[4] for r in tm) / len(tm)
    n = len(tm)
    sd = (sum((r[1] - current) ** 2 for r in tm) / n) ** 0.5
    return {"steps_s": speed, "rps": speed / STEPS_PER_REV,
            "current_a": current, "current_sd": sd, "target_a": target,
            "rps_tm": rps_tm, "vbus": vbus, "sync_frac": sync,
            "samples": len(pll)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--speed", type=float, default=6.0,
                    help="speed to hold, RPS; must leave the current loop headroom")
    ap.add_argument("--coarse", action="store_true",
                    help="(default) sweep +-90 degrees around the derived value")
    ap.add_argument("--range", nargs=2, type=float, metavar=("LO", "HI"),
                    default=None, help="offset range in electrical degrees")
    ap.add_argument("--step", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=3.0, help="measurement window")
    ap.add_argument("--settle", type=float, default=2.5)
    ap.add_argument("--dmode", type=int, default=0, help="0=FORWARD, 1=REVERSE")
    ap.add_argument("--out", default=None, help="write results as JSON")
    args = ap.parse_args()

    if args.range:
        lo, hi = args.range
        n = int(round((hi - lo) / args.step))
        offsets = [lo + i * args.step for i in range(n + 1)]
    else:
        step = 15.0
        offsets = [DERIVED_FANG + i * step for i in range(-6, 7)]

    # Visit outward from the derived value, nearest first. If something is going
    # to misbehave it misbehaves at the far end, and by then there is already
    # enough of a curve to see which way the peak lies.
    offsets.sort(key=lambda a: abs(a - DERIVED_FANG))

    ecu = Ecu(args.port)
    fang_was = ecu.query("AT+FANG?", "+FANG:")
    algo_was = ecu.query("AT+ALGO?", "+ALGO:") or "0"
    results = []

    print(f"holding {args.speed:.2f} RPS, {len(offsets)} offsets, "
          f"{args.seconds:.1f} s each, FANG was {fang_was}")
    print(f"{'FANG':>7} {'RPS':>7} {'I (A)':>8} {'sd':>6} {'Idem':>7} {'sync':>6}")

    try:
        ecu.cmd("AT+OSC=0"); ecu.cmd("AT+TM=0")
        ecu.cmd("AT+PLL=0"); ecu.cmd("AT+HSTATUS=0")
        ecu.cmd("AT+HCLEAR")
        ecu.cmd("AT+MODE=1")     # CLOSED_LOOP_VELOCITY: hold the speed, watch the current
        ecu.cmd("AT+ALGO=1")     # FOC
        ecu.cmd("AT+EMODE=1")    # current mode: bounds the current at any angle
        if (ecu.query("AT+ALGO?", "+ALGO:") or "").strip() != "1":
            print("!! could not select FOC")
            return 1

        for attempt in range(3):
            ecu.cmd(f"AT+DMODE={args.dmode}")
            if (ecu.query("AT+DMODE?", "+DMODE:") or "").strip() == str(args.dmode):
                break
        else:
            print("!! could not leave NEUTRAL")
            return 1

        ecu.cmd(f"AT+SPD={args.speed:.2f}")
        time.sleep(args.speed / 2.0 + 3.0)   # ramp at BLDC_MAX_ACCELERATION

        for off in offsets:
            ecu.cmd(f"AT+FANG={off:.1f}")
            time.sleep(args.settle)
            r = measure(ecu, args.seconds)
            if r is None:
                print(f"{off:7.1f}   -- no telemetry --")
                continue
            r["fang"] = off
            r["speed_cmd"] = args.speed
            results.append(r)
            print(f"{off:7.1f} {r['rps']:7.3f} {r['current_a']:8.3f} "
                  f"{r['current_sd']:6.3f} {r['target_a']:7.3f} "
                  f"{100*r['sync_frac']:5.0f}%")

            if abs(r["current_a"]) > ABORT_CURRENT_A:
                print(f"!! current {r['current_a']:.2f} A exceeds "
                      f"{ABORT_CURRENT_A} A - aborting sweep")
                break
    finally:
        ecu.cmd("AT+CUR=0")
        ecu.cmd("AT+DUT=0")
        ecu.cmd("AT+SPD=0")
        time.sleep(1.5)
        ecu.cmd("AT+DMODE=2")
        ecu.cmd("AT+PLL=0"); ecu.cmd("AT+TM=0"); ecu.cmd("AT+HSTATUS=0")
        ecu.cmd(f"AT+ALGO={algo_was}")
        if fang_was is not None:
            ecu.cmd(f"AT+FANG={fang_was}")
        print(f"left in NEUTRAL, ALGO={algo_was}, FANG={fang_was}")
        ecu.close()

    if results:
        # Only points that actually held the commanded speed are comparable:
        # if the motor fell out of speed the load changed and the current is
        # not measuring the same thing.
        held = [r for r in results if abs(r["rps"]) > 0.9 * args.speed]
        pool = held or results
        best = min(pool, key=lambda r: abs(r["current_a"]))
        print(f"\nleast current at speed: FANG={best['fang']:.1f} deg -> "
              f"{best['current_a']:.3f} A at {best['rps']:.3f} RPS")
        if len(held) < len(results):
            dropped = [r["fang"] for r in results if r not in held]
            print(f"  (excluded, did not hold speed: "
                  f"{', '.join(f'{d:.0f}' for d in dropped)})")
        ordered = sorted(results, key=lambda r: r["fang"])
        print("current required to hold speed (lower is better):")
        span = max(abs(r["current_a"]) for r in ordered) or 1.0
        for r in ordered:
            bar = "#" * int(round(46 * abs(r["current_a"]) / span))
            flag = "" if r in held else "  <- lost speed"
            print(f"  {r['fang']:7.1f} {r['current_a']:7.3f} |{bar}{flag}")
    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=1)
        print(f"-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
