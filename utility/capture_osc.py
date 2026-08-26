#!/usr/bin/env python3
"""Capture the 20 kHz +OSC ring buffer at a settled speed.

+OSC streams 1024 consecutive PWM-ISR samples (51.2 ms) as
    +OSC:<n>,<measured_mA>,<target_mA>,<duty%>,<commutation step>[,<Id_mA>,<Iq_mA>,<angle_deg_e>]
then repeats.  That is the only view fast enough to see commutation-rate
vibration or anything FOC does; +TM and +PLL are 100 Hz and alias it away.

The last three fields were added with FOC and are zero in six-step mode.
Firmware from before that change emits only the first five, which is why the
parser below accepts either width.

    ./capture_osc.py 3.58 --out /tmp/rel.csv --bursts 6
"""

import argparse
import sys
import time

from ecu import Ecu

ACCEL_RPS2 = 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rps", type=float)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--bursts", type=int, default=6, help="1024-sample bursts to keep")
    ap.add_argument("--settle", type=float, default=4.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dmode", type=int, default=0)
    ap.add_argument("--algo", type=int, choices=(0, 1), default=None,
                    help="0 = six-step, 1 = FOC. Omit to leave the ECU as it is.")
    args = ap.parse_args()

    ecu = Ecu(args.port)
    dmode_was = ecu.query("AT+DMODE?", "+DMODE:") or "2"
    rows = []
    try:
        ecu.cmd("AT+OSC=0"); ecu.cmd("AT+TM=0")
        ecu.cmd("AT+PLL=0"); ecu.cmd("AT+HSTATUS=0")
        ecu.cmd("AT+MODE=1")
        if args.algo is not None:
            # Switch algorithms before leaving NEUTRAL: it is a step change in
            # the applied voltage, and doing it while spinning is a transient
            # test rather than a capture.
            ecu.cmd(f"AT+ALGO={args.algo}")
        for _ in range(3):
            ecu.cmd(f"AT+DMODE={args.dmode}")
            if (ecu.query("AT+DMODE?", "+DMODE:") or "").strip() == str(args.dmode):
                break
        else:
            print("!! could not leave NEUTRAL"); return 1
        ecu.cmd(f"AT+SPD={args.rps:.2f}")
        time.sleep(args.rps / ACCEL_RPS2 + args.settle)

        ecu.cmd("AT+OSC=1", settle=0.0)
        ecu.s.reset_input_buffer()
        bursts = 0
        buf = b""
        deadline = time.time() + 40.0
        cur = []
        while bursts < args.bursts and time.time() < deadline:
            buf += ecu.s.read(65536)
            *lines, buf = buf.split(b"\r\n")
            for raw in lines:
                line = raw.decode("ascii", "replace").strip()
                if not line.startswith("+OSC:"):
                    continue
                body = line[5:]
                if not body:              # burst terminator
                    if len(cur) > 900:
                        rows.append(cur)
                        bursts += 1
                    cur = []
                    continue
                p = body.split(",")
                if len(p) in (5, 8):
                    try:
                        row = [int(x) for x in p]
                    except ValueError:
                        continue
                    row += [0] * (8 - len(row))   # pre-FOC firmware: pad Id/Iq/angle
                    cur.append(tuple(row))
        ecu.cmd("AT+OSC=0")
    finally:
        ecu.cmd("AT+OSC=0")
        ecu.cmd("AT+SPD=0")
        time.sleep(args.rps / ACCEL_RPS2 + 2.0)
        ecu.cmd("AT+DMODE=2")
        ecu.close()

    with open(args.out, "w") as f:
        f.write("burst,n,meas_mA,tgt_mA,duty_pct,step,id_mA,iq_mA,angle_deg_e\n")
        for b, burst in enumerate(rows):
            for r in burst:
                f.write(f"{b},{','.join(str(x) for x in r)}\n")
    print(f"{len(rows)} bursts, {sum(len(b) for b in rows)} samples -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
