#!/usr/bin/env python3
"""Capture repeated current steps in CLOSED_LOOP_TORQUE mode.

This is the transient a feed-forward is supposed to win: the current setpoint
steps hard, and the PI would otherwise have to build the whole duty out of
integral action.  A speed ramp does not test that, because the acceleration
limiter makes the demand far slower than the current loop.

Both step levels are positive - no regen - so the bus cannot be pumped by a
supply that will not sink current.  Restores the mode and drive state it found,
including on Ctrl-C.

    ./capture_torque_step.py --out step.log
"""

import argparse
import datetime
import sys
import time

from capture_log import Ecu, frame  # reuse framing / verified command helpers


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--low", type=float, default=0.5, help="low current level (A)")
    ap.add_argument("--high", type=float, default=2.5, help="high current level (A)")
    ap.add_argument("--period", type=float, default=0.30, help="full square-wave period (s)")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    ecu = Ecu(args.port)
    mode_was = ecu.query("AT+MODE?", "+MODE:") or "1"
    dmode_was = ecu.query("AT+DMODE?", "+DMODE:") or "2"
    print(f"mode={mode_was} dmode={dmode_was} emode={ecu.query('AT+EMODE?', '+EMODE:')}")

    try:
        ecu.cmd("AT+OSC=0")
        ecu.cmd("AT+TM=0")
        ecu.cmd("AT+CUR=0")

        # Torque mode: no speed PID, the current setpoint is the command.
        ecu.cmd("AT+MODE=2")
        if (ecu.query("AT+MODE?", "+MODE:") or "").strip() != "2":
            print("!! could not enter torque mode - re-plug the USB and retry")
            return 1
        for _ in range(3):
            ecu.cmd("AT+DMODE=0")
            if (ecu.query("AT+DMODE?", "+DMODE:") or "").strip() == "0":
                break
        else:
            print("!! could not leave NEUTRAL")
            return 1

        # Get it turning before stepping, so the steps happen against a real
        # back-EMF rather than from standstill.
        ecu.cmd(f"AT+CUR={args.low:.2f}")
        ecu.cmd("AT+TM=1")
        time.sleep(2.0)
        print(f"  pre-step speed {ecu.speed()} RPS")

        ecu.cmd("AT+OSC=1", settle=0.0)
        ecu.s.reset_input_buffer()
        print(f"stepping {args.low:g} <-> {args.high:g} A every {args.period / 2:.2f} s "
              f"for {args.seconds:.1f} s -> {args.out}")

        n = 0
        high = False
        end = time.time() + args.seconds
        next_toggle = time.time()
        with open(args.out, "w") as f:
            buf = b""
            while time.time() < end:
                now = time.time()
                if now >= next_toggle:
                    high = not high
                    level = args.high if high else args.low
                    ecu.s.write(frame(f"AT+CUR={level:.2f}"))
                    f.write(f"[{datetime.datetime.now().strftime('%H:%M:%S.%f')}] "
                            f"TX: AT+CUR={level:.2f}\n")
                    next_toggle = now + args.period / 2.0
                buf += ecu.s.read(65536)
                *lines, buf = buf.split(b"\r\n")
                for raw in lines:
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")
                    f.write(f"[{ts}] RX: {line}\n")
                    n += 1
        print(f"  {n} lines")
    finally:
        ecu.cmd("AT+OSC=0")
        ecu.cmd("AT+CUR=0")
        print("coasting down ...")
        time.sleep(3.0)
        ecu.cmd(f"AT+DMODE={dmode_was}")
        ecu.cmd(f"AT+MODE={mode_was}")
        ecu.cmd("AT+TM=0")
        print(f"restored mode={mode_was} dmode={dmode_was}")
        ecu.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
