#!/usr/bin/env python3
"""Capture a +TM/+OSC log from the ECU at a commanded speed.

Spins the motor up, waits for the speed to settle (the firmware slew-limits
acceleration, so this is not instant), captures for a fixed window, then always
ramps down and restores the drive mode it found - including on Ctrl-C or error.

    ./capture_log.py 8            -> 8RPS.log
    ./capture_log.py 8 --seconds 8 --out /tmp/eight.log
"""

import argparse
import datetime
import sys
import time

import serial

BAUD = 2_000_000


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(cmd: str) -> bytes:
    return f"{cmd}*{crc16_ccitt(cmd.encode('ascii')):04X}\r\n".encode("ascii")


class Ecu:
    def __init__(self, port: str):
        self.s = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def cmd(self, c: str, settle: float = 0.15) -> list[str]:
        self.s.write(frame(c))
        time.sleep(settle)
        raw = self.s.read(65536).decode("ascii", errors="replace")
        return [l for l in raw.splitlines() if l.strip()]

    def query(self, c: str, prefix: str) -> str | None:
        for line in self.cmd(c):
            if line.startswith(prefix):
                return line[len(prefix):]
        return None

    def tm(self) -> list[float] | None:
        """Most recent complete +TM line as floats, or None."""
        raw = self.s.read(65536).decode("ascii", errors="replace")
        for line in reversed(raw.splitlines()):
            if line.startswith("+TM:"):
                parts = line[4:].split(";")
                # ">=": field 9 (temperature) was appended later, and the
                # callers here only index the original nine.
                if len(parts) >= 9:
                    try:
                        return [float(x) for x in parts]
                    except ValueError:
                        pass
        return None

    def speed(self) -> float | None:
        t = self.tm()
        return t[3] if t else None

    def target_speed(self) -> float | None:
        t = self.tm()
        return t[2] if t else None

    def close(self):
        self.s.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("rps", type=float, help="target speed in RPS")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--seconds", type=float, default=6.0, help="capture window")
    ap.add_argument("--settle", type=float, default=3.0, help="extra settle time after the ramp")
    ap.add_argument("--out", default=None)
    ap.add_argument("--ramp-from", type=float, default=None, metavar="RPS",
                    help="settle at this speed first, then start capturing and "
                         "command `rps` - captures the transient, which is where "
                         "a feed-forward should differ from PI-only")
    args = ap.parse_args()

    out = args.out or f"{args.rps:g}RPS.log"
    ecu = Ecu(args.port)

    print(f"ver={ecu.query('AT+VER?', '+VER:')}  mode={ecu.query('AT+MODE?', '+MODE:')}"
          f"  emode={ecu.query('AT+EMODE?', '+EMODE:')}  dmode={ecu.query('AT+DMODE?', '+DMODE:')}")
    dmode_was = ecu.query("AT+DMODE?", "+DMODE:") or "2"

    try:
        # Send all commands with telemetry OFF. A saturated 2 Mbaud TX stream
        # appears to be involved in the failure mode where the ECU keeps
        # transmitting but stops acting on commands (recovered only by
        # re-plugging the USB), so keep the link quiet while commanding.
        ecu.cmd("AT+OSC=0")
        ecu.cmd("AT+TM=0")
        # NEUTRAL disables the bridge and makes setTargetSpeed() a no-op, so
        # drive mode has to be confirmed before the speed command is sent.
        for attempt in range(3):
            ecu.cmd("AT+DMODE=0")
            if (ecu.query("AT+DMODE?", "+DMODE:") or "").strip() == "0":
                break
            print(f"  DMODE not accepted (attempt {attempt + 1}), retrying")
        else:
            print("!! could not leave NEUTRAL - aborting")
            return 1

        ecu.cmd("AT+TM=1")   # need +TM to read back the accepted target
        first = args.ramp_from if args.ramp_from is not None else args.rps
        for attempt in range(3):
            ecu.cmd(f"AT+SPD={first:.2f}")
            time.sleep(0.4)
            tgt = ecu.target_speed()
            if tgt is not None and abs(tgt - first) < 0.01:
                break
            print(f"  SPD not accepted (target reads {tgt}, attempt {attempt + 1}), retrying")
        else:
            print("!! speed command never took. If telemetry is still streaming, the link is\n"
                  "   in the state where the ECU transmits but ignores commands - re-plug the\n"
                  "   USB and try again.")
            return 1

        # acceleration_rate is 5 RPS/s, so wait for the ramp plus a settle
        # window, and confirm we actually got there before capturing.
        ramp = first / 5.0 + args.settle
        print(f"spinning up to {first:g} RPS, waiting {ramp:.1f} s ...")
        time.sleep(ramp)
        reached = None
        deadline = time.time() + 10.0
        while time.time() < deadline:
            spd = ecu.speed()
            if spd is not None:
                reached = spd
                if abs(spd - first) < 0.25:
                    break
            time.sleep(0.5)
        print(f"  measured {reached if reached is not None else float('nan'):.2f} RPS")
        if reached is None or abs(reached - first) > 0.5:
            print(f"!! did not reach {first:g} RPS (got {reached}) - aborting rather than "
                  f"writing a log that looks valid but is not")
            return 1

        ecu.cmd("AT+OSC=1", settle=0.0)
        ecu.s.reset_input_buffer()
        print(f"capturing {args.seconds:.1f} s -> {out}")

        n = 0
        end = time.time() + args.seconds
        # In ramp mode the setpoint step goes out a short way into the capture,
        # so the pre-transient baseline is recorded too.
        step_at = time.time() + 1.0 if args.ramp_from is not None else None
        with open(out, "w") as f:
            buf = b""
            while time.time() < end:
                if step_at is not None and time.time() >= step_at:
                    ecu.s.write(frame(f"AT+SPD={args.rps:.2f}"))
                    f.write(f"[{datetime.datetime.now().strftime('%H:%M:%S.%f')}] "
                            f"TX: AT+SPD={args.rps:.2f}\n")
                    step_at = None
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
        ecu.cmd("AT+SPD=0")
        print("ramping down ...")
        time.sleep(args.rps / 5.0 + 2.0)
        ecu.cmd(f"AT+DMODE={dmode_was}")
        ecu.cmd("AT+TM=0")
        print(f"restored dmode={dmode_was}")
        ecu.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
