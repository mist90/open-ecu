#!/usr/bin/env python3
"""Minimal AT-link helper for the ECU, shared by the tuning scripts."""

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
    def __init__(self, port: str = "/dev/ttyACM0", timeout: float = 0.2):
        self.s = serial.Serial(port, BAUD, timeout=timeout)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def cmd(self, c: str, settle: float = 0.15) -> list[str]:
        self.s.write(frame(c))
        time.sleep(settle)
        raw = self.s.read(1 << 20).decode("ascii", errors="replace")
        return [l for l in raw.splitlines() if l.strip()]

    def query(self, c: str, prefix: str) -> str | None:
        for line in self.cmd(c):
            if line.startswith(prefix):
                return line[len(prefix):].strip()
        return None

    def close(self):
        self.s.close()
