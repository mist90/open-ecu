#!/usr/bin/env python3
"""Profile the F410 ADC/current ISR with a bridge-disabled bench image, then restore firmware."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'utility'))
from ecu import frame
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--bench-elf', type=Path, required=True)
parser.add_argument('--restore-elf', type=Path, required=True)
parser.add_argument('--port', default='/dev/ttyACM0')
parser.add_argument('--stlink-serial')
parser.add_argument('--output', type=Path, default=Path('/tmp/opencode/f410-pwm-timing.json'))
args = parser.parse_args()
ELF = str(args.bench_elf.resolve())
RESTORE = str(args.restore_elf.resolve())
assert Path(ELF).is_file() and Path(RESTORE).is_file(), 'Both firmware files must exist'
args.output.parent.mkdir(parents=True, exist_ok=True)
BASE = ['openocd', '-f', 'interface/stlink.cfg', '-f', 'target/stm32f4x.cfg']
if args.stlink_serial:
    assert args.stlink_serial.isalnum(), 'ST-LINK serial must be alphanumeric'
    BASE += ['-c', f'hla_serial {args.stlink_serial}']
symbols = {}
for line in subprocess.check_output(['arm-none-eabi-nm', ELF], text=True).splitlines():
    fields = line.split()
    if len(fields) == 3 and fields[2].startswith('f410_'):
        symbols[fields[2]] = int(fields[0], 16)
assert 'f410_bench_inputs' in symbols, 'Only the bridge-disabled timing image is accepted'
restore_symbols = subprocess.check_output(['arm-none-eabi-nm', RESTORE], text=True)
assert ' f410_bench_inputs' not in restore_symbols, 'Restore image must be a normal firmware build'

def flash(path):
    assert '{' not in path and '}' not in path, 'Unsupported brace in firmware path'
    p = subprocess.run(BASE + ['-c', f'program {{{path}}} verify reset exit'],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=40)
    print(p.stdout, flush=True)
    p.check_returncode()

def collect(port, seconds):
    result = b''
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        result += port.read(4096)
    return result.decode('ascii', errors='replace')

def command(port, cmd):
    port.write(frame(cmd))
    port.flush()
    reply = collect(port, .12)
    if 'ERROR' in reply:
        raise RuntimeError(cmd + ': ' + reply)
    return reply

server = None
connection = None
results = []
logfile = None
with serial.Serial(args.port, 115200, timeout=.03) as uart:
    try:
        flash(ELF)
        boot = collect(uart, .4)
        args.output.with_suffix('.bench-boot.log').write_text(boot)
        assert 'TIMING BENCH' in boot and 'ECU started' in boot, boot
        command(uart, 'AT+TM=0')
        command(uart, 'AT+SPD=0')
        command(uart, 'AT+DMODE=2')
        logfile = args.output.with_suffix('.openocd.log').open('w')
        server = subprocess.Popen(BASE, stdout=logfile, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 5
        while True:
            try:
                connection = socket.create_connection(('127.0.0.1', 6666), timeout=.2)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.05)
        connection.settimeout(5)

        def tcl(text):
            connection.sendall(text.encode() + b'\x1a')
            response = b''
            while not response.endswith(b'\x1a'):
                chunk = connection.recv(65536)
                if not chunk:
                    raise RuntimeError('OpenOCD disconnected')
                response += chunk
            return response[:-1].decode()

        def words(address, count):
            result = tcl(f'mem2array values 32 {address} {count}; list ' +
                         ' '.join(f'$values({i})' for i in range(count)))
            return [int(value, 0) for value in result.split()]

        def write_words(address, values):
            tcl('; '.join(f'mww {address + 4*i} {value}' for i, value in enumerate(values)))

        def capture(label, algo=1, current=0, phases=(2122, 2122, 2122), active=True, osc=False, telemetry=False):
            for cmd in ['AT+OSC=0', 'AT+TM=0', 'AT+DMODE=2', f'AT+ALGO={algo}',
                        'AT+MODE=2', 'AT+EMODE=1']:
                command(uart, cmd)
            write_words(symbols['f410_bench_inputs'], phases)
            if active:
                command(uart, 'AT+DMODE=0')
                command(uart, f'AT+CUR={current}')
            assert f'+DMODE:{0 if active else 2}' in command(uart, 'AT+DMODE?')
            target_reply = command(uart, 'AT+CUR?')
            assert f'+CUR:{current if active else 0:.2f}' in target_reply, target_reply
            status = command(uart, 'AT+STATUS?')
            status_line = next(line for line in status.splitlines() if line.startswith('+STATUS:'))
            fields = status_line.partition(':')[2].split(',')
            assert fields[:2] == ['2', '1'] and int(fields[6]) == algo, status_line
            if telemetry:
                command(uart, 'AT+TM=1')
            initial = words(symbols['f410_diagnostics'], 4)
            assert initial[3] == 0, ('board fault before capture', initial)
            assert words(0x40010044, 1)[0] & 0x8000 == 0, 'MOE must be clear'
            address = symbols['f410_timing_capture']
            write_words(address, [0, 0, 0, 0, 0, 0])
            write_words(address, [40000])
            if osc:
                command(uart, 'AT+OSC=1')
            output = collect(uart, 2.3)
            remaining, samples, total, maximum, minimum, overruns = words(address, 6)
            diagnostics = words(symbols['f410_diagnostics'], 4)
            bdtr = words(0x40010044, 1)[0]
            result = dict(case=label, samples=samples, remaining=remaining,
                          mean_us=total/samples/100 if samples else None,
                          max_us=maximum/100, min_us=minimum/100, overruns=overruns,
                          faults=diagnostics[3], moe=bool(bdtr & 0x8000),
                          status=status.strip(),
                          osc_lines=output.count('+OSC:'), telemetry_lines=output.count('+TM:'))
            print(json.dumps(result), flush=True)
            results.append(result)
            assert remaining == 0 and samples == 40000, result
            assert overruns == 0 and diagnostics[3] == 0 and bdtr & 0x8000 == 0, result
            command(uart, 'AT+OSC=0')
            command(uart, 'AT+TM=0')
            command(uart, 'AT+DMODE=2')
            command(uart, 'AT+SPD=0')

        capture('neutral', active=False)
        capture('FOC zero current', current=0)
        capture('FOC positive saturation', current=6)
        capture('FOC negative saturation', current=-6)
        capture('FOC nonzero phase currents + OSC + telemetry', current=6,
                phases=(2636, 1865, 1865), osc=True, telemetry=True)
        capture('six-step current + OSC + telemetry', algo=0, current=6, osc=True, telemetry=True)
    finally:
        try:
            for cmd in ['AT+OSC=0', 'AT+TM=0', 'AT+SPD=0', 'AT+DMODE=2']:
                command(uart, cmd)
        finally:
            if connection:
                try:
                    connection.sendall(b'shutdown\x1a')
                except OSError:
                    pass
                connection.close()
            if server:
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    server.terminate()
                    server.wait(timeout=5)
            if logfile:
                logfile.close()
            args.output.write_text(json.dumps(results, indent=2))
            flash(RESTORE)
            boot = collect(uart, .4)
            args.output.with_suffix('.normal-boot.log').write_text(boot)
            print('RESTORED:', boot[:400], flush=True)
            for cmd in ['AT+TM=0', 'AT+SPD=0', 'AT+DMODE=2', 'AT+DMODE?', 'AT+SPD?', 'AT+VER?']:
                print(cmd, repr(command(uart, cmd)), flush=True)
