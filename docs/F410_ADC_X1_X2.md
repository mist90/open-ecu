# F410: always-one-scan versus window-qualified ADC x2

Hardware A/B/A comparison, 2026-11-11, after moving dq-filter configuration
to the board and removing the second-order implementation.

## Result

Always-one-scan **x1 works correctly** and reduces the elapsed current-frame
window by approximately **5.4 us / 22%** where x2 can take paired samples.
The trade-off is higher measured high-frequency Iq error: approximately
**26-37% more at 1-6 RPS** in the forward sweep. At 8 RPS, where the x2 build
already falls back to one scan in approximately 98% of frames, the always-x1
build measured less HF Iq error: about **75 versus 85 mA RMS**. The latter
difference persisted when returning to x2.

Both images passed the speed/stop/reverse and current-step probes. No meaningful
difference in current-step t90 was resolved by this measurement. The existing
x2 default is retained for its lower low/mid-speed measurement noise and ample
20-kHz timing margin. x1 remains an explicit build option.

## Controlled conditions

- NUCLEO-F410RB + IHM08M1, unloaded 10-inch MOTOR_1, approximately 40.2 V.
- Release float firmware, 100-MHz core, 20-kHz PWM/current loop, 100-Hz speed loop.
- Board-owned first-order Id/Iq filter at 2000 Hz.
- Boot defaults `FPID=0.5498,516.7920`, `SPID=3.000,2.000`, current limits -6/+20 A.
- Same source and compiler flags; the only definition changed between images
  is `STM32F4_ADC_SOFTWARE_X2=1` versus `0`.
- Each image was flashed with verification, with the rotor stopped for offset
  calibration. Speed probes applied no runtime PI overrides.

**x1:** every PWM-triggered injected sequence measures U, V and W once, using
three sequential ADC ranks. F410 has one ADC, so these are not simultaneous
apertures. There is one completed current frame every 50 us.

**x2:** the same initial U/V/W sequence, plus a second sequence when the
low-side window is wide enough; the HAL publishes sums with the matching
calibration scale. At high duty it publishes a single-scan fallback. Control
still runs once per 50 us.

Both configurations use FOC's existing worst-phase reconstruction from the
two retained currents. All three physical currents are acquired and checked
for overcurrent; this comparison changes acquisition multiplicity.

## Runs and data

1. x2: forward 0.3 -> 1 -> 3 -> 6 -> 8 -> 3 -> 0 RPS; reverse 1 -> 3 -> 0 RPS.
2. x1: the identical speed sequence.
3. Return to x2: 1 -> 3 -> 8 -> 0 RPS, checking repeatability and restoring x2.

Each speed point settled for four seconds before 2.2 seconds of OSC/telemetry
and a 40000-frame non-halting SWD timing capture. In total: **960000 timing
frames**, no board faults, no overruns and no Hall faults. The x1 captures had
**zero paired frames** at every operating point. In x2, all measured frames
were paired through 6 RPS, versus about 2% at 8 RPS.

Analyzed 51 complete OSC blocks in the first x2 run, 50 in x1 and 20 in the
x2 repeat: **123904 raw samples**. Each block contains 1024 samples at 20 kHz;
UART gaps were not concatenated. Partial blocks at deliberate OSC stops were
excluded. Noise entries below are medians of per-block measurements.

### Current ripple and noise

Forward sweep, raw Iq tracking error before the dq feedback filter. HF RMS
uses a Hann-window spectrum from 1 kHz to the 10-kHz Nyquist frequency.

| Speed, RPS | Total Iq error standard deviation x2 / x1, mA | HF Iq error RMS x2 / x1, mA |
|---:|---:|---:|
| 0.3 | 72.2 / 88.2 | 72.5 / 81.3 |
| 1 | 64.9 / 90.0 | 59.8 / 82.1 |
| 3 | 85.9 / 101.3 | 59.7 / 81.7 |
| 6 | 127.6 / 132.8 | 64.4 / 80.9 |
| 8 | 159.5 / 150.0 | 84.5 / 74.8 |

The return-to-x2 HF values were **59.8, 59.2 and 84.8 mA** at 1, 3 and 8 RPS,
respectively. Reverse rotation also showed higher HF noise on x1: 79.2 versus
61.5 mA at 1 RPS and 83.7 versus 59.4 mA at 3 RPS.

At 8 RPS this is a comparison of always-x1 against predominantly-single-scan
x2 with occasional paired frames and its different acquisition code path.
The experiment does not isolate the cause of the remaining 8-RPS difference.
Electrical noise statistics are not calibrated sound-pressure measurements.

### Timing

| Speed, RPS | Mean current-frame window x2 / x1, us |
|---:|---:|
| 0.3 | 24.93 / 19.54 |
| 1 | 24.92 / 19.50 |
| 3 | 24.94 / 19.52 |
| 6 | 24.99 / 19.56 |
| 8 | 20.55 / 19.58 |

The maximum reported PWM-peak-to-control/OSC-completion time over the full
forward/reverse sweeps was **33.54 us for x2**, **27.80 us for x1**, within
the 50-us period. The x2 repeat's maximum was 33.07 us.

The current-frame window starts at the first JEOC and includes the second
conversion gap in x2; it is elapsed time, not pure CPU busy time. Peak-relative
times also include the first JEOC entry phase. The reported values stop before
timing-recorder bookkeeping and the ISR tail; the live overrun guard includes
the recorder work.

### Current steps

Fixed-field 0.4 -> 0.8 A and 0.8 -> 0.4 A, three repetitions per direction:

| Metric | x2 | x1 |
|---|---:|---:|
| Up-step t90 | 0.20 ms | 0.20 ms |
| Down-step t90 | 1.10 ms | 1.10 ms |
| Up-step overshoot | 19.6% | 19.9% |
| Down-step overshoot | 39.1% | 37.3% |

Medians after a centered five-sample offline average. Sub-0.25-ms estimates
are resolution-limited; small-current inverter nonlinearities remain. This
does not establish loaded torque-response equivalence.

## Builds and reproduction

From the repository root:

```sh
cmake -S STM32F410 -B /tmp/opencode/f410-x1 -DCMAKE_BUILD_TYPE=Release \
  -DNUMERIC_PROFILE=float -DENABLE_THROTTLE_BRAKE_CONTROL=OFF \
  -DENABLE_PWM_TIMING_BENCH=OFF -DENABLE_ADC_SOFTWARE_X2=OFF
cmake --build /tmp/opencode/f410-x1 --parallel
```

For x2, use a separate build directory and `-DENABLE_ADC_SOFTWARE_X2=ON`.

| Image | BIN bytes | SHA-256 |
|---|---:|---|
| x2 | 86716 | `72a4a03f3eed6eb1cf6c6d1fdcc8f073d661bf46ae8afd777d5bb23f5e7b7a39` |
| x1 | 86424 | `c27c29e333549c4e8dc15ef026f1f6f4798fb06f863491bf8b4f75ebe428dc77` |

Both report `1.2.0-f410-float`; use binary hashes to distinguish them.
Captures, build/flash logs and scripts are in
`/tmp/opencode/f410-adc-x1-compare-20261111/`: `01-x2.json`, `02-x1.json`,
`03-x2-repeat.json`, the first two runs' `-steps.json` captures,
`analyze_compare.py` and `comparison.json`.

The final installed image is the x2 image above, left in MODE=1, EMODE=1,
DMODE=2, SPD=0, with current and duty zero.
