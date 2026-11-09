# F410 current DMA and pipelined injected auxiliaries

## Acquisition

One PWM event starts **one regular current scan**:

| Group | Sequence | Sampling @25 MHz |
|---|---|---|
| Regular x2 | U/PA0/IN0, V/PC1/IN11, W/PC0/IN10, U, V, W | 15 clocks each |
| Regular x1 | U, V, W | 15 clocks each |
| Injected auxiliaries | Vbus/PA1/IN1, NTC/PC2/IN12, throttle/PA4/IN4 | 144 clocks each |

`ENABLE_ADC_SOFTWARE_X2` / `STM32F4_ADC_SOFTWARE_X2` keep their compatibility
names and **ON** default. They select repeated regular ranks. **There is no CPU
software start of a second current triple**, no SQR switching, and no x1 fallback.

F410 regular EXTSEL cannot select TIM1_TRGO/CH4, while TIM1_CH1/2/3 drive the
motor. The existing hardware bridge remains:

`TIM1_CC4 -> DMA2 stream4/channel6 -> one immutable ADC_CR2.SWSTART command`.

[RM0401 Rev3 table30](https://www.manualslib.com/manual/1551559/St-Stm32f410.html?page=167)
specifies the route. CMS=2 and CCDS=0 generate one up-counting CC4 request per
carrier, ARR−100 (**1 us before peak**, 100 MHz). CH4 is internal; PA11 remains
unconfigured as an output. Both trigger DMA and result DMA2 stream0/channel0
are normal one-shot transfers. Only result DMA TC performs normal control work;
HT and injected JEOC interrupts are disabled.

At each current TC:

1. Validate the current source, both DMA completions/counts, timing and errors;
   copy all stopped current DMA samples.
2. Except for startup, require the **previous owned** injected JEOC and matching
   generation; copy JDR1=Vbus, JDR2=NTC, JDR3=throttle with its original launch
   timestamp. Missing/old/unowned completion fails closed.
3. Clear JEOC/JSTRT and start the next injected auxiliary sequence **before FOC**.
   The CPU writes a saved CR2 command containing JSWSTART and no SWSTART; it never
   RMW-replays a start bit. Both saved commands are built after both groups are
   configured. JAUTO, JDISCEN and injected external triggers remain disabled.
4. Check every original current against the established phase-specific zero and
   **25 A** cutout, then execute one control step using current phase sums and
   the **previous complete auxiliary Vbus**. Rearm current/trigger DMA after FOC.

HAL's `ADC_JSQR` macro right-aligns a three-rank sequence into **JSQ2/3/4**;
the results are read from **JDR1/2/3**. See RM0401 §11.3.9 and §11.12.12, and the
bundled `HAL_ADCEx_InjectedConfigChannel()` implementation.

## Admission and deadlines

Current conversions take **3.24 us x1 / 6.48 us x2**; auxiliaries retain their
**18.72 us** sequence. The long auxiliary conversion now overlaps CPU control.
These are clock calculations, not new hardware measurements.

Auxiliary launch must occur before source + `(period − 21 − 3) us`; at 20 kHz
this is **strictly before 26 us**. The 21 us budget includes 18.72 us conversion
plus ADC/start synchronization allowance. Unsupported rates without room for
currents + auxiliaries + margin are rejected at initialization. Rearm rejects
an observed incomplete auxiliary older than 21 us. The whole current IRQ still
must finish **before source+47 us**, including bookkeeping, leaving 3 us before
the next trigger. It receives no new full-period allowance at DMA entry.

**Both x1 and x2 qualify the consumed current apertures.** The PWM writer tracks
active shadows and queued preloads separately. Every admitted leg retains the
dead-time+2 us settling requirement at its own first aperture. U/V/W first
apertures start0/1.08/2.16 us after the nominal source.

Before each UDIS-gated write, the writer observes UIF to account for an actual
shadow reload, then records the new preload generation and clears only UIF.
At rearm, the carrier position determines whether an extremum occurs before the
next source. The saved opening vector is the generation that will be active
then; closing belongs to the preload loaded at the peak. This avoids combining
an old-high W opening with a new-high U opening that never coexisted.
Unchanged generation and matching live preloads qualify that exact window.
Intervening tracked writes, including ABA, retain conservative per-phase maxima;
unsupported CCR writes invalidate tracking. Nonzero RCR or UDIS fails
qualification. This is PWM-state tracking, not ADC elapsed-time measurement.

Current acquisition timing is fixed from **RM0401 §11.5**:
`Tconv = sampling time +12 ADC clocks`, giving **324 timer ticks x1 /648 x2**
at ADC25 MHz and TIM1100 MHz. By the explicitly chosen port contract, the
hardware trigger-DMA/start delay is negligible and admitted auxiliaries do not
overlap the next current scan. The former102-tick reserve and per-frame
timer/IRQ completion calculation are removed from aperture qualification.
To obtain each phase's last aperture closing time, subtract
only work which must follow it: **264 ticks for U, 156 for V, 48 for W** (the
remaining ranks and the sample's own 12-clock SAR conversion). In x2 these are
the final U/V/W ranks; the first triple closes earlier. The final U/V/W aperture ends are
**0.60/1.68/2.76 us x1** and **3.84/4.92/6.00 us x2** from the nominal source.
CPU/IRQ scheduling does not move these fixed apertures. Frame age, DMA flags,
auxiliary launch admission and the47-us source-relative control deadline are
still checked independently.

x1 accepts all three qualified legs. It can additionally accept **exactly one**
invalid leg only when that leg is both the controller's **exact next FOC
reconstruction choice** and the **unique quantized live CCR maximum**. The board
uses `currentReconstructedPhase()`, which reads FOC's private previous-duty
choice including reset/tie behavior; CCR/status alone cannot infer it. Quantized
highest ties, a mismatched reconstruction choice, or two invalid legs fail
closed. Thus a 95% highest leg can still be reconstructed when both consumed
legs are proven valid. Non-FOC consumers require all three qualified apertures.

x2 qualifies **each physical UVW triple separately** using that rank's fixed
sampling start/end. Each triple needs two valid measured phases. Its single
invalid phase is reconstructed from those two, and only then are the two complete
current vectors averaged. The discarded phase may differ between triples: e.g.
V1 can lose its opening window while W2 loses its closing window. Two invalid
phases in the **same** triple are rejected; non-FOC consumers require all six
measurements to be valid.

When both triples discard the same phase, linear averaging and KCL commute:
the fast path uses the original calibrated phase sums and one exact FOC hint.
Different discarded phases use the wide per-triple reconstruction path. Both
scan masks are calculated from one coherent PWM-state snapshot.

Reconstruction uses calibrated native currents. Original `raw_`, DMA storage,
raw phase sums and raw-protection getters are never replaced with synthesized
ADC codes. A physical sample is converted at the existing x2 calibration scale
by multiplying its raw code by two before conversion. The FOC one-frame choice
is then applied to the already consistent averaged vector and expires after
that update. Capture resets the per-triple choices for every newly owned frame.
Every original sample, including the reconstructed phase, still reaches the
existing raw-current protection. No modulation setting, current threshold,
gain or deadline is changed.
The board applies live-aperture qualification only to bridge-enabled, ready
feedback; GPIO/bridge-off calibration and auxiliary priming do not falsely trip.

Configured timing and strict launch admission separate the auxiliary and current
groups in normal operation. Fixed aperture geometry relies on that contract;
eventual JEOC/TC is not a hardware timestamp proving absence of a fault-induced
shift. Error, generation, source-age and missing-frame watchdogs remain separate
from the nominal ADC timing. There is no periodic checkpoint IRQ or ISR wait.

## Publication and startup

The first current IRQ primes auxiliaries and reports **pending**, without using
zero-initialized Vbus or running control. The second consumes the first completed
auxiliary scan. Initialization/calibration requires a fresh ready pipeline.

SysTick remains 1 kHz, snapshots the newest **complete cached** NTC/throttle pair
under a bounded PRIMASK-preserving lock, and performs thermal calculations after
unlocking. It never starts ADC. `serviceRegular()` returns −1 on error, 0 while
the first auxiliary is pending, and 1 after a snapshot. Current and auxiliary
source ages are separate; neither is relabeled at collection. The live bound
is 105 us at 20 kHz, with the existing independent 2500 us slow-service bound.
Missing progress fails at the next SysTick; DMA/ADC errors latch acquisition off.
Hardware break and existing voltage/thermal protections remain.

## Rotor observation after an acquisition fault

A latched ADC failure or loop overrun disables both acquisition DMAs and parks
the bridge. The port then enables the TIM1 CC4 interrupt at the same priority
as the former current DMA owner. The existing up-count-only compare produces
one observation tick per50 us, without restarting ADC or invoking control.
Deferred Hall processing and the PLL therefore continue through coast-down;
UART speed no longer republishes a frozen pre-fault estimate.

The observation handler enforces NEUTRAL before calling the controller's
observation-only API. A pending old DMA vector cannot clock the PLL a second
time, and acquisition ownership returns only after reset. The other ADC values
remain retained fault data. `f410_rotor_observation_ticks` exposes observation
progress for parked diagnostic checks; it is zero during healthy ADC operation.
`f410_current_window_fault` retains both rejected scan masks, their union,
automatic FOC choice and live preloads before NEUTRAL overwrites them.
`f410_current_reconstructions` counts corrected physical triples by discarded phase.

## Hardware follow-up, 2026-11-09

The final float x2 image passed a UART-only high-range sweep through demands
9/10/10.5/11/11.5/12 RPS, with3 s settling plus10 s observation per point:
78.129 s total, no board faults, overruns, Hall faults or Hall overcaptures.
At demand12 RPS, measured speed was11.1664 RPS (about670 RPM), PWM reached its
95% limit, and bus voltage averaged40.392 V. Thus the full maximum-demand window
passed, but actual12-RPS speed was not reached at this supply/modulation limit.

The phase-correction counters reached452373/427079/478399 physical triples.
Earlier protected runs verified live post-ADC-fault coast to zero in about1.2 s,
and the independent20-kHz observer clock was checked against TIM5. Final state:
NEUTRAL, zero demand/speed/current/duty, acquisition healthy and faults/overruns0.
Installed BIN SHA256:
`b884ad63a8d726905be24879b517b877d309a8ab66fc22dd0be35c99688ee00c`.
Evidence: `/home/ropewalker/ecu-bench/f410-x2-reconstruction-coast-20261109/`.

Raw phase getters retain calibrated x1/x2 sums; Vbus multiplies one previous
physical code by the same scale. NTC/throttle remain physical 12-bit codes.
Timing capture now measures **current-only DMA entry**; its 60-byte ABI and
mid-IRQ arming gate are retained. Historical full-scan/JEOC measurements do not
describe this pipeline and are not rewritten.

Tests: `bash tests/test_stm32_regular/run.sh` and
`bash tests/test_f410_adc_x2/run.sh`. Physical qualification still covers trigger
latency, injected/regular interaction, the valid shunt window and closed-loop
sampling-to-actuation delay. Host/ARM checks do not substitute hardware timing.
