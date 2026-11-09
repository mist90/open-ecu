# G431 independent ADC pipeline

## Rank and DMA layout

Internal OPAMP routing places U on ADC1 and V/W on ADC2. Both **independent**
regular ADCs use the same TIM1_TRGO2 rising edge:

| Mode | ADC1, DMA1 channel1 | ADC2, DMA1 channel2 |
|---|---|---|
| x2 (default) | U1, U2, Vbus, NTC, throttle | V1, W1, V2, W2 |
| x1 | U, Vbus, NTC, throttle | V, W |

U is VOPAMP1; V is VOPAMP2; W is VOPAMP3_ADC2. Auxiliary channels remain
PA0/IN1, PB14/IN5 and PB12/IN11. There are **no dummy ranks or PA6 dummy input**.
`ENABLE_ADC_X2` / `STM32G4_ADC_X2` retain the default x2 calibration scale.
Hardware oversampling, injected conversion and continuous conversion are off.

ADC1 circular DMA holds **two banks of 5/4 physical halfwords**. ADC2 normal DMA
holds **4/2 current halfwords** and stops at TC. Only ADC2 TC runs control.
ADC1 HT/TC flags are retained for provenance, but their interrupts are disabled;
ADC1 DMA transfer error and both ADC overrun interrupts remain enabled/fatal.

Independent mode is intentional: dual CDR's joint-rank completion would make
V2/W2 wait for ADC1's long auxiliary ranks. The bundled `HAL_ADC_Start_DMA()`
explicitly supports each ADC in independent mode, enables its own DR DMA and
arms ADSTART for the configured external trigger. TIM1 stays stopped until both
ADCs and DMAs are ready. No runtime sequence changes or software conversion
starts are needed.

## At each current interrupt

ADC2 TC occurs after all V/W conversions. ADC1 has already stored current U1/U2
and continues the auxiliary tail while the CPU executes control. The handler
copies **current-bank U**, stopped ADC2 V/W, and **previous-bank auxiliaries**.
Vbus is checked every control tick using that previous complete value, normally
about one carrier old. A current frame never waits for its own auxiliary tail.

DMA keeps running even when CPU interrupts are masked. Therefore the copy is
protected by acquisition state, not by assuming a PRIMASK lock freezes DMA:

- The expected PWM source timestamp and current frame parity identify the bank.
- ADC2 requires owned TC, zero remaining transfers and EOS, without either DMA
  transfer error or either ADC overrun.
- ADC1 requires circular+enabled DMA, one-shot ADC mode, at least all current U
  ranks written in the expected bank, and the **previous bank's HT/TC flag**.
  The opposite flag is allowed only when the current bank is already complete.
  NDTR alone is insufficient. Initial startup requires no previous-bank flag.
- Progress/status are sampled again **after copying**, alongside a fresh source
  age. Backwards progress, inconsistent flags, partial/missing U, traversed-bank
  states or crossing into another PWM epoch fail before publication. A single
  bounded resnapshot handles a boundary between status/count reads; no spin.
- Only the **consumed previous boundary flag** is cleared. A current-tail boundary
  arriving during the copy remains available to certify the next frame.

Normal acquisition has exactly one sequence per configured PWM edge. The
independent current generation/deadline check rejects an ISR delayed across a
carrier, while the bank flags reject apparent pointer positions left by missing
or inconsistent ADC1 progress. It is not an unbounded circular-buffer consumer.

ADC2 DMA is rearmed after control and capture. Full acquisition/control must
finish **before source+47 us** at nominal 20 kHz, leaving 3 us to the next trigger.
Prediction is refreshed from TIM1 each frame, avoiding drift from ARR=4249.
Fault handling disables both DMAs, requests asynchronous stop of both ADCs and
clears MOE. No HAL stop/poll/calibration or wait runs in an ISR.

## Epochs, scaling and startup

The first current IRQ records valid currents but has no previous auxiliary
bank. It returns pending readiness, skips control, and keeps both ADCs running.
The next IRQ consumes the first complete auxiliary bank. Zero-initialized
Vbus/NTC codes are never dated fresh. `have_frame_` is separate from the wrapping
sample counter, so counter wrap is not mistaken for startup.

The raw current interface still returns U/V/W sums with each phase's own zero;
odd sums retain half-LSB information. Vbus returns one previous physical code
times the same sum factor. The auxiliary timestamp is the **previous PWM source**,
not current DMA entry or SysTick collection. Current and auxiliary freshness are
checked separately (105 us at 20 kHz).

The 1 kHz SysTick makes a bounded, nested-PRIMASK raw cache snapshot; conversion
and thermal evaluation occur outside the lock. It never launches ADC and has
explicit pending/error/ready results. Missing frame progress fails at the next
tick; the current IRQ also retains the independent 2500 us slow-service watchdog.
G431's existing configured voltage/current/thermal behavior remains; no 25 A
raw threshold or hardware BKIN protection is invented (`TIM_BREAK_DISABLE`
remains the existing setting).

## Calculated timing and qualification

At 42.5 MHz, phase sampling remains **6.5 clocks** plus 12.5 conversion clocks.
ADC2 current TC is nominally **1.79 us x2 / 0.89 us x1**, excluding synchronization
and interrupt latency. ADC1's full bank is **8.31 us x2 / 7.86 us x1**, with the
existing **92.5-clock auxiliary sampling** unchanged.

U2 now occupies ADC1 rank2 rather than the old rank3. Relative sample starts are
U1/V1 at zero, U2/W1 at 19 clocks, V2 at 38 and W2 at 57 clocks. Thus the two U
samples have 0.447 us spacing, while V and W each have 0.894 us spacing. This
aperture skew and the shorter control delay require physical closed-loop
qualification; the arithmetic/sum scale is unchanged.

`BENCH_PROFILING` now measures ADC2 **current-only** DMA IRQ entry. Historical
timer/packed-full-scan measurements are not updated or reinterpreted. Host tests
in `tests/test_stm32_regular` exercise actual driver code with independently
scheduled rank writes, tail completion during copying, bank flags, overwritten
epochs, startup pending, source age, error paths and slow-cache preemption.
