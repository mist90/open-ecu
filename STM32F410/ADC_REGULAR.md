# F410 ADC ownership and timing

- ADC1 injected ranks contain **U, V, W only**. TIM1 triggers one sequential
  12-bit scan per 20-kHz PWM period. Its JEOC IRQ captures all three values,
  checks raw-current protection and runs one controller update.
  `captureInjected()` returns `void`; `INJECTED_SAMPLE_SUM` is always **1**.
  There is no second software scan, pairing state or fallback.
- ADC1 regular ranks are **Vbus/PA1, NTC/PC2, pot/PA4**, drained by
  DMA2 stream 0/channel 0. Sampling is 144 ADC cycles at 25 MHz; an uninterrupted
  three-rank scan takes about 18.72 us. Injected conversions retain priority.
- Every 1 kHz SysTick collects the previous complete DMA scan and requests one
  new scan. Neither getters nor the PWM/ADC IRQ starts or reconfigures regular
  acquisition. The 100 Hz control callback reads the cache.
- Circular DMA reloads three entries but the ADC is **not continuous**. SysTick
  copies the completed, idle DMA buffer into an inactive cache, then publishes
  one atomic index. Higher-priority readers cannot see partially copied fields
  and never spin waiting for SysTick. The CR2 software-start RMW is serialized
  without waiting for conversion completion.
- The timestamp is taken at **launch**, including zero and timer wrap. Normal
  published acquisition age is approximately 1–2 ms. The explicit stale bound is
  **2500 us = two 1 ms ticks + 500 us scheduling allowance**. This replaces the
  PWM-rate Vbus acquisition cadence; voltage thresholds are unchanged.
- Missing DMA completion at the next scheduled SysTick, DMA errors/incoherent
  count or ADC overrun fail closed. The current IRQ also checks age independently
  of SysTick. Startup requires a complete fresh auxiliary frame; faults latch
  the bridge off. NTC raw-rail and existing thermal/voltage protections remain.
- Injected current values, calibration offsets and regular DMA values all use
  physical single-conversion 12-bit counts (`0..4095`); Vbus needs no sum scaling.

Host checks: `bash tests/test_stm32_regular/run.sh` (both numerical profiles),
plus `bash tests/test_f410_adc/run.sh`. The first compiles the
actual driver against fake HAL setup/anonymous MMIO and injects preempting cache
reads around the real atomic publication. These checks do not measure hardware
conversion latency or ISR cycle counts.
