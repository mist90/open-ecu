# F410 ADC ownership and timing

- ADC1 injected ranks contain **U, V, W only**. TIM1 still triggers the first
  scan; the window-qualified software x2 scan, sum scaling, fallback,
  per-scan raw-current protection and loop deadlines remain in the injected IRQ.
- ADC1 regular ranks are **Vbus/PA1, NTC/PC2, pot/PA4**, drained by
  DMA2 stream 0/channel 0. Sampling is 144 ADC cycles at 25 MHz; an uninterrupted
  three-rank scan takes about 18.72 us. Injected conversions retain priority.
- Every 1 kHz SysTick collects the previous complete DMA scan and requests one
  new scan. Neither getters nor the PWM/ADC IRQ starts or reconfigures regular
  acquisition. The 100 Hz control callback reads the cache.
- Circular DMA reloads three entries but the ADC is **not continuous**. SysTick
  copies the completed, idle DMA buffer into an inactive cache, then publishes
  one atomic index. Higher-priority readers cannot see partially copied fields
  and never spin waiting for SysTick. The only acquisition critical section is
  the CR2 software-start RMW, shared with the injected x2 trigger state.
- The timestamp is taken at **launch**, including zero and timer wrap. Normal
  published acquisition age is approximately 1–2 ms. The explicit stale bound is
  **2500 us = two 1 ms ticks + 500 us scheduling allowance**. This replaces the
  PWM-rate Vbus acquisition cadence; voltage thresholds are unchanged.
- Missing DMA completion at the next scheduled SysTick, DMA errors/incoherent
  count or ADC overrun fail closed. The current IRQ also checks age independently
  of SysTick. Startup requires a complete fresh auxiliary frame; faults latch
  the bridge off. NTC raw-rail and existing thermal/voltage protections remain.
- DMA samples stay physical 12-bit values. Only the public Vbus getter multiplies
  by `INJECTED_SAMPLE_SUM`, preserving the shared physical-conversion contract.

Host checks: `bash tests/test_stm32_regular/run.sh` (both numerical profiles,
F410 x2 on/off), plus `bash tests/test_f410_adc_x2/run.sh`. The first compiles the
actual driver against fake HAL setup/anonymous MMIO and injects preempting cache
reads around the real atomic publication. These checks do not measure hardware
conversion latency or ISR cycle counts.
