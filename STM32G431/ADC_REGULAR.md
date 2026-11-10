# G431 ADC ownership and timing

- Injected ADC1 ranks are **U/U**, ADC2 ranks **V/W**. The extra U result is
  padding and is not consumed. Both converters retain two ranks, identical
  6.5-cycle sampling times and unshifted x2 hardware oversampling.
- [RM0440 Rev 7, p. 666](https://www.manualslib.com/manual/3725861/St-Stm32g474rct6.html?page=666)
  permits unequal simultaneous injected sequences only when trigger spacing
  exceeds the longer sequence. Equal durations allow one completion point for
  both ADCs. U/U padding preserves the established current timing and removes
  Vbus without introducing unequal-completion assumptions. Regular conversions
  are independent in injected dual mode and resume after injected conversion.
- ADC1 regular ranks are **Vbus/PA0, NTC/PB14, pot/PB12**. Independent ADC1 DR
  requests use DMA1 channel 1 through DMAMUX, with 92.5-cycle sampling at
  42.5 MHz. An uninterrupted scan takes approximately 7.41 us. Regular
  oversampling is disabled; all DMA values retain physical 12-bit resolution.
- SysTick stays at **1 kHz**, including HAL timeouts. It collects one complete
  prior DMA scan and launches the next; the control callback is divided to
  100 Hz. No polling, channel selection or ADC HAL start/stop runs in an ISR.
  All auxiliary getters return cached values. TIM2 has a true 1 MHz counter
  (`PSC=169` for the 170 MHz, APB1 DIV1 clock tree).
- Circular DMA holds three values; single-scan ADC mode prevents overwrite
  until SysTick launches again. SysTick copies into an inactive cache and
  publishes an atomic index. A preempting current IRQ reads a complete old or
  new frame without waiting.
- Timestamp = **scan launch**, not collection. Published acquisition age is
  approximately 1–2 ms, with an explicit **2500 us = 2 × 1 ms + 500 us** stale
  bound. This is the new Vbus cadence; current scaling and voltage/temperature
  thresholds are unchanged. The Vbus getter multiplies its cached raw value by
  the current sum scale to retain the physical-conversion contract.
- Startup requires a complete fresh auxiliary frame. Missing completion at the
  next scheduled SysTick, DMA error, incoherent count, still-running ADC or
  overrun latches acquisition failure. The PWM IRQ independently enforces the
  stale bound, and an acquisition fault permanently clears TIM1 MOE until reset.

Host checks: `bash tests/test_stm32_regular/run.sh`, both float and fixed.
These exercise the actual driver against fake HAL setup and anonymous MMIO,
including startup, asynchronous/partial DMA, preempting readers, scaling,
overrun/timeout faults and timer wrap. Hardware latency/ISR cycle measurements
require a separate board run.
