# Current-frame ownership and interrupt-state hooks

## One control call per PWM period

The [controller](../libecu/include/bldc_controller.hpp) consumes ready phase
currents and cached bus voltage in one `pwmInterruptHandler()` call. The two
STM32 ports run this tick at **20 kHz (50 µs)**, but enter it differently:

- **STM32F410:** [`ADC_IRQHandler`](../STM32F410/Core/Src/stm32f4xx_it.c)
  calls [`board_adc_interrupt()`](../STM32F410/Core/Src/board.cpp). JEOC
  marks the end of one timer-triggered 12-bit U/V/W scan. The port captures
  all three ranks and checks raw overcurrent and auxiliary freshness before
  the control call. At 100 Hz it checks that injected samples are still arriving.
- **STM32G431:** [`HAL_TIM_PeriodElapsedCallback`](../STM32G431/Core/Src/stm32g4xx_it.c)
  calls [`motor_controller_pwm_interrupt_handler()`](../STM32G431/Core/Src/board.cpp)
  on the **down-counting** TIM1 update only. Both timer extrema raise update
  events; only one runs control. TIM1 also triggers dual injected ADCs with
  hardware ×2 oversampling. The callback does not check JEOS.

Both ports service regular auxiliary DMA from the 1-kHz SysTick. It does not
poll injected currents or run a rotor tick. Hall input capture (TIM5 on F410,
TIM4 on G431) marks a pending reading. The current interrupt observes Hall
and advances the PLL once, including while parked if acquisition continues.
Open loop advances its own phase without replacing the rotor feedback.

The [current handler](../libecu/include/bldc_pwm_frame.hpp) reads commands
directly from `MotorIo::in` and updates `out.electrical` in place. Its
`pwmInterruptHandler(adc, pwm)` overload takes statically bound drivers;
a port that already sampled Hall may pass the decoded sector (0–5, or `0xff`
for invalid) as a third argument. It runs the same single tick. Do not
inspect live results from a PWM write callback or retain a
reference as a snapshot. See [controller I/O](CONTROL_IO.md) for reader rules.
The value-returning `update()` and `updateNative()` algorithm adapters still
serve standalone callers; the board's current tick does not copy their results.

On F410 a higher-priority break latches the bridge off and calls
`cancelPwmTick()`. The handler can return `false` even after some computation
or preload writes: cancellation is not a rollback, and PWM writes must not
re-enable a tripped bridge. The port only captures a requested scope sample
when the handler returns `true`.

The [controller host tests](../tests/test_control/main.cpp) cover the shared
tick, command transitions, both electrical algorithms, parking and capture.
They do not measure either port's interrupt latency. For timing and
acquisition requirements see the [porting guide](PORTING_GUIDE.md#current-acquisition).

## Optional interrupt-state tokens

By default, [`CriticalSection`](../libecu/include/critical_section.hpp) calls
the existing `disable_interrupts()` / `enable_interrupts()` hooks.
A port can define `LIBECU_CRITICAL_SECTION_TOKENS=1` consistently across its
translation units and implement:

```cpp
extern "C" uint32_t save_interrupt_state();
extern "C" void restore_interrupt_state(uint32_t state);
```

`save_interrupt_state()` saves the entry interrupt mask and enters the
critical section. `restore_interrupt_state()` restores that exact mask,
including an already-masked entry state. Each RAII guard stores its own token
and cannot be copied or moved.

Token guards and legacy disable/enable pairs must be released in reverse
acquisition order; the port supplies the matching nesting and memory-ordering
semantics. The [G431 build](../STM32G431/CMakeLists.txt) enables per-guard
tokens; [F410](../STM32F410/Core/Src/board.cpp) uses the nesting-safe pair.
