# Current-frame ownership and interrupt-state hooks

For the full control pipeline, see
[Software and control algorithms](software.md#real-time-execution).

## Reusable control frame

[`MotorAlgorithmFrameT<Profile>`](../libecu/include/motor_samples.hpp) holds
one native input and output. `BldcController` owns persistent frame storage;
the serialized current ISR fills it, runs the selected algorithm and applies
PWM without returning a temporary output aggregate.

- `preparePwmInterruptHandler()` advances Hall/PLL once and snapshots commands
  and rotor state. It performs no ADC reads or PWM writes.
- `completePwmInterruptHandler()` consumes the preparation once, reads ready
  currents and bus voltage, runs FOC or six-step in place, then publishes the
  completed status under a critical section.
- `cancelPwmInterruptHandler()` discards pending work without another Hall/PLL
  tick. The tick already performed during preparation is not rolled back.
- `pwmInterruptHandler()` remains the prepare/complete convenience wrapper.

The port owns acquisition qualification and serializes prepare/complete/cancel.
Repeated preparation preserves the pending snapshot; completion without a
pending preparation does nothing. A live park or fault still prevents drive
output even when commands were captured earlier.

Each algorithm overwrites every output field. Six-step clears unused dq
fields, and parking clears electrical output without applying PWM. A PWM
callback sees the complete previous published status until the new frame is
finished. Value-returning `update()` / `updateNative()` overloads remain
available as compatibility adapters.

`tests/test_control/run.sh float` and `numeric-fixed` exercise publication
ordering, FOC/six-step transitions, parking, split cancellation and guards.

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
semantics. No target-specific interrupt implementation is part of this
header. Existing ports need no new hooks unless they enable the token option.
