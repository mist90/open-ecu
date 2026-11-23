# Current-frame ownership and interrupt-state hooks

The STM32 ports call `BldcController::pwmInterruptHandler()` once per period.
The [controller interface](../libecu/include/bldc_controller.hpp) also exposes
two paths for other interrupt schedules. They differ in when a reader may
inspect status. See [Software and control algorithms](software.md#real-time-execution)
for the full pipeline.

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
  `pwmInterruptHandler(adc, pwm)` and `completePwmInterruptHandler(adc, pwm)`
  accept the same concrete drivers that the board bound to the controller.
  They can avoid virtual calls for ADC reads and FOC PWM writes; the frame
  ownership and publication order are the same.

The port owns acquisition qualification and serializes prepare/complete/cancel.
Repeated preparation preserves the pending snapshot; completion without a
pending preparation does nothing. A live park or fault still prevents drive
output even when commands were captured earlier.

Hall edges only mark a pending reading. Neither those IRQs nor SysTick advance
the PLL if the carrier stops. When the carrier keeps running in `NEUTRAL`, it
continues to track the physical wheel. Tracking runs with interrupts enabled:
a higher-priority hardware break can cancel a frame without changing PLL state.

Each algorithm overwrites the frame's output. Six-step clears unused dq
fields, and parking clears electrical output without applying PWM. During
the PWM write, current, dq and duty fields still describe the previous frame.
Value-returning `update()` / `updateNative()` overloads remain available as
compatibility adapters on the STM32 builds.

## Direct status writes in a single PWM interrupt

Use `prepareOwnedCarrier()` and `completeOwnedCarrier(adc, pwm)` **in the
same single-writer IRQ**. If the board already sampled the Hall pins, pass
the decoded sector (0–5, or `0xff` for an illegal code) to the preparation.
Omitting it uses the usual `HallInterface` read when a Hall sample is due.

This path reads commands from controller status and writes the new electrical
results directly there. There is no atomic copy of the finished frame: the
PWM callback must not inspect status mid-update. Lower-priority readers must
mask the carrier IRQ while copying a snapshot; `getNativeStatus()` returns
a reference and supplies no lock. A higher-priority hardware break may latch
the outputs off, but must not read or change motor status, rotor state or
commands, or enable gates. `parkFromCarrier()` is for the carrier owner.
`writeCarrierDuty()` and `applyCarrierStates()` must never re-enable a
bridge stopped by the break. Use the regular split interface above if
other contexts need to run between preparation and completion.

The [controller host tests](../tests/test_control/main.cpp) compare the
single-call, split and single-writer results in both algorithms. They also
exercise publication order on the regular path, transitions, cancellation
and parking (`tests/test_control/run.sh float` or `numeric-fixed`).

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
