# Hybrid Hall + BEMF Sensorless Rotor Position Estimation

This document describes the hybrid Hall sensor and back-EMF (BEMF) zero-crossing rotor position estimator implemented in open-ecu. The algorithm extends the existing 6-step trapezoidal commutation with a `MotorPLL` so the controller can keep commutating above the speed where Hall sensors become unreliable, without changing the PLL or the commutation controller.

## 1. Overview

open-ecu drives a BLDC motor with 6-step trapezoidal commutation. Rotor position is fed to a `MotorPLL` that integrates angle at the PWM rate and produces the next commutation step. Position events reach the PLL through a single entry point, `MotorPLL::updateHall(uint8_t step)`.

Two independent sources produce those events:

- **Hall sensors** (PB6/PB7/PB8 on the b-g431b-esc1). A GPIO edge interrupt decodes the Hall state into a step 0-5 and calls `updateHall()`. This is the only source used at low speed.
- **BEMF zero-crossing observer** (`BemfObserver`). At high speed the floating phase voltage is sampled through the ADC and watched for a zero-crossing. After a 30-degree delay the observer emits a synthetic step that is fed into the same `updateHall()` call.

The PLL never distinguishes between real Hall edges and synthetic BEMF events. Both look like a step advance on the same input. This keeps the PLL, the commutation controller, and the safety monitor unaware of which sensor is active.

`BemfObserver` is a standalone, platform-independent class in `libecu`. It has no STM32 dependencies and can be unit-tested on the host. All hardware specifics live in the HAL and the application layer.

## 2. Hardware Setup

### Phase Voltage Sensing

Each phase voltage is brought out to a dedicated ADC input through a resistor divider. The divider ratio is set by an external resistor network and is configurable through `BemfVoltageSensorParameters`.

| Phase | GPIO   | ADC Instance | ADC Channel | Injected Rank |
|-------|--------|--------------|-------------|---------------|
| U     | PA4    | ADC2         | IN17        | RANK3         |
| V     | PC4    | ADC2         | IN5         | RANK4         |
| W     | PB11   | ADC1         | IN14        | RANK3         |

### Resistor Divider

The BEMF front end is a 10 kOhm / 2.2 kOhm divider per phase, giving a ratio of about 5.55x. The ADC pin sits at the 2.2 kOhm node to ground. Parameters are stored in `BemfVoltageSensorParameters`:

```cpp
libecu::BemfVoltageSensorParameters bemf_voltage_params;
bemf_voltage_params.r_up = 10000.0f;
bemf_voltage_params.r_down = 2200.0f;
adc_driver.initializeBemf(bemf_voltage_params);
```

### PB5 Mode Control

GPIO PB5 selects between two coupling modes per phase:

- **PB5 LOW** (default at boot): ADC reads the phase through the divider. Used for high BEMF voltages that would otherwise exceed the ADC reference.
- **PB5 HIGH**: ADC is tied directly to the phase through a 3.3V clamp. Used when phase voltages are guaranteed to stay within the ADC input range and the divider ratio would lose resolution.

The active mode is passed into `readPhaseVoltage()` as the `direct_mode` boolean, which selects the right scaling formula in `convertAdcToPhaseVoltage()`.

### ADC Triggering and Sequence

All injected conversions are triggered by `TIM1_TRGO2` so phase voltage sampling is synchronized to the PWM cycle and happens after the current-sensing and Vbus conversions on the same trigger. The ADCs run in dual-mode simultaneous injected mode (`ADC_DUALMODE_INJECSIMULT`).

**ADC1 injected sequence (3 channels):**

| Rank  | Signal            | Source          |
|-------|-------------------|-----------------|
| RANK1 | Phase U current   | OPAMP1 (`VOPAMP1`)         |
| RANK2 | Vbus              | PA0 (`ADC_CHANNEL_1`)      |
| RANK3 | Phase W voltage   | PB11 (`ADC_CHANNEL_14`)    |

**ADC2 injected sequence (4 channels):**

| Rank  | Signal            | Source                       |
|-------|-------------------|------------------------------|
| RANK1 | Phase V current   | OPAMP2 (`VOPAMP2`)           |
| RANK2 | Phase W current   | OPAMP3 (`VOPAMP3_ADC2`)      |
| RANK3 | Phase U voltage   | PA4 (`ADC_CHANNEL_17`)       |
| RANK4 | Phase V voltage   | PC4 (`ADC_CHANNEL_5`)        |

Injected oversampling is enabled with ratio 2 and a 1-bit right shift, giving one extra bit of effective resolution on every channel.

## 3. BEMF Zero-Crossing Detection Principle

### 6-Step Commutation Table

The `CommutationController` drives the inverter with the following static table. Each step energizes two phases (one `UP`, one `DOWN`) and leaves the third floating (`OFF`). The floating phase is where BEMF is observed.

```
Step 0: U=UP,   V=DOWN,  W=OFF   -> W floating
Step 1: U=UP,   V=OFF,   W=DOWN  -> V floating
Step 2: U=OFF,  V=UP,    W=DOWN  -> U floating
Step 3: U=DOWN, V=UP,    W=OFF   -> W floating
Step 4: U=DOWN, V=OFF,   W=UP    -> V floating
Step 5: U=OFF,  V=DOWN,  W=UP    -> U floating
```

The controller picks the floating phase at runtime by querying `getPhaseState()` for each phase and picking the one whose state is `PwmState::OFF`. This is done in `BldcController::findFloatingPhase()`.

### Zero-Crossing Geometry

During a commutation step the floating phase BEMF ramps toward (or away from) the bus rail. It crosses `Vbus / 2` at 30 electrical degrees after the step started, which is the midpoint of the 60-degree step. That crossing is the zero-crossing (ZC) event.

After a ZC is detected the controller still has another 30 degrees to go before the next commutation is due. The observer waits that delay out, then emits the synthetic step. Commutation timing therefore tracks the actual rotor rather than a fixed schedule.

### Edge Detection

The ZC detector does not look for sign or magnitude. It looks for a transition. On every PWM tick it compares the floating voltage against a threshold derived from the bus voltage:

```cpp
float threshold = bus_voltage * params_.zc_threshold;   // zc_threshold = 0.5 by default
bool above = floating_voltage > threshold;

if (above != prev_above_threshold_) {
    // Edge detected — ZC occurred
    prev_above_threshold_ = above;
    zc_detected_ = true;
    // synthetic_step_ = next Hall position (see mapping below)
    // ... compute 30-degree delay ...
}
```

Either rising or falling transitions count. The direction alternates naturally between steps because the floating phase alternates between phases and polarity.

### Synthetic Step Mapping

The observer must output the **next Hall position** (not the next commutation step) because `MotorPLL::updateHall()` expects a Hall sensor reading. The mapping from commutation step `C` to Hall position `H` depends on the commutation table direction:

- **Non-inverse**: `C = (H + 1) % 6` → `H = (C + 5) % 6` → next Hall = `(H + 1) % 6 = C`
- **Inverse**: `C = (H + 5) % 6` → `H = (C + 1) % 6` → next Hall = `(H + 1) % 6 = (C + 2) % 6`

The `is_inverse_commutation` parameter in `BemfObserverParams` selects the correct formula.

### 30-Degree Delay

Once a ZC is recorded the observer counts down a delay in PWM ticks before declaring the synthetic Hall event. The delay corresponds to half of one commutation step period:

```cpp
// 30° = half of one step period (one step = 60°)
// step_period = 1.0 / speed_steps_per_sec  seconds  (time for ONE step)
// half_step   = 0.5 / speed_steps_per_sec  seconds
// delay_ticks = half_step * pwm_frequency
//            = 0.5 * pwm_frequency / speed_steps_per_sec
if (speed_steps_per_sec >= 1.0f) {
    delay_counter_ = 0.5f * pwm_frequency_ / speed_steps_per_sec;
} else {
    // Speed too low — cannot compute a meaningful delay
    zc_detected_ = false;
    return false;
}
```

`speed_steps_per_sec` is the PLL's own estimate of rotor speed in steps per second. The observer therefore self-times the next commutation from the most recent ZC, with no fixed lookup table.

When the delay counter reaches zero, `update()` returns `true` and the caller feeds the synthetic step into the PLL:

```cpp
if (bemf_observer_->update(bemf_v, bus_v, status_.target_position,
                           motor_pll_.getSpeedStepsSec())) {
    motor_pll_.updateHall(bemf_observer_->getSyntheticHallStep());
}
```

## 4. Demagnetization Blanking

Right after a commutation the newly floating phase still carries current from when it was energized. Its voltage is dominated by inductive discharge, not BEMF, and any ZC reading taken during that window is bogus.

`BemfObserver` handles this with a blanking counter. `onCommutation()` is called by `BldcController` every time the commutation step changes, and it resets the blanking window:

```cpp
void BemfObserver::onCommutation(uint8_t new_step) noexcept {
    (void)new_step;
    blanking_counter_ = params_.blanking_cycles;
    zc_detected_ = false;
    delay_counter_ = 0.0f;
    event_pending_ = false;
}
```

While `blanking_counter_` is non-zero, `update()` decrements it and short-circuits before any ZC comparison runs. The previous comparison state (`prev_above_threshold_`) is left untouched so the edge detector is ready the moment blanking ends.

`blanking_cycles` is expressed in PWM cycles. With the application default of 5 cycles at a 20 kHz PWM rate, the blanking window is 250 us. Heavier motors or higher inductance may need a longer window.

## 5. Hybrid Mode Switching

The controller picks its position source based on rotor speed in steps per second, as estimated by the PLL. Three zones are defined by two thresholds:

| Zone                | Speed range                                  | Hall ISR   | BemfObserver        |
|---------------------|----------------------------------------------|------------|---------------------|
| Hall only           | below `transition_speed_low`                 | active     | inactive            |
| Transition          | between `transition_speed_low` and `_high`   | active     | runs, no suppress   |
| BEMF only           | above `transition_speed_high`                | suppressed | active              |

### Hysteresis

A two-threshold scheme with hysteresis prevents mode chatter when the rotor speed hovers near a boundary. The `bemf_was_active_` flag remembers the current source:

```cpp
bool BemfObserver::isBemfModeActive(float speed_steps_per_sec) const noexcept {
    if (bemf_was_active_) {
        // Currently in BEMF mode — drop out at low threshold
        if (speed_steps_per_sec < params_.transition_speed_low) {
            bemf_was_active_ = false;
        }
    } else {
        // Currently in Hall mode — enter BEMF at high threshold
        if (speed_steps_per_sec > params_.transition_speed_high) {
            bemf_was_active_ = true;
        }
    }
    return bemf_was_active_;
}
```

Entering BEMF mode requires crossing `transition_speed_high`. Dropping back to Hall requires falling below `transition_speed_low`. Between the two thresholds the controller stays in whatever mode it was already in. This means the transition zone can show both sources feeding the PLL simultaneously, which is harmless because both push the same step direction.

### Hall Suppression

Suppressing the Hall ISR above the high threshold is a separate decision so the BEMF observer always runs first and the PLL is never fed conflicting steps. The check lives in `BldcController::hallSensorInterruptHandler()`:

```cpp
if (bemf_observer_ &&
    bemf_observer_->shouldIgnoreHall(motor_pll_.getSpeedStepsSec())) {
    return;
}
motor_pll_.updateHall(hall_state);
```

`shouldIgnoreHall()` returns true only when speed is strictly above `transition_speed_high`. Below that, Hall events continue to reach the PLL even while the BEMF observer is running, which keeps the loop synchronized during handover.

## 6. Integration with MotorPLL

The PLL exposes three methods that the rest of the controller uses:

| Method        | Caller                          | Purpose                                            |
|---------------|---------------------------------|----------------------------------------------------|
| `updateHall(step)` | Hall ISR or BemfObserver path | Discrete position update; resets PLL timing reference |
| `updateTick()`     | PWM ISR (TIM1 update)          | Integrates the virtual angle by `angle_per_second_`  |
| `getNextHall(mode)` | PWM ISR                       | Returns the next 0-5 step with 90-degree field offset |

`updateHall()` accepts any step 0-5. The PLL does not tag the source. Whether the step came from a Hall GPIO edge or from `BemfObserver::getSyntheticHallStep()`, the same internal state updates run: the accumulated Hall counter advances, the timing reference is reset, and the PI integrator that drives `angle_per_second_` is nudged toward the new rate.

Because the BEMF path lands on the exact same call as the Hall path, no changes to `MotorPLL` were needed to support sensorless operation. The PLL keeps integrating at the PWM rate via `updateTick()`, the speed estimate keeps coming from its PI integrator, and the commutation controller keeps consuming `getNextHall()` output. The BEMF observer is a pure producer of `updateHall()` events.

## 7. Data Flow

The diagram below shows the two ISRs and how the observer slots into the PWM path. The PWM ISR runs at `PWM_TIMER_FREQ` (20 kHz on the STM32G431 platform).

```
TIM1 Update ISR (20 kHz)
  |
  +-> motor_pll_.updateTick()           // PI angle integrator
  +-> new_position = getNextHall()      // commutation step from PLL
  +-> [if step changed]
  |     +-> commutation_controller_.update(new_position, duty)
  |     +-> bemf_observer_->onCommutation(new_position)   // reset blanking
  +-> read current (DOWN phase)
  +-> read bus voltage
  +-> [if BEMF active]
  |     +-> findFloatingPhase()         // pick the OFF phase
  |     +-> read BEMF voltage (ADC)
  |     +-> bemf_observer_->update()
  |     +-> [if ZC + delay expired]
  |           +-> motor_pll_.updateHall(synthetic_step)
  +-> run current PI controller
  +-> apply commutation

Hall EXTI ISR
  |
  +-> read Hall GPIO state
  +-> [if !shouldIgnoreHall()]
        +-> motor_pll_.updateHall(hall_state)
```

The BEMF block runs after the commutation update so `onCommutation()` is called first and the blanking window is open before any new sample is examined. The current PI controller runs after the BEMF block in `CURRENT_MODE` so the synthetic step, if any, is already in the PLL by the time the duty cycle is computed.

## 8. Configuration Parameters

BEMF parameters live in two structs. `MotorControlParams` carries them at the application level for AT-command tuning. They are copied into a `BemfObserverParams` struct and pushed into the observer at startup.

### `MotorControlParams` (BEMF fields)

Defined in `libecu/include/bldc_controller.hpp`.

| Field                        | Type    | Application default (main.cpp) | Description                                                              |
|------------------------------|---------|--------------------------------|--------------------------------------------------------------------------|
| `bemf_transition_speed_low`  | float   | 500.0 steps/sec                | Below this speed the Hall ISR is never suppressed.                       |
| `bemf_transition_speed_high` | float   | 800.0 steps/sec                | Above this speed Hall events are ignored and BEMF drives the PLL.        |
| `bemf_blanking_cycles`       | float   | 2.0 PWM cycles                 | Demagnetization blanking window after each commutation.                  |
| `bemf_zc_threshold`          | float   | 0.5                            | ZC threshold as a fraction of Vbus. 0.5 means the crossing is at Vbus/2. |

### `BemfObserverParams`

Defined in `libecu/include/algorithms/bemf_observer.hpp`.

| Field                    | Type  | Class constructor default | Description                                                  |
|--------------------------|-------|---------------------------|--------------------------------------------------------------|
| `blanking_cycles`        | float | 10.0                      | PWM cycles to blank after commutation (demagnetization).     |
| `zc_threshold`           | float | 0.5                       | BEMF zero-crossing threshold as a fraction of Vbus.          |
| `transition_speed_low`   | float | 600.0 steps/sec           | Speed below which Hall sensors are used.                     |
| `transition_speed_high`  | float | 1200.0 steps/sec          | Speed above which BEMF is used exclusively.                  |
| `is_inverse_commutation` | bool  | false                     | Selects synthetic step mapping for inverse commutation table. |

The application defaults from `main.cpp` (500 / 800 / 2 / 0.5 / inverse from `BLDC_INVERTION`) override the class constructor defaults at startup via `setParameters()`. The class defaults only apply if `setParameters()` is never called.

### Tuning Notes

- The two thresholds should be far enough apart to give a clean hysteresis band. A gap of 300 steps/sec is the application default.
- `blanking_cycles` scales with PWM frequency. At 20 kHz, 2 cycles is 100 us. If you raise the PWM frequency, raise `blanking_cycles` proportionally to keep the same time window. Keep it short enough that the ZC crossing (which occurs at 30° into the step) is not masked.
- `zc_threshold` can be shifted away from 0.5 to compensate for non-symmetric phase voltage waveforms or diode drops in the inverter. Most setups leave it at 0.5.

## 9. Key Source Files

| File                                                | Role                                                                                       |
|-----------------------------------------------------|--------------------------------------------------------------------------------------------|
| `libecu/include/algorithms/bemf_observer.hpp`       | `BemfObserver` class and `BemfObserverParams` struct.                                      |
| `libecu/src/bemf_observer.cpp`                      | ZC detection, 30-degree delay, blanking, hysteresis.                                       |
| `libecu/include/interfaces/adc_interface.hpp`       | `BemfVoltageSensorParameters`, `getRawPhaseVoltage()`, `readPhaseVoltage()`, divider math. |
| `libecu/hal/stm32g4/stm32_adc.cpp`                  | ADC1/ADC2 injected channel config, PB5 GPIO mode control, OPAMP setup.                     |
| `libecu/hal/stm32g4/stm32_adc.hpp`                  | `Stm32Adc` declaration, `setBemfDividerMode()`.                                            |
| `libecu/src/bldc_controller.cpp`                    | ISR integration: `pwmInterruptHandler()`, `hallSensorInterruptHandler()`, `findFloatingPhase()`. |
| `libecu/include/bldc_controller.hpp`                | `MotorControlParams` with BEMF fields, `setBemfObserver()`.                                |
| `libecu/include/algorithms/motor_pll.hpp`           | `MotorPLL::updateHall()` / `updateTick()` / `getNextHall()` API.                           |
| `STM32G431/Core/Src/main.cpp`                       | Wiring: constructs `BemfObserver`, configures parameters, registers with controller.      |

## References

- STM32 UM3042 "Motor-control sensorless BEMF zero-crossing" application note, section 4.1.2, for the 30-degree ZC geometry referenced in `bemf_observer.cpp`.
- STM32G431 reference manual (RM0440) for ADC dual-mode injected conversions and `TIM1_TRGO2` trigger routing.
- Project README and `AGENTS.md` for the high-level firmware architecture and build instructions.
