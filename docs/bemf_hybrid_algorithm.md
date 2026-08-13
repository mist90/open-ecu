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

The BEMF front end is a 22 kOhm / 2.2 kOhm divider per phase, giving a ratio of exactly 11.0x. The ADC pin sits at the 2.2 kOhm node to ground, so full scale is 3.3 V * 11.0 = **36.3 V of phase voltage** - just above the 36 V over-voltage trip in `MotorControlParams::max_voltage`. Parameters are stored in `BemfVoltageSensorParameters`:

```cpp
libecu::BemfVoltageSensorParameters bemf_voltage_params;
bemf_voltage_params.r_up = 22000.0f;
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

### Coupling mode: ON-time sensing

The bridge uses complementary (synchronous) PWM: the `UP` phase chops between Vbus and ground, the `DOWN` phase has its low side permanently on, and the floating phase has both switches off. `TIM1_CCR4` is set to `CCR_phase / 2` (`Stm32Pwm::calculateAdcTriggerCompare`), so `TRGO2` fires the injected ADC sequence inside the high-side conduction window. This is **ON-time sensing**: while the high side conducts, one phase sits at Vbus, one at 0 V, and the motor neutral is at Vbus/2. The floating phase therefore reads

```
V_float = Vbus/2 + e_c(theta)
```

where `e_c` is the BEMF of the floating winding, referred to the neutral. Across a 60-degree step `e_c` ramps linearly from one trapezoid plateau to the other and crosses zero at 30 degrees. That crossing is the zero-crossing (ZC). A real Hall edge would occur another 30 degrees later, which is where the synthetic Hall event has to be placed.

### Polarity normalisation

Which way `e_c` ramps alternates from step to step. Tracing each floating winding between the step where it is driven `UP` and the step where it is driven `DOWN` shows the BEMF falls on the even steps and rises on the odd ones. Rather than accepting either edge direction, the observer multiplies the error by a per-step sign:

```cpp
int8_t sign = (step & 1u) ? +1 : -1;
if (params_.is_inverse_commutation) sign = -sign;   // sequence runs backwards
if (speed_steps_per_sec < 0.0f)     sign = -sign;   // REVERSE
v_diff = sign * (V_float - V_ref) * polarity_;
```

`v_diff` is then negative before the ZC and positive after it on every step and in both directions, and detection reduces to one "crosses zero upwards" test at the true neutral.

Normalising the polarity is what removes the amplitude dependence of the old two-threshold scheme. Detecting at `0.55 * Vbus` instead of at the neutral means waiting for the BEMF to climb `0.05 * Vbus` past zero, which takes `0.025 * (Vbus / E) * T_step` seconds, where `E` is the BEMF plateau. The resulting phase error is **6 degrees at E = 0.25 * Vbus and 15 degrees at E = 0.1 * Vbus** - it scales with load and speed instead of staying constant, and below `E = 0.05 * Vbus` the signal never reaches the threshold at all and no ZC is ever detected.

`polarity_` is a learned global flip. When `auto_polarity` is set, the observer checks the first valid sample of each step - which is by construction still on the pre-crossing side - and inverts the sign after `POLARITY_FLIP_STEPS` (4) consecutive steps disagree. This covers wiring and commutation-table conventions that the static table gets wrong.

### Neutral reference

By default `V_ref = bus_voltage / 2`. `bus_voltage` comes through a different divider (169k/18k) and a different ADC channel than the phase taps (22k/2.2k), so their tolerance mismatch appears as a fixed offset on `V_ref`.

That offset does **not** land as a fixed commutation phase error, as one might expect. `v_diff` multiplies by a sign that alternates step to step, so a constant offset on `V_ref` alternates with it: odd steps fire early, even steps fire late by the same amount. The mean is unchanged and the symptom is **step-to-step jitter**, i.e. torque ripple at three times the electrical frequency, not a DC phase shift. The harness confirms this - a 2% bus-divider mismatch leaves the bias at 1.77 degrees but lifts the jitter from 1.16 to 1.71 degrees, an alternating component of `sqrt(1.71^2 - 1.16^2) = 1.25` degrees, which matches `(err * Vbus/2) / (2E) * 60` degrees.

Setting `use_virtual_neutral` switches the reference to `(Vu + Vv + Vw) / 3`, which travels the same divider path and cancels that mismatch. The measured difference is then `(2/3) * e_c`, which the observer scales back by 1.5 so the integrator limit means the same thing in both modes.

**This is enabled on the current hardware.** The earlier 10k/2.2k divider saturated the 3.3 V ADC input at 18.3 V of phase voltage, so the phase sitting at Vbus railed out on any bus above that and the mean of the three readings was wrong. The 22k/2.2k divider reaches 36.3 V full scale, above the 36 V over-voltage trip, so all three taps stay in range across the whole operating envelope.

Measured effect, from `tests/test_bemf_observer/run.sh` at 1200 steps/s with 0.3 V rms noise:

| Bus-divider mismatch | Reference | bias | jitter |
|---|---|---|---|
| 0 % | `Vbus/2` | 1.74 | 1.16 |
| +1 % | `Vbus/2` | 1.80 | 1.29 |
| +2 % | `Vbus/2` | 1.77 | 1.71 |
| any | virtual neutral | 1.72 | 1.17 |

The virtual neutral is flat across the whole sweep - the mismatch cancels exactly, as the derivation says it should.

Headroom above the trip is only 0.3 V, so the observer checks every sample rather than trusting the divider: in ON-time sensing one phase always sits at the top rail, so if the largest of the three readings falls below `0.90 * Vbus` the taps must be saturating and the sample reverts to the `Vbus/2` reference. A false trigger of that check is harmless - it is simply the other reference mode - so the check is biased towards firing. `BemfInfo::virtual_neutral` reports which reference was actually used.

### Deadband and confirmation

`zc_deadband_volts` forces everything inside the ADC/EMI noise floor to read as exactly zero, so noise alone can neither create nor cancel a crossing. Setting it to 0 selects an automatic value of 1% of Vbus.

`zc_confirm_samples` (default 2) requires that many consecutive positive samples before a crossing is accepted; a positive blip that falls back to negative discards the candidate. The confirmation costs nothing in timing accuracy because the crossing instant is interpolated from the bracketing samples, not from the sample that confirms it.

### Sub-sample interpolation

At 20 kHz PWM the sample spacing is significant: 3.6 electrical degrees at 1200 steps/s and 7.2 degrees at 2400 steps/s. Quantising the ZC to a whole PWM tick would cost up to a full sample of phase error. The observer keeps the last strictly-negative sample and its age in ticks, and interpolates linearly against the first positive one:

```cpp
const float span = neg_age_ * dt_;
const float frac = v_neg_last_ / (v_neg_last_ - v_diff);   // in (0,1)
t_since_zc_      = span * (1.0f - frac);
```

Keeping the *age* rather than assuming the previous tick means samples that fell inside the deadband, or that were discarded by the rail guard, do not break the bracket.

### Rail guard

A floating phase pinned to 0 V or Vbus is not measuring BEMF - it is freewheeling through a body diode. Samples outside `[rail_margin * Vbus, (1 - rail_margin) * Vbus]` are discarded outright and do not advance any detection state.

### Synthetic Step Mapping

The observer must output the **next Hall position** (not the next commutation step) because `MotorPLL::updateHall()` expects a Hall sensor reading. The mapping from commutation step `C` to Hall position `H` depends on the commutation table direction:

- **Non-inverse**: `C = (H + 1) % 6` -> next Hall = `C`
- **Inverse**: `C = (H + 5) % 6` -> next Hall = `(C + 2) % 6`

The `is_inverse_commutation` parameter selects the correct formula.

### Commutation timing

`BemfTimingMode` selects how the commutation instant is placed after the ZC.

**`DELAY_30DEG`** counts down half a step period using the PLL speed estimate:

```cpp
target = 0.5f * (1.0f - phase_advance) * step_period;
delay_ticks_ = (target - t_since_zc_) / dt_;
```

This is the original scheme. It works, but the delay depends on a speed estimate that is itself driven by this observer's own events: a late ZC produces a late event, which makes the PLL read a lower speed, which lengthens the delay, which makes the next event later still. It also degrades whenever the rotor accelerates inside a step.

**`FLUX_INTEGRATE`** (default). It integrates `v_diff` from the interpolated crossing and fires when the integral reaches `integrator_limit_vs`:

```cpp
integrator_ += 0.5f * (max(v_prev, 0) + max(v_now, 0)) * dt_;   // positive contributions only
fire = (integrator_ >= effectiveLimit());
```

The BEMF plateau `E` is proportional to speed while the 30-degree arc is inversely proportional to speed, so the integral over that arc - `E * T_step / 4` - is **speed-independent**. One constant covers the whole speed range and no speed estimate enters the loop at all. Integration is also a low-pass filter, so per-sample ADC noise averages out instead of directly displacing the commutation instant.

Only positive contributions are accumulated, so a noise dip cannot unwind flux that is already banked. A backstop fires the event anyway if the step runs 45 degrees past the ZC, so a mis-tuned limit cannot strand the PLL.

**Learning the limit.** With `auto_learn_limit` set and `integrator_limit_vs` left at 0, the observer learns the limit during the hybrid handover: on steps where it detected a ZC but the *Hall sensors* placed the commutation, the flux accumulated between the two is exactly the 30-degree integral it should be aiming for, and it is low-pass filtered into `learned_limit_` with `learn_alpha`. Until a value is learned, `FLUX_INTEGRATE` falls back to the timed countdown, so the observer is usable from the first step.

**Phase advance.** `phase_advance` (0..0.9) scales the limit (or the countdown) down, moving the commutation earlier by that fraction of the 30-degree arc.

### Recovering from a late bridge

If the bridge falls more than 30 degrees behind the rotor, the crossing has already happened by the time blanking ends and no negative sample can arrive - the naive detector waits forever and the loop latches permanently behind. When the first valid samples of a step are already positive with no negative bracket, the observer confirms that over `zc_confirm_samples` and then anchors the crossing at the **step start**, not at "now". Anchoring at "now" would add another 30 degrees of lag every step and the loop would run away instead of catching up.

## 4. Demagnetization Blanking

Right after a commutation the newly floating phase still carries current from when it was energized. Its voltage is dominated by inductive discharge, not BEMF, and any reading taken during that window is bogus.

Two mechanisms cover this, and the second is the one that matters:

**Time-based blanking.** `onCommutation()` restarts the window. `update()` short-circuits while

```
time_since_comm_ < max(blanking_cycles * dt, blanking_fraction * last_step_period)
```

A fixed tick count alone is wrong at both ends of the speed range: too short for a large motor at low speed, and longer than the whole 30-degree arc at high speed. At 2400 steps/s a step is only 8.3 PWM cycles, so the original fixed 2-cycle window already consumes a quarter of it. `blanking_fraction` (default 0.20, clamped below 0.45 so blanking always ends before the 30-degree point) makes the window scale with speed.

**Rail guard.** How long demagnetization actually lasts depends on phase current, inductance and speed, so no fixed window covers it reliably - and the sample right after a too-short window is both the most contaminated one and the one the detector leans on hardest. Discarding railed samples (section 3) removes the dependence on getting the window right: a freewheeling phase is pinned to a rail by definition, so it is filtered out whether or not the timer agreed.

## 5. Hybrid Mode Switching

The controller picks its position source based on rotor speed in steps per second, as estimated by the PLL. Three zones are defined by two thresholds:

| Zone                | Speed range                                  | Hall ISR   | BemfObserver        |
|---------------------|----------------------------------------------|------------|---------------------|
| Hall only           | below `transition_speed_low`                 | active     | inactive            |
| Transition          | between `transition_speed_low` and `_high`   | active     | runs, no suppress   |
| BEMF only           | above `transition_speed_high`                | suppressed | active              |

The transition zone is where `auto_learn_limit` acquires the flux limit, so it should be wide enough for a few tens of commutations.

### Hysteresis

A two-threshold scheme with hysteresis prevents mode chatter when the rotor speed hovers near a boundary. The `bemf_was_active_` flag remembers the current source. The PLL speed is **signed** - negative in REVERSE, because the Hall sequence descends - so both `isBemfModeActive()` and `shouldIgnoreHall()` compare magnitudes:

```cpp
const float speed = std::fabs(speed_steps_per_sec);
if (bemf_was_active_) {
    if (speed < params_.transition_speed_low)  bemf_was_active_ = false;
} else {
    if (speed > params_.transition_speed_high) bemf_was_active_ = true;
}
```

Comparing the signed value made BEMF mode unreachable in REVERSE and produced a negative 30-degree delay that fired immediately.

### Hall Suppression

`shouldIgnoreHall()` returns false whenever the observer has lost lock (`sync_lost_`), so the Hall sensors automatically take the loop back while the observer keeps running and re-synchronises. `sync_lost_` is raised when no zero-crossing arrives within `max_step_periods` step periods and cleared on the next successful event.

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
  +-> read Vu, Vv, Vw from the injected ADC registers   // sampled mid-ON-time
  +-> findFloatingPhase()                               // pick the OFF phase
  +-> read bus voltage
  +-> [if isBemfModeActive(speed, duty)]
  |     +-> bemf_observer_->update(BemfObserverInput{...})
  |     +-> [if it returned true]
  |           +-> motor_pll_.updateHall(getSyntheticHallStep())
  +-> [if hall_update_pending_]                         // deferred, debounced
  |     +-> hall_state = getCurrentPosition()
  |     +-> [if !shouldIgnoreHall(speed, duty)]
  |           +-> motor_pll_.updateHall(hall_state)
  +-> motor_pll_.updateTick()                           // PI angle integrator
  +-> new_position = getNextHall(dmode)                 // commutation step
  +-> run current PI controller  (CURRENT_MODE)
  +-> [if step changed]
        +-> commutation_controller_.update(new_position, duty)
        +-> bemf_observer_->onCommutation(new_position) // restart blanking

Hall EXTI ISR
  |
  +-> hall_update_pending_ = true       // re-read in the PWM ISR, 50 us later,
                                        // so EMI-induced bounce is hidden
```

Both position sources reach the PLL before `updateTick()` runs, so a synthetic event and a Hall event arriving in the same cycle are resolved by the PLL rather than by ISR ordering. `onCommutation()` is called last, at the point the bridge actually changes, which is what makes `time_since_comm_` line up with the real step boundary: the new switch states take effect on the next timer update event.

## 8. Configuration Parameters

BEMF parameters live in two structs. `MotorControlParams` carries the application-level subset for AT-command tuning; the full set lives in `BemfObserverParams`. `main.cpp` reads the class defaults with `getParameters()`, overrides what it cares about, and pushes the result back with `setParameters()`.

### `BemfObserverParams`

Defined in `libecu/include/algorithms/bemf_observer.hpp`.

| Field | Type | Default | Description |
|---|---|---|---|
| `min_duty` | float | 0.15 | Minimum duty for ON-time sensing; below this the Hall sensors are used |
| `transition_speed_low` | float | 600 st/s | Below this the Hall ISR is never suppressed |
| `transition_speed_high` | float | 1200 st/s | Above this Hall events are ignored and BEMF drives the PLL |
| `is_inverse_commutation` | bool | false | Selects the synthetic step mapping and flips the slope sign |
| `blanking_cycles` | float | 2.0 | Hard floor of the blanking window, in PWM cycles |
| `blanking_fraction` | float | 0.20 | Blanking as a fraction of the step period, clamped below 0.45 |
| `zc_deadband_volts` | float | 0.0 | Noise floor in Volts; 0 selects 1% of Vbus |
| `zc_confirm_samples` | uint8 | 2 | Consecutive positive samples required to accept a crossing |
| `use_virtual_neutral` | bool | false | Reference `(Vu+Vv+Vw)/3` instead of `Vbus/2` - needs unclipped dividers |
| `auto_polarity` | bool | true | Learn the global BEMF slope sign at runtime |
| `rail_margin` | float | 0.05 | Discard samples this fraction of Vbus away from either rail |
| `timing_mode` | enum | `FLUX_INTEGRATE` | Commutation timing strategy |
| `integrator_limit_vs` | float | 0.0 | Flux from ZC to commutation in V*s; 0 means learn it |
| `phase_advance` | float | 0.0 | 0..0.9: fraction of the 30-degree arc removed |
| `auto_learn_limit` | bool | true | Learn the flux limit from Hall-driven steps |
| `learn_alpha` | float | 0.2 | LPF coefficient for the learned limit |
| `max_step_periods` | float | 2.0 | Declare loss of lock if no event within this many step periods |
| `amplitude_lpf_alpha` | float | 0.2 | LPF coefficient for the BEMF amplitude estimate |

### Tuning Notes

- **Start with the defaults.** `integrator_limit_vs = 0` plus `auto_learn_limit = true` means the observer calibrates itself during the Hall-to-BEMF handover. Read the learned value back with `getLearnedIntegratorLimit()` and pin it in `main.cpp` once it is stable.
- **`zc_deadband_volts` is a floor, not a filter.** Sizing it above the noise costs samples in the 30-degree arc, which matters at high speed where there are only a handful. The sweep in `tests/test_bemf_observer` shows the tradeoff: at 2400 steps/s with 0.8 V rms noise, going from a 0.24 V deadband to 2.4 V raises the mean error from 3.7 to 11.2 degrees and starts dropping steps. Prefer the automatic 1% of Vbus and let `zc_confirm_samples` and the integrator do the noise rejection.
- **`blanking_cycles` scales with PWM frequency**; `blanking_fraction` does not. If you raise the PWM frequency, raise `blanking_cycles` proportionally to keep the same absolute demagnetization floor.
- **Residual bias.** Firing on the tick at or after the target adds on average half a PWM period of lag - 1.8 degrees at 1200 steps/s, 3.6 at 2400. It is a known constant and can be dialled out with `phase_advance` if it matters.
- **Pick the transition speeds from measurement, not from a guess.** Capture logs across the speed range with `utility/capture_log.py` and run `tests/test_bemf_replay` - the zero-crossing jitter column shows where the detector becomes trustworthy. On MOTOR_1 (40 poles, 31 V bus), from 8-second captures at each speed:

  | steps/s | RPS | ZC jitter |
  |---|---|---|
  | 619 | 5.16 | 12.5 deg |
  | 652 | 5.43 | 10.5 deg |
  | 742 | 6.18 | 4.6 deg |
  | 787 | 6.56 | 5.1 deg |
  | 829 | 6.91 | 6.2 deg |
  | 960 | 8.00 | 1.5 deg |

  The detector goes from unusable to good between 652 and 742 steps/s, so *both* thresholds are placed above that knee (720 / 828 steps/s). Setting `transition_speed_low` far below the knee is the dangerous half: `transition_speed_high` only decides where BEMF *takes over*, but once it has the loop it keeps it all the way down to `transition_speed_low`. The original 500 steps/s put that fallback in the 13-degree region.
- **The thresholds are per motor and belong in RPS.** One electrical step is 60 degrees, so `steps/sec = RPS * num_poles * 3`. The same numeric threshold in steps/sec means completely different mechanical speeds on a 40-pole and an 8-pole motor; `main.cpp` declares `BLDC_BEMF_LOW_RPS` / `BLDC_BEMF_HIGH_RPS` inside the per-motor block and converts.
- **Check the band is worth having.** The usable sensorless range is `transition_speed_high` up to whatever the bus voltage allows. On MOTOR_1 at 31 V that is 828 steps/s to about 1200 steps/s - under 1.5:1 - because the motor is voltage-limited at ~10 RPS. A higher bus widens the band at the top; nothing widens it at the bottom except a cleaner signal.
- **A wide transition zone helps.** `auto_learn_limit` only learns on steps that the Hall sensors commutated, so the band between `transition_speed_low` and `transition_speed_high` needs to be crossed slowly enough to collect a few tens of steps.

## 9. Key Source Files

| File                                                | Role                                                                                       |
|-----------------------------------------------------|--------------------------------------------------------------------------------------------|
| `libecu/include/algorithms/bemf_observer.hpp`       | `BemfObserver` class and `BemfObserverParams` struct.                                      |
| `libecu/src/bemf_observer.cpp`                      | Polarity normalisation, ZC interpolation, flux integration, blanking, rail guard.          |
| `libecu/include/interfaces/adc_interface.hpp`       | `BemfVoltageSensorParameters`, `getRawPhaseVoltage()`, `readPhaseVoltage()`, divider math. |
| `libecu/hal/stm32g4/stm32_adc.cpp`                  | ADC1/ADC2 injected channel config, PB5 GPIO mode control, OPAMP setup.                     |
| `libecu/hal/stm32g4/stm32_adc.hpp`                  | `Stm32Adc` declaration, `setBemfDividerMode()`.                                            |
| `libecu/src/bldc_controller.cpp`                    | ISR integration: `pwmInterruptHandler()`, `hallSensorInterruptHandler()`, `findFloatingPhase()`. |
| `libecu/include/bldc_controller.hpp`                | `MotorControlParams` with BEMF fields, `setBemfObserver()`.                                |
| `libecu/include/algorithms/motor_pll.hpp`           | `MotorPLL::updateHall()` / `updateTick()` / `getNextHall()` API.                           |
| `STM32G431/Core/Src/main.cpp`                       | Wiring: constructs `BemfObserver`, configures parameters, registers with controller.      |
| `tests/test_bemf_observer/`                         | Host harness: commutation accuracy and amplitude estimate under noise.                     |

## 10. Measured behaviour

`tests/test_bemf_observer/run.sh` builds a host harness that models a trapezoidal BLDC, samples the floating phase once per PWM cycle the way the injected ADC does, and closes the commutation loop on the observer. The rotor is kinematic, so the metric is the true rotor angle at which the observer emits its synthetic event, in electrical degrees, against the sector boundary it names. "bias" is the mean, "jitter" the standard deviation about the mean, "missed" the number of steps that produced no event and had to be forced by the harness watchdog.

20 kHz PWM, 24 V bus, 8 degrees of modelled demagnetization:

| Case | timing | bias | jitter | max\|e\| | missed |
|---|---|---|---|---|---|
| clean, 1200 st/s | `DELAY_30DEG` | 1.24 | 0.98 | 3.60 | 0 |
| | `FLUX_INTEGRATE` | 1.23 | 0.98 | **2.47** | 0 |
| clean, 2400 st/s | `DELAY_30DEG` | 2.42 | 1.96 | 7.20 | 0 |
| | `FLUX_INTEGRATE` | 2.42 | 1.96 | 7.20 | 0 |
| weak BEMF (E = 0.08 Vbus) | `DELAY_30DEG` | 1.24 | 0.98 | 3.60 | 0 |
| | `FLUX_INTEGRATE` | 1.23 | 0.98 | **2.47** | 0 |
| 0.3 V rms noise | `DELAY_30DEG` | 1.52 | 1.42 | 6.02 | 0 |
| | `FLUX_INTEGRATE` | 1.74 | **1.16** | **4.82** | 0 |
| 0.8 V rms noise | `DELAY_30DEG` | 1.85 | 2.87 | 10.84 | 0 |
| | `FLUX_INTEGRATE` | 1.65 | **1.71** | **7.24** | 0 |
| 0.8 V rms + 1 % EMI spikes | `DELAY_30DEG` | 2.04 | 3.06 | 16.85 | 1 |
| | `FLUX_INTEGRATE` | 1.70 | **1.84** | **7.24** | **0** |
| 0.8 V rms, 2400 st/s | `DELAY_30DEG` | 3.35 | 3.22 | 28.82 | 5 |
| | `FLUX_INTEGRATE` | 3.76 | 3.09 | 38.43 | 6 |

Reading the numbers:

- **Flux integration wins on jitter and worst case, not on bias.** At 0.8 V rms the jitter is 1.71 vs 2.87 degrees and the worst single step is 7.24 vs 10.84 - the integrator's low-pass action. Under EMI spikes it is the difference between 0 and 1 dropped step.
- **The comparison flatters `DELAY_30DEG`.** The harness hands it the *exact* rotor speed. In the firmware it gets the PLL estimate that its own events produce, so the positive-feedback path described in section 3 is not represented here at all. Treat the delay column as an optimistic bound.
- **Neither mode is exercised on acceleration**, which is where the flux integral's real advantage lies: it is a path integral in angle, so any speed trajectory gives the same result at the same angle, while a countdown locks in a duration at the ZC and is wrong the moment the speed changes. The harness runs at constant speed only.
- The residual 1-4 degrees of bias is the half-PWM-period firing granularity described in section 8, not a detection error. It is the same for both modes because both fire on the tick at or after their target.
- 2400 steps/s is the hard case for both: ~8 PWM samples per step leaves only ~4 between the end of blanking and the crossing.

### What the legacy detector measured

The pre-rewrite detector (fixed `0.55`/`0.45 * Vbus` hysteresis pair, fixed 2-cycle blanking, speed-derived countdown) was carried in the harness during the rewrite and has since been removed. For the record, on the same cases it produced:

| Case | bias | jitter | missed |
|---|---|---|---|
| clean, 1200 st/s | 9.6 | 1.0 | 0 |
| clean, 2400 st/s | -77.6 | 90.0 | 1333 |
| weak BEMF (E = 0.08 Vbus) | -67.2 | 90.0 | 666 |
| 0.8 V rms noise | -67.6 | 90.7 | 100 |
| 0.8 V rms + 1 % EMI spikes | -72.4 | 93.5 | 124 |

A jitter of ~90 degrees is not jitter, it is loss of lock - the loop was not tracking at all in those rows. The two outright failures, 2400 steps/s and weak BEMF, are exactly the two failure modes predicted analytically in section 3: a fixed 2-cycle blanking window against an 8-cycle step, and a signal that never climbs `0.05 * Vbus` past the neutral. These numbers are no longer reproducible from the current harness.

## 11. BEMF amplitude estimate (for model-based current control)

`BemfObserver::getAmplitude()` returns a `BemfAmplitude` describing the back-EMF the observer is currently seeing. It exists so a current command can be computed from the motor's physics instead of leaving a PID to discover the operating point every time.

### How it is measured

Inside a step the floating winding sweeps the linear transition of the trapezoid from `-E` to `+E`, so the polarity-normalised signal `v_diff` is a straight line whose slope carries the amplitude. The observer runs a least-squares fit over every sample of the step that survived the rail guard:

```
slope s = (n*sum(k*v) - sum(k)*sum(v)) / (n*sum(k^2) - sum(k)^2)   [V per PWM tick]
E       = 0.5 * (s / dt) * T_step                                   [V, line-to-neutral peak]
ke      = E * T_step / (pi/3)                                       [V*s per electrical radian]
```

Three implementation points:

- **The abscissa is the PWM tick index, not seconds.** With seconds the sums are O(1e-8) and the normal-equation denominator `n*sum(t^2) - sum(t)^2` loses most of a `float` mantissa to cancellation. With tick indices the sums stay O(1..1e3) and the fit is exact to working precision.
- **The fit uses the raw `v_diff`, before the deadband.** The deadband flattens exactly the samples nearest the crossing, which would bias the fitted slope towards zero.
- **The fit runs in `onCommutation()`**, so it sees the whole step. That costs one step of latency on a quantity that changes on a mechanical timescale.

A fitted slope that comes out negative means the polarity is wrong or the step was garbage; the sample is dropped rather than folded in. Results are low-pass filtered with `amplitude_lpf_alpha` and retain their last good value, so the caller can read them every PWM cycle.

Because it is a fit over `n` samples rather than a peak reading, the noise on the estimate falls as `1/sqrt(n)`.

### Accuracy

From `tests/test_bemf_observer/run.sh`, against the modelled amplitude:

| Case | E true | E est | error |
|---|---|---|---|
| clean, 1200 st/s | 6.000 | 5.936 | -1.1 % |
| clean, 2400 st/s | 9.000 | 8.722 | -3.1 % |
| weak BEMF (0.08 Vbus) | 2.000 | 1.979 | -1.1 % |
| 0.3 V rms noise | 6.000 | 5.883 | -2.0 % |
| 0.8 V rms noise | 6.000 | 5.983 | -0.3 % |
| 0.8 V rms + 1 % spikes | 6.000 | 6.079 | +1.3 % |
| 0.8 V rms, 2400 st/s | 9.000 | 8.927 | -0.8 % |

Noise barely moves it - the fit averages it out. The consistent small **negative** bias is a real systematic, not noise: commutation runs a couple of degrees late, so the first samples of a step still catch the tail of the trapezoid *plateau*, which is flat and drags the fitted slope down. It scales with the commutation lag, which is why it is -3 % at 2400 steps/s (lag ~2.4-3.8 degrees) and -1 % at 1200 (lag ~1.2 degrees). Fitting only the middle of the step would remove it; at 1-3 % it is smaller than the winding-resistance drift you get from a 40 K temperature rise, so it is left alone.

### Using it for a feed-forward current command

In 6-step drive two phases are in series across the bridge, so the loop equation is

```
duty * Vbus = 2*E + 2*R_phase*I + 2*L_phase*dI/dt
```

Both driven phases contribute BEMF, hence `2*E`. In steady state the feed-forward duty for a target current is

```
duty_ff = (2*E + 2*R_phase*I_target) / Vbus
```

and the existing PID then only has to trim the model error - resistance drift with temperature, dead-time voltage loss, MOSFET and diode drops - instead of building the whole operating point out of integral action. That is what removes the windup and the slow recovery from saturation.

For a current *step* the inductive term matters:

```
duty_step = (2*E + 2*R_phase*I_target + 2*L_phase*(I_target - I_now)/dt) / Vbus
```

which is a deadbeat command; in practice clamp it, since `2*L/dt` is a large gain at 20 kHz.

**`ke_v_s_per_rad` is the field that makes this work at all speeds.** `E` is only measured while the observer is receiving samples - above `min_duty`, with a phase floating, and realistically above the transition speed. But `ke` is a motor constant. Once learned, `predictBemfVolts(speed)` gives `E = ke * (pi/3) * speed_steps_per_sec` from the PLL speed alone, so the feed-forward model stays valid down to standstill where the observer itself is inactive:

```cpp
const libecu::BemfAmplitude a = controller.getBemfAmplitude();
const float e = a.valid ? a.peak_volts
                        : observer.predictBemfVolts(pll_speed_steps_sec);
const float duty_ff = (2.0f * e + 2.0f * R_phase * i_target) / bus_voltage;
```

Two parameters the observer cannot supply and that have to come from a motor datasheet or a bench measurement: `R_phase` and `L_phase` (line-to-neutral). A DC injection test at standstill gives `R_phase`; a small-signal step or an LCR meter across two terminals gives `2*L_phase`.

`BldcController::getBemfAmplitude()` is a `CriticalSection`-guarded passthrough, since the estimate is updated from the PWM ISR.

## References

- STM32 UM3042 "Motor-control sensorless BEMF zero-crossing" application note, section 4.1.2, for the 30-degree ZC geometry referenced in `bemf_observer.cpp`.
- VESC firmware, `motor/mcpwm.c` (`mcpwm_adc_int_handler`, `update_adc_sample_pos`, `rpm_thread`) for the flux-integration commutation scheme.
- STM32G431 reference manual (RM0440) for ADC dual-mode injected conversions and `TIM1_TRGO2` trigger routing.
- Project README and `AGENTS.md` for the high-level firmware architecture and build instructions.
