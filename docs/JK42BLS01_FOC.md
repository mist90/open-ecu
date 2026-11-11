# JK42BLS01 FOC defaults

With `MOTOR_1` undefined, `motor_config.hpp` selects the eight-pole JK42BLS01.
The defaults below were qualified on STM32G431 with the float control profile
at approximately 20 V on 2026-11-11:

| Setting | Default |
|---|---|
| Algorithm / electrical mode | FOC / current |
| FOC electrical angle | +120 degrees |
| Effective phase resistance / inductance | 1.6 ohm / 1.7 mH |
| Current-loop design bandwidth | 500 Hz |
| FOC Kp / Ki | 5.340708 V/A / 5026.548 V/(A*s) |
| Current-mode speed Kp / Ki | 0.025 / 0.025 |
| Six-step current Kp / Ki | 0.15 / 200, independently tuned |

The shared main derives the FOC gains as `Kp = 2*pi*f*L`, `Ki = 2*pi*f*R`.
`BLDC_SIX_STEP_USE_MOTOR_MODEL=false` selects the separately tuned six-step
PI. The inverse motor/Hall alignment requires +120 degrees; the scooter's
-60-degree offset produced reverse acceleration in a voltage-mode check.
Feed-forward remains disabled because back-EMF has not been identified.

## Model provenance

The [manufacturer's JK42BLS01 drawing, page 1](https://static.jkongmotor.com/JK42BLS01+outline-aidlrBprKrkloSRrlrnkrjikn.pdf)
specifies **1.8 ohm phase-to-phase resistance**: nominally 0.9 ohm per phase
in the balanced terminal-equivalent star model. No verified inductance was
found for this exact model.

Biased voltage steps with a stationary stator field were captured through the
real 20 kHz ADC/OSC path. The fitted electrical time constant was 1.029–1.089 ms;
voltage/current increments gave an effective resistance of 1.584–1.621 ohm
and inductance of 1.630–1.765 mH. The rounded 1.6 ohm / 1.7 mH values describe
the observed drive, including inverter and voltage/current scaling effects.
They are **not bare-winding datasheet values or an independent LCR measurement**.
The difference from nominal resistance has not been independently resolved.

## Hardware qualification

- Current steps: 0.2 to 0.4 A, 0.2 to 0.6 A, and 0.6 to 0.2 A.
- Fitted 95% response time: 0.85–0.90 ms; steady tracking-error RMS: 0.020–0.021 A.
- Forward velocity points: 2, 5, 10 and 20 RPS; reverse: 5 and 20 RPS, including
  command-to-zero stops. The original speed PI overshot on startup/transitions;
  the 0.025 / 0.025 tuning completed this sweep.
- At 2 RPS, speed SD was approximately 0.88 RPS: mean tracking does not imply
  smooth low-speed rotation.
- Full current IRQ: maximum 23 us in 120,000 sampled IRQs while rotating,
  with OSC enabled and no overruns against the 50 us period. No Hall or ADC
  acquisition faults were observed.

Current accuracy is measured by the controller's own feedback, not an external
current probe. These are bounded unloaded float-profile tests, not qualification
of the full rated speed/current range or fixed-profile current-mode operation.
The drive boots in NEUTRAL; always finish bench work with `AT+DMODE=2` and `AT+SPD=0`.
