# Root-free FOC voltage limit

`FocControlCore` uses `PolygonVoltageLimit` in both float and fixed profiles.
The current PI outputs are voltages. The limiter sits before inverse Park/SVPWM
and feeds the applied-minus-requested correction back to the PI integrators.
The outer speed controller receives the same actual-clipping saturation flag.

## Circumscribed 32-gon

Let `L` be the inradius and `x = abs(Vd)` after clamping `Vd` to `[-L,L]`.
The positive q headroom is

```text
q_limit = min(k=0..7) [sec(k*pi/16)*L - tan(k*pi/16)*x]
Vq = clamp(Vq_requested, -q_limit, q_limit)
```

The axis-aligned faces are tangent to the original circle. In exact arithmetic
the polygon contains that circle and extends its radius by at most
`sec(pi/32)-1 = 0.48385724%`. This is a radial bound, not a relative bound on
the added q headroom: at `abs(Vd)=L`, q can still reach `L*tan(pi/32)`, about
`0.0984914*L`. The limiter retains d priority and both voltage signs.

The polygon is fixed in **rotating dq coordinates**. It is not the stationary
alpha/beta inverter hexagon and does not implement overmodulation or a
transition to six-step operation. Constant attainable Vd/Vq still generate
sinusoidal differential phase voltages.

## Finite arithmetic and cost

- Coefficients are constants: no runtime trig, root, or division in the limiter.
- Fixed-point tangent intercepts/slopes use directed Q28 rounding. Q30 fractions
  cannot represent the largest coefficients (approximately 5.13 and 5.03).
  Products remain wide until the final Q18 floor; their full-range bound is
  below `2^62`.
- Binary face selection uses rounded-down adjacent-line crossings. Taking the
  minimum of the selected and preceding faces recovers the exact quantized
  envelope, including tiny limits whose crossings share an integer count.
  The crossing predicate must be strict `>`.
- An interior diamond accepts common unsaturated requests directly. A flat-face
  shortcut handles small d. These shortcuts preserve the full polygon law.
- Fixed q headroom is rounded inward relative to the ideal polygon. The deficit
  is less than `1 + 2*L.raw()/2^28` Q18 counts: less than 17 counts over the entire
  representable positive range, and less than 1.04 counts at L=20 V.
- Float calculations use only single precision. Adjacent faces account for
  crossing rounding; tests permit `2e-6*L` geometric error in either direction.
- The bus-derived inradius and reciprocal remain cached. Face products are
  evaluated on demand: preparing every face on each changed bus sample made
  the cache-miss path slower than the original root in the measured ARM fixture.

Wide PI requests are compared with the voltage bound **before narrowing**.
Saturation means that at least one output was actually clamped. Points between
the old circle and the new polygon therefore do not trigger false anti-windup.
The existing back-calculation uses the actual applied output; fixed corrections
retain the voltage-request representation's resolution.

## Programmed ceiling and physical duty span

The configured `max_modulation` supplies an optional operating ceiling. Current
feedback does not remove the inverter's physical voltage limit: independent
duty clipping would otherwise apply a different dq voltage than the PI expects.

To fit every polygon corner at every rotor angle, the effective inradius is

```text
span = max_duty_cycle                   // low-side-clamped PWM
span = 2*max_duty_cycle - 1              // centered PWM
m = min(max_modulation, span*cos(pi/32))
L = max(0, m*Vbus/sqrt(3))
```

The cosine is constant and `m` is prepared when parameters change. For example,
with 95% maximum duty, the inradius modulation ceiling is about 94.5425% in
low-side-clamped mode or 89.5666% in centered mode. A configured 20% ceiling
already has enough reserve and stays 20%. The final duty clamp remains a
backstop for finite-arithmetic rounding.

This change does not affect current cutouts, Hall supervision or timing guards.
The exact square root for scalar modulation telemetry remains in the slow
controller update, outside the PWM current-loop body.

## Verification

```sh
bash tests/test_voltage_limit/run.sh
bash tests/test_profile_math/run.sh
bash tests/test_foc/run.sh float
bash tests/test_foc/run.sh fixed
bash tests/test_control/run.sh float
bash tests/test_control/run.sh numeric-fixed
```

The limiter suite checks independent trigonometric half-planes, rational fixed
envelopes, tiny/full-range inputs, vertex neighbors, all signs, pre-narrowing
saturation, actual PWM voltage reconstruction and anti-windup. It runs under
ASan/UBSan. When ARM GCC and Python `unicorn`/`pyelftools` are available, it also
executes the compiled Thumb-2 limiter against an arbitrary-precision oracle and
rejects roots, division and soft-float arithmetic in that probe. Set
`LIBECU_TEST_ARM_PYTHON` to select that Python environment.

On the 2026-11-05 comparison fixture, executed ARM instruction counts were:

| Path | Circular | Polygon |
|---|---:|---:|
| Saturated limiter, maximum in sweep | 129 | 101 |
| Interior limiter fixture | 47 | 23 |
| Complete FOC fixture, cache-hit maximum | 622 | 597 |
| Complete FOC fixture, changed-bus maximum | 681 | 659 |

These are executed instruction counts for those fixtures, not hardware cycles
or a whole-firmware WCET bound. Board-specific measurements belong in the port's
hardware test journal.
