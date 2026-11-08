# Hall/PLL parameter publication

Run `./tests/test_parameter_publication/run.sh`; set `BUILD_DIR` to retain all
temporary build activity under a chosen artifact directory.

The tests compile the actual Hall/PLL wrappers and controller in float/fixed at
8, 16 and 20 kHz. They verify:

- coefficient equivalence, including independent fixed-point rounding and Hall
  persistence ceil boundaries;
- rejected preparation leaves the output, active parameters and live state intact;
- preparation is state-independent, and config-only publication preserves every
  ISR-owned field changed in between (including sub-LSB state and fault latches);
- re-publishing PLL gains preserves its exact future trajectory;
- an ISR injected immediately before the controller's interrupt mask is retained;
- invalid controller parameters do not enter a critical section.

A separate narrow-`PllStepGain` profile exercises the wrapper's independent step
coefficient range check; only Hall/PLL wrappers are linked for that case. It does
not claim support for that synthetic profile in the full motor controller.

The controller materializes each prepared bundle using a compiler memory input
before `CriticalSection`, preventing LTO from moving coefficient calculation
inside the mask. Target disassembly is the check for generated commit code;
these host tests do not measure interrupt-mask duration on hardware.
