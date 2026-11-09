/** @file pwm_timing_bench.hpp
 * @brief SWD-accessible bounded timing capture and optional synthetic sensor inputs
 */
#ifndef OPEN_ECU_F410_PWM_TIMING_BENCH_HPP
#define OPEN_ECU_F410_PWM_TIMING_BENCH_HPP
#include <cstdint>
extern "C" {
enum : uint32_t { F410_TIMING_DRIVE_ONLY = 1 };
/** @brief Clear fields, then write remaining LAST to arm a bounded capture. */
struct F410TimingCapture {
    uint32_t remaining;
    uint32_t samples;
    uint32_t total_cycles;
    uint32_t max_cycles;
    uint32_t min_cycles;
    uint32_t overruns;
    // The first six words retain the original SWD ABI. All totals are uint32:
    // use bounded captures (e.g.200000 frames/10 s at20 kHz) to avoid overflow.
    uint32_t flags;
    uint32_t skip_frames; // Counts only selected frames; permits settling without live SWD.
    uint32_t entry_total_cycles;
    uint32_t entry_max_cycles;
    uint32_t entry_min_cycles;
    uint32_t peak_total_cycles;
    uint32_t peak_max_cycles;
    uint32_t peak_min_cycles;
    uint32_t paired_frames; // Current-only DMA frames containing two physical triples (no x1 fallback).
};
static_assert(sizeof(F410TimingCapture) == 60, "Update the F410 timing decoder");
extern volatile F410TimingCapture f410_timing_capture;
#ifdef PWM_TIMING_BENCH
/** @brief Synthetic raw inputs; the ADC still supplies the real 20 kHz IRQ clock. */
struct F410BenchInputs {
    uint32_t phase_adc[3];
    uint32_t bus_adc;
    uint32_t hall_position;
    uint32_t hall_direction; // 1=forward sequence, 5=reverse sequence
};
extern volatile F410BenchInputs f410_bench_inputs;
#endif
}
#endif
