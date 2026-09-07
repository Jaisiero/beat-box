# Colored solver workgroup sizing

Baseline: merged PR #38 (`c18869a`), RTX 4090 at 3840x2160.

Colored solver dispatches previously inherited the 32-thread group size used by
integration and other simulation passes. They scan a body/manifold grid but only
process the current color. The remaining active lanes execute substantial
per-body/contact work. The best tested group size for these passes was four.

`color_solve_dispatch.slang` couples the shader group size with indirect argument
generation. It applies to AVBD primal sweeps (including its post-stabilization
cascade) and the PGS/TGS colored prepare/solve/relax kernels. Integration, color
assignment, overflow and other passes keep their original sizes. The shared
CPU/GPU layouts and C++ command recording do not change.

## Correctness

Global invocation IDs still visit the same body/manifold indices. The full grid
and color filters remain intact. Contacts/bodies within a color are independent;
existing barriers preserve the order between colors, sweeps and substeps. The
indirect counts use the same constant as `numthreads`, including cascade counts.
Iteration counts, contact data, floating-point expressions and overflow ordering
are unchanged. There are no additional buffers, passes or CPU/GPU transfers.

## Candidate selection

A GPU compact-contact-list experiment was rejected. Its initial 32-thread groups
increased TGS substeps from 2.67 to 3.70 ms and unprofiled F10 mean frame time from
11.43 to 12.97 ms. Groups of 8, 4 and 1 recovered performance, but the best compact
variant offered little advantage over tuning the original full-grid kernels.
The final patch contains none of that list construction or allocation code.

Full-grid TGS trials with groups of 1, 4, 8 and 32, and AVBD trials with groups of
4, 8, 16 and 32, preserved exact replay states. Four-thread AVBD groups reduced
its sampled main phase from approximately 1.14 to 0.97 ms and post-stabilization
from 0.80 to 0.70 ms. This is consistent with better latency hiding for sparse,
heavy shader invocations; hardware occupancy counters were not collected.
These group sizes are measured on this GPU, not a universal hardware optimum.

## Final comparisons

Measurements and raw logs are stored under
`/root/beat-box/work/solver-dispatch/`. `baseline-runtime` is an isolated copy of
merged PR #38. `final-frames-*` contains alternating unprofiled 4K runs using
`tools/benchmark_sdf_frames.py`; `final-profile` contains separate timestamped
runs. Each benchmark uses 600 steps and excludes the first 20. Frame intervals
include rendering/publication but do not measure scanout or Moonlight latency.

Two alternating pairs per scene/solver. Means and p95 values average the two
per-run statistics; maximum is the worst observed interval across both runs.

| Scene | Solver | Mean before → after | p95 before → after | Worst before → after |
|---|---|---:|---:|---:|
| F10 breakable pool | AVBD | 9.705 → 9.019 ms | 13.432 → 12.671 ms | 17.542 → 17.232 ms |
| F10 breakable pool | TGS | 11.541 → 11.252 ms | 14.500 → 14.065 ms | 17.360 → 17.078 ms |
| Timber drop fixture | AVBD | 6.713 → 6.567 ms | 11.309 → 11.056 ms | 15.503 → 14.333 ms |
| Timber drop fixture | TGS | 8.848 → 8.566 ms | 12.973 → 12.615 ms | 16.109 → 15.526 ms |
| F7 box pool | AVBD | 7.609 → 7.330 ms | 11.274 → 11.056 ms | 15.068 → 12.985 ms |
| F7 box pool | TGS | 9.729 → 9.733 ms | 13.287 → 13.292 ms | 16.683 → 16.727 ms |
| F11 recycling | AVBD | 7.133 → 7.012 ms | 10.986 → 10.806 ms | 14.623 → 13.405 ms |
| F11 recycling | TGS | 8.977 → 8.708 ms | 13.289 → 12.888 ms | 15.382 → 15.354 ms |

AVBD mean improvements range from 1.7% to 7.1%. TGS improves 2.5%–3.2% in the
SDF scenes; F7 is effectively unchanged (mean +0.004 ms, p95 +0.005 ms). These
runs do not prove that every individual frame improves. F10 publication-frame
means fall from 11.850 to 10.644 ms for AVBD and 12.779 to 12.406 ms for TGS.

Separate F10 GPU samples (first 20 steps excluded) confirm the targeted reduction:

| GPU phase | Before | After |
|---|---:|---:|
| AVBD main solve | 1.143 ms | 0.971 ms |
| AVBD post-stabilization | 0.799 ms | 0.701 ms |
| TGS substeps | 2.647 ms | 2.548 ms |

The remaining costs include CPU solver command recording, other simulation
stages, path tracing and frame acquisition. These overlapping CPU/GPU spans must
not be summed into a frame budget.

Validation of the final patch:

- 8,280 candidate steps exactly match the original with Vulkan synchronization
  validation: timber, F10, F11, F5 and F7, using TGS, PGS and AVBD.
- 9,600 further candidate steps exactly match the baseline and repeats in the
  unprofiled benchmarks; 1,200 more match in the GPU profiling comparisons.
- All eight CTest tests pass.
- Normal-mode 4K F11 completes 99 AVBD and 118 TGS fragment publications with
  profiling and Vulkan synchronization validation, without validation or
  invariant errors.
