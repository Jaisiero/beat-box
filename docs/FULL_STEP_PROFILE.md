# Full simulation-step profile

Measured on the RTX 4090 in LXC 110 at commit
`a638e580a9d30280b90c5130d86f9d4154f79c8b`.
This corrects the scope of the fracture-construction report: the dominant
construction kernels are not the dominant costs of an entire frame.

## Experiment

F10, AVBD and TGS, 1,800 deterministic steps each, 3840×2160 window.
Repeat each solver at render scale 1 and 0.5 (1920×1080 internal tracing).
Only `BB_FRAME_TIMING=1` is enabled; `BB_RESPAWN_TIMING` and the CPU
verification flags are disabled. All four runs complete, and changing render
scale preserves every DET state within each solver, with zero invariant errors.

The first 20 steps are excluded. Steps 21–600 capture the active pile and
fractures; 601–1799 show later behavior. The final step exits before the frame
summary. This is one diagnostic pair per solver, not a repeated FPS benchmark.

Raw logs, the runner and parsed results:
`/root/beat-box/work/full-step-profile/{run.py,run.log,f10-*.log,analysis.json}`.

## Active-pile costs at 4K

Means over steps 21–600, milliseconds.

| Span | AVBD | TGS |
|---|---:|---:|
| Entire CPU frame span | 9.867 | 11.970 |
| Before simulation: GUI/events/acquire and frame setup | 4.145 | 5.090 |
| CPU simulation phase, including recording and completion | 4.987 | 6.061 |
| CPU command recording/submission within that phase | 1.420 | 2.066 |
| CPU completion wait within that phase | 3.481 | 3.914 |
| GPU simulation, first-to-last solver graph timestamp | 2.971 | 3.707 |
| GPU path tracing, preceding render sample | 2.474 | 2.678 |

**These rows overlap and must not be summed.** The completion wait includes
simulation execution and any outstanding MAIN render dependency. The front
span includes acquisition/pacing, not only useful CPU execution. The GPU
simulation timestamps cover shared setup through finalization, excluding the
tiny trailing configuration-readback copy. Path tracing belongs to the
preceding render sample and can overlap CPU work.

The CPU recording span is substantial: approximately 1.4 ms for AVBD and
2.1 ms for TGS. It is not SDF construction time or a memory-transfer measurement.

GPU simulation breakdown:

| Stage | AVBD | TGS |
|---|---:|---:|
| Narrow phase, nested inside setup | 0.773 | 0.532 |
| Other shared setup, including broad phase and chain sort | 0.156 | 0.163 |
| Preparation: islands, sleeping, coloring and solver preparation | 0.165 | 0.196 |
| AVBD main solve / TGS substeps | 0.974 | 2.549 |
| AVBD velocity reconstruction, impact and post-stabilization | 0.703 | — |
| TGS contact preparation | — | 0.069 |
| Finalization | 0.201 | 0.198 |

AVBD main solve plus post work accounts for about 56% of its measured GPU
simulation span; narrow phase contributes 26%. TGS substeps account for about
69%, narrow phase 14%. Broad phase is contained in the small remaining setup
span; these measurements do not support prioritizing it above solve or
narrow phase.

## Render-isolation result

| Span | AVBD 4K → half scale | TGS 4K → half scale |
|---|---:|---:|
| GPU path tracing | 2.474 → 0.630 | 2.678 → 0.702 |
| GPU simulation | 2.971 → 2.983 | 3.707 → 3.695 |
| CPU simulation phase | 4.987 → 4.368 | 6.061 → 5.828 |
| Before-simulation span | 4.145 → 2.320 | 5.090 → 2.927 |
| Entire CPU frame span | 9.867 → 7.358 | 11.970 → 9.550 |

The physical states are identical. The GPU simulation span remains essentially
unchanged while frame time and the CPU simulation wait change. Therefore the
roughly 10 ms frame cannot be attributed wholly to physics kernels. These
results do not identify acquisition's exact internal cause or isolate every
driver cost.

## Later steps and fracture frames

At full render scale, AVBD's main solve drops to 0.155 ms and post work to
0.141 ms in steps 601–1799, while narrow phase remains at **0.961 ms**. It
accounts for 54% of the measured 1.796 ms GPU simulation span. This is a more
important persistent SDF cost than construction of new fields.

TGS remains at 2.766 ms in substeps, with 0.561 ms narrow phase and 4.003 ms
total GPU simulation. Its steady-state priority remains the solver.

Frames with publication spend about **1.02 ms AVBD / 1.05 ms TGS** in the CPU
scene-edit span, against about 0.007 ms in ordinary frames across the run.
There are 27 AVBD and 56 TGS publication frames. This span includes the whole
fracture/edit path and associated waits, not solely SDF construction. Publication
frames occur earlier in the run, so their whole-frame mean must not be compared
against the later settled pile as an isolated fracture overhead.

## Optimization order justified by these measurements

1. Solver execution: AVBD primal/post work during active motion, and especially
   TGS substeps. Preserve iterations, convergence and color dependencies.
2. CPU command recording/submission: repeated solver sweeps and indirect
   dispatch/barrier recording. The existing grouped-sweep implementation still
   records each color dispatch and its dependency.
3. SDF narrow phase, particularly AVBD's later 0.96 ms despite much cheaper
   solving. Contact reuse would need correct wake-up, topology-change and
   warm-start invalidation; no unchecked reuse is enabled by this report.
4. For frame throughput, path tracing and render/simulation scheduling also
   matter. A render-scale change is an isolation experiment, not a shipped
   reduction in visual quality.

No solver settings, rendering resolution defaults or synchronization behavior
are changed by this profiling follow-up.
