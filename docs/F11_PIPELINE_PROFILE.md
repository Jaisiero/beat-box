# F11 accumulated-fragment pipeline profile

Baseline: `70caa6d8`, RTX 4090, 3840x2160, AVBD and TGS. F11 defaults retain settled debris (kill plane -20, spawn every 45 stepped frames). The recycling benchmark's `BB_KILL_Y=.4` does not represent this accumulation workload.

Two workloads were measured with `BB_FRAME_TIMING=1` and `BB_AS_TIMING=1`, without geometry verification or `BB_RESPAWN_TIMING` (which adds a host wait when profiling SDF construction):

- Normal cadence, 7,200 deterministic steps per solver.
- Accelerated accumulation, spawn every 5 steps, 3,600 steps per solver. This is deliberately heavier than default F11.

Normal cadence grows from about 4–5 ms to 7–8 ms per measured CPU frame. Final body high-water counts are 332 AVBD and 353 TGS. TLAS remains approximately 0.09 ms; solve and contacts grow with retained debris.

## Heavy accumulation

Means over steps 1801–3599. GPU scopes and host scopes overlap; do not add rows indiscriminately.

| Scope (ms) | AVBD | TGS |
|---|---:|---:|
| GPU narrow phase | 1.043 | 0.771 |
| GPU main solve | 2.895 | — |
| GPU post work | 2.164 | — |
| GPU TGS substeps | — | 4.746 |
| Previous GPU trace sample | 4.575 | 4.784 |
| GPU TLAS build | 0.111 | 0.132 |
| GPU BLAS build, publication samples only | 0.075 | 0.075 |
| CPU scene edits, all frames | 0.089 | 0.132 |
| CPU device completion span | 0.496 | 0.517 |
| Whole CPU frame | 12.334 | 12.185 |

Final body high-water counts: 839 AVBD, 1,024 TGS. These are not necessarily awake/live-body counts. The BLAS statistic does not include partitioning, SDF construction or the whole publication path.

Distinct from sustained solve/render cost, both solvers show large completion spikes at steps 1012, 2036 and 3060. The existing device-wide idle takes 8.2–9.8 ms at those points, including frames with no scene edits. The largest observed frames reach 22 ms. This locates the host stall; its periodic internal driver/presentation cause is not established by these measurements.

Evidence and runners: `/root/beat-box/work/f11-profile/` (`f11-{2,3}.log`, `stress-{2,3}.log`). Deterministic replay forces one step per frame; it does not measure real-time catch-up amplification.

## Guarded completion fast path

The renderer records the MAIN submission ordered before the step and the COMPUTE_0 submission completed by its existing host wait. After processing scene edits, it skips the additional device idle only when both submission indices are unchanged, a completed simulation snapshot exists, and no scene update is pending. New publications and non-stepped updates retain the previous device completion boundary. All task-graph barriers and render/compute dependencies remain unchanged.

At step 1012, before either original baseline/candidate trajectory differs:

| CPU span (ms) | AVBD baseline | AVBD candidate | TGS baseline | TGS candidate |
|---|---:|---:|---:|---:|
| Additional completion | 8.229 | 0.0001 | 9.774 | 0.00008 |
| Whole frame | 17.267 | 9.092 | 22.318 | 12.104 |

This demonstrates removal of that redundant host stall, not an equivalent gain in GPU solving. Sustained solver and trace cost remains. Publication frames still take the guarded completion path.

## Reproducibility limitation

The initial 3,600-step heavy replays first differ between baseline and candidate at AVBD step 2046 and TGS step 1093. A fresh **unmodified baseline** AVBD replay itself differs from the original baseline at step 1778; the candidate also differs from its earlier run at step 1649. Therefore long heavy-F11 hashes are not presently a reliable standalone regression gate. The candidate's 2,200-step synchronization-validation replay completes without validation errors or invariant violations, but that does not prove absence of every race. No deterministic/solver settings were changed to hide this limitation.

Do not interpret full-run performance differences after state divergence as a strictly state-matched A/B. The table above uses a step in the original matching prefix. Further work should isolate the pre-existing heavy-scene reproducibility issue and reduce sustained solve/render cost separately.

Regression coverage: exact 600-step baseline/candidate replays for F10 and F7 with AVBD/TGS, and F9 with PGS/soft PGS (7,200 records across 12 runs). All eight CTest targets pass.

Normal real-time accelerated F11 also completed 584 stepped frames with synchronization validation enabled, including minimize/restore, without validation errors. This exercises the real-time loop as a smoke test, not a state-matched performance comparison.
