# Frame pipeline bottlenecks and CPU recording overlap

After merged PR #37 (`ae98244`), profile the whole frame rather than treating all
simulation-phase CPU time as solver execution. F10 at 3840x2160 on the RTX 4090,
30-second normal-mode runs, first 20 stepped frames excluded:

| Mean active-frame cost | AVBD | TGS |
|---|---:|---:|
| CPU wait before recording the step | 1.50 ms | 1.91 ms |
| CPU simulation command recording/submission | 1.37 ms | 2.12 ms |
| GPU solver stages, including narrow phase | 3.29 ms | 4.09 ms |
| GPU narrow phase (included above) | 0.82 ms | 0.55 ms |
| GPU path tracing | 2.43 ms | 2.58 ms |
| GPU final TLAS build | 0.10 ms | 0.11 ms |

These are overlapping CPU/GPU spans, not additive frame-budget components. AVBD
enters full-sleep stasis sooner: 599 measured active frames versus 1,774 for TGS.
The table prioritizes work within each run; it is not an equal-trajectory solver
speed comparison. The timestamp queries themselves are diagnostic overhead.

The next priorities are solver work/command overhead (particularly TGS substeps)
and path tracing. The routine TLAS build is substantially smaller. CPU preparation
also matters, and can overlap prior rendering without running the solver against
buffers that rendering is still reading.

## Change

Previously each step called `device.wait_idle()` before recording its solver graph.
Now a wait-only COMPUTE_0 submission depends on the latest MAIN queue timeline
value, followed by the existing solver graph submission on COMPUTE_0. In the
installed Daxa implementation, `wait_queue_submit_indices` becomes a Vulkan
timeline semaphore wait at `ALL_COMMANDS`. Subsequent compute commands remain
ordered after rendering, with the semaphore memory dependency intact. The CPU
can record those commands while the previous render finishes.

There is no new GPU overlap between tracing and simulation. The optimization is
CPU recording overlapping GPU work. The submit is deliberately outside the task
graph, so correctness does not depend on resource history surviving parity
rebinding or optional task-graph submit hooks.

Retained completion boundaries:

- Simulation completion before CPU consumption of SimConfig and fracture events.
- Fracture manifest waits before CPU AS metadata/handle retirement.
- The final frame synchronization before publication timing collection, upload
  slot reuse and host TLAS bookkeeping.
- Existing scene-load, reset, resize and shutdown lifetime rules.

No shader code, physics settings, iteration counts, buffers or contact rules change.

## Unprofiled frame comparison

Use `tools/benchmark_sdf_frames.py` with complete old/new runtime directories.
The benchmark verifies 4K geometry and exact state equality. Each run has 600
steps; the first 20 are excluded. F10 uses four alternating pairs per solver;
the timber drop fixture uses two. Values average the per-run statistics.

| Scene | Solver | Mean frame before → after | Full-frame p95 before → after |
|---|---|---:|---:|
| F10 pool | AVBD | 9.939 → 9.474 ms | 13.290 → 13.293 ms |
| F10 pool | TGS | 11.667 → 11.518 ms | 14.347 → 14.368 ms |
| F9 timber drop fixture | AVBD | 7.360 → 6.582 ms | 10.617 → 11.053 ms |
| F9 timber drop fixture | TGS | 9.306 → 8.843 ms | 13.009 → 12.949 ms |

Mean improvements are 4.7% / 1.3% in F10 and 10.6% / 5.0% in the timber fixture.
Tail latency does not improve uniformly: timber TGS publication-frame p95 rises
from 14.926 to 15.927 ms (only ten publications per run), and timber AVBD's
full-frame p95 rises by 0.44 ms. This is an average-throughput improvement, not
proof that every slow frame is fixed. The raw logs retain these outliers.
Intervals are externally timed between flushed DET records; they include
rendering and publication but are not scanout or Moonlight latency measurements.

## Diagnostics and validation

`BB_FRAME_TIMING=1` adds:

- `sim_order_ms`: CPU cost of establishing the render-to-simulation dependency.
- `sim_submit_ms`: CPU recording/submission of the solver graph.
- `sim_wait_ms`: CPU wait for compute completion, including outstanding render
  work required by the queue dependency.
- `[RENDER-GPU] trace_ms`: a completed sample of the preceding ray-tracing work.

A render query pool is reused only after the simulation completion boundary
covers its producer. Frames between sampled steps skip new queries while one is
pending. Reading query availability alone would not be sufficient: an old result
could still be available before a newly submitted reset executes. No extra CPU
completion wait is added for profiling. The legacy `[PERF] sim` remains a CPU
submission/completion span and now includes queued render dependencies; use GPU
stage timestamps to measure shader execution itself.

- 4,080 candidate steps exactly match the baseline with Vulkan synchronization
  validation enabled: timber fracture, F10, F11 recycling and F5, both solvers.
- 7,200 further candidate steps exactly match the baseline and repeats in the
  unprofiled 4K benchmarks.
- Normal-mode 4K F11 tests with profiling and Vulkan synchronization validation
  complete 93 AVBD and 118 TGS publications without validation or invariant errors.
- All eight CTest tests pass.

Test-host artifacts: `/root/beat-box/work/frame-pipeline/`. The `baseline-runtime`
contains merged PR #37; `instrumented-runtime` has the old host wait plus the
new profiler. `frames-10`, `frames-10-confirm` and `frames-3` hold raw frame samples.
