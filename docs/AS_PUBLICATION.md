# Acceleration-structure publication

## Redundant work removed

`build_AS()` previously ran a graph that copied bodies into the current and next simulation buffers, built changed BLAS, and built a TLAS. It then submitted another pair of full body copies. Both scene loading and runtime fracture publication subsequently called `update_TLAS()`, which refreshed the GPU instance transforms and built the final TLAS before rendering. The intermediate TLAS was never traced.

Publication now performs the existing scratch-to-current-to-next body copies and builds changed BLAS once. The extra copy submission and intermediate TLAS build/size preparation are removed. The final instance-update and TLAS-build graphs remain separate, retaining their shared-resource dependency across COMPUTE_0 submissions.

The fixed-capacity per-parity TLAS objects allocated in `create()` are rebuilt in place instead of being destroyed and recreated by every `update()`. This remains a full BUILD, not an UPDATE/refit. Existing AS/scratch size checks enforce the 1 MiB capacities before each build. Scene reset changes instance counts and rebuilds the same objects; teardown still destroys them.

## Synchronization and ownership

Runtime publication now defers its host completion wait to the renderer's existing final scene-edit boundary. Initial loading keeps synchronous completion before its immediate TLAS update. Both body parities are still written by the AS graph, so subsequent COMPUTE_0 body-list tasks inherit the MAIN dependency. The final instance/TLAS graphs and all GPU barriers remain.

Each SimConfig upload uses a separate immutable staging slot. Slots are recycled only after the renderer's device completion boundary; they are owned by the rigid-body manager and released at teardown. A second publication cannot overwrite an earlier upload still being read by COMPUTE_0. Later cull/spawn manifest waits complete preceding MAIN AS work before CPU handle retirement or staging reuse. See `FRACTURE_PUBLICATION_STALLS.md` for the follow-up and timing methodology.

Both callers still update the final TLAS before tracing, including paused scene loading and the full-AS diagnostic fallback.

## Instrumentation

`BB_AS_TIMING=1` adds:

- `[AS-PUBLISH] record_ms`: CPU graph recording/submission time.
- `host_wait_ms`: CPU time inside the existing device completion wait; not a PCIe bandwidth measurement.
- `copy_gpu_ms`: GPU timestamps spanning primitive/body upload and the next-parity copy.
- `blas_gpu_ms`: GPU timestamps around changed BLAS construction.
- `blas_count`: number of build records in that publication.
- `[AS-TLAS] gpu_ms`: GPU time around the final TLAS build, read on the next update when available. The last sample at shutdown may not be emitted.

Publication timestamp slots are retained until the existing completion boundary, allowing multiple publications in one frame without overwriting pending queries. `deferred=1` identifies an asynchronous runtime publication; its `host_wait_ms=0` does not mean the later frame has no wait. Final TLAS queries are read without adding a completion wait. Timing is opt-in and does not change solver dispatches. The legacy `[RESPAWN-MS] blas_gpu` field still describes the host-observed `build_AS()` span; use the new field for actual BLAS GPU time.

## Historical PR #35 paired benchmark

RTX 4090, Release build, F10, 480 steps, 860x640. Three alternating baseline/candidate pairs per solver. Baseline is merged PR #34 (`f76cc4a`, same source as `b8476a7`) with equivalent timing instrumentation. The first standalone instrumentation run overlapped a later CPU build and is not used in the accepted comparison. Accepted paired runs had no concurrent build or simulation. Both variants enabled `BB_AS_TIMING` and `BB_RESPAWN_TIMING`; the latter includes its existing SDF timing wait.

Medians of the three per-run means, milliseconds:

| Metric | AVBD baseline | AVBD new | TGS baseline | TGS new |
| --- | ---: | ---: | ---: | ---: |
| Successful publication, host total | 1.659 | 1.414 | 1.740 | 1.485 |
| Publication p95, median per-run p95 | 2.041 | 1.671 | 2.078 | 1.665 |
| Entire fracture event handling, host | 1.731 | 1.478 | 1.887 | 1.596 |
| AS recording/submission, CPU | 0.126 | 0.111 | 0.114 | 0.087 |
| Existing publication wait, CPU | 0.694 | 0.620 | 0.734 | 0.649 |
| Changed BLAS construction, GPU | 0.066 | 0.066 | 0.066 | 0.066 |
| Final TLAS construction, GPU | 0.100 | 0.100 | 0.102 | 0.102 |

Successful publication improves by about 14.7% in each solver. These are publication timings, not full-frame/FPS gains. Nested phases must not be added together. AVBD has 26 successful publications per replay and TGS 56; they evolve different physical workloads. All twelve paired runs match every replay state against their solver's reference. The eliminated intermediate TLAS cost about 0.10 ms of GPU time; the duplicate body copies about 0.006 ms.

Reproduce on the GPU host, with the application stopped:

```sh
DISPLAY=:0 python3 tools/benchmark_as_publication.py \
  --baseline work/as-publication/instrumented-runtime \
  --candidate build/Release --output work/as-publication/perf --profile
```

The runner writes raw logs and summary JSON, checks complete replays and exact baseline equality, and excludes initial loading from AS publication statistics. Process wall time includes initialization and must not be interpreted as frame time.

## Historical PR #35 validation evidence

Evidence is retained at `/root/beat-box/work/as-publication/` on LXC 110. The production synchronization-validation replays cover F10 (480), F11 recycling (1500), and the F3 fractured frame (900), for both solvers, matching their existing baselines. All eight CTest tests pass. GPU oracles for F10/F11 match production (3,960 more steps). Both solvers also match the F10 reference through an in-process reset (1,920 steps) and the full-AS rebuild fallback (960 steps). Including the 5,760 paired benchmark steps, this is 18,360 checked replay steps.

Two normal 30-second F11 runs at 3840x2160, one per solver with synchronization validation and AS profiling enabled, complete 209 runtime publications without reported validation errors or NaNs. This does not replace a pixel-by-pixel visibility oracle for transient missing fragments.
