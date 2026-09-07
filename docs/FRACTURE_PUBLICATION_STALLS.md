# Fracture publication stalls

## Work removed from the critical path

Runtime publication regenerated voxel BLAS AABBs using one host-recorded dispatch per voxel body, including unchanged bodies whose primitive offsets moved in the dense layout. A large pool therefore recorded hundreds of dispatches for each fracture. The old shader also counted every preceding occupancy word again for each occupied cell.

`voxel_primitive_batch.slang` uses one dispatch, one 64-thread group per body. Each group visits 64-cell tiles and maintains its solid-count prefix. Within a tile, each lane derives its rank from two occupancy words and the preceding-bit popcount. The destination is still the canonical ascending occupancy-bit rank. No atomics, group barriers, truncation, contact changes, or solver-budget changes are involved. Grid dimensions, origin and primitive offsets come from published GPU body/shape data after finalization and layout barriers.

Initial scene loading retains the original per-body path. With `BB_FRAGMENT_VERIFY`, runtime batches are compared byte-for-byte against that independent GPU implementation, including non-contiguous occupancy and fragment offsets. These diagnostic allocations/readbacks do not run in production.

Runtime `build_AS(true)` enqueues its work and leaves completion to the renderer's existing final scene-edit synchronization. Initial loading uses the synchronous default. This removes the intermediate per-publication host drain, not all CPU/GPU synchronization.

## Lifetime requirements

- The AS graph publishes both body parities and carries their queue history to COMPUTE_0 consumers. Its GPU copy/build dependencies remain intact.
- Every SimConfig upload owns an immutable staging slot. Slots cannot be reused until the renderer completes all queues at the final scene-edit boundary. This prevents a second publication from modifying input still consumed by the first. The pool grows to the maximum simultaneous upload count and is reused; buffers are owned and destroyed with RigidBodyManager.
- A later cull/spawn transaction submits on MAIN after prior AS work and waits for its manifest before the CPU rewrites AS staging or retires handles. Earlier frame tracing has also completed before that point. Its GPU body-list readers are ordered against subsequent AS copies through shared task buffers.
- The final frame completion boundary remains before in-place TLAS rebuild and tracing. Children are not exposed to tracing before their geometry and AS inputs are ready.
- Profiling retains a query slot for each outstanding publication and collects it after that boundary. The old single query pool could not be reset safely for multiple uncollected publications.

## Timing interpretation

`[AS-PUBLISH] deferred=1` means `host_wait_ms` is zero at the publication call. A completion wait still exists later in the frame. Legacy `[RESPAWN-MS] total` and `[FRACTURE-MS] publish` now measure enqueue-side CPU spans, not completed publication latency, and must not be used alone to claim an end-to-end speedup.

The acceptance comparison uses unprofiled, validation-disabled F11 replays at 3840x2160, with identical state sequences and frame intervals recorded externally between flushed DET lines. Frames containing a `[FRACTURE] respawn` are classified separately. These intervals include application rendering/pacing, not Moonlight network/decoder latency. They use fixed-step replay rather than normal catch-up pacing. First 20 steps are excluded. Two alternating pairs per solver limit run-order bias; complete raw logs and per-frame samples are preserved.

Evidence: `/root/beat-box/work/batched-voxel-publication/` on LXC 110. `batch-only-*` measurements are the intermediate batching-only experiment; final measurements include deferred completion and immutable staging. The isolated baseline is merged PR #35 (`4da5adf`, source equivalent to `e4c2097`).

## Results and limits

Two alternating F11 pairs per solver at 3840x2160, without profiling or validation, give these averages of per-run statistics (milliseconds):

| Metric | AVBD baseline | AVBD candidate | TGS baseline | TGS candidate |
| --- | ---: | ---: | ---: | ---: |
| Publication-frame mean | 11.337 | 10.748 | 12.734 | 12.413 |
| Publication-frame p95 | 14.677 | 14.620 | 15.993 | 15.794 |
| Publication-frame maximum, average of run maxima | 17.446 | 16.121 | 21.262 | 19.474 |
| All-frame mean | 10.372 | 10.374 | 11.776 | 11.882 |

Publication-frame means improve by 5.2% and 2.5%, respectively. The p95 improvement is small. The overall mean does not improve (TGS is about 0.9% higher in these samples); these runs do not establish a general FPS gain or complete elimination of user-visible stalls. Maximum observations are not latency guarantees. No solver iterations, fracture events, or physical steps were dropped to achieve these results.

## Validation

All eight CTest tests pass. The final candidate matches existing reference states for F10 (480 steps), F11 recycling (1500) and the F3 fractured frame (900), both solvers, in production and GPU-oracle modes under Vulkan synchronization validation: 11,520 steps. In-process reset adds 1,920 matching steps; full-AS fallback adds 960. The unprofiled 4K pairs add 6,000 candidate steps matched against 6,000 baseline steps. Total final-candidate exact replay coverage: 20,400 steps.

The primitive oracle compares the new GPU output byte-for-byte with the independent previous implementation during runtime publications. Normal 30-second 4K F11 runs pass for both solvers with synchronization validation and AS profiling, completing 208 publications without reported validation errors or NaNs. Every publication has its profiling sample collected, including multiple publications in one frame. These checks do not replace a per-pixel visibility oracle or a reproduction of the user's exact drag-and-release sequence.
