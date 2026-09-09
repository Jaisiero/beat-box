# Batched GPU scene edits

Retirement and spawning now share one GPU dispatch, one compact result readback, and one AS/body publication when both are requested in the same stepped render frame. Retirement runs first so a full body pool can immediately reuse a retired ID. The host mirrors retirement before spawning; duplicate dirty IDs are coalesced by the AS dirty mask.

The manifest stores retired IDs separately from the optional spawned ID. A refused spawn preserves successful retirements. An empty operation clears the manifest. Static bodies and pinned template shapes are preserved.

## Synchronization

Fracture and scene-edit recording no longer wait on MAIN before submitting work. Initial READ_WRITE-to-compute barriers order edits after previous MAIN readers, including tracing. The existing simulation completion is retained. The final compute-to-host barrier and submission wait remain: the CPU must finish preceding readers before retiring or reusing AS resources. Reset, scene-switch and AS completion protections are unchanged.

When retirement and spawning coincide, the scene-edit controller uses one result wait instead of two preliminary waits plus two result waits. AS-internal waits are excluded from this count. Fracture batches remain separate from this combined operation.

## Validation

On RTX 4090, synchronization validation enabled:

- All eight CTest tests passed.
- F11: 1,500 steps per solver (AVBD and TGS), both GPU-oracle and production modes; all four traces exactly match the merged PR #30 baseline.
- F10: 480 production steps per solver, exactly matching PR #30. Total new replay coverage: 6,960 steps.
- A 30-second F11 run at 3840x2160 exercised the normal render loop with AVBD and synchronization validation, completing 1,024 simulation steps without validation errors.
- A real GPU oracle checks full-pool spawn refusal, retire-and-reuse in one dispatch, static preservation, private-shape release, pool invariants and empty operations.

F11 publication counts decrease from 128 to 125 for AVBD and from 145 to 142 for TGS. Each run contains three coincident retirement/spawn operations, explaining the exact reduction. Summed publication durations are 300.91 to 279.35 ms for AVBD and 284.18 to 290.13 ms for TGS. These single runs use validation and an 860x640 window: they demonstrate fewer publications, not a reliable frame-time or solver speedup.

Evidence: `/root/beat-box/work/gpu-scene-edits/` on LXC 110. Baseline is merged commit `f22f454aa470bddc50cd01e70f161459d3a06b68`.

## Remaining host coordination

- SimConfig still returns to the CPU for sleep/min-height decisions and spawn cadence. Its separate submission/wait is removed by [simulation-owned publication](SIM_CONFIG_PUBLICATION.md).
- The CPU consumes impact events and records per-parent fracture work; a GPU work queue would require suitable scratch allocation and indirect scheduling.
- Compact geometry metadata still returns to the CPU for AS handles, build-size queries and command recording. BLAS/TLAS construction itself executes on the GPU.
- Renderer-wide queue waits require a separate lifetime/dependency audit before removal, especially for resources shared with tracing.

The next priority is separating telemetry from simulation control and replacing eligible host waits with GPU dependencies. Initial scene authoring, input and resource creation remain host responsibilities.
