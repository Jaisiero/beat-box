# Resident impact payloads

The strongest pending impact already lived in GPU scratch memory, but publication duplicated its position and normal into the host-visible event bridge. Fracture setup then loaded that duplicated payload from the bridge.

The bridge now contains `FractureEventSummary` records with only a body ID and impulse. Setup reads the full `FractureEvent` from `FractureImpactScratch::pending[body_id]` in device-local memory. CPU scheduling and diagnostics need only the summary; the obsolete event-slot argument is removed from production inputs, setup push constants and verification interfaces.

Each published record shrinks from 32 to 8 bytes, enforced by C++ size assertions. At the 1,024-event capacity, the event array drops from 32 KiB to 8 KiB, excluding the unchanged header/padding. This is a 75% reduction in logical event payload written to host-visible memory, not a measured 75% reduction in PCIe traffic or a frame-time claim.

## Lifetime and synchronization

Impact reduction and publication write pending payloads on COMPUTE_0. Both production batching and the per-parent verification path explicitly wait on that queue's latest submission in their MAIN submission. The installed Daxa implementation translates `wait_queue_submit_indices` into a timeline semaphore wait at ALL_COMMANDS. This supplies the cross-queue memory dependency for device-local pending data; the existing MAIN barriers still order subsequent setup/partition reads.

The CPU reads the summary only after simulation completion and acknowledges its generation as before. Acknowledgement changes the bridge scalar; it does not clear pending payloads. The next simulation publication clears or replaces them. Existing render-loop completion boundaries prevent that next simulation from overwriting data while fracture setup consumes it. Future removal of host waits must preserve this reverse lifetime dependency too.

No new host wait or separate readback submission is added. Event ordering, maximum selection, position/normal tie breaking, catch-up retention, capacity and acknowledgement semantics are unchanged.

## Remaining host work

The CPU still consumes the compact event summary, checks body eligibility, records bounded per-parent commands, acknowledges generations and prepares AS metadata. This does not replace the event loop with a GPU work queue or remove all CPU/GPU coordination.

Evidence is stored in `/root/beat-box/work/gpu-impact-payloads/` on LXC 110. The isolated baseline is PR #33 commit `dedd00e`, included in merged baseline `5bed593`.

## Validation and timing

All eight CTest tests pass. F10 (480 steps) and F11 (1,500 steps), both AVBD and TGS in GPU-oracle and production modes, exactly match the baseline (7,920 steps). F3 adds 900 oracle steps per solver with exact agreement: 9,720 synchronization-validated replay steps total. Both solvers also pass 30-second F11 runs in the normal 3840x2160 render loop without reported validation errors or NaNs.

The impact oracle covers full capacity, unconsumed catch-up retention, maximum selection, position and normal tie breaking, acknowledgement and reuse. Geometric assertions now inspect pending payloads; the host bridge exposes only summaries.

Three alternating baseline/candidate pairs of 480-step F10 runs, without validation oracles and at 860x640, give these medians of per-run mean `[FRACTURE-MS] split` durations:

| Solver | Baseline | New |
| --- | ---: | ---: |
| AVBD | 0.3123 ms | 0.3101 ms |
| TGS | 0.3165 ms | 0.3216 ms |

Every timing replay matches exactly. These small differences do not establish a speedup; the measured interval is the host-observed partition batch, not the whole frame or impact-publication bandwidth. The guaranteed reduction is the published record size and removal of its duplicated geometric payload.
