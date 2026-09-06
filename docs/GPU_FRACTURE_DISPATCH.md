# GPU-derived fracture dispatches and local connectivity

The production fracture input carries a body ID, event slot and command-count bound instead of a VoxelShape copy. GPU setup reads the live parent and shape, seeds fracture sites and writes indirect arguments into the parent context. Voronoi assignment, flood initialization and census use the generated workgroup count. Their shaders read dimensions and occupancy offsets from that same resident context; planning reads its cell count there. Each invocation caches dimensions locally.

## Connectivity

Grids of up to 64 cells use exact local union-find in one invocation. A 64-entry shared-memory forest joins same-site occupied neighbors and always links the larger root to the smaller root, preserving minimum-cell component labels. No other invocation accesses the forest, so it needs no atomics or group barriers. Only one flood command is recorded for these fragments.

Larger grids retain the original flood algorithm and iteration formula: eight passes plus two for each right shift of the cell count while it exceeds one. Each pass uses the GPU-generated cell dispatch arguments. The CPU retains the command-count bound using shape metadata already mirrored for AS publication. This does not add a transfer. A scene-wide fixed command schedule was tested and rejected because extra empty commands penalized mixed-size scenes.

The absolute limit remains 40 passes, protected by the existing 65,536-cell capacity assertion. The CPU bound is intentionally not presented as migrated: removing it without a performance cost requires a different GPU scheduling approach.

## Synchronization and verification

A compute-write-to-read barrier after setup covers shader reads and indirect command fetches. Existing compute barriers between flood iterations, census, planning, allocation and packing remain. Shared scratch is reused sequentially per parent, and AS resource retirement still follows batch completion.

Standalone census/plan verification inputs retain explicit dimensions/cell counts with a null context address. Runtime fracture uses the GPU context in both verification and production paths. The small-grid GPU oracle independently compares every label against CPU breadth-first traversal on 24 grids, including full/empty grids, a 64-cell chain, partial workgroups, holes and site boundaries.

## Scope

CPU impact consumption, body eligibility checks, event ordering, command-count bounds, per-event command recording and AS metadata publication remain. This change moves workgroup sizing and runtime shape parameters to GPU and accelerates small-fragment connectivity. It does not remove the batch result readback or replace the CPU event loop with a GPU work queue.

Evidence is stored in `/root/beat-box/work/gpu-fracture-dispatch/` on LXC 110. The isolated comparison runtime contains PR #32 commit `63af919`, included in merged baseline `1747228`. Final evidence uses the `accepted` prefixes; earlier variants were retained for comparison.

## Timing

F10 uses 480 steps per run, three alternating baseline/candidate pairs per solver, an 860x640 window and no validation/oracle overhead. All replay states match. Median of the per-run mean `[FRACTURE-MS] split` duration:

| Solver | Baseline | New | Reduction |
| --- | ---: | ---: | ---: |
| AVBD | 0.3580 ms | 0.3238 ms | 9.6% |
| TGS | 0.3680 ms | 0.3276 ms | 11.0% |

Each run processes 31 fracture batches with AVBD or 64 with TGS. These are host-observed partition/planning/allocation/packing batch durations, including completion; they are not solver-step or frame times.

A single baseline/candidate pair also screens larger and mixed geometries:

| Scene / solver | Baseline split | New split |
| --- | ---: | ---: |
| F3 / AVBD | 0.3275 ms | 0.2653 ms |
| F3 / TGS | 0.3271 ms | 0.2774 ms |
| F11 / AVBD | 0.3665 ms | 0.3753 ms |
| F11 / TGS | 0.3334 ms | 0.3340 ms |

Those single-pair measurements are not a robust speedup estimate. In particular, F11 shows no established gain; its AVBD sample is 0.009 ms slower per batch. F3 uses 900 steps and F11 1,500 steps per solver/variant, with exact replay agreement.

## Final validation

- All eight CTest tests pass.
- F10 (480 steps) and F11 (1,500 steps), both AVBD and TGS, in GPU-oracle and production modes: all states exactly match the baseline (7,920 steps).
- F3 adds 900 oracle steps per solver with exact baseline agreement: 9,720 synchronization-validated replay steps total.
- The 24-grid small-component GPU oracle passes for each verification run.
- Both solvers pass 30-second F11 runs in the normal 3840x2160 render loop under synchronization validation, without reported validation errors or NaNs.
- All timing replays also match their baseline states exactly.
