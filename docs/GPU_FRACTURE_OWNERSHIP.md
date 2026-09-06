# GPU fracture ownership

This change moves runtime fracture decisions and physical payloads to GPU storage. The CPU still records Vulkan commands and mirrors the compact metadata needed to create acceleration structures. It does not download voxel labels, live bodies, inertia or surface arrays during normal fracture processing.

## Runtime ownership

| Operation | Owner after this change |
| --- | --- |
| Contact impulse candidates, maximum impact per body, deterministic tie breaking | GPU |
| Pending impacts across catch-up simulation steps | GPU; host acknowledges the consumed generation |
| Parent pose, impact-to-grid transform, material-dependent fracture sites | GPU |
| Voronoi partition, connectivity, component census | GPU |
| Sliver merging, component ordering and dense label remap | GPU |
| Occupancy/SDF/surface slices, shape slots and body slots | GPU transactional allocator |
| Cropped occupancy packing | GPU indirect dispatch |
| SDF, surface list, center of mass and inertia | GPU |
| Child pose, velocity, inverse mass/inertia and shape origin | GPU |
| Kill-plane retirement and F11 spawn randomization | GPU |
| Runtime body array, active list, primitive offsets, AABBs and instance transforms | GPU |
| BLAS handles, build-size queries, AS backing regions and build command recording | CPU |
| Execution of BLAS/TLAS builds | GPU |
| Initial scene authoring, input, UI, resource creation and explicit diagnostic dumps | CPU |

The allocator has separate free lists for occupancy, SDF nodes, surface entries, shape IDs and body IDs. The legacy primitive pool index remains reserved with zero capacity. BLAS backing storage remains in the AS manager because Vulkan build-size queries and handle creation remain host operations.

## Publication sequence

1. Solvers write one candidate per manifold. A per-body reduction selects the strongest contact; ties use contact position and normal. Publication walks persistent body IDs in ascending order. The old eight-entry atomic ring is replaced by a pending slot for every possible body, so contact scheduling cannot discard arbitrary fracture events. Bodies with fewer than six voxels are rejected before event publication: they cannot produce two retained components of at least three voxels.
2. A fracture batch records partitioning, planning, reservation and packing for all pending parents in one command list. Shared scratch is reused sequentially with compute barriers. There is one compact result completion/readback for the batch, rather than one result wait per parent.
3. Reservations modify private copies of the free lists. All children must fit before descriptors or edits are published. Packing consumes the parent's occupancy before its private slices are returned to the pool. Shared authored templates remain pinned. Capacity refusal retains the whole parent.
4. The CPU consumes child IDs, counts, bounds and allocation offsets solely to mirror build metadata and record AS work. It does not upload a replacement physical body or a placeholder AABB array.
5. GPU SDF construction feeds GPU body finalization, primitive generation and instance transforms. These MAIN-queue submissions feed the following AS graph through explicit barriers. Normal execution does not wait on the host between SDF construction, publication and AS input consumption.
6. The completed canonical body array is copied to both simulation parities. Active IDs and the ID-to-row table are generated on GPU after those bodies exist. Initial scene load follows this order too.

The hardware used for validation, an RTX 4090, reports `accelerationStructureIndirectBuild=false`. This implementation therefore retains compact host build metadata and CPU-recorded AS commands. It is not a zero-readback renderer. Initial uploads and explicitly enabled CPU verification are also intentional exceptions.

## Synchronization and lifetime

| Producer / consumer | Dependency |
| --- | --- |
| Solver candidate writes / impact reduction / publication | Compute write to compute read/write barriers |
| Simulation / runtime geometry editing | Existing simulation completion and MAIN tracing completion before shared resources are retired |
| Partition scratch reused by the next parent | Compute read/write to compute read/write barrier |
| Allocation manifest / indirect packing dispatch | Compute write to all reads, including indirect command reads |
| Occupancy packing / parent retirement | Packing completion before allocator commit; next partition begins after a compute barrier |
| Batch manifest / host AS metadata | Compute write to host read plus MAIN submission completion |
| SDF / body finalization / AABB generation | Ordered MAIN submissions and compute visibility barriers |
| Generated AABBs / AS input reads | Compute write to acceleration-structure-build read and subsequent read visibility |
| Scene publication / both simulation buffers | Completed AS publication and explicit copies before active-list generation |

Host waits that protect AS resource reuse are retained. Reset and scene switching wait for both queues before replacing the old scene, even when the previous frame is still tracing. Removing a geometry readback does not make it safe to retire an in-flight BLAS or overwrite buffers used by another queue.

## F10 and reset

F10 reuses F7's 432 falling-box placements and static pool. Dynamic boxes are solid 4 x 4 x 4 voxel grids with 0.25 m voxels, density 5 and fracture impulse 30, matching F9's wood threshold. Walls and floor remain static OBBs with no fracture threshold. The existing 512-shape/1024-body capacities remain bounded; when fragments cannot fit, admission leaves the parent intact.

Fracture sites remain procedural. The GPU uses the former mt19937 integer stream, but GPU transcendental rounding can change a boundary voxel compared with the old CPU seeding path. F11 now uses a deterministic GPU PCG stream for spawning; it intentionally does not reproduce the former host RNG sequence.

Reset now rebuilds the authored scene and reinitializes GPU ownership. Runtime CPU body records contain build metadata, so re-uploading them would not restore the original scene.

## Verification

`BB_CENSUS_VERIFY`, `BB_POOL_VERIFY`, `BB_FRAGMENT_VERIFY` and `BB_SDF_VERIFY` deliberately enable host inspection. The per-parent diagnostic route uses the same GPU recording helper as the production batch route, while allowing each result to be checked before scratch reuse.

GPU tests cover full impact capacity, catch-up retention, strongest-impact selection, tie breaking, acknowledgement, indirect occupancy packing, private-parent retirement and failure in the last pool after earlier reservations succeeded. Separate planning tests cover empty occupancy, all-sliver fallback, more than 128 roots and shape-capacity rejection. Runtime oracles compare cropped occupancy, independent free lists and child physical properties.

Run the automated comparison from the built runtime directory:

```sh
DISPLAY=:0 python3 ../../tools/verify_gpu_fracture.py --output gpu-fracture-validation
```

It runs both solvers through the 900-step fracture fixture, two 480-step F10 replays and a 1,500-step F11 recycling stress test in both production and oracle modes. It rejects Vulkan errors, oracle failures, solver invariant violations and any step-by-step state-hash mismatch. CPU tests remain available through `ctest --test-dir build/Release --output-on-failure`.

## Results (RTX 4090, Linux LXC 110)

- Release build and all eight CTest tests passed.
- The automated production/oracle matrix completed 13,440 simulation steps with exact state-hash agreement, repeated F10 agreement and no Vulkan validation errors or solver/oracle failures.
- F6 completed 600 steps per solver and matched the control snapshot exactly.
- F10 completed 480 steps, reset in-process and repeated all 480 steps exactly with each solver. Final GPU dumps contained 486 bodies with AVBD and 541 with TGS. The six original static bodies (floor, light and four walls) retained their original positions, shape type and flags; all captured poses and velocities were finite.
- An additional 3840 x 2160 F10 replay completed 480 steps per solver and implementation, with Vulkan synchronization validation enabled and identical state hashes.

### Controlled publication comparison at 4K

The control is the `baseline-scene` snapshot from this migration: GPU setup, impact reduction, allocation, culling and spawning were already enabled, but it still performed per-parent readback and CPU body/AABB staging. This comparison isolates the final batching/publication changes; it is **not** a whole-PR speedup relative to PR #29's different impact/RNG behavior.

Measurements are `[FRACTURE-MS] total`, the host-observed fracture-processing duration including its GPU completion boundaries. They are not solver step times or total render frame times. One 480-step replay per solver/variant, no CPU oracles, identical physical states, synchronization validation enabled:

| Solver | Fracture batches | Mean before / after | p95 before / after | Maximum before / after |
| --- | ---: | ---: | ---: | ---: |
| AVBD | 31 | 5.159 / 4.029 ms | 7.906 / 5.297 ms | 8.894 / 5.669 ms |
| TGS | 64 | 5.265 / 4.318 ms | 7.121 / 6.025 ms | 10.728 / 6.520 ms |

The mean reduction in this sample is 21.9% for AVBD and 18.0% for TGS. These are observed timings, not fixed-clock guarantees or a claim that all SDF/solver cost disappeared. Separate repeated 860 x 640 runs showed the same direction of improvement.

Raw logs, CSVs, GPU dumps and isolated control runtimes are retained in `/root/beat-box/work/gpu-ownership/`. `reviewed/` contains the final automated matrix; `extra-check.log` contains the reset/F6/4K checks; `performance-4k.json` contains the timing rows above. Control executable SHA-256: `8d91ba3bda634b521e1abd9e39ee4232af469a961cbd95290cc1caf58abcf36e`. Matrix executable SHA-256 (retained as `baseline-reviewed`): `995d9aca5d477337581cdf5114f7523e78a23cb05e04aeb6e5588742e1fc5e94`.

The committed-source rebuild also passed all eight CTest tests and another 900-step GPU oracle replay per solver, matching the matrix executable exactly. Its executable SHA-256 is `34c4118711adc6143a634a33bb890a28d3571e3a4c501284920bc0cf6808bd7e`; logs are in `committed/` and `ctest-committed.log`.
