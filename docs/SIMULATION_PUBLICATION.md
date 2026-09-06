# Simulation and TLAS publication

## Changes

The `device.wait_idle()` between instance updates and TLAS construction is
removed. Both graphs retain the same external instance `TaskBuffer`: the
compute write records its producer queue, and the AS graph waits for that
queue's latest submission through Daxa's internal semaphores. Rendering consumes
the published `TaskTlas` in the same way. FIFO ordering alone is not assumed to
make writes visible.

The `sim_done_tsem` semaphore, its counters, and spans are removed: in the
vendored Daxa 3.6 revision, `TaskGraph::submit` ignores `TaskSubmitInfo`.
Therefore, the previous `additional_*_timeline_semaphores` fields did not provide
synchronization. The effective dependency already came from external resources.

Waits immediately after `simulate()` now wait for the latest submission on
`QUEUE_COMPUTE_0`, where the solver runs, rather than globally waiting for the
other queues. Pre-simulation waits and resource lifetime, scene change,
readback, and fracture boundaries that still need the current protocol are
preserved. This change does not eliminate all CPU/GPU communication or alter
the physics algorithm.

The debug BLAS reads geometry from the LBVH node buffer. That buffer is added
as `BUILD_READ` to the build task and registered with its graph.

## Graph compiler limitation

A combined graph and a combined graph with two submissions were tested. Both
failed during `TaskGraph::complete`, before simulation execution. GDB located
an `ArenaDynamicArray8k<TaskBarrier>` with a null allocator when inserting the
barrier for `blas_instance_data`, between compute writes and AS reads.
This branch does not modify Daxa: it retains the two existing graphs without
the intermediate CPU wait. The exact internal cause requires a separate minimal
reproducer; the failure is not attributed to Vulkan or a GPU race.

## Validation

- Release build and all five CTest tests pass.
- Fracture fixture: 900 steps with AVBD and 900 with TGS.
- F7: 1800 steps with AVBD and 1800 with TGS.
- All four CSVs and every DET checkpoint exactly match the PR25 controls.
  Vulkan synchronization validation reports no errors; the fixture also enables
  pool verification.
- Interaction at 3840x2160 with Vulkan validation: drag/release fractures in F9,
  BVH and reset, pause/accumulation, F5/F7/F9, camera movement, and resizing.
  No reported errors. Captures: `ui-fracture.png` and `ui-bvh.png`.
- F6 A/B against the previous executable, three repetitions of 600 steps per
  solver and variant. All CSVs and checkpoints match within each solver.
  Vulkan validation and profiling were disabled for performance measurements.

| Solver | Before, ms/frame | After, ms/frame | Reduction |
| --- | ---: | ---: | ---: |
| AVBD | 4.450 | 4.024 | 9.6% |
| TGS | 6.034 | 5.804 | 3.8% |

Median wall-clock time between checkpoints 1 and 600, divided by 599.
Includes simulation, publication, rendering, and harness overhead at 860x640;
**these are neither solver-only GPU times nor 4K rendering measurements**.
Hardware: RTX 4090, LXC 110. This is a local measurement, not a guarantee for
other scenes, resolutions, or architectures.

Evidence: `/root/beat-box/work/publication/`, including `checks.log`,
`timings.json`, per-run logs/CSVs, and backtraces from the attempts to combine
graphs.

## Visibility after fractures

A subsequent check detected two preexisting publication defects: transposed
rotation in CPU-created instances and indirect dispatch using the group count
from before the fracture. The 32 -> 34 body reproduction, fix, and regression
test are documented in [FRAGMENT_VISIBILITY.md](FRAGMENT_VISIBILITY.md).
Physics replays and synchronization validation did not cover this visibility
condition; instance instrumentation did detect it.
