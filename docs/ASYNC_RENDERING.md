# Independent simulation and rendering

F11 previously put a CPU wait for every simulation step between event polling and
ray tracing. A slow solve therefore delayed camera updates and presentation, even
though simulation and rendering used separate GPU queues.

## Scheduling and ownership

The main thread pumps events and simulation completion independently of the
render deadline. Physics retains a fixed timestep with at most one solve in
flight. Presentation defaults to 60 Hz (`BB_RENDER_HZ`, 1–240).
`BB_ASYNC_SIM=0` selects the synchronous scheduler for A/B measurements.
Replay defaults to synchronous; `BB_ASYNC_SIM=1` exercises asynchronous scheduling.

Three GPU snapshot slots hold rigid bodies, persistent-ID mappings, LBVH nodes,
islands, contact islands, debug points/lines/axes and simulation configuration.
Only active ranges are copied, on COMPUTE, before submitting the next solve.
Rendering selects the newest completed snapshot and can reuse it while physics
runs. CPU draw counts are captured with the same snapshot.

The instance generator and TLAS build run on MAIN and read the selected snapshot.
The TLAS has stable identity so Daxa retains its MAIN read/build dependencies.
Geometry edits invalidate old snapshots after the existing retirement boundary;
the first render of new topology waits for a matching capture.

Daxa's conservative cross-graph queue tracking can include later submissions on
a producer queue. Render aliases therefore use explicit timeline waits for the
selected capture, rather than sharing the solver task handles. Each slot records
its last MAIN reader. It is recycled only after that reader and its previous copy
complete, with an explicit semaphore dependency protecting alias reuse. The
selected slot is never overwritten. If no slot is free, capture is skipped and
physics continues. Copy sources retain normal COMPUTE task tracking; scene-edit
writes on MAIN have an explicit MAIN-to-COMPUTE dependency.

Accumulation resets when the selected pose changes, rather than when an unrelated
simulation completion is observed. Debug instance uploads use staging instead of
writing mapped storage while a prior render could still consume it.

No shader algorithms, contact counts, solver iterations or physics parameters
are changed. The BLAS pool and geometry retirement rules remain in place. Scene
reset, reload, resizing and shutdown still use their existing completion barriers.

## Input and diagnostics

Cursor coordinates are cached from GLFW motion events. Querying them every pump
with `glfwGetCursorPos` synchronizes with X11 and measured up to 11.5 ms in the
4K streaming session. The initial position and discrete button events still query
once; steady-state physics scheduling makes no pointer round trip.

Mouse rays and press/release state are latched on the CPU and written to the pick
bridge immediately before a new step, after the previous step completes. A press
is retained between physics steps while held; release cancels an unconsumed press.
The result shown by pick diagnostics is a completed CPU copy.

Debug vertex buffers now bind to the simulation parity **after** advancing the
step, rather than the preceding render's parity. Contact/axis diagnostics and
LBVH visualization are suppressed across a topology change until the next step
has regenerated them for the new body layout. This avoids showing stale contact
vertices or an old tree alongside newly published fragments.

## Scope and measurements

This removes the per-step CPU solver wait and makes the physics clock independent
of presentation. It does not make GPU execution free: ray tracing and simulation
still share GPU resources. Geometry fragmentation/publication retains compact
host metadata and some completion waits; this change does not claim that all
fracture processing runs asynchronously in the background.

`tools/benchmark_async_render.py` compares both schedulers at the same resolution,
render cap and F11 spawn cadence. `BB_PACING_TRACE` records CPU frame-loop intervals
without GPU timestamps. Report physics throughput alongside P95/P99 frame times;
showing an unchanged snapshot more often is not a solver speedup. These intervals
are not Moonlight/display presentation timestamps.

## Validation

All eight CTest targets pass. Both F11 solvers match all 3,600 reference DET
records (7,200 total) with synchronization validation enabled, at 3840×2160,
spawning every five steps and using the default kill plane. A further 600 records
cover an in-process reset (two identical 300-step runs). A 38-second interaction
run covers pause/resume, AVBD/TGS switching, F9/F11/reset, contacts/axes/LBVH,
mouse input and accumulation without reported validation errors.

At 30 Hz presentation, AVBD maintains 59.98 physics steps/s, including the dense
fragment interval (59.99 steps/s). This measures independent cadence rather than
claiming a faster solver. The following 60 Hz results use two 70-second runs per mode/solver, reverse
mode order on the second run, and exclude the first ten seconds. Both modes
include the cursor fix and snapshot ownership changes; this isolates scheduling.

| Solver | Scheduler | P99 interval (ms), run range | Worst interval (ms) | Physics steps/s, run range |
| --- | --- | ---: | ---: | ---: |
| AVBD | Sync | 16.79–16.81 | 16.91 | 60.00–60.00 |
| AVBD | Async | 16.85–16.92 | 18.23 | 59.97–59.98 |
| TGS | Sync | 16.78–16.79 | 25.75 | 59.99–59.99 |
| TGS | Async | 17.46–17.83 | 21.34 | 59.98–59.98 |

No measured interval exceeded 33.33 ms. Async TGS has a modest P99 overhead
relative to the corrected synchronous mode; there is no claim that async alone
improves this workload's frame tails. Its benefit is independent presentation
and physics cadence without requiring a completed solve for every render.

Before caching cursor events, the same three-slot AVBD async workload measured
58.70 steps/s and a 21.92 ms P99; after the cursor fix it maintains approximately
60 steps/s. The pointer-query round trip, rather than shader work, caused this
avoidable loss. These before/after numbers are diagnostic single runs, whereas
the table contains the repeated final comparisons.

Earlier single-snapshot schedulers were rejected: one improved frame intervals
but reduced AVBD throughput to approximately 52 steps/s. Their results are not
counted as validation of the three-slot implementation. An earlier original
baseline TGS teardown stalled in NVIDIA's vkDestroyDevice lock; its repeat exited
normally. An incomplete uncapped prototype replay was also rejected.
