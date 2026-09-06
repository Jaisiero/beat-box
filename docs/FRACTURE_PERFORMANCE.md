# Fracture publication and TGS

## Changes

- TGS now classifies material impacts once during contact preparation, before warm start
  and iterative bias corrections. It uses approach velocity and normal effective mass
  to estimate the stopping impulse, with a gravity-step gate against resting loads.
  Speculative positive gaps do not trigger this classifier. Both colored and overflow
  preparation paths call it. It does not emit an event on every substep/iteration.
- `fracture.slang` shares threshold checks and event publication between TGS and AVBD.
  AVBD retains its existing pre-impact stopping-impulse estimator. This is impact
  fragmentation, not a stress-based structural-integrity model.
- Partitioning already uses a zero carve radius and does not mutate occupancy. It no
  longer clones a shared input grid before knowing whether a split exists. Shared
  source grids stay resident; private parent grids are retired after fragment emission.
- A single component after the conservation merge means no topology change: return
  without uploading pools or rebuilding acceleration structures.
- Fragment emission copies the parent record before `rigid_bodies.push_back`, avoiding
  a dangling reference if the vector grows while constructing subsequent fragments.
- The fragment body IDs explicitly invalidate their BLAS. Hashing CPU voxel AABBs is
  insufficient because these are zero placeholders; their actual geometry is GPU-built.
  Bodies whose geometry did not change retain their BLAS.
- `update_sim` and `update_active_rigid_body_list` already refresh both parities.
  Respawn no longer calls each twice or advances the render clock to repeat uploads.
- `BB_RESPAWN_TIMING` now also prints `[FRACTURE-MS]`: live-state synchronization,
  partition/split and publication, covering the whole operation rather than only respawn.

## Synchronization

Partition dispatches and label/occupancy copies now share one command submission and
one completion wait. Compute-write to transfer-read and transfer-write to host-read
barriers remain explicit. Scratch storage and staging are not reused/destroyed before
that submission completes.

Live body synchronization waits for prior MAIN rendering before host geometry edits,
then for the COMPUTE_0 copy before reading the staging allocation. Simulation/readback
has already completed on COMPUTE_0 at the caller. Pool rebuild and derived readback
wait for their own MAIN queue submission instead of draining every device queue.
Diagnostic oracle readbacks retain completion waits too.

Publication is still synchronous. The CPU still groups component labels, allocates
fragment grids and consumes mass properties before publishing bodies/AS. This patch
does not claim an asynchronous fracture pipeline or zero-cost topology edits. Existing
AS publication waits remain; removing them needs explicit resource-lifetime dependencies,
not simply deleting barriers. Daxa's installed TaskGraph submit implementation ignores
additional timeline arguments, so these are not used as a substitute for completion.

## Verification (RTX 4090, LXC 110)

Release build and all three existing CTest targets pass. These tests are not a complete
fracture suite; the GPU runs below exercise the actual fracture path.

| Run | Complete fracture cost | Notes |
| --- | --- | --- |
| F10 AVBD baseline | 4.242 / 3.285 ms | Instrumented pre-fix host path |
| F10 AVBD final | 2.786 / 1.865 ms | Same automatic drop, normal rendering, pool checks |
| F10 TGS final | 3.267 / 3.514 ms | Vulkan synchronization validation and pool checks |
| F9 AVBD drag/release, 1280x720 | 1.810-3.210 ms | Eight fracture publications, 5 to 25 bodies |
| F9 AVBD drag/release, 3840x2160 | 4.181-6.217 ms | Five publications, validation enabled, 5 to 17 bodies |

These are observed wall times, not a statistically controlled latency guarantee. The
4K and TGS validation runs include validation overhead and are not direct solver-speed
comparisons. The large user-reported pause has not been isolated to one defect by these
small tests; the measured synchronous work and unnecessary rebuilds are reduced.

F10 TGS impact events: 224.846 and 231.109 kg*m/s. Both materials fracture; no repeated
resting-load fracture events occurred in that run. AVBD baseline impacts were 231.157.
Separate solver trajectories need not produce exactly equal impact estimates.

Final TGS/F9 validation logs contain no Vulkan validation/synchronization errors,
voxel-conservation warnings or pool invariant failures. Runtime evidence is retained
in `/root/beat-box/work/fracture-audit` (including `baseline-full.log`,
`final-avbd-benchmark.log`, `final-tgs-validation.log`, `f9-drag.log`, `f9-4k.log`).

The repeatable fixture `tests/scenes/fracture_frame_drop.txt` (F3, TGS, automatic
start, 8 seconds, pool and Vulkan synchronization validation enabled) exercises
fragment growth and impacts that produce only one component. No-split publication
was skipped (~0.001 ms; total partition/check still ~0.5-1.8 ms). The body set grew
from 3 to 24, exercising vector reallocation; there were no conservation/pool or
Vulkan errors. See `tgs-fragments.log` for all events. This intentionally very weak
material fixture is a regression case, not a recommended material setting.

## Follow-up: distinguish publication from the following simulation steps

`BB_RESPAWN_TIMING` now records the publication frame and three following loop
intervals (`FRACTURE-FRAME`), including their step counts and simulation-phase wall
time. These are CPU loop intervals, not display/presentation timestamps. No new
GPU completion wait is added. The existing SC completion wait also makes a pair
of optional narrow-phase GPU timestamp queries readable (`FRACTURE-NP`, every measured step when enabled). With catch-up, this query reports the last step before readback.
The query pool is reset on COMPUTE_0 inside the narrow-phase callback; the existing
per-step completion synchronization prevents reuse while a prior step is running.

The F9 4K investigation reproduced a 357.6 ms frame with 354.6 ms spent executing
four simulation steps. Fracture publication itself was approximately 2 ms.
A subsequent timestamped run measured up to 76.9 ms in narrow phase alone. Thus
removing publication waits alone cannot solve the reported hitch.

Changes:

- Remove the TLAS update inside `respawn_after_fracture`. All three callers run
  inside the renderer's scene-edit block (fracture, cull, soak spawn), which already
  publishes the final TLAS before ray tracing. BLAS construction, both body-buffer
  uploads and the final TLAS synchronization remain. This removes duplicate work,
  not the dependency protecting the renderer from unpublished geometry.
- Stop real-time catch-up once the simulation phase has consumed one fixed-step
  duration of wall time. Keep fixed dt, solver iterations, the four-step hard cap
  and the existing backlog cap. A single expensive step can still exceed budget;
  it no longer schedules three more expensive steps before returning to rendering.
  Deterministic fixed-step tests are unchanged by this scheduling rule.
- Before sampling eight SDF nodes, reject sample cubes whose rotated conservative
  AABB cannot reach the other shape's grid (including the existing 2 cm band).
  Clamp neighboring-cell traversal to the grid; outside cells are always empty.
  Both full 899-row fracture-fixture CSVs are identical before/after this spatial
  optimization. No claim of an exact global SDF or different solver convergence.

The final interactive run grew F9 to 43 bodies. Recorded four-frame event windows
peaked at 48.5 ms, with one step in the expensive frames. However, narrow phase
still reached 79.9 ms elsewhere in the run. These manual impacts produce different
trajectories and fragment counts, so the maxima are diagnostic observations, not
a controlled speedup ratio or an upper latency bound. Residual F9 motion remains;
this is a partial contact fix and a catch-up/publication improvement, not a claim
that destruction is fully stable or fits a 16.7 ms frame budget.

Evidence: `/root/beat-box/work/destruction-resume`, especially `f9-v2.log`,
`f9-profile.log`, `f9-final.log`, `final-*.csv` and `validation-*.log`.

Final Release build and all four CTest targets pass. Both solvers complete 900
fixed steps of the fracture fixture with pool checks and Vulkan synchronization
validation enabled. There are no validation errors, synchronization hazards,
conservation warnings or pool invariant failures. The validated CSVs match the
normal final fixed-step CSVs. The instrumentation does not add a shader ABI field
or a new environment flag; it extends `BB_RESPAWN_TIMING` and `BB_POCKET_TRACE`.

`BB_RESPAWN_TIMING` also reports `[SDF-BUILD] gpu_ms=... shapes=...` for the
voxel-pool GPU dispatch chain. Queries are read after the existing MAIN submit
wait; this instrumentation introduces no additional wait. See
[SDF_PERFORMANCE.md](SDF_PERFORMANCE.md) for controlled measurements and rejected
experiments.

`BB_RESPAWN_TIMING` additionally reports `[AVBD-STAGES]` GPU intervals for setup,
preparation, primal/dual, post-stabilization and publication. These queries reuse
the existing readback wait. See `SDF_PERFORMANCE.md` for interval definitions.
