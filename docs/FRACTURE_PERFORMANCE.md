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
