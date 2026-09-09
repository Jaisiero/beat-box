# Cache source SDF gradients without dropping contact directions

Reference: PR50 (`3353a29`, resolved AVBD contact links).

## Contact audit

F11 captures at steps 900 and 1800 contain, respectively, 4,175 manifolds /
7,773 contacts and 3,595 manifolds / 6,474 contacts. There are 2,849 and 2,759
body pairs. `tools/audit_contact_redundancy.py` finds no exact duplicate
manifolds, constraint rows, points within a manifold, duplicate pair keys,
empty manifolds, or manifold normals within one degree for the same pair.
These snapshots do not establish that all future contact sets are minimal.

The extra manifolds represent distinct contact directions. Arbitrarily keeping
one manifold per pair would lose directions at corners and concave regions.
The existing reducer already retains at most four points per manifold and six
normal directions per SDF pair. This change preserves those limits, feature
keys, warm starts, and all emitted contacts. It does not reduce solver work by
removing constraints.

A conservative source-surface bounds experiment skipped approximately 23% of
samples on captured contact-bearing pairs. It preserved the replay but did
not improve measured narrow-phase time; its bounds calculations and checks
consumed the savings. That experiment is not included.

## Implementation

Near-surface SDF collision detection previously sampled the source shape's
SDF again to obtain its local gradient for every visited source voxel and body
pair. That eight-node interpolation depends on geometry, not the current body
pose or the other body. Surface construction now computes it once per emitted
surface voxel. Narrow phase loads the cached raw gradient and performs the
same pair rotations and normalization as before. The zero-gradient exposed
face fallback is unchanged. Target SDF sampling and SAT are unchanged.

`voxel_sdf_node` and `voxel_sdf_sample` move unchanged into the shared
`voxel_sdf_sampling.slang` header, so build and collision paths use the same
sampling expression. The cache is not normalized or quantized.

The `voxel_surface` allocation retains its 32,768-entry uint index prefix and
adds a float4 gradient tail using the same slot numbers: 512 KiB extra GPU
storage, 640 KiB total. The pool allocator still counts surface slots; no
additional allocator, buffer handle, CPU transfer, task, or queue wait is
introduced. Diagnostic surface readbacks still read the index prefix.

## Lifetime and synchronization

- Initial construction and every dirty shape rebuild write both the canonical
  surface index and its gradient. Only the published `surf_count` is read.
- Recycled surface slots receive new gradients before their new shape is
  published. Unchanged shapes retain their cache.
- SDF finalize precedes surface construction with the existing compute-write
  to compute-read/write barrier. Surface construction now consumes that SDF.
- The existing build completion barrier and publication synchronization publish
  the entire surface allocation, including its new tail, before simulation
  consumes it. Rendering does not consume this cache.
- Inertia computation still only reads occupancy and writes derived properties;
  it does not race with surface-gradient writes.
- Fragment finalization recenters `grid_origin`. A cell-local gradient is
  translation invariant; the cache contains neither world coordinates nor body
  rotations. Floating-point cancellation can differ under recentering in
  general, so exact replay is checked explicitly rather than inferred solely
  from that mathematical property.

## Measurements and validation

F11 AVBD, 3840x2160, requested render rate 144 Hz, spawn interval one step,
70 seconds per run, NVIDIA compute occupancy priority enabled. Order:
reference, cache, cache, reference. Both variants use the enlarged allocation;
only the surface-build and collision shaders differ. Fine profiling and
synchronous SDF build profiling are disabled for this comparison.

| Run | Steps | Narrow phase | Mean step | P99 step | Worst step |
|---|---:|---:|---:|---:|---:|
| Reference 1 | 4,199 | 1.123 | 4.221 | 9.746 | 13.635 |
| Cache 1 | 4,199 | 1.099 | 4.166 | 9.556 | 12.951 |
| Cache 2 | 4,198 | 1.096 | 4.102 | 9.449 | 12.603 |
| Reference 2 | 4,198 | 1.106 | 4.120 | 9.377 | 12.798 |

All times are milliseconds. Averaged across repetitions, narrow phase improves
1.5% (17 us per step). Full-step mean improves 0.9%, P99 0.6%. The reference
runs also drift appreciably, so the whole-step and tail differences should not
be interpreted as a robust improvement to spikes. Both cache runs have lower
narrow-phase means than both references. No run exceeds the 16.667 ms step
budget, but this does not guarantee future worst-case latency.

- F11 AVBD: all 1,800 replay checkpoints exactly match the reference, and the
  final captured manifold geometry matches after sorting by body pair/key.
  Synchronization validation reports no errors.
- F11 TGS: all 600 checkpoints exactly match its existing reference, with
  synchronization validation enabled on the candidate.
- F5 and F6 AVBD: each candidate exactly matches its 600-step reference;
  synchronization validation is enabled on candidates.
- All 10 CTest targets pass.

A separate F11 1,200-step replay enables `BB_RESPAWN_TIMING` for both variants.
It produces the same checkpoints and 48 post-initialization build batches.
Surface construction averages 0.007981 ms before and 0.009382 ms after:
approximately 1.4 us extra per build batch. Total SDF build averages 0.080535
versus 0.082135 ms; maxima are 0.204288 versus 0.207392 ms. The initial scene
build is excluded from those batch statistics. This profiling mode introduces
its existing timestamp-readback wait, so these numbers isolate construction
cost and are not used as unprofiled frame-latency measurements.

Validation disables optional NVIDIA occupancy priority because the installed
validation layer predates that extension. Performance runs enable it. No
`BB_DETERMINISTIC` flag is used: post-stabilization remains active.

Raw logs are in `/root/beat-box/work/contact-reduction/`: `ref1.log`,
`gradient1.log`, `gradient2.log`, `ref2.log`, `gradient-verify.log`,
`gradient-final-*.log`, and `gradient-build-profile-*.log`. GPU step statistics
use `tools/analyze_gpu_timeline.py`; narrow-phase means use the existing
`[FRACTURE-NP] gpu_ms` samples. Contact snapshots can be audited with
`tools/audit_contact_redundancy.py`.

