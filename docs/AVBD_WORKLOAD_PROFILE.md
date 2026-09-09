# Correlate AVBD contact workload with GPU step cost

Reference: PR51 (`3148cdb`). This PR adds optional diagnostics and an analyzer;
it does not change the solver, contact reduction, coloring or iteration count.

## Enable and interpret

Run from the normal runtime directory, with the viewport resized to 3840x2160:

```sh
BB_SCENE=11 BB_SOLVER=2 BB_AUTOSTART=1 BB_RUN_SECONDS=70 \
BB_RENDER_HZ=144 BB_NV_COMPUTE_PRIORITY=1 BB_FRACTURE_SPAWN_STEPS=1 \
BB_AVBD_WORK_PROFILE=1 BB_GPU_TIMELINE=1 BB_FRAME_TIMING=1 ./beat-box > f11.log
python3 tools/analyze_avbd_workload.py f11.log
```

Use scene 7 for the non-destructible box pool. The analyzer is in the source
repository's tools directory, not necessarily the runtime directory.

`[AVBD-WORK]` records per-color awake dynamic body counts, potential contact
endpoint visits per sweep, maximum contacts per body, and maximum manifolds
per body. These are collected while prepare already walks the canonical
manifold chains. Both awake endpoints count the shared contacts; the sum is
not a unique contact count. Main-sweep convergence early-outs and contact
activation can reduce actual subsequent work, which these counters do not
measure. Existing global counts provide candidates, manifolds and depth.

Stage timing and workload are joined by the exact integer GPU begin timestamp,
not by adjacent log lines. `SimConfig.frame_count` can advance during render-only
frames, so the log calls it `frame`; chronological sample ordinals group the
first 1,200 measured solver steps. The analyzer also accepts the older `step`
label in the initial experiment logs, with the same frame semantics. Absolute
timestamps are never converted through floating point. Missing associations
and malformed color arrays fail explicitly.

`useful_lane_fraction` is awake bodies divided by all lanes dispatched across
used colors, including padding to the existing four-thread group size. It is
not a hardware occupancy measurement. `sum_color_max_contacts` sums each
color's maximum body contact count: a workload proxy for serial color batches,
not an exact GPU critical-path model.

## Storage, synchronization and observer cost

- Four 32-entry uint arrays add 512 bytes to the existing SimConfig allocation
  and its existing snapshot copy. No new readback submission or CPU wait.
- The earlier body-color reset clears disjoint array entries. Existing graph
  dependencies publish that reset before prepare atomically accumulates counts.
- Each awake body accumulates local counts during its existing chain traversal,
  then performs four optional atomics to its color's counters. Sleeping and
  static bodies are excluded; invalid packed indices are excluded.
- Counters are behind `PROFILE_AVBD_WORK`, enabled only by
  `BB_AVBD_WORK_PROFILE`. Normal runs do not perform those atomics or the extra
  contact-count loads. Existing graph-tail readback and completion handling
  publish the counters alongside the configuration for the measured step.
- The flag enables existing stage timestamp queries. Query reads use existing
  simulation completion handling. Validation confirms synchronization and
  exact replay; it is not used for performance measurements.

The profiled F11 run averages 4.213 ms, versus 4.130 ms for a subsequent run
with counters disabled. Prepare averages 0.319 versus 0.297 ms. This single
comparison contains run variation and profiling overhead; instrumentation is
not free and is disabled in the restored interactive session.

## Observed workloads

RTX 4090, 4K, requested rendering 144 Hz, 70-second runs, AVBD with normal
post-stabilization, no validation layer. F11 uses a one-step spawn interval.
There are 4,199 F11 solver samples and 590 F7 solver samples; F7 settles and
stops running the full solver during the remainder of the observation.
Scenes differ in geometry, population and motion, so this is not an isolated
SDF-versus-box cost per object benchmark.

The worst measured GPU step in each run:

| Metric | F11 SDF fragments | F7 boxes |
|---|---:|---:|
| Full GPU step, ms | 12.833 | 2.362 |
| Narrow phase, ms | 1.257 | 0.069 |
| Setup including narrow phase, ms | 1.848 | 0.210 |
| Prepare, ms | 0.492 | 0.194 |
| Main solve including dual, ms | 6.211 | 0.977 |
| Post-stabilization, ms | 4.039 | 0.776 |
| Finalize, ms | 0.238 | 0.200 |
| Bodies / awake dynamic bodies | 781 / 774 | 438 / 430 |
| Candidate pairs | 5,758 | 1,524 |
| Manifolds | 4,286 | 860 |
| Potential contact endpoint visits per sweep | 17,798 | 3,286 |
| Maximum contacts on one awake body | 173 | 24 |
| Maximum manifolds on one awake body | 53 | 8 |
| Body colors | 8 | 5 |
| Maximum support layer index | 11 | 11 |

Narrow phase is part of setup; do not add it a second time. Stage totals omit
small timestamp-boundary overhead compared with the full GPU step.

F11's worst step has body counts by color of
`250, 199, 147, 101, 51, 20, 5, 1`. The maximum body contacts by color are
`100, 94, 173, 119, 112, 96, 86, 96`. In particular, the last color runs one
body with 96 contacts. Each color is ordered after its predecessors, while
one thread currently processes a body's contact chain. A low body count in
that batch does not imply a cheap batch.

Across the F11 run, Pearson correlation with main-solve time is:

| Workload | Correlation |
|---|---:|
| Sum of maximum body contacts per color | 0.977 |
| Maximum manifolds on one awake body | 0.947 |
| Maximum contacts on one awake body | 0.925 |
| Total awake bodies | 0.896 |
| Potential contact endpoint visits | 0.876 |
| Global manifold count | -0.145 |
| Candidate pair count | -0.552 |

These are descriptive correlations, confounded by the evolution and sleeping
of the pile. They do not prove causation. They do show why global manifold
count alone is a poor predictor: many persistent contacts remain while their
bodies sleep, whereas the expensive transient concentrates work on active
bodies. Main plus post account for about 75% of the slowest 1% of F11 steps
(4.457 + 3.393 out of 10.527 ms average stage total), and about 80% of its worst
step. No 40 ms spike was reproduced in these runs.

## Next optimization targets

1. Reduce the serial work of heavily connected bodies in primal and post:
   investigate cooperative contact evaluation while preserving the ordered
   accumulation of the body's system. Shared poses change between colors, so
   caching world-space constraint data for the whole step would be invalid.
2. Evaluate coloring priorities that account for contact work, not merely body
   count. A different order can change finite-iteration convergence and needs
   penetration/stability comparisons, not just a faster timestamp.
3. Compact awake bodies by color and support layer to eliminate inactive lanes
   and empty layer/color batches. This targets scheduling waste, especially
   after settling; it does not by itself eliminate the serial 173-contact body.
4. Contact reduction remains a possible algorithmic change, but the existing
   contact audit found no obvious duplicates. Do not cap contacts per body or
   merge distinct directions solely to make this counter smaller.

## Validation and artifacts

- F11: all 1,800 profiled replay checkpoints match the PR51 reference exactly.
- F7: all 1,800 checkpoints match with profiling disabled/enabled.
- Synchronization validation enabled on both profiled replays; optional NVIDIA
  priority disabled there because the installed layer predates the extension.
- Final source: another exact 600-step F11 replay with synchronization validation;
  all 11 CTest targets pass.
- Analyzer tests cover out-of-order logs, neighboring integer timestamps above
  2**53, malformed arrays, missing stages and constant correlations.
- Raw logs and JSON analyses: `/root/beat-box/work/avbd-workload/`.
  `live-11-1.log`, `live-7-1.log` contain counters; `live-11-0.log` is the
  unprofiled control. `replay-*.log` are correctness runs, not timing evidence.
