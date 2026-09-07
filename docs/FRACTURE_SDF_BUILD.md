# Fracture and SDF construction costs

Measured on the RTX 4090 in LXC 110, against merged PR #39
(`fc4346e476a6841e38c4e7ac8645c49200119387`). The target is work performed
when new voxel fragments are created, not SDF collision queries.

## Measured bottlenecks

The timber-drop fixture exposes larger fragment grids than F10. Its expensive
construction work was the serial surface scan, serial mass-property scan and
census/plan/packing stage. The distance transform itself was smaller.

Separate diagnostic runs use the same stage timestamps in the old and new
shader implementations. Values below are GPU means per fracture batch, in ms;
initial scene construction is excluded. AVBD produced 14 batches, TGS 10.

| Timber stage | AVBD before → after | TGS before → after |
|---|---:|---:|
| Census, plan, allocation and packing | 0.319 → 0.168 | 0.314 → 0.165 |
| Surface extraction | 0.191 → 0.027 | 0.197 → 0.026 |
| Mass properties | 0.214 → 0.148 | 0.211 → 0.147 |
| Connected components | 0.116 → 0.116 | 0.109 → 0.110 |
| Distance transform, three axes | 0.071 → 0.071 | 0.076 → 0.074 |
| BLAS build | 0.101 → 0.104 | 0.099 → 0.098 |

The surface stage improves about 86%, mass properties 31%, and the combined
census/plan/packing stage 47%. The latter is a combined measurement, not an
isolated claim about census performance. Maximum AVBD surface cost falls from
0.417 to 0.054 ms, inertia from 0.461 to 0.332 ms, and census/plan/packing from
0.628 to 0.317 ms.

F10 uses smaller fragments: AVBD surface extraction goes from 0.0182 to
0.0055 ms, mass properties from 0.0224 to 0.0204 ms, and census/plan/packing
from 0.0386 to 0.0313 ms. TGS shows the same surface/plan improvement and little
change in inertia. These savings are substantially smaller than in timber.

AS copy costs are about 0.003 ms in timber and 0.010 ms in F10; the BLAS
publication helper reports zero host wait for these fracture batches because
publication is deferred. This does not imply the entire fracture path is
asynchronous: reading the partition manifest still waits for its submission.
Timber partition wait spans fall from 0.533 to 0.384 ms (AVBD) and 0.532 to
0.373 ms (TGS). Those CPU wait spans may include preceding queue work.

AS command recording still has variable CPU costs: approximately 0.2 ms per
timber batch in AVBD and 0.5 ms in TGS, with occasional multi-millisecond
outliers. This patch does not change AS recording or synchronization.

## Implementation and ordering

- `voxel_sdf.slang`: 64 lanes classify surface cells in tiles. Two shared
  bitmasks provide stable ranks, so the output has exactly the original
  x-fastest order and first-empty-neighbor direction. All lanes participate in
  the barriers, including the last partial tile. Tile counts are read before
  the masks can be reused; output visibility uses the existing dispatch barrier.
- `voxel_sdf.slang`: mass properties iterate set occupancy bits, skipping empty
  voxels. Ascending words and least-significant set bits preserve the original
  accumulation order and arithmetic. Padding bits in the last word are masked.
  The reduction remains serial to preserve its floating-point behavior.
- `fragment_plan.slang`: 64 lanes prefetch census tiles into group memory.
  Lane zero processes them in the original label order, retaining centroid
  calculations, tie-breaking, sliver merges and final sorting. Overflow uses
  a uniform group exit after a barrier. No lane exits while another needs it
  at a group barrier.
- `rigid_body_manager.cpp/.hpp`: `BB_RESPAWN_TIMING` additionally reports
  `[SDF-STAGES]` and `[FRACTURE-STAGES]`. Queries are read after existing
  completion boundaries. No new runtime readback or host wait is introduced.
  GPU plan fixtures now include a sparse root in a partial final tile, alongside
  empty input, all-sliver, 131-root and capacity-rejection cases.

The detailed timestamps use ALL_COMMANDS boundaries. They can serialize
otherwise independent surface and inertia dispatches; stage sums are diagnostic
and must not be added to unprofiled frame times. Profiling is disabled in the
frame comparison.

## Verification reference repair

The merged baseline's `BB_SDF_VERIFY` path reserved new fragment reference
slices but did not populate SDF values, surface entries or mass properties.
It therefore reported mismatches against zero-filled data after the first
fracture. The same failure was reproduced on the untouched baseline.

`scene_manager.hpp` now authors those CPU references only under
`BB_SDF_VERIFY`, from the independently packed CPU occupancy. It uses brute
point-to-cell distances rather than the GPU EDT. Comparisons skip retired
shape slices and unallocated pool capacity; non-finite GPU distances fail
the comparison. Mass-property comparisons keep the existing 0.1% relative
tolerance and add a 1e-6 absolute allowance for near-zero components, rejecting
non-finite values. This avoids treating 1e-8 off-diagonal cancellation residues
as failures. This debug work is absent from normal-mode timings.

## Reproduction

Evidence resides in `/root/beat-box/work/fracture-sdf-build/`.
`instrumented-profile-*` and `sparse-profile-*` contain comparable stage
measurements; `stage-summary.json` excludes initial construction.
`baseline-runtime/` preserves the merged baseline.

Run each solver separately with `BB_RESPAWN_TIMING=1`, `BB_AS_TIMING=1`,
`BB_DET_STEPS=600` and either `BB_SCENE=10` or `BB_SCENE=3` plus
`BB_SCENE_FILE=/root/beat-box/tests/scenes/fracture_timber_drop.txt`.
The normal batch path must be used for timing; verification flags select an
intentionally slower diagnostic path.

Unprofiled comparison:

```sh
DISPLAY=:0 python3 tools/benchmark_sdf_frames.py \
  --baseline work/fracture-sdf-build/baseline-runtime \
  --candidate build/Release --output work/fracture-sdf-build/bench-10 \
  --scene 10 --steps 600 --repeats 2 --width 3840 --height 2160
```

For timber, use `--scene 3` and
`--scene-file tests/scenes/fracture_timber_drop.txt`.

## Validation

- The optimized shaders exactly reproduce 8,280 candidate steps across timber,
  F10, F11, F5 and F7 with TGS, PGS and AVBD and Vulkan synchronization validation.
- Another 2,400 profiled candidate steps match the baseline in timber and F10.
- The repaired CPU references pass 480 timber steps, 600 F10 steps and 120 steps
  each in F5/F6 with synchronization validation. Surface entries and counts
  match exactly; SDF and mass properties satisfy the stated tolerances.
- GPU plan boundary fixtures pass, including a live root in the last partial
  tile and uniform capacity rejection.

- All eight CTest tests pass on the final build.
- Unprofiled benchmarks add 14,400 matching candidate steps across six
  alternating pairs per scene/solver (600 steps each).

## Unprofiled frame results

Six alternating baseline/candidate pairs per scene and solver, 3840×2160.
The first 20 steps are excluded. Means and p95 average the per-run statistics;
maximum is the worst across all six runs. Both the initial two pairs and the
additional four pairs are included, without discarding outliers.
Evidence: `bench-*`, `extended-bench-*`, and `frame-summary.json`.

Intervals between flushed DET records include rendering, publication and the
next simulation step. They do not measure scanout or Moonlight latency.

| Scene / solver | All-frame mean before → after | Publication-frame mean before → after | Publication p95 before → after | Publication worst before → after |
|---|---:|---:|---:|---:|
| Timber / AVBD | 6.539 → 6.548 ms | 9.682 → 9.242 ms | 12.999 → 12.810 ms | 14.120 → 14.502 ms |
| Timber / TGS | 8.489 → 8.606 ms | 12.075 → 11.982 ms | 14.555 → 14.805 ms | 15.912 → 16.979 ms |
| F10 / AVBD | 9.151 → 9.285 ms | 10.960 → 10.954 ms | 13.539 → 13.736 ms | 18.116 → 17.221 ms |
| F10 / TGS | 11.229 → 11.262 ms | 12.879 → 12.463 ms | 14.888 → 14.846 ms | 16.093 → 15.640 ms |

Publication-frame means improve 0.44 ms in timber/AVBD and 0.42 ms in F10/TGS.
F10/AVBD is unchanged; timber/TGS improves only 0.09 ms. Each variant contains 84 timber/AVBD, 60 timber/TGS,
162 F10/AVBD and 336 F10/TGS publication samples.

**This is a construction-kernel improvement, not a demonstrated global FPS
improvement.** The whole-frame means are unchanged or slightly worse, with
approximately 1.4% increases for timber/TGS and F10/AVBD. Several individual
intervals differ by roughly 6–7 ms; presentation and other frame work are
included, but these measurements do not establish the cause. Worst-frame
results are mixed. The data does not support claiming that every fracture
frame or the entire simulation is faster.

Next priorities are the remaining serial mass reduction and plan processing,
connected components, AS command-recording outliers, and the larger simulation /
frame-acquisition spans. Further parallel reductions must retain acceptable
mass-property accuracy and convergence; the exact-replay checks remain the
gate for changes intended to preserve trajectories.
