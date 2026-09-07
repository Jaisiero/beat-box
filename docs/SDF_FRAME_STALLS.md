# SDF contact cost after fracture

Fracturing increases the number of touching voxel bodies. The old narrow phase
walked a pair's surface list six times, repeating its SDF samples, neighboring-cell
search and 15-axis SAT for each directional manifold. Most of that work does not
depend on the destination manifold. In the measured F10 runs it cost substantially
more than the changed BLAS builds.

The new sampler traverses each surface sample and neighboring cell once. Near
contacts go to the dominant MTV direction's reducer; deep SDF contacts retain the
original gradient membership and may enter multiple reducers. OBB contacts retain
all six face tests. Each reducer receives the same ordered subsequence as before,
including tie breaks, normal accumulation, overlap/interior counters, margins,
penetration caps and feature IDs. Manifold emission and wedge filtering still run
in face order. No solver iterations, contacts, physics steps or fractures are
skipped. This adds no GPU buffers, transfers or synchronization boundaries.

`voxel_finish_axis_manifold` now only finishes the already collected reducer;
`voxel_pair_collision_detection` owns the shared traversal and six reducers.

## Measurements

RTX 4090, LXC 110, baseline merged PR #36 (`5bd1397`). Both variants use identical
instrumentation code. Tests ran without concurrent builds or other simulations.
The narrow-phase measurements use GPU timestamps and the existing simulation
completion wait. Frame measurements below disable profiling and Vulkan validation.

Mean narrow-phase GPU time, 480-step replays, first 20 steps excluded:

| Scene | Solver | Six traversals | Shared traversal |
|---|---|---:|---:|
| F9 timber drop fixture | AVBD | 1.167 ms | 0.414 ms |
| F9 timber drop fixture | TGS | 1.151 ms | 0.374 ms |
| F10 breakable pool | AVBD | 2.191 ms | 0.709 ms |
| F10 breakable pool | TGS | 1.496 ms | 0.506 ms |

Unprofiled 3840x2160, two alternating pairs per solver, 600 steps each, first 20
excluded. Values are averages of each run's statistic, not pooled percentiles:

| Scene | Solver | All-frame mean, before → after | Publication-frame p95, before → after |
|---|---|---:|---:|
| F9 timber drop fixture | AVBD | 8.403 → 7.407 ms | 15.172 → 13.830 ms |
| F9 timber drop fixture | TGS | 10.540 → 9.443 ms | 16.543 → 15.283 ms |
| F10 breakable pool | AVBD | 12.099 → 10.009 ms | 15.785 → 13.833 ms |
| F10 breakable pool | TGS | 13.243 → 11.811 ms | 16.040 → 15.458 ms |

These are externally measured intervals between flushed deterministic-step
records. They include publication, rendering and the next simulation step, but
are not scanout or Moonlight latency. Publication samples per run: 10 for timber,
27 for F10 AVBD and 56 for F10 TGS. TGS still has occasional full-frame intervals
over 16.7 ms; this is not a universal frame-time guarantee. The repeatable tests
have not reproduced a long whole-image freeze. Interactive confirmation is still
needed to attribute the user's remaining perceived pause conclusively.

A separate normal-mode F9 exercise grabs the steel club (GPU pick ID 4), lifts,
swings and releases it into the frame, then observes six seconds of chain fracture.
Two alternating pairs per solver verified the input and successful fragmentation.
Candidate AVBD had no measured stepped frame over 16.7 ms in either run (worst
16.067 ms); candidate TGS still had seven such frames across both runs (worst
19.250 ms). The remaining TGS peaks include render/acquire and substep time.
These mouse trajectories are wall-clock driven and fracture counts differ, so
this is functional stress evidence, not an exact-workload speed comparison.
The deterministic benchmark above supplies the controlled performance comparison.

The timber fixture uses F9's 24x16x4 geometry, 0.25 voxel size, density 12 and
strength 30. The previous `fracture_frame_drop.txt` is a smaller 8x8x2 frame and
remains useful as a separate regression case.

## Diagnostics and reproduction

`BB_FRAME_TIMING=1` prints CPU front/acquire, simulation, post-simulation edits,
final synchronization, render submission and garbage collection durations on
stepped frames. The edits interval also includes readback and diagnostic output.
It enables existing narrow-phase and solver-stage GPU queries, read after the
existing completion wait. Unlike `BB_RESPAWN_TIMING`, it does not add a synchronous
SDF-builder timestamp read. Diagnostic output/query overhead means those runs
must not be compared with unprofiled frame benchmarks.

Run the unprofiled comparison on the GPU host (both runtime directories must have
an executable and matching shader/include trees):

```sh
DISPLAY=:0 python3 tools/benchmark_sdf_frames.py \
  --baseline /path/to/reference-runtime --candidate build/Release \
  --output /path/to/results --scene 10 --steps 600 --repeats 2

DISPLAY=:0 python3 tools/benchmark_sdf_frames.py \
  --baseline /path/to/reference-runtime --candidate build/Release \
  --output /path/to/timber-results --scene 3 \
  --scene-file tests/scenes/fracture_timber_drop.txt --steps 600 --repeats 2
```

The benchmark requires xdotool, verifies the target window's 4K geometry, rejects
incomplete/invalid replays and requires exact state equality across variants and
repetitions. It saves raw logs and every measured interval, including maxima and
counts over a 60 Hz budget. Performance is measured separately from validation.

## Validation

- 4,080 candidate steps exactly match the six-traversal implementation across
  timber fracture, F10, F11 recycling and F5 concave contacts, with both solvers.
- All 4,800 candidate steps in the paired 4K benchmark also match exactly.
- 10,320 additional candidate steps pass Vulkan synchronization validation and
  exact production/oracle comparisons for F3 fracture, F10 and F11 recycling,
  with both solvers. GPU census, allocation, finalization and primitive-generation
  oracles report matches; no validation errors or solver invariant violations.
- All eight CTest tests pass.

Raw measurement and validation artifacts are retained on the test host under
`/root/beat-box/work/fracture-frame-stalls/`.
