# SDF pair prefilter optimization

## Implementation

A voxel pair can emit six directional manifolds. Its unchanged OBB bounds test
now runs once before those six passes, instead of inside each pass. Rejected
pairs emit nothing; accepted pairs retain the same sample/SAT/manifold order,
thresholds and warm-start behavior. The monitored-pair coverage bit is set before
the rejection. No buffer, shader ABI, dispatch count or barrier changes.

`NarrowPhaseSdfInfo` compiles this as an internal specialization using
`BB_SDF_PAIR_PREFILTER`. It is not a user setting or environment override.
`SceneManager::load_scene` updates the scene's voxel presence on every load;
`RigidBodyManager` chooses the SDF pipeline for scenes with voxel shapes and the
original pipeline for OBB-only scenes, including switches back from F9 to F7.
Fracturing an existing voxel scene retains the SDF selection.

Without the specialization define, the collision source has exactly the same
non-comment tokens as merge baseline e45d8b6. The original OBB compilation path is
preserved while SDF optimizations can evolve independently. There are two compiled
variants of the same entry point, not extra dispatches per simulation step.
An edit of collision detection can therefore rebuild two pipeline variants.
`voxel_sdf.slang` remains unchanged; the earlier build shortcut is not retained.

## Measurement

`BB_RESPAWN_TIMING` logs all narrow-phase query samples (formerly only >=5 ms)
and the voxel-pool GPU build chain. Queries are read after existing submit waits;
there are no additional waits. All instrumentation remains opt-in.

With the application closed, on the Linux simulation host:

```sh
python3 tools/benchmark_sdf.py --runtime build/Release --output work/sdf-benchmark --repeats 3
```

The tool clears inherited BB_* and validation-layer overrides, checks completion,
query counts and CSV lengths, and records source/CSV hashes. F3's weak fracture
fixture, F5 and F6 run for 300 fixed steps; F7 runs for 1800 and reports first full
sleep, sleep count at 900, and whether full sleep persists. `--solver 3` selects
TGS; `--steps` and `--pool-steps` change measurement lengths, not solver iterations.
F7 has no SDF builds. A late full sleep does not erase a failed 900-step checkpoint.

RTX 4090, AVBD, three runs per scene. Values are medians of per-run mean GPU
narrow-phase time. The original was measured in three alternating original/early-
candidate runs; the final isolated variant was remeasured in three runs after the
C++ build. Startup compilation, rendering and validation-layer overhead are excluded.

| Scene | Original | Final SDF variant | Reduction |
| --- | ---: | ---: | ---: |
| F3 fracture fixture | 0.31322 ms | 0.29509 ms | 5.79% |
| F5 | 0.46470 ms | 0.44246 ms | 4.78% |
| F6 | 0.49114 ms | 0.46999 ms | 4.31% |

The earlier alternating candidate independently measured 5.90%, 4.69% and 4.21%.
Savings are approximately 18–22 microseconds per step in these scenes. They are
not whole-frame speedups or a bound on arbitrary interactive F9 fracture stalls.
The six surface scans remain; this is an incremental optimization.

## Validation and limits

All nine final SDF physics CSVs match their original controls byte-for-byte.
AVBD and TGS both complete the 900-step fracture fixture with Vulkan synchronization
validation and pool verification without reported errors; their full CSVs match
the merged contact-fix baseline. Four CTest targets pass. Independent surface-list/count checks on both captures
also pass. A 60-second interactive F9 test at 4K performed 11 fracture publications;
its capture contained 34 bodies with valid surface lists and no reported pool errors.
This smoke test is not a controlled F9 performance or convergence comparison.

F7 settling remains variable and is not claimed fixed by this PR. Eight original
controls in the current campaign reached full sleep by 1800 steps. The initial
single-pipeline candidate completed two of three, leaving 15 sleepers in the other.
The final isolated version reached full sleep at steps 576 and 617 in two runs;
one run still had 26 sleepers at 1800, despite using the original OBB source path.
This remains an unresolved stability limitation, not evidence of a specific driver
bug, memory race or SDF arithmetic error. The PR remains a draft for review.
No sleep tolerance or solver iteration budget has been changed to hide the result.

Previous controls also found a failure of the original at the 900-step checkpoint
(it settled at 1094), so the earlier categorical attribution to the unused SDF
build shortcut was premature. That shortcut remains withdrawn.

Evidence: `/root/beat-box/work/sdf-pair-prefilter/` contains alternating runs,
`isolated/results.json`, final validation logs/CSVs and build logs. Earlier evidence
is in `work/sdf-performance/` and `work/f7-regression/`; historical directory names
`final/` and `shipping/` do not imply approval. Initial `baseline-*` runs using an
empty scene-file override were invalid and are excluded.
