# SDF profiling and F7 regression investigation

Base: merge cf4a96e, including corrected fragment surface counts.

## Final change

Only measurement/tooling changes remain. Both `collision_detection.slang` and
`voxel_sdf.slang` are byte-identical to the merge baseline. The attempted
zero-distance shortcut has been withdrawn; there is no retained solver or SDF
algorithm change and no claimed production speedup in this PR.

`BB_RESPAWN_TIMING` logs every narrow-phase GPU query, including samples below
5 ms. It also measures the voxel-pool GPU build chain (SDF, surface list and mass
properties). Results are read after the existing submit wait; no extra queue wait
is introduced. Instrumentation remains opt-in.

```sh
python3 tools/benchmark_sdf.py --runtime build/Release --output work/sdf-benchmark --repeats 3
```

Close the application first. The tool checks completion, timestamp counts and CSV
rows; records shader/CSV hashes; clears inherited BB_* and validation overrides;
and measures F3's fracture fixture, F5 and F6 for 300 steps. It additionally runs F7
for 1800 steps and records first full sleep, the original 900-step checkpoint and
whether full sleep persists. `--steps`, `--pool-steps` and `--solver 3` customize
measurement. A later full-sleep result does not erase a failed 900-step checkpoint.
F7 has no voxel builds. Timings exclude startup compilation and rendering.

## Withdrawn experiments

Moving the OBB prefilter outside six voxel face passes reduced narrow-phase GPU
time by about 5-6% on RTX 4090: F3 0.31373 to 0.29516 ms, F5 0.46715 to 0.44199 ms,
F6 0.49393 to 0.46962 ms (median of three run means). Nine SDF CSVs matched, but
F7 did not consistently meet the existing 900-step criterion. Keeping the six
helper early-outs and only caching the boolean also failed that gate. Both were
withdrawn. Direct node loads and a shared SAT table were slower; SAT unrolling
failed to improve every scene. None is retained.

Skipping an axis-transform node whose squared distance was already zero was
mathematically exact for the nonnegative distance field. Three alternating runs
measured fracture rebuilds at 0.05032 versus 0.04743 ms; initial F9 builds at
0.29421 versus 0.29085 ms. These microsecond savings did not fix the full fracture
stall. All 18 F3/F5/F6 CSVs matched, and CPU/GPU SDF errors on F5/F6/F9 were zero.
AVBD/TGS completed 900 fracture steps with synchronization validation, pool checks
and independent surface-capture analysis passing. Four CTest targets passed.
Despite those results, this candidate was conservatively withdrawn during the F7 investigation.

## F7 findings and corrective action

The first interactive observation was made during settling and was insufficient
to diagnose persistent instability. In controlled 35-second interactive runs,
**both original and candidate reached 432 sleeping bodies**. Neither remained in
permanent motion. A suspicion that AVBD's maximum-velocity statistic retained an
old peak was rejected: `entry_avbd_prepare` already resets it every step.

Three alternating 1800-step runs then measured:

| Shader variant | First full sleep, by run | Sleeping at step 900 |
| --- | --- | --- |
| Original | 575, 711, 530 | 432, 432, 432 |
| Zero-distance candidate | 879, 965, 1255 | 432, 26, 28 |

All six ended fully asleep. This small sample initially suggested slower settling
under the candidate, but the expanded original-shader controls below invalidate
the claim that failure at 900 steps uniquely identifies a candidate regression.
There is no evidence here of NaNs, memory corruption or endless instability.
F7 contains only OBBs and does not dispatch the modified SDF build shader, so the
causal mechanism is **not established**. Earlier checkpoints show matching main
solve hashes and a divergence after post-stabilization; this localizes that
comparison but does not prove a driver/compiler defect or a specific race.

Three further original-shader runs (the updated benchmark, `reverted/`) reached
full sleep at steps **790, 752 and 1094**. At step 900 they had 432, 432 and 21
sleeping bodies. All three stayed fully asleep after settling, and all nine SDF
CSVs matched the merged baseline. Thus the original also fails the old 900-step
gate. Attribution to the unused SDF shader is **not demonstrated**, and withdrawal
must not be described as a proven fix for F7 settling variability.

The conservative implementation action is to restore the original shader. The
diagnostic correction is to distinguish a failed settling-time checkpoint from
persistent instability and avoid attributing it to a change without adequate
controls. Sleep thresholds and solver iterations are unchanged. The benchmark now exposes both settling
time and the 900-step gate so eventual success cannot hide this difference.
The original interactive configuration is restored after verification.

Evidence on host: `work/sdf-performance/` (initial experiments) and
`work/f7-regression/` (real-*, settle-*, reverted/). The earlier directory names
`final/` and `shipping/` contain subsequently withdrawn candidates, not approvals.
Initial `baseline-*` runs with an empty scene-file override are invalid; use
`base-*` instead. The benchmark omits that variable for built-in scenes.
