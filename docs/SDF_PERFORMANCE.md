# SDF pipeline performance

Base: merge cf4a96e, including corrected fragment surface counts.

## Measurement

`BB_RESPAWN_TIMING` now logs every narrow-phase GPU query, including samples below
5 ms. It also measures the GPU voxel-pool build chain (SDF, surface list and mass
properties) with timestamps read after its existing submit wait. No additional
queue wait is introduced. Queries and logging remain opt-in.

Run the repeatable narrow-phase benchmark with the application closed:

```sh
python3 tools/benchmark_sdf.py --runtime build/Release --output work/sdf-benchmark --repeats 3
```

It checks process completion, sample counts and physics CSV rows, and records
shader/CSV hashes. It clears inherited BB_* and validation-layer overrides.
`--solver 3` selects TGS. GPU intervals exclude startup compilation and rendering.

## Rejected narrow-phase experiments

Moving the identical OBB prefilter outside the six voxel face passes saved about
5-6% of narrow-phase GPU time on the RTX 4090 (AVBD, 300 steps, three repeats):

| Scene | Original | Rejected candidate |
| --- | ---: | ---: |
| Weak-frame fracture fixture | 0.31373 ms | 0.29516 ms |
| F5 | 0.46715 ms | 0.44199 ms |
| F6 | 0.49393 ms | 0.46962 ms |

All nine SDF physics CSVs matched, but repeated F7 controls exposed intermittent
failure to put the full pool to sleep. The original slept all 432 dynamic bodies
in four controls; the early-return candidate failed in two of five runs. Keeping
the six helper early-outs and merely caching the prefilter boolean also failed
in one of three runs. Neither candidate is retained. The reason a change in the
voxel branch affects this OBB scene is not established.

Direct interior node loads and a shared per-pair SAT table were slower; SAT loop
unrolling did not improve every scene. These experiments are also discarded.
The shipping narrow-phase source remains identical to the merge baseline.

Evidence on the simulation host: `work/sdf-performance/`. `base-*` are valid
baselines; the initial `baseline-*` attempt used an empty scene-file override and
must be excluded. `final/` contains the subsequently rejected prefilter variant,
not a validated shipping result.

## Draft SDF build optimization — stability gate unresolved

The axis distance transform skips a node whose input squared distance is already
zero. All candidate distances are nonnegative, so its exact minimum and existing
output are zero. Each thread owns its entire column, and the input column is
loaded before writes. Dispatches, barriers and the contact shader are unchanged.
RTX 4090, three alternating original/optimized repetitions. GPU intervals cover
all pool-build kernels and their barriers, excluding CPU publication and readback.
For the weak-frame fixture, each run's mean excludes its initial scene build and
includes its seven fracture rebuilds; the table uses the median of these means.
For F5/F6/F9 it is the median of the initial scene builds.

| Build workload | Original | Optimized | Reduction |
| --- | ---: | ---: | ---: |
| Weak-frame fracture rebuilds | 0.05032 ms | 0.04743 ms | 5.74% |
| F5 initial pools | 0.08250 ms | 0.08112 ms | 1.67% |
| F6 initial pools | 0.04845 ms | 0.04797 ms | 0.99% |
| F9 initial pools | 0.29421 ms | 0.29085 ms | 1.14% |

These are microsecond savings, not a fix for the entire fracture stall. The F9
measurement is an intact scene build, not an interactive fracture latency test.
The zero shortcut leaves the algorithm's worst-case quadratic column scan intact.

All 18 300-step F3/F5/F6 physics CSVs (original and optimized) match the original
baseline byte-for-byte. CPU SDF verification on F5, F6 and F9 reports maximum
absolute error zero in all repetitions. Surface-list verification also passes.
Raw alternating build evidence: `work/sdf-performance/build-*.log`,
`build-*.csv`, `build-results.json`, `build-ab.sh`, `zero-validation.sh`.


## Validation and unresolved gate

Both AVBD and TGS complete 900 fixed fracture steps with Vulkan synchronization
validation and pool checks, without reported validation errors. Their full CSVs
match the merged contact-fix baseline. Independent capture analysis reports no
surface-list/count errors for either solver. All four CTest targets pass.
The updated benchmark tool also completes an actual three-scene run, including
build timestamp collection (`shipping/`; the directory name does not mean approval).

However, this candidate is **not ready to merge**. With the original contact shader
and the zero-distance build shortcut, three F7/AVBD repeats ended with 23, 432 and
29 sleeping bodies. Five subsequent controls with both original shaders slept all
432 bodies. F7 contains only OBBs and does not dispatch the SDF build shader; the
causal mechanism is unresolved. No claim that this is harmless variability or a
proven SDF arithmetic regression is justified. The earlier prefilter experiments
are likewise withheld rather than declared safe based only on their SDF CSVs.

The draft preserves the small candidate and instrumentation for review, not as a
validated replacement for the merged simulation. Investigate the F7 discrepancy
before accepting any optimization. This PR does not resolve the full fracture stall.
