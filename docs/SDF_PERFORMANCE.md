# SDF and solver pipeline optimization

## Changes

A voxel pair can emit six directional manifolds. The unchanged OBB bounds test
runs once before those passes instead of once inside each pass. Rejected pairs
emit nothing; accepted pairs retain the sample/SAT/manifold order and thresholds.
The monitored-pair coverage bit is set before rejection. There is one narrow-phase
pipeline; the provisional SDF specialization and scene-selection state are removed.
`voxel_sdf.slang` is unchanged. The earlier build shortcut remains withdrawn.

AVBD records one C++ task per primal color sweep instead of one per color.
With the current configuration this replaces 800 primal task entries with 25,
and binds the primal pipeline once per sweep. It preserves all 800 indirect GPU
dispatches, their order, push constants, iteration counts and per-color memory
dependencies. Explicit compute barriers separate colors; Daxa task attachments
provide sweep-boundary and indirect-buffer dependencies.

The convergence defect was uninitialized reference-edge fields in OBB incident
vertices, subsequently consumed as contact-history IDs. Both fields are now
initialized before clipping. See [CONVERGENCE_ORIGIN.md](CONVERGENCE_ORIGIN.md)
for captures, the causal chain and the synchronization audit. No convergence
thresholds were relaxed to obtain the measurements below.

## Measurement

`BB_RESPAWN_TIMING` reports every narrow-phase query, the voxel-pool build chain,
and five GPU AVBD stage intervals. Queries are read after existing submit waits;
no additional queue wait is introduced. Stage markers use a SimConfig attachment
to remain ordered and exist only when profiling is enabled.

- `setup_ms`: pick/reset, sorting, BVH, broad/narrow phase and contact-chain sort.
- `prepare_ms`: advection, islands, sleep/coloring, warm start and depth preparation.
- `main_ms`: primal/dual iterations.
- `post_ms`: finalization, impact handling and positional post-stabilization.
- `finalize_ms`: debug contacts and body publication.

Narrow phase is included in setup, not additive to it. Instrumented GPU intervals
exclude host recording, waits, rendering and startup compilation. The six SDF
surface scans remain and are still a significant optimization opportunity.

With the application closed on the Linux host:

```sh
python3 tools/benchmark_sdf.py --runtime build/Release --output work/sdf-benchmark --repeats 3
```

The tool clears inherited BB_* and validation overrides, checks full completion,
query counts and CSV lengths, and records shader/CSV hashes. F3's weak fracture
fixture, F5 and F6 run 300 fixed steps; F7 runs 1800. `--solver 3` selects TGS.

RTX 4090, AVBD, three alternating runs of the fixed original bounds-test placement
and fixed pair-prefilter version. Both controls include the contact-ID correction.
Values are medians of per-run mean GPU narrow-phase time:

| Scene | Original placement | Pair prefilter | Reduction |
| --- | ---: | ---: | ---: |
| F3 fracture fixture | 0.31342 ms | 0.29584 ms | 5.61% |
| F5 | 0.46610 ms | 0.44265 ms | 5.03% |
| F6 | 0.49045 ms | 0.47062 ms | 4.04% |

After pair rejection is optimized, the unbatched F6 profile is approximately
0.558 ms setup (including 0.471 ms narrow phase), 0.101 ms preparation,
0.236 ms main solve, 0.187 ms post-stabilization and 0.008 ms publication.
The batched version changes GPU main/post times only to 0.237/0.187 ms:
its principal target is CPU recording overhead, not the solver arithmetic.

For total step cost, three alternating unbatched/batched F6 runs used
`BB_SCENE=6 BB_SOLVER=2 BB_AUTOSTART=1 BB_RUN_SECONDS=20`, with profiling and
validation disabled. Each log contains 216 timed steps before sleep. The existing
`[PERF] sim` interval includes recording, submission, GPU execution and readback;
it excludes rendering. Weighting each interval by its step count gives unbatched
means of 3.130, 3.168 and 3.853 ms, versus 2.643, 2.674 and 2.490 ms batched.
The median drops **3.168 -> 2.643 ms (16.6%)**. Run-to-run host timing noise is
visible; this is a scene-specific step-cost measurement, not a whole-frame or
worst-case fracture-latency guarantee.

## Validation

All checkpoint traces and physics CSVs match across the fixed reference,
pair-prefilter and batched versions: three runs of each, on F3/F5/F6/F7.
Across all 15 F7 AVBD runs, every checkpoint and CSV row is identical. All 432
dynamic boxes sleep at step 591 and remain asleep through 1800. This includes
a profiling-disabled run with Vulkan synchronization validation. Two additional TGS F7 replays also have identical CSVs and reach
full sleep at step 885. These measure this fixture, not universal solver rankings.

Both solvers complete 900 fracture-fixture steps with Vulkan synchronization
validation and pool verification, without reported errors. Independent CPU
occupancy-to-surface-list/count checks pass for both final captures.
The validated 900-step fracture CSVs match unbatched controls for both solvers.
All five CTest targets pass. The replay checker also rejects the original divergent
early-contact captures.
A 45-second interactive F9 smoke test at 4K performs 13 fracture publications;
the final capture has 48 bodies and passes independent surface-list/count checks.
No pool errors are reported. This manual test is not a controlled performance
comparison. No claim is made that finite tests prove every possible SDF scene stable.

Raw evidence: `/root/beat-box/work/convergence-origin/`, including
`fixed-reference-{1,2,3}/`, `fixed-optimized-{1,2,3}/`, `batched/`,
`validation-*`, `tgs-f7-*` and `realtime-*`. Earlier draft performance and
convergence interpretations in `work/sdf-pair-prefilter/` are historical;
the corrected controls above supersede their unresolved F7 conclusions.
