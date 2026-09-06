# TGS with SDF: profiling and task batching

## Findings

TGS and AVBD share the once-per-step narrow phase. TGS does not resample the SDF
at every substep: its solver reuses prepared contacts and updates separation from
rotated local anchors and current poses. Different trajectories do generate
different contact counts, so cross-solver narrow-phase times are not identical-work
comparisons.

The default TGS configuration has eight substeps, each with a warm-start sweep,
two biased solve sweeps and two relaxation sweeps. Including initial preparation,
that is 41 color sweeps, each recording 32 color dispatches: 1312 colored solver
tasks per step, plus overflow and integration tasks. AVBD already batches its
color sweeps in C++.

TGS colors contact edges rather than bodies. Multiple directional SDF manifolds
for a body pair compete for colors. Each active color dispatch scans the manifold
range and filters by color; unused colors have zero workgroups but still carry
command and synchronization overhead. F6's sampled PERF checkpoints use only 2-3
colors out of 32 and show no overflow. Therefore serial overflow is not the
bottleneck demonstrated by this fixture. A future compact contact list or different
GPU scheduling scheme needs its own performance and convergence validation.

## Accepted change

Record each TGS color sweep as one task and bind its pipeline once: 1312 colored
solver task entries become 41. All indirect dispatches, color ordering, overflow
passes, eight substeps, two solve/relax sweeps and floating-point shader operations
are unchanged. The serial diagnostic still bypasses colored sweeps and uses the
original overflow path.

Within each sweep, global compute-write to compute-read/write barriers publish
body velocity and manifold impulse updates to the next color. Dispatch arguments
and color IDs are read-only. GraphColorSolveTaskHead keeps its existing resource
access declarations, so Daxa supplies the final dependency to overflow/integration
and the producer-to-sweep indirect argument visibility. There is no new CPU/GPU
readback or wait.

## GPU profile

Extend existing opt-in BB_RESPAWN_TIMING with TGS-STAGES, using the same query
pool and existing readback wait as AVBD. The pending record retains its solver ID
so the log labels match the executed graph. Six ordered markers delimit:

- setup: reset/sorting/BVH/broad and narrow phase/contact-chain sorting;
- prepare: advection/islands/sleep/contact coloring and dispatcher preparation;
- contacts: initial TGS contact preparation including overflow;
- substeps: velocity integration, warm start, biased solves, position integration
  and relaxation, including overflow;
- finalize: contact debug/publication.

Narrow-phase time is included in setup. Profiling is disabled in normal operation.
GPU intervals exclude host recording, readback latency and rendering.

RTX 4090, median of three per-run means, 300 fixed steps (1800 for F7):

| Scene | Setup before | Preparation before | Contact preparation before | Substeps before | Substeps batched |
| --- | ---: | ---: | ---: | ---: | ---: |
| F3 fracture fixture | 0.4224 ms | 0.1428 ms | 0.0343 ms | 1.2473 ms | 1.2355 ms |
| F5 | 0.4421 ms | 0.1454 ms | 0.0251 ms | 0.8729 ms | 0.8764 ms |
| F6 | 0.5979 ms | 0.1482 ms | 0.0308 ms | 1.1033 ms | 1.1072 ms |
| F7 | 0.1948 ms | 0.1827 ms | 0.0407 ms | 1.5574 ms | 1.5628 ms |

GPU solve cost is essentially unchanged by batching. For context, AVBD's F6 main
solve plus post-stabilization measured approximately 0.424 ms in the preceding
campaign. TGS still has substantially more GPU stage time with the current solver
configuration; host batching does not claim to eliminate that remaining gap.

## Total simulation step

Three alternating runs per variant use F6, 20 seconds, autostart, profiling and
validation disabled. The existing PERF sim measurement includes CPU recording,
submission, GPU work and readback, excluding rendering. Per-run means weight each
reported interval by its timed-step count.

| Variant | Run means | Median |
| --- | --- | ---: |
| TGS original | 4.978, 4.504, 4.434 ms | 4.504 ms |
| TGS batched | 3.667, 3.871, 3.631 ms | 3.667 ms |
| AVBD, same session | 2.626, 2.611, 2.721 ms | 2.626 ms |

TGS step cost drops **18.6%**. It remains approximately **39.7% slower than AVBD**
in this scene/configuration. These are whole-step results for each solver's own
trajectory, not equal-contact or equal-accuracy microbenchmarks: TGS logs 619 timed
active steps before sleep, AVBD 216. The physical comparisons proving batching
safe are the same-solver fixed-step replays below. Host timing noise is visible.

## Reproduction and checks

```sh
python3 tools/benchmark_sdf.py --runtime build/Release --output work/tgs-benchmark --solver 3 --repeats 3
```

All full checkpoint traces and physics CSVs agree across three baseline and three
batched runs for F3, F5, F6 and F7. An additional profiling-disabled F7 replay also matches: all seven reach full
sleep at step 885 and remain asleep through 1800. No sleep/contact tolerances or solver budgets are
relaxed. Shader code is unchanged.

AVBD and TGS each complete 900 fracture-fixture steps with Vulkan synchronization
validation and pool verification, without reported errors. Their complete CSVs
match the pre-change controls; independent surface-list/count checks also pass.
All five CTest targets pass. A 45-second interactive TGS F9 test at 4K makes
nine fracture publications; the final capture contains 30 bodies with valid
surface lists/counts and no reported pool errors. This manual smoke test is not
a controlled convergence or latency benchmark.

Raw evidence is preserved in `/root/beat-box/work/tgs-sdf/`: `baseline/`,
`batched/`, `validation-*`, `no-profile-f7.*`, `realtime-*`, build and test logs.
