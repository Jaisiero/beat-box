# Fragment census and surface authority

## Changes

Large fracture grids now compute component voxel counts, integer coordinate sums,
and inclusive bounding boxes on the GPU. The census follows connected-component
labeling in the same submission; it does not add a completion wait. Within each
subgroup, equal labels are reduced before issuing global atomics, reducing
contention on a component's counters. A compact summary is read only after the
existing label-publication wait and a write-to-host-read barrier.

The host sorts summaries by label, preserving deterministic component identities,
centroid ties, and largest-fragment selection. The existing nearest-centroid
merge of components smaller than three voxels is unchanged. Merged counts and
bounds are unions of summaries, replacing the subsequent per-cell census and
bounding-box passes.

Grids smaller than 1024 cells use an exact, single-pass CPU census. This avoids
three GPU dispatches when their cost exceeds the small amount of host work.
The cutoff is a conservative local heuristic, not an architecture-independent
optimal crossover. `BB_CENSUS_VERIFY=1` deliberately forces GPU census even for
small grids, verifies exact results, and reports stage timing.

Surface voxel counting is now authoritative in the existing GPU surface builder.
The duplicate CPU neighbor checks are executed only for census/SDF validation.
The PR27 finalization hook already refreshes the GPU-written shape record on the
host before the next split; no extra readback is introduced. The previously
unused occupancy readback from `carve_and_label` is removed. Per-cell labels are
still downloaded because cropping and occupancy packing remain on the CPU.

## Synchronization and bounds

- Labeling writes precede census initialization/accumulation through the existing
  compute-write to compute-read/write barrier.
- Initialization, accumulation, and compaction have separate dispatches with
  explicit compute-write to compute-read/write barriers. No workgroup or subgroup
  operation substitutes for those inter-dispatch barriers.
- Subgroup reductions include partial-dispatch lanes as inactive contributions;
  only one lane performs global atomics for each distinct subgroup label.
- The final write-to-host-read barrier covers both compute-written summary data
  and transfer-written label staging. The existing MAIN submission wait completes
  before either is read. Scratch/output buffers are reused only after completion.
- The compact summary holds 128 components. A larger count falls back to the full
  host-label census; it never truncates components or drops geometry.
- Scratch is bounded by `BB_MAX_VOXEL_SDF_F32S`: 65536 records of 44 bytes, about
  2.75 MiB. Coordinate sums fit uint32 because each coordinate is at most its
  linear cell index, so the sum is bounded by `N*(N-1)/2`. A static assertion
  prevents increasing the capacity beyond this bound without revisiting types.

## Validation

`BB_CENSUS_VERIFY=1` compares all component fields bit-for-bit against a CPU
reference and checks finalized surface counts against the old CPU neighbor test.
It also runs four synthetic GPU cases once: a non-cubic partial dispatch, 257
components (overflow fallback), the maximum 65536-cell coordinate sum, and an
empty component set. Validation adds work and is excluded from end-to-end timing.

All seven CTest tests pass. Fracture replay runs complete 900 steps per solver
in both the production small-grid path (incremental AS) and forced-GPU validation
path (full AS). Every physics CSV row and all 900 DET checkpoints match PR27
within each solver. The 600-step F6 CSV also matches PR27 for both solvers, and
all twenty short benchmark CSVs match within each solver. Vulkan synchronization
validation reports no errors, including the GPU edge cases and surface checks.
The final 4K F9 drag/release chain reached 34 bodies, checking ten censuses and
their finalized surface counts without validator errors; debris remains visible.

The CPU test covers expected counts/sums/bounds, canonical ordering, overflow
recovery, maximum sums, and empty output. Shader dependency checks limit census
algorithm changes to its three entry points.

## Measurements and scope

The first unconditional GPU implementation was not useful for tiny grids. In a
128-cell fixture, GPU census took roughly 8 microseconds while the fused CPU
reference took 2–3 microseconds. This is why the production path keeps small
cropped fragments on CPU.

An instrumented F9 chain at 3840x2160, without Vulkan validation, measured four
1536-cell censuses: GPU 6.624–7.936 microseconds versus CPU reference
9.020–14.810 microseconds. Two 1216-cell censuses measured GPU 8.192–8.960 versus
CPU 9.410–10.290 microseconds. These are stage-local samples, comparing GPU
execution timestamps against a CPU wall-clock census; they exclude label
transfer, completion waits, and the rest of fracture processing. They are not
a whole-frame or solver speedup claim.

A final paired benchmark against PR27 (`d0d6457`, included in merge `4a698a8`)
uses ten repetitions per solver/variant, alternating order, a warmed shader cache,
and the 128-cell fracture fixture at 860x640. Validation is disabled. Because
this fixture is below the GPU cutoff, it measures the fused CPU census and the
removal of redundant CPU surface counting, not a GPU-census speedup.

| First fracture phase, median | PR27 | This branch |
| --- | ---: | ---: |
| AVBD split preparation | 0.2134 ms | 0.2055 ms |
| TGS split preparation | 0.2133 ms | 0.1986 ms |
| AVBD entire event | 1.4160 ms | 1.3113 ms |
| TGS entire event | 1.6459 ms | 1.3506 ms |

The preparation phase includes labeling, its existing wait, census, merging, and
CPU packing. Entire-event medians are noisy: AVBD ranges are 1.135–1.777 ms versus
1.177–1.802 ms; TGS ranges are 1.169–1.925 ms versus 1.191–2.272 ms. The much larger
apparent total reduction cannot be attributed to the small census saving; AS
publication is unchanged and varies substantially. There is no demonstrated
large whole-event or whole-frame speedup.

```sh
python3 tools/benchmark_fragment_finalization.py \
  --baseline /path/to/pr27-beat-box --candidate build/Release/beat-box \
  --cwd build/Release --fixture tests/scenes/fracture_frame_drop.txt \
  --output work/gpu-census/final-paired --trials 10 --skip-steady
```

Evidence is retained under `/root/beat-box/work/gpu-census/`, including experiments,
replay CSVs, validator logs, and paired timings. The benchmark utility now records
`first_fracture` (sync/split/publish/total) in addition to respawn timing and accepts
`--skip-steady` for changes that execute only when something fractures.

The next migration boundary is remapping and packing cropped occupancy on GPU.
That requires preserving labels across multiple events and removing the host
occupancy upload that would otherwise overwrite GPU output. Pool allocation and
Vulkan AS object management remain host operations in the current architecture.
Steady SDF narrow-phase/solver cost is unchanged by this fracture-only work.
