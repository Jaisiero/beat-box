# GPU-resident fragment occupancy

Runtime fracture now keeps the cell labels and cropped occupancy on the device.
The production path no longer allocates a label staging buffer or copies the full
occupancy pool back to the GPU after a split. Occupancy, SDF and surface storage
use device buffers rather than host-mappable allocations. Scene creation uploads
only the initial occupancy; SDF and surfaces are built on the device.

## Data flow

1. Voronoi, connected components and census run on GPU. CPU consumes the compact
   component manifest at the existing label-completion boundary.
2. CPU retains deterministic sliver merging, component ordering and slot ownership.
   It checks capacity for **all** children against shadow free lists before changing
   the parent. A failed reservation preserves the whole parent.
3. CPU sends a sorted source-label/target-label map (8 bytes per component) and
   cropped shape metadata. A GPU invocation packs one complete output occupancy
   word, including zero padding. Reused slices need neither atomics nor a separate
   clear pass.
4. Packing is recorded at the beginning of the next label or SDF command list.
   It adds no production queue submission or CPU wait. SDF, surfaces, inertia,
   finalization and AABB generation consume the result on GPU.
5. CPU still records the existing BLAS/TLAS build commands and publishes bodies.

There is deliberately no claim of a fully GPU-driven fracture pipeline yet.
Live-body synchronization, merging, allocation ownership, body/shape metadata
uploads, and the AS command descriptors remain host responsibilities. The new
`GpuPoolTransaction` layout runs as host-side admission today; a real compute
validation also exercises its reserve/retire/publish/rollback methods. Migrating
its production ownership requires coordinating culling, spawning, publication
and slot retirement together, rather than uploading allocator snapshots per split.

## Synchronization and lifetime

- `carve_and_label` completes MAIN before CPU reads the compact census or reuses
  the mapped remap buffer. Any earlier queued packing is recorded before the new
  label dispatch and completes within that same submission.
- Packing reads the previous labels and writes disjoint cropped occupancy words.
  COMPUTE read/write dependencies publish occupancy and order label readers before
  scratch overwrite by another fracture or EDT build.
- The final fracture's packing runs in the SDF-build submission. Its existing
  completion wait remains the boundary before AS preparation/publication.
- Parent pool slices remain allocated throughout child admission and allocation.
  They are retired only after every child's offsets have been assigned. Packing
  reads labels, not parent occupancy; subsequent label work cannot overtake it.
- Initial occupancy uses a staging transfer with TRANSFER_WRITE -> COMPUTE_READ.
  Its scene-load-only wait protects staging lifetime. Runtime fracture has no
  occupancy upload.
- A transaction's `publish()` changes allocator metadata; it is **not** a Vulkan
  barrier and does not authorize early reuse of resources referenced by in-flight
  command buffers. Retirement is forbidden during reservation/construction.

## Recycled-tail correction

The CPU pool helper previously called either `resize` or `fill`. An allocation
starting inside the old vector but ending beyond it therefore retained stale bits
in its reused prefix. F11 exposed this at step 1126: GPU packing wrote complete
words correctly, while the CPU reference ORed into an uncleared word. The helper
now grows storage if needed and always clears the entire allocation. A dedicated
regression covers partial reuse plus growth, complete reuse, and pure append.
F11 also exposed empty-vector uploads during its initial no-dynamic-body state;
`allocate_fill_copy` now skips those forbidden zero-byte Vulkan copies.

## Exceptional and diagnostic paths

Census output holds 128 components. Overflow downloads labels and reconstructs the
complete manifest on CPU; it never truncates fragments. This exceptional path adds
one readback submission/wait. `BB_CENSUS_VERIFY` or `BB_SDF_VERIFY` also downloads
labels and compares GPU occupancy word-for-word with the original CPU packing.
The contact-geometry export explicitly reads live GPU occupancy, since the host
occupancy mirror is no longer authoritative after fracture.

`BB_POOL_VERIFY` runs a small compute allocator check once: reserve all pools,
reject premature retirement/publication, publish a valid replacement, then exhaust
the last pool and verify that committed allocations survive unchanged. CPU tests
exercise 100,000 randomized operations against an independent bitmap plus free-list
metadata exhaustion, overlaps, integer overflow and transaction rollback.

## Validation and performance

The final paired benchmark used 10 trials per solver, alternating baseline and
candidate with warm shader caches and verification/validation disabled. Baseline
is PR28 (`ebb1d8c`); both variants used `fracture_frame_drop.txt` and identical
pre-fracture traces. Results below are medians in milliseconds.

| Solver | Phase | Baseline | Candidate |
| --- | --- | ---: | ---: |
| AVBD | Split | 0.1999 | 0.2019 |
| AVBD | Whole fracture event | 1.3250 | 1.4348 |
| TGS | Split | 0.2262 | 0.2156 |
| TGS | Whole fracture event | 1.5897 | 1.5427 |

This is a transfer/ownership migration, **not a demonstrated overall speedup**.
AVBD's whole-event median is about 0.11 ms higher in this small fixture; TGS's is
about 0.05 ms lower. Whole-event ranges overlap substantially (AVBD baseline
1.17–1.68 ms, candidate 1.18–2.25 ms; TGS baseline 1.23–2.35 ms, candidate
1.25–2.04 ms). The SDF/publication interval now includes GPU packing, so its
increase must not be interpreted as a slower SDF kernel alone.

The reservation workspace is persistent; each fracture copies only active free
ranges and allocator headers instead of allocating/clearing a new 98 KB object.
The CPU tail-clear correction was added after this timing run; it affects recycled
storage and the diagnostic oracle, not this fixture's first-fracture allocations.

Validation: eight CTest tests pass. Both solvers match all 900 deterministic steps
and CSV records of PR28 in the fracture fixture, with and without diagnostic
readbacks. Both match the 600-step F6 SDF fixture. GPU packing matched 27 AVBD and
25 TGS dirty-shape outputs word-for-word, with no Vulkan synchronization validation
errors in those runs. F11 additionally exercises spawning, culling and recycled
pool slices: 1,500 steps per solver, 131 AVBD and 194 TGS packing checks, zero
Vulkan errors and zero oracle failures after the recycled-tail fix. A 4K F9
drag/release chain passed 57 packing checks; the F12 contact capture contained
all 1,740 original solid voxels across its live bodies.

Server evidence: `/root/beat-box/work/gpu-packing/`, including `paired-reuse`,
`reuse-verify*`, `reuse-normal*`, `steady-*` and `ctest-final.log`.
