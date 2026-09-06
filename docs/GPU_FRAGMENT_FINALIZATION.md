# GPU fragment finalization

## Scope and ordering

After cropping a fragment, its reduced center of mass and inertia already exist
on the GPU. Previously, respawn allocated a staging buffer, copied all derived
records, submitted the copy, waited, and calculated the final body properties on
the CPU. It then uploaded the corrected shape origins again.

`fragment_finalization.slang` now consumes those derived records directly. It
calculates mass, inverse inertia, COM recentering, bounds, and the rigid velocity
shift `v_new = v_parent + omega x delta_position`. It wakes the affected body and
seeds its instance transform. The shader runs in the existing voxel-AABB
submission, before AABB generation and BLAS/TLAS construction. It serves both
AVBD and TGS; it does not change either solver's contact equations.

The AS upload hook now receives the primitive scratch, body scratch, and instance
buffers. Both full and incremental AS preparation invoke it **after all CPU
writes**. Calling it at the old location would let later host uploads overwrite
the GPU-finalized bodies and transforms.

Synchronization is explicit:

1. The existing derived-build completion boundary remains.
2. The finalizer reads derived data and uploaded body/instance inputs, behind a
   write-to-compute read/write barrier.
3. A compute-write to compute-read barrier publishes new shape origins to voxel
   AABB generation.
4. Existing write-to-read, AS-build-read, and host-read barriers publish the
   outputs. The existing primitive-generation completion wait remains.
5. Only after that completion does the CPU refresh the changed body/shape
   records from the existing mapped buffers. The following AS graph copies the
   finalized body scratch and builds acceleration structures from the new inputs.

This removes **one separate staging allocation/copy/submission/wait per respawn**.
It does not remove all CPU/GPU communication. Host records are still needed for
the next split. Event handling, label census, cropping/repacking, pool allocation,
AS descriptor construction, and other completion boundaries remain on the CPU.
The new push-constant contract is isolated in `fragment_finalization.inl`, so
editing the algorithm does not invalidate unrelated shaders.

## Accuracy and validation

`BB_FRAGMENT_VERIFY=1` enables an optional CPU oracle of the previous calculation.
It adds a diagnostic derived readback and must be **disabled for timing**. Every
changed body is checked for finite mass, inverse mass/inertia, position, rotation,
velocities, bounds, shape origin, and wake state. The numerical limit is
`abs(cpu-gpu) / max(1, abs(cpu), abs(gpu)) <= 1e-6`; exceeding it aborts.
The reciprocal is refined, but CPU/GPU floating-point results are not promised
to be bit-identical.

- Fracture fixture: 900 steps per solver, with both incremental and full AS builds,
  pool checks and Vulkan synchronization validation. All four runs pass.
- 27 finalized records per AVBD run and 25 per TGS run were checked. Maximum
  scaled differences were `2.38419e-7` and `2.04771e-7`, respectively.
- Both solvers finish the fixture at rest, without contacts deeper than 100 mm.
  AVBD ends at 9 mm maximum penetration; TGS at 1 mm. Full and incremental build
  paths produce identical final fixture metrics within each solver.
- Small numerical differences change later fracture topology. Consequently,
  whole-chain traces are **not** identical to the CPU implementation.
- F7: 1800 steps per solver under Vulkan synchronization validation; all physics
  CSV rows and DET checkpoints match the PR26 controls exactly.
- All 20 F6 benchmark physics CSVs match within each solver.
- Interactive F9 at 3840x2160 reached 46 bodies after chained drag/release
  fractures, with 56 finalized records checked and no numerical or Vulkan
  validation errors. A capture confirms visible debris; this is not an exhaustive
  per-frame visibility proof.
- All six CTest tests pass, including analytical mass, inertia, COM shift, angular
  velocity, wake-state, and empty-record checks for the CPU validation oracle.

## Paired measurements

RTX 4090, LXC 110, 860x640 window, ten runs per solver and variant, alternating
CPU/GPU order. Baseline is PR26 (`ce29bab`, also contained in merge `0c6c705`).
The script warms both shader caches before measuring. Validation is disabled.
Raw evidence is retained at `/root/beat-box/work/gpu-finalization/`.

| Measurement (median) | CPU finalization | GPU finalization | Interpretation |
| --- | ---: | ---: | --- |
| First fixture respawn, AVBD | 0.962 ms | 0.966 ms | No measurable gain |
| First fixture respawn, TGS | 1.239 ms | 1.112 ms | 10.2% lower in this sample |
| F6 whole frame, AVBD | 4.157 ms | 4.195 ms | Essentially unchanged |
| F6 whole frame, TGS | 5.783 ms | 5.771 ms | Essentially unchanged |

Respawn timing includes pool work, finalization, AS preparation/build, and
simulation upload. The first fracture is compared because it has identical input
state and six resulting bodies; the harness rejects a changed pre-fracture DET
trace. Later splits have different workloads and are not used for a speedup claim.
Timing ranges overlap substantially (AVBD respawn: CPU 0.849–1.400 ms,
GPU 0.908–1.446 ms; TGS: CPU 0.921–1.433 ms, GPU 0.936–1.437 ms).
The observed TGS reduction is not a guarantee of a repeatable 10% improvement.

F6 measures elapsed time between DET checkpoints 1 and 600, divided by 599.
It includes simulation, publication, rendering, and harness overhead; these are
not solver-only GPU timings or 4K measurements. Its steady narrow phase does not
execute fragment finalization. This migration therefore does not address the
main steady-state SDF cost.

Reproduce from the repository root with both executables retained:

```sh
python3 tools/benchmark_fragment_finalization.py \
  --baseline /path/to/pr26-beat-box --candidate build/Release/beat-box \
  --cwd build/Release --fixture tests/scenes/fracture_frame_drop.txt \
  --output work/gpu-finalization/paired --trials 10
```

## Remaining GPU work

A separate instrumented F6 run (600 steps per solver) measures GPU timestamps:

| Stage median | AVBD | TGS |
| --- | ---: | ---: |
| Narrow phase | 0.415 ms | 0.454 ms |
| Setup, including narrow phase | 0.501 ms | 0.538 ms |
| Preparation | 0.098 ms | 0.147 ms |
| Main solve / substeps | 0.104 ms | 0.877 ms |
| Post solve / contact stage | 0.118 ms | 0.025 ms |
| Finalization | 0.008 ms | 0.008 ms |

Narrow phase is nested within setup: do not add those two rows. The remaining
rows describe different passes in the two solvers, not equivalent algorithms.
These profiles point to narrow phase for AVBD and substeps for TGS as more useful
steady-state targets than moving a few fragment calculations off the CPU.
Results are specific to F6; densely interlocked F9 debris needs its own profile.
