# Solver convergence and synchronization audit

Test environment: LXC 110, Ubuntu 24.04, RTX 4090, locally patched Daxa 3.6,
Slang 2026.10. The PR preserves the pre-existing Daxa migration in a separate
commit before the audit fixes.

## Physical and geometric fixes

- Cuboid inverse inertia was calculated from inverse mass. For a unit cube of
  mass 5, this gave 30 instead of 1.2, amplifying angular response 25 times.
  `SceneManager::load_scene` now passes mass; analytical C++ tests cover scaling
  and static bodies.
- OBB SAT used the wrong relative-basis multiplication for the row-axis transform
  convention. `relative_box_axes` explicitly constructs C[B axis][A axis] from
  world-axis dot products. C++ tests cover noncommuting rotations and invariance
  under a common world rotation.
- Parallel overlapping edge segments used an incorrect projection sign.
  Contact extraction now uses the shared, tested closest-segment helper.
- Persistent friction impulses are reprojected through world space when the
  contact basis or A/B order changes.
- Sleeping could accept tens of millimeters of overlap if the penetration trend
  stalled. OBB contacts now enforce a 10 mm limit and keep their entire island
  awake while it is exceeded; previously sleeping neighbors must also participate
  in extraction. Voxel contacts retain the existing discrete-surface policy.
- AVBD positional extraction now starts at the geometric 10 mm limit, with a
  stiffness floor of 32000. Its residual already included margin and slop; using
  that residual directly delayed the trigger another 5.5 mm. The comparison now
  removes those offsets while retaining the slop in the correction itself.

TGS now defaults to 8 substeps, two biased/relaxation sweeps per substep and
120 Hz contact frequency, with the existing frequency cap. This increases solver
work from the former 4 substeps / one sweep / 30 Hz. One sweep left the 12-cube
tower oscillating, despite remaining upright. The serial diagnostic remains off. No more aggressive
graph coloring or reordering was required for the measured TGS pool result.

## Memory synchronization

The audit covered the 42 task headers and manual transfer, readback, voxel,
acceleration-structure and presentation paths. Corrections include:

- Read/write declarations for four tasks that wrote through read-only SimConfig
  attachments, and an islands read/write attachment for the new sleep veto.
- Current-frame readback, compute-to-transfer-to-host publication and completion
  waits, including the raw buffer-address fracture-event bridge.
- TaskBuffer swaps that preserve Daxa queue history; redundant `set_buffer` calls
  that reset that history are avoided.
- No concurrent velocity writes to shared static or sleeping bodies; atomic reads
  for fields concurrently modified by atomic operations; corrected island roots.
- GPU completion before renderer destruction.

These changes do not establish correctness for every architecture or indirect
access. GPU-Assisted Validation 1.4.313 fails internally while instrumenting Slang
buffer pointers (`BufferDeviceAddressPass: FindOffsetInStruct has unexpected
non-composite type`). Host-visible memory tested here is coherent. Dynamic
fracture coverage and non-coherent memory remain limitations of this validation.

## F7 acceptance measurements

Each run contains 7,200 fixed steps. Both checkers pass: the final 120 samples
have all 432 cubes asleep, integer maximum speed 0 mm/s and contact depth <= 10 mm;
independent final OBB geometry also has no pair deeper than 10 mm.

| Solver / run | Maximum independent overlap | Continuous full rest from frame |
|---|---:|---:|
| TGS, run 1 | 5.875 mm | 809 |
| TGS, run 2 | 4.289 mm | 671 |
| AVBD, run 1 | 9.904 mm | 662 |
| AVBD, run 2 | 9.944 mm | 957 |
| TGS, synchronization validation | 4.477 mm | 755 |
| AVBD, synchronization validation | 9.955 mm | 594 |

TGS uses the final default 8 substeps / 2 sweeps / 120 Hz. AVBD uses its default
iterations with the corrected extraction threshold and stiffness floor. AVBD is
closer to the tolerance boundary; these measurements are not a zero-overlap claim
or a guarantee for every landing or scene. Run-to-run contact scheduling varies.

Both validation runs completed with zero Validation Error, Validation Warning
or SYNC-HAZARD reports. The Khronos validation shared library was also confirmed
loaded in the running process. This is synchronization validation, not successful
GPU-Assisted Validation of every buffer-address access.

At 1,800 steps, scene 3's 25 tower cubes, scene 6's 9 voxel bodies and scene 8's
single cube all reach full rest with both solvers. Voxel SDF, surface and primitive
checks match their CPU reference; relative inertia error remains below 4.73e-7.
Release build and `ctest` pass. The independent geometry self-test passes; the
stricter checkers reject historical all-sleeping F7 captures with deep overlap.

The corresponding server logs are `work/solver-review/accept-tgs-r{1,2}.*` and
`work/solver-review/depth-pool-s2-r{1,2}.*`.

## Cost sample

A separate 12-second F7 run measured a weighted mean of 5.80 ms per simulation
step for the final TGS defaults (712 sampled steps). This timer includes CPU
submission and waiting for GPU completion; it is not a GPU timestamp benchmark.
On the same corrected code with 120 Hz configured, overrides of 4 substeps / one
sweep and 8 substeps / one sweep measured 2.55 and 3.91 ms respectively. The contact
frequency cap and evolving pile differ between runs, so these are indicative
end-to-end costs, not isolated kernel speedups or timings of the original solver.

## Reproduction

See [TESTING.md](TESTING.md) for fixed-step pool commands, the serial reference,
full-barrier diagnostic and synchronization validation. Both pool checkers are
required: sleeping alone hid 64–101 mm overlaps after the first geometry fixes.
The pose checker independently runs 15-axis OBB SAT against other cubes and the
known pool boundaries; it does not reuse engine contacts or their extraction cap.

Runtime headers and shaders are now synchronized on every build, including a
shader-only build. Unchanged timestamps preserve the shader cache. Previously a
successful no-op build could leave an old shader in the executable directory.

The private Voxagon engine is not available for implementation comparison. These
results demonstrate convergence of this project's tested scene and configuration,
not equivalence with that engine.
