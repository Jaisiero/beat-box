# AVBD profiling, tangent-arm reuse, and valid F11 stress tests

Base: `3a02cea`, merged PR47. RTX 4090, 3840x2160, F11 AVBD, 144 Hz render
target, NVIDIA compute occupancy priority enabled. Timing comparisons use 70-second
runs with `BB_FRACTURE_SPAWN_STEPS=1`; no validation layer or fine profiling.

## A prerequisite found by longer validation

The old dense test exceeded the 65,536 AS primitive capacity. Spawning reserved a
body and changed scene metadata before AS publication rejected it (for example,
65,563 primitives). Subsequent steps used a scene whose publication had failed.
Two unchanged reference replays then diverged at step 856. An in-process reset
also produced validation-counter violations. This is not evidence that tangent
reuse caused a solver regression, and timings after these errors are invalid.

The GPU scene-edit pass now checks the prospective primitive total before
committing a spawn. It includes static bodies, models retired bodies as the same
one-primitive tombstones used by publication, and excludes the stable ID of the
replaced slot. Spatial row indices are not stable IDs. If the budget is exceeded,
the temporary body reservation is released and the spawn seed/edit remains
uncommitted. A cull in the same transaction can still succeed. The capacity is
passed from the existing host AS limit; no new CPU readback or synchronization is
introduced. This guard covers scene spawning, not every possible AS failure.

The replay harness now rejects `ERROR:` and `AS rebuild failed`, even when the
application exits successfully. Earlier exploratory timing results are not used
to substantiate the final optimization.

## Local primal optimization

Both tangent constraints of a contact use the same two world-space anchor arms,
computed from the step-start rotations and local anchors. Previously each call to
`avbd_dir_C` recomputed them. The primal pass now obtains them from the first
call and passes them to `avbd_dir_C_from_arms` for the second direction.

The directional dot products, drift deadband, motion corrections, friction cone,
Jacobian accumulation order and LDL factorization are unchanged. This is local
reuse within one contact evaluation; it does not cache a neighbor's current pose
across colors or sweeps. It applies to main and post-stabilization primal passes.
No iteration counts, contacts, sleeping thresholds or solver barriers are changed.

## Fine profiling

`BB_AVBD_FINE_TIMING=1` emits `[AVBD-DETAIL]` with separate intervals for primal,
dual, velocity/impact, depth cascade, and symmetric post-stabilization. Queries
use the existing completed-simulation readback boundary; there is no additional
CPU/GPU wait. Default runs record no extra markers. Fine profiling adds ordered
timestamp markers and is diagnostic, not the basis of performance A/B claims.

## Validation

GPU scene-edit verification covers full-pool refusal, AS-budget refusal after
retirement, allocation rollback, an exact-capacity successful spawn, static-body
preservation, and non-identity body-row ordering. Pool invariants and unchanged
RNG state are checked after rejection. The test runs with synchronization
validation and the optional NVIDIA extension disabled because the installed
validation layer predates that extension.

After the capacity guard, 1,800-step dense reference runs match exactly across
repeats, and tangent reuse matches the corrected reference. Post-stabilization
remains enabled. All 10 CTest targets pass.

## Valid performance comparisons

Both variants include the AS-capacity guard. Order: reference, reuse, reuse,
reference. Each row covers 4,199 measured steps; no AS errors were reported.

| Run | Mean step | P99 step | Main (primal + dual) | Post block | Worst step |
|---|---:|---:|---:|---:|---:|
| Reference 1 | 4.522 ms | 12.019 ms | 1.325 ms | 1.071 ms | 19.344 ms |
| Reuse 1 | 4.449 ms | 11.133 ms | 1.303 ms | 1.017 ms | 16.527 ms |
| Reuse 2 | 4.459 ms | 11.318 ms | 1.308 ms | 1.022 ms | 19.751 ms |
| Reference 2 | 4.525 ms | 11.697 ms | 1.319 ms | 1.072 ms | 18.103 ms |

The repeatable improvement is approximately 1.5% over the full step and 4.8% in
the post block. Worst-step improvement is **not** established: a reused-arm run
still reached 19.75 ms. The settled final HUD averages were around 3.02–3.05 ms
with reuse and 3.08–3.10 ms without it. No claim is made that every frame remains
below 16.67 ms.

## Final fine-profile intervals

A separate corrected-scene run with reuse enabled measured these means over
4,199 steps: primal 1.259 ms, dual 0.072 ms, velocity/impact 0.031 ms, depth
cascade 0.509 ms, and symmetric sweeps 0.470 ms. These averages include both
spawning and the settled pile; they are not costs for a fixed contact count.

The final TGS build also matches all 600 existing reference checkpoints with
synchronization validation and fine timing requested, exercising the six-query
TGS path in the larger query pool. The NVIDIA extension is disabled for that
validation run, as described above.

## Rejected exploratory candidates

- Assemble only the Hessian's lower triangle.
- Reuse the current body's motion across manifold visits.

Neither showed a convincing improvement and neither is included. A post-only
reuse variant was prepared while investigating the dense mismatch, but was not
needed once the invalid AS publication was identified.
