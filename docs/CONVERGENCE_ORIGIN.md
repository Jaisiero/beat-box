# Contact identity and convergence regression

## Cause

`OBBvsOBBCollision::compute_incident_face` initialized vertex positions and the
incident edge IDs, but left `FeaturePair.in_reference` and `out_reference`
uninitialized. Local `out` arrays do not guarantee zero-filled members.
`ortographic_clip` copies the feature record and only replaces the reference
edge involved in a clipping intersection. An untouched vertex, or the opposite
edge of a clipped vertex, therefore carried undefined data. The reference/incident
role swap in `axis_collision_detection` could promote that data into incident IDs.

`contact_history.slang::is_same_contact` uses those incident IDs together with its
position gate to match contacts. Unstable IDs changed which previous anchors and
impulses were reused. This explains why modifying an unrelated shader path could
change settling even in an OBB-only scene. This was an implementation defect;
the evidence does not require a driver bug or more aggressive coloring.

The fix initializes both reference fields to zero before clipping. Zero already
represents the absence of a clipping reference edge. Positions, normals, solver
iterations, time step, sleep tolerances and contact thresholds are unchanged.
The same OBB contact generator is used with both AVBD and TGS.

Slang documents uninitialized local data as undefined; see the official
[declarations reference](https://docs.shader-slang.org/en/latest/external/slang/docs/language-reference/declarations.html).

## Direct evidence

Three original F7 runs agreed at step 1 and diverged at step 2. Full step-2
captures contained 22 manifolds with differing feature metadata despite matching
geometric contact data (187 manifolds total). For persistent body pair (174, 180),
key 2, one contact had position (-2.86535645, 23.5694866, 16.4977818), penetration
-0.00803291798 and features [0, 10, 9, 6] versus [0, 0, 9, 6]. Another contact
changed [10, 0, 6, 10] to [0, 0, 6, 10].

After initialization, three step-2 captures agree completely after mapping packed
body indices to persistent IDs. Five independent 1800-step F7 runs have identical
full checkpoint traces and byte-identical physics CSVs. All 432 dynamic boxes
first sleep at step 591 and remain asleep through step 1800. CSV recorded frame
592 corresponds to solver step 591. Post-stabilization remains enabled.

The subsequent fixed-reference, SDF-prefilter and batched-task variants each ran
three additional F7 replays: all match, including every checkpoint and CSV row.
F3 fracture, F5 and F6 also match across all three variants and repetitions.
These are same-machine, same-input results, not a cross-device determinism claim.

## Synchronization in the batched primal sweep

Each primal sweep still executes 32 indirect color dispatches in the original
Gauss-Seidel order. Between colors an explicit global compute-write to
compute-read/write barrier makes updated body poses and rotations visible to
later colors, and covers SimConfig writes. The indirect argument buffer is
read-only inside the sweep.

The unchanged AvbdTaskHead declares the argument buffer as compute/indirect read,
body/config/contact/state/color buffers as compute read/write, and the contact
links/map as compute read. Daxa therefore supplies producer-to-sweep dependencies
(including dispatch-argument visibility) and the final dependency to the next
sweep or dual task. The last color does not need a duplicate internal barrier.
No barrier can repair an uninitialized function-local feature record.

## Reproduction

With the application closed, run the benchmark three times per variant using
`tools/benchmark_sdf.py`. Do not set `BB_DETERMINISTIC`: that diagnostic disables
post-stabilization and would test a different solver configuration.

```sh
python3 tools/check_solver_replays.py --logs work/run-{1,2,3}.log --csv work/run-{1,2,3}.csv
python3 tools/check_solver_replays.py --captures work/early-{1,2,3}.txt.contacts.json
```

The checker compares complete checkpoint traces, physics CSVs and canonical
captures, including feature IDs. It rejects duplicate persistent body IDs and
post-stabilization-disabled logs. Its tests exercise packing-order invariance and
feature corruption detection. It rejects the original divergent captures and
accepts the fixed captures; the unit tests alone are not the physics validation.

Raw evidence is in `/root/beat-box/work/convergence-origin/`: `early-*`,
`fixed-early-*`, `fixed-f7-*`, `fixed-reference-*`, `fixed-optimized-*` and `batched/`.
Earlier draft conclusions attributing the variability to SDF arithmetic were
premature. The provisional second narrow-phase pipeline has been removed.
