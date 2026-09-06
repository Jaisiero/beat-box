# Invisible fragments during publication

## Verified cause

When F9 grew from 32 to 34 bodies after a fracture, the instance updater still
used the indirect dispatch from the last simulation step: one group of 32
threads. Instances 32 and 33 remained unchanged until the next step.

Those instances had been initialized by C++. `get_transform_matrix()` gathered
the columns of `Quaternion::to_matrix()`, although the values stored in its
x/y/z vectors were already the rows required by Vulkan's representation.
The resulting rotation was transposed. Translation was correct.
The update shader produced the correct rotation.

The TLAS could be fully built while still containing the wrong matrix:
ray tracing candidate traversal and the manual body intersection used different
transforms. The next step expanded the dispatch and repaired the pending
instances, explaining why they reappeared.

The disappearance also reproduced with the previous `device.wait_idle()`
restored. Adding that wait does not fix it. Both defects already exist in
PR26's base commit, `272284a`; removing the wait did not introduce them.

## Fix

- C++ preserves the matrix rows when writing instance data.
- Publication dispatches directly from `current_rigid_body_count`, updated after
  fractures, removals, and spawns. It no longer depends on the group count
  calculated before editing the scene.
- Solver mathematics and SDF algorithms are unchanged.

## Validation

The CPU test applies the matrix's twelve floats as three Vulkan rows and
compares the result against an independent `q * point * conjugate(q)` rotation.
It covers identity, a 90-degree turn, a general rotation, translation, and
multiple points. It detected ten discrepancies before the change and none after.

Temporary instrumentation compared the actual GPU-generated AABBs of retained
BLASes; it found no incorrect changes in the examined chain. It then compared
the body map, BLAS addresses, masks, and matrices of published instances.
With the corrected matrix oracle, it detected exactly instances 32 and 33 when
crossing from 32 to 34 bodies. Their rotations were the transpose of the expected
values. Evidence: `/root/beat-box/work/vanishing/coverage-verify.log`.

The instrumentation adds readbacks and is not part of the final executable.
Logs and the diagnostic patch are retained in the working directory.
Physics validation alone does not verify rendering visibility.

After the fix, an F9 chain crossed 32 -> 34 again and reached 56 bodies without
any instance or geometry discrepancies in the instrumentation
(`fixed-verify.log`). All five CTest tests pass, including the test that failed
before the change.

The final executable, without instrumentation, completed the fracture fixture
(900 steps) and F7 (1800 steps) with AVBD and TGS under Vulkan synchronization
validation: all four CSVs and every DET checkpoint match the PR25 controls,
with no reported errors. Logs: `final-*.log`, `final-*.csv`, and `final-checks.log`.
