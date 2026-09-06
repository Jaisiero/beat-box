# Fragment contact origin audit

## Confirmed defect: surface count restored from the parent

`apply_fracture` creates each cropped shape with `VoxelShape ns = shape`. Before
this fix, it changed dimensions and pool offsets but inherited the parent's
`surf_count`. The GPU surface builder correctly wrote the fragment's count.
However, `respawn_after_fracture` subsequently uploaded the CPU shape records to
publish the corrected center-of-mass/grid origin. This restored the stale parent
count. Later pool uploads also propagated the stale CPU metadata.

Narrow phase trusted that count and consumed entries beyond the fragment's valid
surface list. These could be unused entries or another allocation's voxel sites.
A memory barrier cannot repair this overwrite. The access can stay inside the
large shared Vulkan buffer while being outside the fragment's logical list, so
absence of a Vulkan validation error does not establish correctness here.

The fix counts exposed occupied cells while cropping each component. A cell is a
surface cell if any of its six neighbors is outside the parent grid or belongs to
a different component. This matches the GPU builder's one-entry-per-surface-cell
contract. The subsequent CPU upload now preserves the correct count. No solver
iterations, contact-normal rules or GPU barriers are changed by this fix.

## Captured counterexample

The automatic weak-frame fracture fixture, AVBD, step 150 (capture frame 152),
contains a fragment with persistent body ID 7, shape index 5, dimensions 2x3x2,
voxel size 0.5 m and five occupied cells. Its stored contacts include voxel sites
(3,5,0), (3,2,0), (2,5,0), and (2,2,0), outside that grid.

The first contact is at (-1.71716046, 0.711829185, 11.238781). Transforming it with
the captured **step-start pose**, before the solver moves the body, gives grid
coordinates (3.5,5.5,0.5). Its distance to the nearest exposed face of the actual
fragment is 1.76777 m. The contact claims 43.7 mm penetration against body 6.
It is a normal key-94 contact, not the center-to-center recovery (none of the 14
contacts in this capture uses the recovery feature sentinel).

This identifies a real input-geometry error rather than a TAB display delay.
It is a deterministic fixture reproduction; it is not a capture of the exact
interactive frame originally observed by the user.

## Capture and independent verification

F12 and the existing `BB_DET_DUMP` / `BB_DUMP_AT_SECONDS` mechanisms now also write
`<scene-path>.contacts.json`. The legacy text format cannot represent arbitrary
fractured shapes and must not be described as an exact fragment replay.

The sidecar contains persistent body IDs, simulation row indices via array order,
current poses/velocities, available AVBD start poses, actual fragment occupancy,
CPU/GPU surface counts, GPU surface lists and stored manifold contact geometry.
It is an on-demand diagnostic readback; it adds no per-frame transfer or new flag.
GPU work finishes before staging copies, with transfer-to-host barriers and scoped
completion waits. The diagnostic intentionally stalls when requested.

Run `python tools/analyze_contact_capture.py <capture.json>` (NumPy required).
The analyzer constructs exposed voxel rectangles directly from occupancy, without
using the simulation SDF, and measures signed point-to-surface distances. It also
checks every live shape's complete GPU surface list and both counts against an
independent occupancy enumeration. Negative distances mean inside solid material.
For TGS or unavailable AVBD start poses, the `before` measurement uses the current
pose; do not interpret it as a TGS step-start measurement.

After the fix, captures at steps 150 and 900 with both solvers have no surface-list
or count discrepancies. At 900 steps, the weak fixture ends with AVBD: 18 dynamic
bodies asleep, 9 mm penetration, zero speed; TGS: 9 asleep, 1 mm penetration,
163 mm/s residual speed. Fragment counts differ from the defective baseline
because phantom contacts no longer generate the same secondary impacts.

## Separate issues still visible in TAB

Normal voxel contacts still use sample centers and generated tangential offsets,
not clipped surface witness points. The corrected capture still shows offsets of
roughly half a voxel, and some larger offsets for rotated features. Also, TAB emits
stored pre-solve contact positions after the solver has moved the bodies. These
are separate issues from reading another fragment's surface entries. The generic
center-to-center recovery remains approximate too. This change does not claim
that every displayed contact now lies on a physical surface or that all residual
TGS motion is eliminated.

Evidence is retained under `/root/beat-box/work/contact-origin` and the local
`outputs/contact-origin` directory, with pre-fix `step-150` and fixed `fix-*` captures.

## Final validation and limits

Release build and all four CTest targets pass. Both solvers complete 900 fixture
steps with Vulkan synchronization validation, pool checks and contact capture:
no validation errors, synchronization hazards, conservation warnings or pool
failures. The full metric CSVs match their normal runs. The analyzer also detects
injected stale-count and invalid-surface-entry faults in otherwise valid captures.

A 4K F9 drag/release run produced a capture with 14 bodies and 32 contacts. All
referenced GPU surface lists and counts match occupancy. The worst point in this
capture is about 179 mm outside one body and 125 mm inside the other, illustrating
the remaining sample-center placement issue, not an out-of-grid surface entry.
Further fractures occurred after that capture; the scene still had residual motion
at the end of the run. This is not an acceptance pass for every F9 contact point.

F7 AVBD ends with all 432 bodies asleep. F7 TGS does not pass the full-sleep
criterion: the initial run ends with 430 asleep, and repeats end with 431 and 429.
The cropped-fragment count path does not execute in this non-fracturing scene,
but the variability must still be recorded rather than claiming that every
regression check passes. TGS stability remains open.
