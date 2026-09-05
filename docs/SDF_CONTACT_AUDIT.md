# SDF contact continuity audit

The reported symptom is small positional jumps, especially between fragments in F9.
This change fixes contact-generation/history inconsistencies shared by AVBD and
TGS. It does not add a teleport, change solver iteration counts, or clamp body motion.

## Empty cells must not trigger penetration recovery

`voxel_sample_body` previously counted every near-zero interpolated SDF gradient as
an interior sample. `voxel_axis_collision_detection` uses that count to synthesize a
center-to-center contact with penetration equal to half the sampled voxel size.
An undefined gradient alone does not prove that the sample is inside solid material.
The zero-gradient recovery count now also requires occupancy of the sampled cell.
Empty samples continue through cell SAT, so their finite-sized cubes still collide
with the walls of a cavity; they are not simply discarded.

A concrete counterexample is a 3 x 3 x 1 voxel ring with its central cell empty.
Each of the eight grid nodes surrounding that empty cell touches both solid and empty
space, so the node builder assigns zero distance at all eight corners. Trilinear
interpolation is therefore identically zero inside the cavity, with zero gradient.
The old predicate classified this empty cavity as interior. The occupancy check does
not. This is a geometric counterexample, not a claim that this exact configuration
was captured in the user's scene.

Occupied cells with undefined gradients still use the existing penetration recovery.
This patch does not replace that approximate recovery with an exact union-distance
query; deeply embedded solids can still require substantial positional correction.

## Undefined gradients in thin fragments must not reject every SAT axis

The near-surface SAT filter rejected an axis when `dot(mtv, gA) >= 0`.
For an undefined own-shape gradient, `gA` was explicitly set to zero, making
that condition true for **every** axis. This routed genuine contacts into the
half-voxel recovery instead of resolving their measured contact depth.

A one-cell-thick slab can have opposing real exposed faces and a zero gradient.
The fallback now retains a six-bit mask of exposed voxel faces from occupancy.
It accepts directions facing an exposed face and rejects directions facing only
internal faces. Opposite exposed faces remain separate rather than cancelling.
The same fallback handles the other body's undefined gradient in empty cavities.
When a gradient is defined, the existing signed-gradient filtering is retained.

`voxel_contact_math.hpp` shares the face-direction predicate with C++ tests.
Tests cover an isolated voxel, both sides of a thin slab, diagonal directions,
internal slab faces, empty masks and inward directions at a single exposed wall.
This local face test is a fallback for an undefined SDF direction, not an exact
global minimum-translation solver for arbitrary interlocked concave unions.

## Keep current contact geometry and persistent friction anchors

Voxel manifolds previously kept their old world-space normal while its dot product
with the new normal exceeded 0.9 (about 25.8 degrees). The contact depths had already
been computed along the new normal. Rotating surfaces could consequently solve an
outdated plane and then change direction abruptly at the threshold.

The manifold now keeps its measured normal and tangent basis. History is rejected
when the normals are incompatible (dot <= 0.7), and the inherited normal force/impulse
is projected onto the new normal. Tangential impulses still use the existing basis
reprojection.

Exact feature matches also kept their old world-space contact position until the
new point moved 10 cm. Voxel contacts now retain the newly measured position.
Body-local sticking anchors, feature matching, and proximity matching remain in use;
these carry static friction independently of the refreshed world-space geometry.
The existing OBB position hysteresis is outside this change.

## Fragment publication review

`sync_live_bodies` preserves live poses before republishing geometry. The fragment
fixup shifts the center of mass and local grid origin together, preserving world
voxel placement, and transfers velocity with omega cross COM displacement.
`update_sim` resets collision counts for both buffer parities, so publication starts
with fresh contact history rather than inheriting the parent's old manifolds.
No additional buffer, dispatch, host readback, or memory barrier is introduced here.

## Validation render path

The reduced-resolution measurement mode exposed Vulkan error
`VUID-vkCmdBlitImage-dstImage-00224`: swapchain images lacked `TRANSFER_DST`
usage, although the upscale task blits into them. `gpu_context.hpp` now requests
that usage alongside storage and transfer-source usage. This adds no queue wait.
Earlier reduced-resolution validation logs must not be described as error-free;
final validation is repeated after correcting the image usage.

## Validation

Release build and all three CTest targets pass, including the new thin-face checks.
GPU: RTX 4090, LXC 110, default solver settings, 900 fixed steps per F5/F6 run.
Final runs use normal rendering; the earlier control used reduced-resolution rendering.
These compare physics outcomes, not rendering performance or bitwise determinism.

| Scene / solver | Before: sleeping / max speed / penetration | Final: sleeping / max speed / penetration |
| --- | --- | --- |
| F5 AVBD | 9 / 0 mm/s / 6 mm | 9 / 0 mm/s / 8 mm |
| F5 TGS | 9 / 0 mm/s / 0 mm | 9 / 0 mm/s / 1 mm |
| F6 AVBD | 9 / 0 mm/s / 5 mm | 9 / 0 mm/s / 5 mm |
| F6 TGS | 9 / 0 mm/s / 3 mm | 9 / 0 mm/s / 1 mm |

All four final runs have zero `deep100` and `deep200` at the end. These are reported
contact depths, not an exact volume-overlap measurement. AVBD F5's 2 mm increase is
recorded rather than presented as a penetration improvement.

The existing `tests/scenes/fracture_frame_drop.txt` fixture was run for 600 fixed
steps with each solver, pool checks and Vulkan synchronization validation:

- AVBD: 12 publications, 23 total bodies, 21 sleeping, 0 mm/s final speed, 39 mm
  reported penetration. Sleeping does not prove geometric convergence.
- TGS: 12 publications, 23 total bodies, 10 sleeping, 755 mm/s final speed, 8 mm
  reported penetration. A separate 1800-step run still ended with 55 mm/s residual
  speed, 19 sleeping and 0 mm reported penetration.
- Neither validated run reports a Vulkan error, synchronization hazard, pool
  invariant failure or voxel-conservation warning.

F9 at 3840 x 2160 with AVBD and Vulkan synchronization validation: grab/release the
club, then lift and release a frame. Six publications grew the scene from 5 to 25
bodies. Full fracture event handling measured 4.16-4.80 ms in this run. No Vulkan
errors, synchronization hazards or physics NaNs were reported. **This run still has
one or two oscillating bodies after the main impact**, with final reported speed
1.482 m/s, 21 sleeping and 16 mm penetration. It is not an acceptance pass for the
user's complete stability requirement; the changes should remain a draft pending
further investigation of that residual contact motion.

A separate 120-step F9 run with `BB_RENDER_SCALE=0.25` and synchronization validation
reports no Vulkan errors after the swapchain usage fix.

Reproduce the fixed-step tests from `build/Release` with `DISPLAY=:0`, `BB_SCENE=5`
(or 6), `BB_SOLVER=2` (AVBD) or 3 (TGS), `BB_DET_STEPS=900`, and
`BB_METRICS_CSV=/absolute/output.csv`. The fragment fixture uses `BB_SCENE=3`,
`BB_SCENE_FILE=/root/beat-box/tests/scenes/fracture_frame_drop.txt`,
`BB_DET_STEPS=600`, `BB_POOL_VERIFY=1`, and `BB_RESPAWN_TIMING=1`.
Validation adds `VK_INSTANCE_LAYERS=VK_LAYER_KHRONOS_validation` and
`VK_LAYER_ENABLES=VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT`.

Raw logs and CSVs are in `/root/beat-box/work/sdf-audit`: `baseline-900`,
`baseline-f5-3`, `baseline-f6-2`, `baseline-f6-3`, and the final `v3-*` files.
The existing metrics report contact penetration, speed and sleeping bodies; they do
not directly measure the largest per-body positional correction in a solver step.
Passing these checks therefore does not establish that every possible F9 jump is gone.
