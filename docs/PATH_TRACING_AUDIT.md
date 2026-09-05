# Path tracing audit

Branch: `fix/path-tracing-black-pixels`, based on `32ec437`.
This first pass is a review, not a production rendering fix. Physics is unchanged.

## P1: Shadow rays select incompatible SBT records

`lighting.slang:614` traces `ShadowPayload` with hit-record offset 1 and miss
index 0. `MainRayTracingPipeline` builds hit records [PROCEDURAL_HIT, LBVH_HIT]
and miss records [HIT_MISS, SHADOW_MISS]. Normal body instances have offset 0.
Consequently shadows select the LBVH intersection/any-hit group and the normal
miss shader, both designed around the main path rather than a shadow query.
The LBVH any-hit and normal miss use `HitPayload`, not `ShadowPayload`.
The shadow flag starts false and no dedicated accepted-hit path sets it true.
This is invalid payload routing and unreliable occlusion, independently of NaNs.

Correct the shadow route as a unit: real geometry intersection, compatible payload,
miss index 1, initial occluded=true cleared on miss, and no debug any-hit invocation.
Review the debug-instance masks/offsets at the same time.

Vulkan requires matching payload structures across all stages using that ray:
https://docs.vulkan.org/spec/latest/chapters/interfaces.html

## P1: Unwritten light direction used by MIS

`compute_direct_lighting_cubes` has an `out light_dir`, but line 576 declares
another local `light_dir`. The caller then uses the unwritten output for its
BRDF PDF and MIS weight. Assign the output itself and initialize all outputs
before early returns. This is not fixed by adding barriers.

## P1: Diffuse sampling admits a zero PDF

`random_float` converts a 32-bit integer to float and multiplies by 2^-32.
Float32 rounds UINT32_MAX to 2^32, so the result can be exactly 1. The hemisphere
sampler then computes sqrt(1-u)=0. On axis-aligned normals the throughput update
`brdf*cos/pdf` can evaluate 0/0. An IEEE float32 numerical reproduction returns
NaN. This establishes a numerical defect, not its frequency after GPU compiler
optimization. Generate a strictly half-open [0,1) sample using 24 random bits;
use the analytic Lambertian cancellation (throughput *= albedo), with explicit
handling for degenerate samples before launching another ray.

## P1: Light sampler does not match rectangular emitter geometry

The active light sampler and emission MIS PDF use only get_half_size(0), equal
areas and a cube centered at the body origin. F9's light has half-extents
(11, 0.2, 6): sampled points and face areas are not its actual surface. Compute
centers, extents and face areas from the actual bounds, and share that geometry
between light sampling and the inverse PDF. Otherwise the estimator remains
biased even after numerical fixes.

## P2: Accumulation can retain non-finite values and lose precision

The history is RGBA16F. The recurrence has no sample/history finite check, and
0*NaN does not clear poisoned history. The current host clear does precede the
trace at frame 0; that earlier ordering problem is already fixed. A later NaN
or half-float overflow can persist. Half precision also loses progressively
smaller updates as sample count grows. Evaluate RGBA32F history, a stable
running mean, and diagnostics that count/reject invalid samples without silently
presenting black as a valid sample. Fix upstream sources first.

## Validation and limits

Inspected active raygen, miss/hit groups, shadow calls, sampling/PDFs, history
format and task ordering. The missing STBN asset is not used by the active
sampler: its texture reads are commented out, and the path uses XorShift.

An audit-only shader probe marks non-finite raw radiance magenta before tone
mapping. It is injected only into a temporary runtime copy; tracked shaders
remain unchanged. F9 screenshots at 1280x720, both without accumulation and after enabling key 0,
showed zero magenta pixels. The accumulated capture also contained zero pure-black
RGB pixels. This probe checks the current raw sample, not the stored HDR history.
This does not rule out rare failures, poisoned history, or finite incorrect
values. Visible dark samples alone are not proof of NaN. GPU simulation's
`nan=0` diagnostic does not inspect path-traced radiance.

No rendering changes have been claimed fixed or benchmarked by this report.
