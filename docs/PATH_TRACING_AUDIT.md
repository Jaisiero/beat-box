# Path tracing corrections

Branch: `fix/path-tracing-black-pixels`, based on solver commit `32ec437`.

## Corrected defects

- **Accumulation dependency:** `RayTracingTaskHead` declared the history image read-only,
  although raygen reads and writes it. It now uses Daxa `RAY_TRACING_SHADER_READ_WRITE`,
  allowing the task graph to include shader writes in image dependencies. No global
  GPU wait was added. The existing frame-zero clear was already before tracing.
- **Shadow payload routing:** shadow rays used hit offset 1 (the LBVH debug group)
  and miss index 0 (`HitPayload`) with a `ShadowPayload`. They now select real
  geometry with offset 0 and shadow miss index 1. The payload starts occluded;
  a miss clears it. Opaque, terminate-on-first-hit rays skip closest-hit and any-hit.
  Debug BVH instances use mask 2 and shadow rays mask 1, excluding debug overlays.
  The offset ray origin and direction now aim at the same sampled light point.
- **Unwritten MIS input:** a local `light_dir` hid the output parameter in
  `compute_direct_lighting_cubes`. All outputs are initialized and the actual
  output direction is assigned before the caller evaluates the BRDF PDF.
- **RNG endpoint and grazing division:** converting all 32 random bits directly to
  float could round to 1, producing a zero cosine/PDF. `pt_unit_float` uses 24 bits
  and is strictly in [0,1). Cosine-weighted Lambertian throughput now uses its
  analytical result, `throughput *= albedo`, avoiding `brdf*cos/pdf` at grazing angles.
- **Rectangular lights:** F9's light has half-extents (11, 0.2, 6), but the sampler
  treated every extent as 11. Sampling and the inverse MIS PDF now use actual bounds,
  face areas and the same face distribution, including bounds not centered at zero.
- **History precision and recovery:** RGBA32F replaces RGBA16F. RGB stores a linear
  running mean; alpha counts accepted samples per pixel. Non-finite or negative
  samples are rejected rather than averaged as black. Invalid history is reset.
  Reset frames do not read the previous history. At 4K this costs about 126.6 MiB
  for history instead of 63.3 MiB. This is a deliberate precision/memory tradeoff.
- **Ray parameter and display:** transformed ray directions are not renormalized,
  preserving the world ray parameter. ACES input is bounded only above its already
  saturated white range to prevent polynomial overflow for extreme finite HDR.

## Diagnostic mode

`BB_RT_VALIDATE=1` marks invalid radiance or history magenta. With accumulation,
this mark persists until history reset, so rare failures remain visible in a later
capture. The diagnostic uses the sign of the sample count, with no extra GPU
buffer, CPU readback or per-frame wait. It is disabled by default; 0/false/off and
1/true/on are accepted. It detects non-finite/negative values, not every possible
finite rendering error. Normal rendering rejects invalid accumulated samples too.

## Code map

- `include/shared.inl`: actual history access and diagnostic flag.
- `src/renderer_manager.cpp`: history format at creation/resize and runtime diagnostic setting.
- `src/acceleration_structure_manager.cpp`: debug-instance visibility mask.
- `include/path_tracing_math.hpp`: shared C++/Slang random conversion and box-face geometry.
- `src/shaders/path_tracing/lighting.slang`: shadow routing, light sampling and PDFs.
- `src/shaders/path_tracing/ray_tracing.slang`: throughput, history and display safety.
- `tests/path_tracing_tests.cpp`: endpoint regression and six-face rectangular geometry.

## Validation

Release build and all three CTest targets pass (`math_tests`,
`runtime_diagnostics_tests`, `path_tracing_tests`). GPU tests on RTX 4090 in LXC 110:

- F9 accumulated at 3840x2160: zero diagnostic magenta pixels; visually no scattered
  black speckles. After removing desktop borders, the local-contrast metric finds
  60 pixels at object corners and the cursor, versus 999 in the similarly cropped
  user capture. The camera framing differs, so this is not an equal-sample benchmark.
- F7 simulated, then paused and accumulated at 1280x720: zero diagnostic markers.
- F9/F7 scene changes and 1280x720 to 3840x2160 resize completed successfully.
- Vulkan validation layer confirmed loaded, with synchronization validation enabled:
  zero `Validation Error`, `SYNC-HAZARD` or `VUID-` lines in the three validation runs.
- Temporary runtime-only fault injection: a NaN at sample 5 in an 8x8 block produced
  exactly 64 persistent magenta pixels. With `BB_RT_VALIDATE=0`, the same block
  recovered to ordinary finite displayed colors, with zero magenta or pure-black
  pixels in the capture. The injected shader was removed and runtime/source equality
  verified before the final run. No fault-injection code is shipped.

Captures/logs remain under `work/render-audit` on the server. Validation is evidence
for these runs, not proof of all possible GPU memory or numerical behavior.

The user's 2131x1125 capture has 1042 pixels over 40/255 darker than their 5x5
median, with median brightness above 60/255. This metric includes some geometric
edges and the cursor, so images also require visual inspection. Counting only pure
black pixels or checking a single frame's NaNs was insufficient in the initial audit.

The active sampler is XorShift; missing STBN assets are not used by its commented-out
texture reads. Physics `nan=0` does not validate rendering. These changes correct
multiple concrete defects; attributing every original speckle to one cause requires
isolating each change, rather than inferring NaNs from appearance alone.
