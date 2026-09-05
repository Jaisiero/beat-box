# Testing & measurement

beat-box is measured **headlessly** through environment-variable hooks read at startup, so a run needs
no keyboard input and self-terminates. This is the canonical, version-controlled way to A/B solvers,
gate quality regressions, and check determinism — prefer it over eyeballing the window (a run-to-run
settled pile diverges, so the eye is a poor judge).

Build first: `cmake --build build/Release --config RelWithDebInfo` (exe lands in
`build/Release/RelWithDebInfo/beat-box.exe`).

## Environment hooks

| Var | Effect |
|-----|--------|
| `BB_SOLVER=0..3` | select solver at startup: 0 PGS, 1 PGS_SOFT, 2 AVBD (default), 3 TGS_SOFT |
| `BB_AUTOSTART=1` | start simulating immediately (no Space needed) |
| `BB_SCENE=N` | load scene N (1..8) at startup instead of the default scene_7 |
| `BB_SCENE_FILE=path` | load a data-driven scene from a text file (overrides `BB_SCENE`): one body per line — a cube `px py pz [half_extent] [mass] [restitution] [friction]`, or a concave voxel piece `vox <l\|cross\|frame> px py pz [qx qy qz qw]` (the scene_5 shape set; quat normalized). `#`/blank lines skipped; a floor + emissive light are auto-added. Iterate on a repro scene with no rebuild — see `scratch/wedge_repro.txt` for a voxel example. |
| `BB_SCENE_DUMP=path` | after the scene builds, write its dynamic cubes to `path` in the `BB_SCENE_FILE` format (round-trips any built-in scene into an editable file) |
| `BB_RUN_SECONDS=N` | auto-exit after N wall-clock seconds (reproducible captures) |
| `BB_METRICS_CSV=path` | write one ground-truth metrics row per stepped frame: `frame,solver,manifolds,sleeping,pen_mm,maxv_mm,miny_m,deep100,deep200` |
| `BB_ASSERT_MAX_DEEP200=N` / `BB_ASSERT_MAX_PEN=N` / `BB_ASSERT_MAX_MAXV=N` | **exit code 2** + `[METRICS] ASSERT FAILED:...` if the metric exceeds N after `BB_ASSERT_AFTER` warmup steps (default 60). For scriptable pass/fail. Thresholds are per-scene. |
| `BB_ASSERT_MIN_MINY=Y` | **exit code 2** if the LOWEST body sinks below `Y` meters (floor escape / explosion gate — pen/deep can look fine while a body free-falls through the floor) |
| `BB_POCKET_TRACE=path` | write the deep-pocket oscillator trace CSV (off by default; per-frame disk I/O) |
| `BB_DETERMINISTIC=1` | skip the non-convergent alpha=0 post-stab so runs are cross-launch reproducible (also enables the `dbg_*` hashes) |
| `BB_DET_STEPS=N` | exactly one sim step per render frame for N steps, then exit — a bit-identical step sequence for determinism checks |
| `BB_DET_INPROC=1` | after N steps, reset to the identical initial state and run N again in the SAME process, comparing a cumulative path-hash |
| `BB_COMPILE_ONLY=1` | compile all pipelines (warm `spirv_cache/`) then exit before the render loop — for the `warm_shader_cache` build target |

**Cold start:** the first launch after a `.slang` (or `shared.inl`) edit recompiles the ~69 pipelines
(~100s, `[COMPILE N]` progress); Daxa caches the SPIR-V in `spirv_cache/`, so later launches are ~2s.
To pay that once as a build step instead of on the first real launch, run (on a GPU machine):
```
cmake --build build/Release --config RelWithDebInfo --target warm_shader_cache
```

The startup pipeline compile prints `[COMPILE N] <name>` progress (~100s cold, ~2s warm from
`spirv_cache/`). `[PERF]` lines carry the live diagnostics; the ground-truth quality metrics
(`pen`, `deep100`, `deep200`, `maxv`, `miny`) are computed every step regardless of flags.

## Canonical recipes

Solver A/B on scene_7 (metrics to CSV, then diff / plot):
```
BB_SOLVER=2 BB_AUTOSTART=1 BB_RUN_SECONDS=8 BB_METRICS_CSV=avbd.csv ./beat-box.exe
BB_SOLVER=3 BB_AUTOSTART=1 BB_RUN_SECONDS=8 BB_METRICS_CSV=tgs.csv  ./beat-box.exe
```

Quality-regression gate (non-zero exit fails CI):
```
BB_SOLVER=2 BB_AUTOSTART=1 BB_RUN_SECONDS=20 BB_ASSERT_MAX_DEEP200=30 BB_ASSERT_AFTER=60 ./beat-box.exe
```

Per-scene metrics (e.g. the ramp/slider, scene_4):
```
BB_SOLVER=2 BB_SCENE=4 BB_AUTOSTART=1 BB_RUN_SECONDS=10 BB_METRICS_CSV=scene4.csv ./beat-box.exe
```

Determinism check (two runs, per-field first-diverge; `NONE` everywhere == bitwise-deterministic;
exit 0 = deterministic, 1 = diverged, 2 = no data):
```
pwsh tools/determinism_det.ps1
```

**Full regression gauntlet** — scenes 1-8 with calibrated per-scene thresholds, one command
(exit 0 = all pass; non-zero = the number of failing scenes):
```
pwsh tools/gauntlet.ps1
```

Text-scene parser round-trip self-test (dump == dump(load(dump)); exit 0 = identical):
```
pwsh tools/scene_roundtrip_test.ps1
```

Host-side math unit tests (Quaternion — CPU only, no GPU needed):
```
ctest --test-dir build/Release -C RelWithDebInfo --output-on-failure
```

## `tools/` vs `scratch/`

- **`tools/`** (version-controlled) holds durable regression harnesses — start with `determinism_det.ps1`.
- **`scratch/`** is gitignored: ad-hoc experiments, one-off plots, run logs. Promote a script to `tools/`
  (generalizing any hardcoded path, as `determinism_det.ps1` does) once it stabilizes into something
  worth keeping.

## TGS pool and synchronization audit

`BB_DET_STEPS` now honors an explicit `BB_SCENE` or `BB_SCENE_FILE`; scene 3 is
only its fallback. Use the pool (scene 7) as the primary TGS stability regression:

```sh
cd build/Release
DISPLAY=:0 BB_SOLVER=3 BB_SCENE=7 BB_AUTOSTART=1 BB_DET_STEPS=1800 \
  BB_METRICS_CSV=pool.csv ./beat-box
python3 ../../tools/check_pool_metrics.py pool.csv
```

The checker requires a quiet final 120 samples (integer max speed 0 mm/s), no
contacts deeper than 200 mm, and all 432 cubes asleep. An application exit code 0
alone is not a stability pass. `deep200` counts contacts, not bodies, and `pen_mm`
is capped at 250 by the OBB narrow-phase extraction cap.

`BB_TGS_SUBSTEPS=1..32` selects the recorded TGS loop count and its matching shader
step size (default remains 4). This is a convergence experiment, not a substitute
for fixing races. Compare the same number of full simulation steps.

`BB_SYNC_FULL_BARRIERS=1` disables task reordering and inserts ALL_COMMANDS
read/write memory barriers before every graph task. Diagnostic only: it cannot
repair races within a dispatch or replace cross-queue/host synchronization.

Vulkan synchronization validation, with installed Khronos layers:

```sh
VK_INSTANCE_LAYERS=VK_LAYER_KHRONOS_validation \
VK_LAYER_ENABLES=VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT \
DISPLAY=:0 BB_SOLVER=3 BB_SCENE=7 BB_AUTOSTART=1 BB_RUN_SECONDS=8 ./beat-box
```

Validation without a hazard report is not proof that buffer-device-address shader
accesses or intra-dispatch races are correct. Review those accesses separately.

### TGS serial scheduling reference

`BB_TGS_SERIAL=1` replaces TGS prepare, warm-start, biased solve and relaxation
with the single-invocation overflow kernels extended to cover every manifold.
It traverses colors in the same order as the normal graph, and traverses manifold
indices within each color; the final pass handles overflow contacts. Gravity,
position integration, narrow phase and coloring retain their normal implementation.
This isolates parallel contact execution without intentionally changing color order.
It does not serialize the whole simulation and does not prove absence of races in
other stages. It is a slow diagnostic, disabled by default, and does not alter AVBD.

Compare separate runs with `BB_SCENE=7 BB_SOLVER=3 BB_AUTOSTART=1 BB_DET_STEPS=1800`,
with and without `BB_TGS_SERIAL=1`, writing different `BB_METRICS_CSV` files.
Do not treat early termination or shader compilation failure as a solver result.

The shared `closest_segment_parameters` helper is exercised by `math_tests` on
parallel overlapping edges, reversed endpoints, crossing segments, disjoint segments,
a point against a segment and two points. The shader's `edges_contact` uses this same
helper. These geometric cases catch the old parallel branch's wrong projection sign;
they do not establish that this branch is the dominant cause of scene 7 instability.

### Convergence work and independent final geometry

`BB_TGS_SWEEPS=N` (1 to 16, default 1) repeats the biased solve and relaxation
sweeps inside each TGS substep. It does not repeat warm starting, change dt, add
contact refreshes, change coloring or modify AVBD. Use it to separate convergence
work from scheduling and substep frequency.

`BB_DET_DUMP=/absolute/path/poses.txt` exports final GPU poses before a fixed-step
run exits. For scene 7, `python3 tools/check_pool_geometry.py poses.txt` reconstructs
the pool floor and walls and measures OBB overlaps independently of the engine's
manifolds and extraction cap. Run `--self-test` for analytic geometry checks. Its
pair IDs are indices in the dump, not persistent GPU body IDs. A sleeping count of
432 does not prove that boxes have stopped overlapping.

Cuboid inverse inertia requires mass, not inverse mass. A unit cube of mass 5 has
inverse inertia 1.2 on all axes; supplying inverse mass produced 30. The regression
checks cover this value, inverse scaling with mass and the static zero-mass case.
The relative OBB basis is also checked with noncommuting rotations and invariance
under a common world rotation; SAT indexes it as C[B axis][A axis].
