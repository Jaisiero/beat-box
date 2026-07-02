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
| `BB_SCENE_FILE=path` | load a data-driven scene from a text file (overrides `BB_SCENE`): one cube per line, `px py pz [half_extent] [mass] [restitution] [friction]` (trailing fields optional, `#`/blank lines skipped); a floor + emissive light are auto-added. Iterate on a repro scene with no rebuild. |
| `BB_SCENE_DUMP=path` | after the scene builds, write its dynamic cubes to `path` in the `BB_SCENE_FILE` format (round-trips any built-in scene into an editable file) |
| `BB_RUN_SECONDS=N` | auto-exit after N wall-clock seconds (reproducible captures) |
| `BB_METRICS_CSV=path` | write one ground-truth metrics row per stepped frame: `frame,solver,manifolds,sleeping,pen_mm,maxv_mm,miny_m,deep100,deep200` |
| `BB_ASSERT_MAX_DEEP200=N` / `BB_ASSERT_MAX_PEN=N` / `BB_ASSERT_MAX_MAXV=N` | **exit code 2** + `[METRICS] ASSERT FAILED:...` if the metric exceeds N after `BB_ASSERT_AFTER` warmup steps (default 60). For scriptable pass/fail. Thresholds are per-scene. |
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

Determinism check (two runs, per-field first-diverge; `NONE` everywhere == bitwise-deterministic):
```
pwsh tools/determinism_det.ps1
```

## `tools/` vs `scratch/`

- **`tools/`** (version-controlled) holds durable regression harnesses — start with `determinism_det.ps1`.
- **`scratch/`** is gitignored: ad-hoc experiments, one-off plots, run logs. Promote a script to `tools/`
  (generalizing any hardcoded path, as `determinism_det.ps1` does) once it stabilizes into something
  worth keeping.
