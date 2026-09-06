# Shader compilation boundaries

The previous physics translation unit (`RB_sim.slang`) registered 44 compute entry
points and included collision detection, solvers and LBVH together. `extensions.slang`
also coupled contact history to 69 registered stages. Editing a function could therefore
invalidate dozens of unrelated SPIR-V artifacts even when its generated code was unused.

This refactor changes source ownership and pipeline source paths. Task heads, push-constant
layouts, entry names, dispatch sizes, task graph order and barriers are unchanged. The 59
moved RB/AVBD entry bodies were compared token-for-token against commit `5e16318`, including
both preprocessor branches; no executable tokens in those bodies changed.

## Source ownership

All paths below are relative to `src/shaders/simulation` unless stated otherwise.

| Source | Registered compute entry points | Responsibility |
| --- | ---: | --- |
| `passes/dispatch.slang` | 5 | Indirect dispatch dimensions |
| `passes/sort.slang` | 4 | Morton keys and radix sorting |
| `passes/bvh.slang` | 3 | BVH construction |
| `passes/broad_phase.slang` | 1 | Overlapping body pairs |
| `passes/narrow_phase.slang` | 1 | Contact generation, including SDF/SAT/history |
| `passes/reordering.slang` | 3 | Body reordering and canonical contact chains |
| `passes/islands.slang` | 10 | Body/contact island construction and packing |
| `passes/integration.slang` | 3 | Advection, position integration and body publication |
| `passes/sleep.slang` | 3 | Quiet timers, vetoes and sleep application |
| `passes/solver_pgs_tgs.slang` | 9 | PGS/TGS solve, color and overflow passes |
| `passes/pick.slang` | 1 | Mouse spring |
| `passes/contact_debug.slang` | 1 | Contact visualization |
| `passes/avbd_step.slang` | 2 | AVBD setup and velocity reconstruction |
| `passes/avbd_warmstart.slang` | 1 | AVBD warm start |
| `passes/avbd_primal.slang` | 1 | AVBD body minimization and local linear solve |
| `passes/avbd_dual.slang` | 1 | AVBD force/penalty update |
| `passes/avbd_depth.slang` | 3 | AVBD depth propagation |
| `passes/avbd_impact.slang` | 2 | AVBD impact estimate and velocity correction |
| `passes/avbd_trace.slang` | 1 | AVBD contact diagnostics |
| `passes/avbd_coloring.slang` | 4 | AVBD body coloring |
| `voxel_sdf.slang` | 6 | SDF, surface list, inertia and primitive generation |
| `voxel_fracture.slang` | 4 | Carving, partition assignment and component labels |

The separate `coloring.slang` retains its 10 PGS/TGS contact-coloring entries. The AS,
ray-tracing and GUI shader families remain separate.

`contact_history.slang` now owns the Manifold extension and matching logic and is included
only by collision detection. Root shader helpers split rigid-body math, advection,
quaternion integration, transforms and state copying. `body_activity.slang` shares the
awake-body predicate without pulling in a solver. AVBD shares graph traversal through
`avbd_common.slang` and constraints through `avbd_constraints.slang`; neither includes a
file containing all AVBD entry points. There is no replacement umbrella include.

## Measured cache behavior

LXC 110, RTX 4090, installed Daxa/Slang build. Each probe adds a harmless comment to a
runtime shader copy, launches with `BB_COMPILE_ONLY=1`, counts actual changed SPIR-V cache
files, and restores the original bytes/timestamps. The source checkout is not edited.
Times include pipeline creation and application initialization, not just Slang compilation.

| Edit | Previous registered consumers (source analysis) | Measured cache writes after refactor | Full startup |
| --- | ---: | ---: | ---: |
| No edit, warm cache | - | 0 | 1.32 s |
| Collision detection | 44 | 1 | 6.53 s |
| Contact history (formerly in extensions) | 69 | 1 | 6.48 s |
| PGS/TGS solver algorithms | 44 | 9 | 20.15 s |
| AVBD primal | 15 | 1 | 2.22 s |
| Voxel fracture kernels | 10 | 4 | 3.72 s |
| Ray-tracing lighting | 7 | 7 | 9.53 s |
| No edit after all probes | - | 0 | 1.27 s |

The old consumer counts come from the previous include graph, not a timed pre-refactor
probe. Ray tracing is the control case: a lighting edit recompiles only its own stages.
These are individual measurements, not a guaranteed latency or statistical benchmark.

Daxa's installed cache validates disk dependencies by modification time. Rewriting identical
files can invalidate it. `cmake/sync_runtime_sources.cmake` already copies only changed
contents; preserve that behavior in deployment tools. Startup messages now say `[PIPELINE]`
because cache hits still create Vulkan pipelines and must not be counted as compilations.

Common ABI/layout headers (`shared.inl`, `math.hpp`) still have broad consumers. Core body
math changes also invalidate its actual consumers. This is intentional; the refactor does
not hide dependencies, bypass barriers, change the compiler or introduce parallel compilation.

## Reproduction and regression guards

Project-local include dependencies, conservatively including conditional branches:

```sh
python3 tools/shader_dependencies.py
python3 tools/shader_dependencies.py --json
python3 tests/shader_dependencies_tests.py
```

The structural tests check registered entry points and limit the consumers of frequently
edited algorithms, so adding an umbrella include fails a test. CMake registers this fourth
CTest target when a Python 3 interpreter is available; Python is not a runtime dependency
of Beat Box.

With Beat Box closed, run the actual cache benchmark against its runtime copy:

```sh
DISPLAY=:0 python3 tools/benchmark_shader_cache.py \
  --runtime build/Release --output work/shader-cache-probe
```

It writes logs and `results.json`, checks the expected invalidation counts and verifies zero
cache writes on warm launches. It refuses the source checkout and source-file symlinks.

Release build and all four CTest targets pass. F5/F6, both AVBD and TGS, execute 900 fixed
steps and produce CSVs identical to the pre-refactor versions (899 emitted records each).
F7 ends with all 432 dynamic bodies asleep: AVBD 9 mm and TGS 5 mm reported penetration,
zero final speed and no deep100/deep200 contacts. The weak fracture fixture executes 600
steps per solver with synchronization validation: both CSVs are also identical to the
pre-refactor versions, with 12 fragment publications each and no Vulkan errors, sync hazards
or physics NaNs. This preserves the residual fragment motion documented in the SDF audit;
it does not claim to fix the destruction solver.

Evidence: `/root/beat-box/work/shader-modules`, with `cache-probe/results.json`,
`f5-*.csv`, `f6-*.csv`, `f7-*.csv`, `fragments-*.csv` and their logs.

The narrow-phase entry now has two pipeline specializations: the original and an
SDF pair-prefilter variant. Scene metadata selects one; there is still one dispatch.
The dependency report counts entry points, so its one narrow-phase entry represents
two compilations when collision detection changes. See `SDF_PERFORMANCE.md`.
