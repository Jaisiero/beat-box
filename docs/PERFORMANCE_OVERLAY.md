# Performance overlay

An always-visible, non-interactive ImGui pass draws after path tracing, upscaling
and collision/axis overlays, immediately before presentation. It uses the existing
Daxa ImGui dependency, installs no input backend/callbacks and never enters the
path-tracing accumulation image. Font size scales from 1080p to 4K.

- **SIM GPU**: average GPU elapsed time across the simulation graph, including
  broad/narrow phase, solver, impact selection and configuration readback.
- **RENDER GPU**: average GPU elapsed time from camera/config upload through the
  final overlay. Includes path tracing, accumulation clear, upscale and debug
  draws; excludes the preceding snapshot capture, AS publication and TLAS build.
- **Hz / FPS**: completed simulation steps and submitted render frames per elapsed
  wall-clock second, independently counted. FPS excludes resize-containing intervals
  (and their render counts); simulation Hz remains actual wall-clock throughput.
  FPS is application throughput, not a Moonlight/display presentation measurement.
- **WORST STEP / RENDER**: largest completed GPU measurement in the application session.
- **FRAME / WORST**: average and largest wall-clock interval between render-loop
  frames, including pacing, CPU work and stalls. It can exceed GPU render time.

Values refresh every half second. Pause reports zero simulation Hz after the
current reporting interval and retains the last GPU step cost. Scene switch,
scene reset and solver change reset current averages but **never clear peaks**.
Only restarting the application clears them. The first application frame has no
interval to measure; subsequent intervals containing swapchain reconstruction are excluded from
FRAME, WORST FRAME and FPS. Scene-load and OS scheduling stalls still count.
GPU timings and their session peaks are unchanged.

Each GPU timer uses sixteen pairs of timestamp queries. A slot can be read or
reused only after its exact submission has completed. Query availability alone
is insufficient because an older value may remain available until a queued GPU
reset executes. Pending samples carry an epoch so old-scene results cannot mix into new-scene
averages, but those late results still update the session peaks. If every slot is busy, instrumentation skips a sample
rather than delaying rendering or simulation. No per-frame CPU/GPU wait is added.

The final pass declares swapchain color-attachment read/write access. Daxa orders
it after rendering/upscaling and before present. An explicit MAIN vertex/index
read-to-transfer-write dependency protects the ImGui utility's internally reused
buffers; its own transfer-to-shader/index barrier protects each upload.

Validation: all nine CTest targets pass, including independent cadence, pause,
averaging and persistent-session-peak tests. Both AVBD and TGS match 600 F11 reference steps
with synchronization validation enabled (1,200 exact records). A 30-second 4K
interaction test covers render scale 0.5, Tab, accumulation, pause, solver changes,
F9/F11 and reset without reported synchronization errors. The 4K HUD was visually
inspected in a separate live run. A follow-up trace asserts that all three peaks
are nondecreasing through pause, solver changes, F9/F11 and reset, including
late GPU samples.

A single 70-second A/B against merged PR44, at 4K with AVBD and F11 spawning every
five steps, excludes the first ten seconds:

| Metric | PR44 | Overlay |
| --- | ---: | ---: |
| Physics steps/s | 59.983 | 59.982 |
| Mean frame interval, ms | 16.741 | 16.737 |
| P99 frame interval, ms | 16.838 | 16.889 |
| Worst frame interval, ms | 18.214 | 18.183 |

This verifies preserved cadence in the tested workload; it does not claim zero
GPU cost or statistical significance for the small timing differences.


## Investigating the approximately 90 ms session peak

`[SLOW-FRAME]` now records intervals above 33.33 ms with CPU phase totals and
resize/control/publication tags. Totals include all simulation-only pumps
between rendering boundaries. They describe the interval that just ended;
per-pump `[FRAME-PHASES]` fields alone must not be interpreted as the cause of
the preceding `wall_ms`. Unattributed time is reported separately.

A paused F11 run reproduced **88.34 ms**, of which **84.08 ms** was resize work,
with zero simulation steps and zero geometry publications. The application
starts at 860×640; changing it to 3840×2160 recreates the swapchain and render
images. Daxa 3.6's `ImplSwapchain::recreate` calls `daxa_dvc_wait_idle` before
cleanup and swapchain creation. The measured resize bucket includes that whole
operation plus application image allocation, not just the idle wait.

A separate 90-second active F11/AVBD run at 4K, spawning every five steps,
recorded one 88.24 ms startup interval (80.95 ms resize). It processed 51 fracture
body events and had no other interval above 33.33 ms. After the first ten seconds:

| Metric | Measured |
| --- | ---: |
| Worst frame interval | 18.36 ms |
| P99 frame interval | 17.17 ms |
| Physics cadence | 59.98 Hz |
| Session worst GPU step | 12.00 ms |
| Session worst GPU render | 6.34 ms |

This reproduces a peak very close to the reported 89.66 ms and isolates a resize
cause. The original 89.66 ms event had no phase trace, so its exact attribution
cannot be recovered retrospectively. The follow-up resize filter excludes such intervals from FRAME/WORST FRAME/FPS,
while the unfiltered slow-frame trace retains their full cost. Other peaks remain
session-wide and are not reset.

A resize-filter regression run recorded an unfiltered 82.13 ms interval while
WORST FRAME remained at 18.89 ms. All nine CTest targets pass, including exclusion
of resize duration and frame count from FPS without changing simulation Hz.

## Dense F11 at 60 versus 144 Hz

`BB_HUD_TRACE=1` logs the published HUD values every half second without enabling
per-frame or per-stage tracing. `BB_FRAME_TIMING` also includes these records.
This makes a reported current/peak contradiction auditable without clamping or
otherwise hiding the values. The same metric owns the mean and session maximum;
scene/solver resets clear only its mean. No numerical inconsistency was reproduced.

Two 90-second AVBD runs at 4K used `BB_FRACTURE_SPAWN_STEPS=1` and stage tracing,
reaching 995 bodies. The last 30 seconds give the following averages of published
HUD windows (stage rows are per-step averages):

| Metric | Render target 60 Hz | Render target 144 Hz |
| --- | ---: | ---: |
| Simulation GPU interval | 7.40 ms | 17.90 ms |
| Simulation cadence | 59.99 Hz | 44.09 Hz |
| Render GPU interval | 5.21 ms | 5.18 ms |
| Render cadence | 59.75 FPS | 133.77 FPS |
| Narrow phase | 1.17 ms | 1.15 ms |
| AVBD main | 2.25 ms | 6.56 ms |
| AVBD post stabilization | 1.66 ms | 3.51 ms |
| Session worst simulation step | 16.53 ms | 40.92 ms |
| Session worst GPU render | 6.37 ms | 6.49 ms |

The evidence points to GPU contention when rendering more frequently: independent
cadences do not guarantee independent GPU execution resources. Timestamp intervals
include execution delays between markers, not just exclusive shader work. This is
not a measurement of warp divergence or proof of a convergence regression. Both
runs use the same scene/spawn settings but are not identical step-by-step replays;
their different physics throughput also changes the trajectories.

All 357 published HUD samples satisfied current <= session worst for simulation
and render; all three maxima were nondecreasing. Nine CTest targets pass. The
reported approximately 20 ms GPU render peak was not reproduced in these runs;
its original cause remains unassigned. The low-volume HUD trace is left enabled
in the interactive session to capture future reports.
