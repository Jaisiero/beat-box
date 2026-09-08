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
  wall-clock second, independently counted. FPS is application throughput, not a
  Moonlight/display presentation measurement.
- **WORST STEP / RENDER**: largest completed GPU measurement since reset.
- **FRAME / WORST**: average and largest wall-clock interval between render-loop
  frames, including pacing, CPU work and stalls. It can exceed GPU render time.

Values refresh every half second. Pause reports zero simulation Hz after the
current reporting interval and retains the last GPU step cost. Scene switch,
scene reset and solver change clear peaks. The first frame interval after reset
is excluded; subsequent resize or OS scheduling stalls are deliberately counted.

Each GPU timer uses sixteen pairs of timestamp queries. A slot can be read or
reused only after its exact submission has completed. Query availability alone
is insufficient because an older value may remain available until a queued GPU
reset executes. Pending samples carry a reset epoch so old-scene results cannot
repopulate cleared peaks. If every slot is busy, instrumentation skips a sample
rather than delaying rendering or simulation. No per-frame CPU/GPU wait is added.

The final pass declares swapchain color-attachment read/write access. Daxa orders
it after rendering/upscaling and before present. An explicit MAIN vertex/index
read-to-transfer-write dependency protects the ImGui utility's internally reused
buffers; its own transfer-to-shader/index barrier protects each upload.

Validation: all nine CTest targets pass, including independent cadence, pause,
averaging and peak-reset tests. Both AVBD and TGS match 600 F11 reference steps
with synchronization validation enabled (1,200 exact records). A 30-second 4K
interaction test covers render scale 0.5, Tab, accumulation, pause, solver changes,
F9/F11 and reset without reported synchronization errors. The 4K HUD was visually
inspected in a separate live run.

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
