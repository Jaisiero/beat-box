# Frame scheduling and CPU/GPU dependency audit

The normal loop runs zero to four fixed simulation steps, then publishes AS and renders once. A forced scene-load step counts toward the four-step limit. The catch-up loop also stops after its measured CPU simulation span reaches one fixed timestep. The deterministic benchmark instead forces one step per render frame; it does not exercise normal catch-up scheduling.

## Current critical path

1. CPU GUI/events and scene updates.
2. Swapchain acquisition. Daxa 3.6 first waits its GPU frame timeline to enforce frames in flight, then calls `vkAcquireNextImageKHR`. Time here is not solely CPU work or solely vsync.
3. For each step, enqueue a COMPUTE_0 wait on the latest MAIN submission, record/submit the solver, then wait for its completion on the CPU.
4. Read the completed configuration and fracture events once after the normal step batch, and process scene edits/publications.
5. Unconditionally call `device.wait_idle()` on stepped/updated frames, collect publication timestamps and release upload staging.
6. Update instances and build TLAS on COMPUTE_0. Their shared instance buffer carries the GPU dependency between the separate task graphs.
7. Record/submit path tracing and GUI on MAIN, with shared-resource producer dependencies, then present.

The explicit MAIN-to-COMPUTE dependency serializes simulation after the preceding render. Separate queues alone do not overlap those GPU phases. The CPU can record the next simulation while preceding rendering runs, but waits at each simulation completion boundary.

## What can change safely, and what needs redesign

- **Batch catch-up submissions:** no normal intermediate fracture/configuration CPU consumption was found between regular catch-up steps. One completion wait after a batch is a candidate. It must preserve parity queue history, staging lifetimes, readback ownership, and the current time-based catch-up cap. Blindly queuing four expensive steps would defeat the cap. The forced-load path also reads sleep state after its step.
- **Order after render once per batch:** MAIN normally has no new submission between catch-up steps, so repeating the same wait-only submission is redundant. This alone is a small CPU saving, not removal of render/simulation serialization.
- **Narrow the publication lifetime wait:** the unconditional device idle is broader than needed in ordinary no-edit frames. Publication frames still require the completion boundary used to release staging and inspect host-visible output. Replace this only with tracked publication submission completion, not an unconditional deletion.
- **Overlap rendering and simulation:** render reads live body/map/LBVH/island/contact data. The two simulation parity buffers are not an independently owned render snapshot. Correct overlap needs a published snapshot and AS lifetime ownership, including geometry/AABBs while fracture updates them. A semaphore cannot make concurrent conflicting accesses safe by itself.
- **Command recording:** earlier F10 profiles measured about 1.42 ms AVBD and 2.07 ms TGS CPU recording/submission. This is significant next to roughly 2.97/3.71 ms of GPU simulation. Reuse/compaction must be measured across the complete frame, preserving color ordering and barriers.

## Measurement changes

`BB_FRAME_TIMING` now separates `front_cpu_ms` (everything before acquisition, including any scene-update waits) and `acquire_ms` from the existing `front_ms`. These are wall-clock spans, not CPU utilization. Existing solver timestamps, simulation submit/wait spans, publication sync and render submission spans remain available. No new GPU wait is introduced.

The preceding full-step report remains the source of measured baseline costs. Its rows overlap and must not be added. New scheduling changes need normal real-time runs with step-count distributions as well as deterministic state comparisons; a one-step-per-frame replay cannot establish a catch-up improvement.

## Located CPU-side stall

Splitting the front span localized it to `WindowManager::update()`, rather than acquisition. GLFW 3.4's X11 implementation of `glfwGetWindowAttrib(GLFW_ICONIFIED)` calls `getWindowState`, a synchronous X11 property query. The application performed it on every frame. The change reads the initial state once, then maintains it with `glfwSetWindowIconifyCallback`; resize-to-zero handling and minimized event waiting remain intact.

F10 at 4K, 600 deterministic steps, means excluding the first 20:

| CPU wall span (ms) | AVBD before | AVBD callback | TGS before | TGS callback |
|---|---:|---:|---:|---:|
| Event processing | 3.487 | 0.007 | 4.352 | 0.007 |
| Swapchain acquisition | 0.014 | 0.002 | 0.017 | 0.002 |
| Simulation submit and completion | 5.071 | 5.506 | 6.150 | 6.513 |
| Whole frame | 9.231 | 6.105 | 11.230 | 7.138 |

These are one instrumented diagnostic pair per solver, not the repeated uninstrumented benchmark. Some waiting moves to simulation completion, but the whole CPU frame still improves. The X11 server's internal cause was not independently traced; these measurements establish the cost of the synchronous query in this environment, not a universal GLFW cost. All 600 deterministic states match the reference exactly for each solver, with zero invariant violations.

Raw evidence: `/root/beat-box/work/solver-recording/record-{pipeline,window-cache}-{2,3}.log`. No solver, timestep, rendering quality, or GPU synchronization change is included. The contact-compaction experiment was discarded because it did not improve the measured whole frame.

## Repeated uninstrumented comparison

Two alternating baseline/candidate repetitions for each solver, F10, 600 steps at 4K. All eight runs reproduce identical per-step states within each solver (4,800 step records, zero invariant violations). No timing flags or validation layer are enabled in this comparison. Baseline runtime is commit `647e5f0`.

| Mean interval between deterministic step records (ms) | Baseline | Callback | Reduction |
|---|---:|---:|---:|
| AVBD, all sampled frames | 9.092 | 6.119 | 32.7% |
| TGS, all sampled frames | 11.414 | 7.177 | 37.1% |
| AVBD, publication frames | 11.012 | 7.403 | 32.8% |
| TGS, publication frames | 12.783 | 8.089 | 36.7% |

Intervals include the frame loop between flushed step records; these are not isolated GPU step times or displayed FPS. First 20 steps are excluded. The callback change removes a host-side round trip, not the physical render-to-simulation dependency. Raw runs and summary: `/root/beat-box/work/solver-recording/window-ab/`.

Normal real-time F10 also completed with Khronos validation and synchronization validation enabled, including minimize/restore. No validation errors were reported. All eight CTest targets pass. This smoke test checks the window path; it does not establish a performance result for batched catch-up or asynchronous rendering, which remain unchanged.
