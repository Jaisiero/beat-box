# Simulation-owned SimConfig publication

Previously, each stepped render frame waited for simulation, then submitted a separate SimConfig copy graph and waited again. Each PGS, AVBD and TGS graph now ends with that copy and its host-visibility barriers. The existing simulation completion also completes the snapshot, so the renderer consumes it without a separate submission or wait.

The copy task is shared with the standalone readback graph. Non-stepped updates keep the standalone path. A frame-local completion flag is set only after simulation completion and cleared when an in-process reset replaces the scene. Paused frames without updates perform neither simulation nor readback.

## Dependencies and scope

- SimConfig is copied from the current simulation parity to its corresponding host buffer.
- The host destination is registered as a simulation graph attachment.
- Compute-write to transfer-read and transfer-write to host-read barriers are preserved. The compute-write to host-read barrier also preserves publication of directly mapped simulation results.
- CPU access follows the existing COMPUTE_0 submission completion. No delayed snapshot is used for sleep, fracture or culling decisions.
- AS lifetime protections and renderer-wide synchronization are unchanged.

This removes one dedicated readback submission/wait from a normal stepped render frame. Deterministic replay previously invoked the standalone readback twice; both calls now consume the same completed snapshot. The transfer itself remains, and catch-up frames copy once per simulation step instead of once per render frame. This is a coordination reduction, not a fully GPU-driven control path or a measured FPS claim.

## Remaining migration

Sleep/min-height decisions, spawn cadence and impact dispatch scheduling still need GPU-side control. Removing the data transfer requires splitting telemetry from those control dependencies; simply delaying the current SimConfig readback could change retirement and sleep behavior. Host metadata for AS commands also remains.

## Validation

Evidence is stored in `/root/beat-box/work/sim-config-publication/` on LXC 110. The comparison executable is PR #31 commit `5b7dc5ab447d3d8fc73f07d2562bce0d3bdb0bc2`, also contained in merged baseline `a177abf`.

F10 (480 steps) and F11 (1,500 steps), with both AVBD and TGS, match all baseline replay states exactly under Vulkan synchronization validation (3,960 steps).

All eight CTest tests pass. PGS F6 matches the baseline for 300 steps. Both AVBD and TGS pass F10 in-process reset/replay (480 + 480 steps each), and each pass matches the baseline exactly. New executable deterministic coverage totals 6,180 steps.

The normal render loop also passes 30-second F11 runs at 3840x2160 with each of AVBD and TGS, synchronization validation enabled, without validation errors or reported NaNs.
