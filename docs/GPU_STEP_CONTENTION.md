# F11 GPU step contention

Baseline: merged `a519cc8` (PR46). RTX 4090, NVIDIA 595.84, 3840x2160,
AVBD F11, 144 Hz render target, 60 Hz simulation, render fairness enabled.
Each timing run lasts 70 seconds with `BB_FRACTURE_SPAWN_STEPS=1` and no validation layer.

## Finding

The longest simulation intervals overlap two ray-tracing intervals. The reference
20.94 ms step overlaps 4.69 and 4.63 ms of rendering. This is evidence of shared
GPU contention, **not** a measurement of idle/stall time: overlapping queues can
make progress together. Lowering only render cadence to 60 Hz reduced the maximum
to 16.33 ms (144 Hz control: 20.89 ms). Splitting the ray dispatch into eight
horizontal bands did not help (20.90 vs 20.79 ms) and increased rendering cost;
that experiment is not included.

## Mitigation

When the device exposes both `VK_NV_compute_occupancy_priority` and its feature,
the Daxa backend enables it and sets HIGH (0.75) occupancy priority on each
COMPUTE command buffer. MAIN/TRANSFER remain normal, and occupancy throttling is
zero. This favors latency-sensitive simulation dispatches without changing
solver work, queue dependencies, barriers, rendering resolution or samples.

This is an optional NVIDIA optimization. Unsupported devices keep the original
behavior. `BB_NV_COMPUTE_PRIORITY=0` disables it for comparisons. This is an
occupancy hint, not preemption or a guaranteed frame deadline. Existing render
admission fairness is retained. Feature structures and extension-name storage
stay alive through device creation; command state is set after every successful
compute command-buffer begin, including recycled recorders.

The backend patch is separate from the existing Daxa compatibility patch and
is applied idempotently by CMake. A private compatibility header supplies the
extension ABI for the older Vulkan headers used by Daxa 3.6.

## Measurements

| 70-second run | Mean step | P95 step | P99 step | Worst step |
|---|---:|---:|---:|---:|
| Priority off | 13.70 ms | 17.07 ms | 18.28 ms | 20.94 ms |
| Priority on | 6.32 ms | 9.92 ms | 11.03 ms | 13.74 ms |
| Priority on, repeat first | 6.39 ms | 9.83 ms | 10.97 ms | 13.74 ms |
| Priority off, repeat second | 13.65 ms | 17.52 ms | 18.38 ms | 20.98 ms |

These are all measured GPU step spans, including spawning and settling, rather
than a single final HUD average. The first pair's final HUD render cadence was
121.92 versus 139.83 FPS; worst GPU rendering was 5.74 versus 6.06 ms. The
simulation improves despite a small increase in individual rendering cost.

## Diagnostics

`BB_GPU_TIMELINE=1` logs integer timestamps and completed submission indices for
simulation and rendering. With `BB_FRAME_TIMING=1`, AVBD/TGS stage logs also
include their six boundary timestamps. These logs are opt-in.

```sh
python3 tools/analyze_gpu_timeline.py capture.log --top 10
```

The analyzer subtracts integer timestamps before conversion to floating point.
The legacy ray-tracing query reader now waits for its recorded submission to
complete before trusting availability, matching the existing HUD timer safety
rule. This adds no blocking wait and does not change the HUD's session peaks.

## Validation and limitations

- 600 exact reference checkpoints each for AVBD and TGS with priority enabled,
  retaining normal solver post-stabilization (no `BB_DETERMINISTIC`).
- The same 1,200 checkpoints with synchronization validation and priority disabled.
- The installed Vulkan validation layer uses header version 313 and rejects the
  new extension's feature structure as unknown. The enabled path therefore does
  **not** have a clean validation-layer run; updating the layer is needed for that
  coverage. The driver itself exposes and executes the extension successfully.
- All 10 CTest targets pass. A final 20-second live 4K run with priority active
  reaches 13.72 ms worst step and about 140 FPS, with no reported NaNs or coloring
  violations.
- Fresh patch application and already-applied reverse checks pass.
- A synthetic timestamp above 2^53 verifies integer subtraction and overlap logic.
- Measurements cover this RTX 4090 F11 workload, not a guarantee that every scene
  stays below 16.67 ms or that other GPUs/drivers obtain the same improvement.

## References

- [Vulkan extension specification](https://docs.vulkan.org/refpages/latest/refpages/source/VK_NV_compute_occupancy_priority.html)
- [Khronos ABI definitions](https://github.com/KhronosGroup/Vulkan-Headers/blob/main/include/vulkan/vulkan_core.h)
