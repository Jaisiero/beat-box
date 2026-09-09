# Share GPU time with pending simulation steps

At 4K with a 144 Hz render target, dense F11 kept submitting ray tracing while
an asynchronous AVBD step was already taking too long. Independent CPU cadences
still share GPU execution resources: physics fell to roughly 40–44 Hz and
session worst steps reached 35–41 ms in the measured runs.

The real-time asynchronous scheduler now briefly defers a due presentation when
an outstanding step is at least half a simulation interval old (8.33 ms at 60 Hz).
It retries through the existing event/completion pump in at most 1 ms. Once the
last render is one simulation interval old, it stops deliberately deferring it.
This bounds the policy's delay; it does not guarantee display FPS against GPU,
swapchain or OS stalls. There is no new GPU wait, readback or queue dependency.

Fast steps, paused operation without a pending step, initial snapshots, targets
at or below the simulation frequency, and deterministic replay scheduling retain
the existing behavior. The configured render rate remains an upper target.
`BB_RENDER_FAIRNESS=0` disables the policy for A/B comparison. `BB_HUD_TRACE=1`
reports `deferred_render_polls` alongside the unchanged GPU cost/peak measurements.
It counts deferred pump attempts, not dropped display frames.

## Measured impact

RTX 4090, F11/AVBD, 3840×2160, 144 Hz render target, spawning every simulation
step (`BB_FRACTURE_SPAWN_STEPS=1`), up to 995 bodies. Four 70-second runs used the
same executable: off/on, then on/off. GPU stage tracing was enabled in all runs.
Means below average published HUD windows from seconds 40–70; peaks cover the
entire run. The changing simulation throughput means trajectories are not an
identical replay, so these are repeated workload comparisons.

| Policy / run | Mean SIM GPU | Simulation Hz | Render FPS | Worst step |
| --- | ---: | ---: | ---: | ---: |
| Off / 1 | 19.73 ms | 40.09 | 138.63 | 35.65 ms |
| On / 1 | 12.68 ms | 59.99 | 119.34 | 20.91 ms |
| On / 2 | 11.36 ms | 60.00 | 119.86 | 20.87 ms |
| Off / 2 | 18.91 ms | 43.93 | 138.85 | 41.11 ms |

The tradeoff is fewer presentations under simulation pressure. Frame interval
P99 in the first pair changed from 8.84 to 11.23 ms; session worst frame changed
from 16.80 to 19.12 ms. Image quality, solver equations, iteration counts and
contact generation are unchanged. This is GPU scheduling relief, not a claim
that individual AVBD kernels execute faster in isolation. The remaining roughly
21 ms step peaks still exceed the 16.67 ms simulation budget.

## Validation

- Ten CTest targets pass, including bounded render retry and persistent peaks.
- AVBD and TGS each match 600 reference checkpoints with synchronization
  validation enabled and post-stabilization retained. Replay scheduling bypasses
  the new policy; these checks guard accidental changes outside its scope.
- Four real-time stress runs preserve current <= peak and nondecreasing session
  peaks. Logs show no NaNs, AVBD coloring violations or reported sync hazards.
- A separate 20-second real-time 4K stress run enables synchronization validation
  and asserts that render deferrals actually occurred.

Reproduce the workload from the runtime directory with DISPLAY configured:

```sh
BB_SCENE=11 BB_SOLVER=2 BB_AUTOSTART=1 BB_RUN_SECONDS=70 \
BB_RENDER_HZ=144 BB_RENDER_FAIRNESS=1 BB_FRACTURE_SPAWN_STEPS=1 \
BB_FRAME_TIMING=1 ./beat-box
```

Resize the window to 3840×2160 before the measured window; repeat with fairness
zero, reverse run order, and compare simulation throughput as well as FPS.
Raw experiment logs on the GPU host are under `work/dense-pile/`.
