# F11 step-tail investigation

The 19.751 ms peak in the corrected PR48 scene is mostly AVBD work:
setup 1.600 ms, prepare 0.432 ms, main solve 9.769 ms,
post-stabilization 7.655 ms, finalize 0.289 ms. Narrow phase itself is
1.169 ms within setup. Render intervals overlap the step, but overlap is not
measured stall time and must not be subtracted from the GPU duration.

## Controlled experiments

RTX 4090, 3840 x 2160, F11 AVBD, spawn interval 1 step, 70 seconds,
4,199 measured steps per run. All runs include the merged AS-capacity guard,
compute occupancy priority and render fairness. Fine timing is disabled.
The requested render rate is 144 Hz except for the explicit 30 Hz control.
The scene retains all contact rows, ten main iterations, the depth cascade,
and the same post-stabilization sweeps. No AS publication errors were logged.

One- and two-thread workgroups were rejected. A one-thread run initially
reduced the maximum to 13.955 ms, but its repeat reached 20.896 ms while both
runs increased the mean to about 4.64 ms. Two threads reached 21.034 ms.
The original four-thread workgroup remains unchanged.

The admission experiment uses the existing recent GPU step average. When it
exceeds half the step budget, render fairness starts deferring immediately
while a simulation step is pending. Previously it only started after the
pending step's CPU age exceeded half the budget. The maximum frame-age
allowance is unchanged, and the loop still polls input and completion.
No query, queue wait, physics iteration, or contact is added or removed.

The lower-render-rate control tests competition for the GPU. It is not a
proposed runtime setting. Workgroup timing alone does not identify whether
register allocation, divergent lane lifetimes, or another hardware scheduling
effect dominates; hardware counter attribution has not been established.

## Reproduction and attribution

Use `BB_GPU_TIMELINE=1 BB_FRAME_TIMING=1` and analyze the resulting log with
`python3 tools/analyze_gpu_timeline.py run.log --top 10`.
The tool attributes stage intervals by their integer GPU timestamps, even
when completion/readback messages are interleaved, and counts steps exceeding
the 16.667 ms budget. Missing or ambiguous stage matches are left unattributed.
No new runtime queries or synchronization are needed for this analysis.

## Final admission A/B (candidate, reference, reference, candidate)

Reference is merged commit `32e1b61`. Both executables use the same shaders.
Times are milliseconds; FPS/Hz are means of the HUD reporting windows.

| Run | Mean step | P99 step | Worst step | Steps > 16.667 ms | Render FPS | Sim Hz |
|---|---:|---:|---:|---:|---:|---:|
| Early admission 1 | 4.339 | 10.110 | 12.448 | 0 | 130.12 | 59.99 |
| Reference 3 | 4.463 | 11.450 | 17.490 | 2 | 136.09 | 59.99 |
| Reference 4 | 4.480 | 11.614 | 22.503 | 6 | 135.93 | 59.99 |
| Early admission 2 | 4.336 | 9.988 | 12.760 | 0 | 131.01 | 59.99 |

This is a contention mitigation with a presentation-throughput tradeoff:
about 3% lower mean step cost and 13% lower P99, but approximately 4% fewer
rendered frames. Heavy intervals can approach 60 FPS even though the target
remains 144 Hz. Fast workloads retain the previous admission behavior.
Zero missed simulation budgets across these 8,398 candidate steps is not a
guarantee for arbitrary workloads or an isolated unexpected heavy step.

Worst CPU frame intervals were 21.26/21.67 ms for the candidate versus
20.34/20.88 ms for the reference. This change does **not** establish an
improvement in worst presentation intervals. Worst render GPU spans were
6.30/6.20 ms versus 6.25/6.00 ms. The optimization is specifically for the
simulation tail, without weakening its constraints or changing convergence.

The separate 30 FPS control had mean step 3.971 ms and maximum 12.585 ms.
It reinforces the contention explanation but is not the default configuration.

Raw server logs and summary: `/root/beat-box/work/step-tail/`.

## Validation

- All 10 CTest targets pass, including early admission, fast-history and
  presentation-starvation boundary cases in `render_pacing_tests`.
- The timeline parser test checks delayed/interleaved readbacks and integer
  timestamp precision at absolute GPU ticks above double's exact range.
- A 45-second real-time F11 run with synchronization validation exercises
  render deferrals without VUIDs, AS errors, NaN or AVBD violation reports.
  NVIDIA occupancy priority is disabled for this run because the installed
  validation layer predates the optional extension. Its timings are excluded
  from the performance comparison.
- All 1,800 dense AVBD replay checkpoints exactly match the corrected PR48
  reference, with post-stabilization active. Deterministic stepping bypasses
  render fairness, so this is a physics regression check, not a test of the
  admission policy; the real-time run and unit tests cover that policy.
