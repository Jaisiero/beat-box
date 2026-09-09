# Skip empty AVBD color and support-layer batches

Reference: merged PR52 / `642e508`. Workload profiling identified expensive
serial color batches, but also batches containing no awake dynamic body.
Previously, any color below the global color count dispatched the whole body
grid. The cascade dispatched every such color at every layer through the
maximum occupied layer, including holes in that layer/color grid.

## Change

The existing support-depth reduction now ORs each awake body's color into its
clamped support layer's bit mask. Depth reset clears all masks beforehand.
The ordinary color dispatcher uses the union of those masks. The cascade
uses the individual mask for its layer. Empty batches receive zero workgroups.
The ordinary dispatcher moves after depth reduction so it consumes complete
masks; it is still before the first primal sweep.

The change adds twelve uints (48 bytes) to SimConfig and one atomic OR per
awake colored body in an existing pass. It adds no pass, submission, readback
or CPU wait. Indirect commands and their ordering barriers are still recorded;
their workgroup count becomes zero for an empty batch. Nonempty batches retain the same body indices, workgroup size,
color order, layer order, constraints and floating-point calculations. This
is not contact reduction, a new coloring algorithm or cooperative solving.

## Correctness and synchronization

- Membership uses exactly primal's dynamic/awake predicates and layer clamp.
  Unreachable support depth clamps to the last layer in both paths. Awake
  bodies with zero contacts are still included, preserving their inertial update.
- Colors, sleeping flags and support depths do not change between mask
  construction and the last post-stabilization sweep. Pose corrections do
  not alter membership in this already constructed schedule.
- Depth reset clears one disjoint mask per invocation, before its body-count
  guard. The existing group has 32 invocations, covering all twelve masks even
  for a small scene. Existing task dependencies order reset before reduction.
- The reduction's atomic OR permits multiple bodies to share a layer/color.
  The existing SimConfig dependency publishes its writes to the dispatchers.
- Dispatcher writes already depend on indirect-command reads in the solver.
  Offsets in DispatchBuffer and the existing indirect-command barriers remain
  unchanged. No CPU reads the masks to decide which work to submit.
- Skipped batches previously returned before updating any body. Empty colors
  therefore cannot change residual accumulation or convergence decisions.

## Validation

- F11 AVBD: all 1,800 checkpoints exactly match the merged reference.
- F7 AVBD: all 1,800 checkpoints exactly match the box-pool reference.
- F11 TGS: all 600 checkpoints exactly match its reference.
- These runs enable synchronization validation, disable optional NVIDIA
  occupancy priority because the layer predates the extension, and keep
  post-stabilization active (no BB_DETERMINISTIC flag).

The optional workload log reports the actual number of dispatched colors and
occupied cascade batches. Its analyzer uses those counts for the lane-ratio
estimate, while accepting older logs that used the full color/layer rectangle.
A regression test checks that a sleeping-only color is excluded from that
estimate. This diagnostic adjustment does not affect scheduling.

The final 1,800-step F11 replay also checks the reported mask counts and passes
all 11 CTest targets. In that replay, nonzero cascade batches decrease from
95,216 to 40,711 in total (57.2% fewer). At the 173-contact-body checkpoint,
there are 45 occupied batches instead of the previous 8 colors x 12 layers =
96. These are workgroup-producing batches, not removed CPU-recorded commands.

## Measurements

F11 AVBD, 3840x2160, requested render rate 144 Hz, spawn interval one step,
70 seconds per run, normal NVIDIA compute occupancy priority. Profiling
counters and the validation layer are disabled. The order is reference,
candidate, candidate, reference. Both variants use the same enlarged SimConfig
and dispatcher placement; the reference shaders retain the full color/layer
rectangle and do not build or consume the masks.

| Run | Mean step | P99 step | Worst step | Main | Post | Prepare |
|---|---:|---:|---:|---:|---:|---:|
| Reference 1 | 4.121 | 9.495 | 12.811 | 1.144 | 0.924 | 0.295 |
| Active batches 1 | 4.052 | 9.471 | 12.715 | 1.113 | 0.880 | 0.296 |
| Active batches 2 | 4.064 | 9.663 | 12.522 | 1.119 | 0.884 | 0.296 |
| Reference 2 | 4.138 | 9.540 | 12.102 | 1.151 | 0.931 | 0.296 |

All times are milliseconds; each run has 4,198 or 4,199 GPU step samples.
Averaged across pairs, the full step improves 1.7%, main 2.7%, and post 4.9%.
Prepare increases about 0.7 us. P99 does not improve (about 0.5% higher), and
worst values remain variable. This removes scheduling waste; it does not solve
the serial work of a heavily connected body or establish a lower worst-case
latency bound. No contacts or solver iterations were removed.

Raw logs: `/root/beat-box/work/active-batches/`, `baseline1.log`, `active1.log`,
`active2.log`, `baseline2.log`; stage means come from `[AVBD-STAGES]` and full
step statistics from `tools/analyze_gpu_timeline.py`.


## Rejected packed-matrix experiment

An additional prototype stored only the 21 lower-triangle elements used by
LDL factorization and omitted upper-triangle assembly. It preserved all 1,800
F11 checkpoints with synchronization validation, but did not improve timing:
4.051913 ms mean step with the existing matrix versus 4.053641 ms packed;
main 1.114151 versus 1.111287 ms, post 0.878775 versus 0.878320 ms.
The prototype is not included. These results do not establish whether the
compiler already eliminated the unused elements; they only show no useful
measured gain. Logs: `matrix-base.log`, `matrix-packed.log`, `packed-verify.log`.
