# Contiguous AVBD contact indices

Reference: PR53 (`6f4d3d0`). Two independent experiments target memory access
and wasted lanes without removing constraints or changing the color order.

## Contact stream

The previous cache stored `(packed manifold index, next node)` at each linked
node's index. Primal and post-stabilization still chased that chain every
sweep. The new cache gives each awake body a span in a contiguous uint stream.
Entries remain in the original canonical chain order and reference the shared
manifold store; this does not duplicate mutable manifold/contact data.

The color validator already traverses every dynamic body's chain. It now
reserves an interval of that length for each awake body using a GPU atomic
cursor. Prepare fills the reserved interval during its existing traversal.
Primal and post read `stream[offset + visit]` instead of following next-node
pointers. The initial prototype counted nodes in a second prepare traversal;
the final version reuses the validator's existing count.

Valid chains own disjoint nodes, so total requested entries cannot exceed the
node pool. A malformed over-capacity request receives a sentinel span and
uses the original bounded direct traversal, rather than writing outside the
cache or truncating a body's valid constraint set. Empty bodies receive a
zero-length span and still execute their inertial update.

## Storage and barriers

The existing 512 KiB tail of `avbd_state` is reused, after the body-state array:

| Region | Entries | Bytes |
|---|---:|---:|
| Packed manifold index stream | 65,536 uints | 262,144 |
| Body offset/count spans | 1,024 uint2s | 8,192 |
| GPU allocation cursor | 1 uint | 4 |

Total tail usage is 270,340 bytes, within the existing reservation. A host
static assertion checks capacity and body-count agreement. SimConfig is unchanged. There is no new
buffer allocation, dispatch, readback submission or CPU wait.

- Color reset clears the contact cursor in a previous task. Body-state prefix
  writes and the cursor occupy disjoint memory.
- Color validation reserves disjoint spans. Its existing AVBD-state dependency
  publishes span metadata before prepare fills the stream.
- Prepare writes complete spans before primal/post consume them through the
  same AVBD-state resource, which tracks the whole allocation including tails.
- Existing dispatcher-to-indirect-read dependencies remain in place.
- Sleeping flags, colors, topology and support layers stay fixed throughout
  these sweeps. Waking bodies rebuild their spans
  before solving. Rendering does not consume these auxiliary lists.

## Validation and measurement

F11, RTX 4090, 3840 x 2160, requested render rate 144 Hz, AVBD,
NV compute priority enabled, 70 seconds per live run, fragment spawning every
simulation step. GPU timeline and stage timers enabled; validation and workload
atomics disabled during timing. Each run contains 4,198 or 4,199 step samples.
ABBA order compares PR53 against the final validator-count implementation.

| Run | Mean step ms | P99 ms | Maximum ms | Prepare mean ms | Primal mean ms | Post mean ms |
|---|---:|---:|---:|---:|---:|---:|
| Baseline A1 | 4.04838 | 9.45478 | 12.48397 | 0.29506 | 1.10913 | 0.87856 |
| Contiguous B1 | 3.99088 | 9.33728 | 11.31203 | 0.29581 | 1.07531 | 0.85759 |
| Contiguous B2 | 4.00065 | 9.41482 | 11.17648 | 0.29543 | 1.07685 | 0.85810 |
| Baseline A2 | 4.05676 | 9.51043 | 12.49562 | 0.29620 | 1.11471 | 0.88051 |

Averaging the two runs per variant gives approximately 1.4% lower mean step,
3.2% lower primal time and 2.5% lower post-stabilization time. Preparation is
essentially unchanged. The lower observed maxima are encouraging, but do not
establish a worst-case bound or reproduce the previously reported 40 ms spike.
GPU timestamps measure elapsed intervals, including possible queue contention;
these are not isolated hardware stall or cache-hit measurements.

The final algorithm matched every recorded DET state in F11 and F7 AVBD
replays of 1,800 steps, and F11 TGS for 600 steps, with synchronization
validation enabled. Post-stabilization remained enabled (BB_DETERMINISTIC was
not set). The rebuilt final configuration is checked again with an F11 replay.
All 11 CTest tests pass. Replay equality covers the recorded state, not every
GPU byte or every possible scene.

## Rejected compact color-list experiments

A separate experiment appended awake body indices into per-color lists during
the existing depth reduction, dispatching only those entries. It preserved
recorded replay results, but was not retained:

| Variant | Mean step ms | Primal mean ms | Post mean ms |
|---|---:|---:|---:|
| Contiguous only, two-run average | 3.98059 | 1.07890 | 0.85465 |
| Compact list, four bodies per group, two-run average | 4.08971 | 1.15891 | 0.89554 |
| Contiguous only, additional reference | 3.97896 | 1.07573 | 0.85631 |
| Compact list, one body per group, single run | 3.95819 | 1.02648 | 0.88090 |

Four-body compaction regressed overall time by about 2.7%. A dense checkpoint
had 788 padded logical lanes instead of 6,272, illustrating that fewer logical
lanes does not imply better hardware utilization. Hardware occupancy was not
measured. Reduced independent workgroups and uneven contact workloads are
possible explanations, not established profiling findings.

The one-body variant improved primal but worsened post-stabilization. Its
combined improvement was only about 1.3% in the initial pair, below the 2%
screening threshold for continuing that experiment. It remains inconclusive
rather than a demonstrated universal regression. Neither variant's additional
SimConfig fields, lists, analyzer changes or dispatch remapping are shipped.

Raw logs are retained on the benchmark host under
`/root/beat-box/work/contiguous-contacts/`: `baseline3`, `opt1`, `opt2`,
`baseline4`, `listbase1`, `compact1`, `compact2`, `listbase2`, `onebase1`,
and `one1` (each with a `.log` suffix).

## Follow-up arithmetic experiments (2026-09-09)

Two additional candidates were tested independently against this PR and
rejected. Neither changes the shipped shader.

1. **Packed lower Hessian:** explicitly store the 21 entries consumed by LDL^T
   instead of a 6 x 6 array, omit upper-triangle accumulation, and preserve the
   operation order of every lower-triangle entry. F11 and F7 each matched all
   1,800 recorded DET states with synchronization validation. The baseline mean
   step was 3.98895 ms; two candidate runs measured 3.98917 and 3.98911 ms.
   Primal was 1.07455 ms before and 1.07337/1.07398 ms after; post was
   0.85569 ms before and 0.85726/0.85887 ms after. No useful improvement was
   observed. This is consistent with the compiler already eliminating unused
   entries, but generated machine code/register counts were not inspected.
   The final control run was interrupted when the existing input-reload
   service restarted Xorg and is excluded. This is not a completed ABBA result.
2. **Reuse self motion across manifolds:** compute the current body's pose
   delta once per solve while continuing to read each neighbor's current pose.
   F11/F7 again matched all 1,800 recorded states with synchronization
   validation. A fresh reference/candidate pair after the Xorg restart measured
   4.08363/4.32677 ms per step, 1.09318/1.16631 ms primal, and
   0.86046/0.93031 ms post. P99 increased from 9.50093 to 10.31843 ms.
   The screening comparison failed and was not promoted to a full repeated
   experiment. A register-lifetime or scheduling explanation is only a
   hypothesis; these measurements do not identify the hardware cause.

Runs used the same F11 4K/requested-144-Hz 70-second configuration described
above. Do not compare absolute times across the Xorg restart. Sources and raw
logs remain in `work/packed-hessian/` and `work/self-motion/` on the benchmark
host, including the interrupted control log. The final runtime was restored
to this PR's validated contact-stream implementation.
