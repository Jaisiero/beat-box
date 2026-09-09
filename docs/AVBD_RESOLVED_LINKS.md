# Resolve AVBD contact links once per step

Reference: merged PR49 (`5e9490e`). AVBD primal and all post-stabilization
sweeps previously followed `node -> scratch manifold id -> packed manifold`
for every visit. The scratch-to-packed mapping is invariant after manifold
island packing and throughout the solve.

The existing AVBD prepare pass now writes `(packed manifold, next node)` for
each node visited by an awake dynamic body. Primal reads that pair directly.
This preserves the canonical chain order, manifold/contact contents, body
colors, floating-point constraint arithmetic, iteration counts and cascade
order. It does not compact or drop contacts, nor does it cache changing poses.

## Storage and synchronization

The AVBD state allocation gains a fixed tail of 65,536 `uint2` entries
(512 KiB). Body-state layout and existing buffer handles remain unchanged.
The host allocation expression and `avbd_resolved_links` agree on the offset:
`sizeof(AvbdBodyState) * BB_MAX_RIGID_BODY_COUNT`. There are no per-step CPU
allocations, transfers, readbacks, new tasks, or new queue waits.

- Manifold island packing precedes prepare, so map indices are final.
- Each node belongs to exactly one body's canonical chain; cache writes are
  disjoint between prepare invocations.
- Prepare skips static and sleeping bodies, as does primal. Sleep decisions
  precede prepare and are not changed between prepare and post-stabilization.
- Node visits are bounded by the current clamped node count; invalid mapped
  indices are represented by `MAX_U32`, matching the previous lookup helper.
- `AvbdTaskHead::avbd_state` already declares the entire buffer as compute
  read/write. Its task dependencies publish prepare writes to later passes;
  the extension is inside that same allocation, not an undeclared buffer.
- Existing inter-color compute barriers and inter-step queue ordering remain
  unchanged. Render does not consume this buffer. Old links belonging to
  dormant bodies are never read and are rebuilt before those bodies solve.

## Measurement protocol

F11 AVBD, 4K, requested render rate 144 Hz, spawn interval one step,
70 seconds / 4,199 measured steps per run. Existing render admission and
NVIDIA compute occupancy priority remain enabled. Fine timestamps are off.
Order is reference, cached links, cached links, reference. Both variants use
the enlarged allocation so the comparison includes cache population and
consumption, without changing allocator placement between variants.

Raw logs: `/root/beat-box/work/contact-links/`.

## Results

All times are milliseconds. Main includes the unchanged dual pass; the code
change itself targets primal, which is also used by post-stabilization.

| Run | Mean step | P99 step | Main | Post | Prepare | Render FPS | Worst step |
|---|---:|---:|---:|---:|---:|---:|---:|
| Reference 1 | 4.357 | 10.364 | 1.244 | 0.996 | 0.301 | 129.21 | 20.780 |
| Cached links 1 | 4.238 | 9.816 | 1.170 | 0.944 | 0.302 | 131.85 | 17.921 |
| Cached links 2 | 4.223 | 9.599 | 1.169 | 0.931 | 0.305 | 131.45 | 12.987 |
| Reference 2 | 4.341 | 10.379 | 1.240 | 0.995 | 0.296 | 130.26 | 19.127 |

Across the repeated pairs, main and post each improve about 6%, the full step
about 2.7%, and P99 about 6.4%. The prepare increase is approximately 5 us.
Render throughput improves slightly; this change does not alter admission
policy or trade solver iterations for speed. All runs contain 4,199 steps and
report no AS errors, NaNs or AVBD violations.

Worst steps remain variable. A candidate run still reached 17.921 ms, so this
is not a claim that every future step fits inside 16.667 ms.

## Validation

- All 10 CTest targets pass.
- The final F11 implementation exactly matches all 1,800 corrected reference
  checkpoints, including post-stabilization, with synchronization validation.
- F7's non-destructible box pool exactly matches its 600-step reference; the
  candidate run also enables synchronization validation.
- Validation disables optional NVIDIA compute occupancy priority because the
  installed layer predates the extension. Performance runs enable it and are
  separate from validation. No solver determinism flag disables post-stabilization.
- Final cleanup only removes an unused local and updates comments; GPU replays
  above were repeated on that final source.
