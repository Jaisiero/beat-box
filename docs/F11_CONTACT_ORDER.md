# Complete contact ordering in heavy F11

Base: `b1e06117` (merged #40–#42). Two independent ordering defects were found while investigating the non-reproducible heavy-F11 replays recorded in `F11_PIPELINE_PROFILE.md`.

## Per-body chain truncation

`entry_chain_sort` sorted only the first 24 manifold links. The remaining suffix stayed in atomic insertion order. This also meant the selected prefix itself depended on insertion order; the resulting whole chain was not ordered by persistent pair identity.

The register-array insertion sort remains for chains of up to 24 nodes. Longer chains use bottom-up linked-list merge sort, with constant temporary storage and O(n log n) work. Nodes are owned by individual bodies, so simultaneous chain rewrites are disjoint. No contact is discarded and no extra dispatch, global allocator, host transfer or wait is introduced.

The first fixed AVBD heavy-F11 replay reached 33 nodes per body and exceeded 24 in 448 of 3,600 steps. The first changed state relative to the old implementation was step 1166, exactly the first over-cap chain. Two fixed AVBD replays then matched all 3,600 states; the old version differed across repeated runs.

## TGS and soft-PGS serial overflow ordering

Contacts that cannot fit the 32 solve colors use a serial solver bucket. Its pre-step, solve and relaxation traversed packed manifold indices. Packing depends on parallel allocation, so serial execution alone did not provide a deterministic order.

The graph-color solve dispatcher now gathers valid overflow contacts once per step, sorts them by the same unique persistent-pair code used by color arbitration, and publishes an index list. In-place heapsort bounds sorting to O(n log n) without temporary allocation. Empty overflow skips gathering/sorting. All three overflow passes consume that list; the serial diagnostic retains color order and uses the ordered list for its final overflow pass.

The list reserves 128 KiB plus a count in the existing DispatchBuffer allocation, after the existing indirect slots. Existing indirect offsets are unchanged. This also replaces repeated whole-contact scans in the serial bucket with traversal of just its actual contacts.

The dispatcher task declares read access to colors, bodies and collisions, read/write access to the dispatch buffer and diagnostic configuration. Existing solver task accesses include compute reads and indirect-command reads, so Daxa tracks the producer-to-consumer dependency. No cross-queue wait or barrier is removed.

## Verification

DET_HASHES enables GPU diagnostics: maximum original chain length, monotonic key order, bounded termination, and preservation of node count/sum/XOR. The overflow check verifies its count against the validator and strict key order. Diagnostics reset before narrow phase and report through the existing completed configuration readback. Normal execution does not perform these diagnostic walks.

`benchmark_sdf_frames.py` rejects nonzero ordering errors as well as its previous validation/invariant checks. The diagnostic checksums are not a formal set-equality proof; they supplement actual replay and validation runs.

Evidence directory: `/root/beat-box/work/f11-chain/`. Workload: F11 at 4K, spawn every five steps, 3,600 steps per replay; this accelerates accumulation relative to the normal 45-step cadence and retains the default kill plane.

## Repeated replay result

With the complete chain sort alone, AVBD repeated all 3,600 steps exactly. TGS still first differed at step 1092. With the ordered overflow list, the first TGS overflow is precisely step 1092 (four contacts outside the colors); two full TGS replays now match. The maximum overflow was seven contacts and six steps used overflow in this workload.

With both fixes, two runs per solver (14,400 step records total) reproduce identical DET states within AVBD and within TGS, with zero chain/list diagnostic errors. Maximum chain lengths are 33 AVBD and 37 TGS. The existing steps, iterations, materials, sleep settings and generated contacts are unchanged; correcting the accumulation order necessarily changes trajectories once an old unordered path is reached.

This is primarily a correctness/reproducibility result. It does not establish a large mean-frame improvement: overflow occurred in only six steps, and post-fix heavy trajectories are not the old trajectories. The sustained solve and path-tracing costs identified in the previous report remain optimization targets.

## Final validation

With Khronos synchronization validation enabled, the final code preserves all 600 reference states in F10/F7 with AVBD and TGS, and F9 with PGS and soft PGS. Both final 3,600-step heavy-F11 validation runs also match their corrected references exactly. These eight validation runs cover 10,800 step records without validation or ordering errors. All eight CTest targets pass.

The DET `viol` output now includes graph-color violations and ordering errors as well as AVBD violations, so the existing replay runner cannot silently ignore TGS coloring failures. The reset size is derived from the contiguous diagnostic block's offsets rather than another hard-coded field count.

A real-time accelerated-F11 smoke test also completes 595 stepped frames with synchronization validation, including minimize/restore, without reported validation errors.
