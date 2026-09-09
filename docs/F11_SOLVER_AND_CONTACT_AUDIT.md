# Heavy F11: repeated solve work and collision audit

Reference: `7642dc9`, with complete contact-chain ordering and deterministic TGS overflow from this PR. This comparison does not use the old unordered trajectories.

## Retained optimization

AVBD computed the same body translation and small-angle rotation deltas inside each normal and tangent constraint evaluation. `AvbdMotion` computes them once for each manifold visit in the primal and dual passes. The cache is local to that invocation: it is rebuilt when another color or sweep visits the pair, after the existing inter-color dependency. No pose is cached across sweeps, no GPU buffer is added, and no CPU readback is introduced.

The primal entry also passes the two supported stabilization modes as literals to the implementation. Slang can specialize main and post-stabilization constraint arithmetic instead of carrying a general push-constant alpha through every contact. The host already emits only alpha=1 for the main solve and alpha=0 for post-stabilization. Iterations, timestep, relaxation, margins, contact generation, contact order, warm-start matching and barriers are unchanged.

Two profiled 3,600-step AVBD runs preserve every reference DET record. In the loaded second half (steps 1801–3600), mean GPU times are:

| GPU scope | Reference | Candidate | Change |
|---|---:|---:|---:|
| Main AVBD sweeps, including dual | 2.874 ms | 2.204 ms | -23.3% |
| Post-stabilization scope | 2.103 ms | 1.769 ms | -15.9% |
| Main + post scopes | 4.978 ms | 3.973 ms | -20.2% |

The narrow phase remains approximately 0.99 ms in this workload. It is significant, but smaller than the combined solve scopes. The post scope also includes velocity reconstruction and impact passes; these are existing timestamp boundaries, not isolated-kernel estimates.

Evidence: `/root/beat-box/work/f11-solver-opt/`, `baseline-{0,3}.log` and `motion-{0,1}.log`. Hardware: RTX 4090; F11 at 3840x2160; one deterministic simulation step per rendered frame; new structure every five steps; default kill plane. Physics hashes include post-stabilization: `BB_DETERMINISTIC` is not enabled.

## Whole-frame A/B

The final unprofiled comparison ran baseline/candidate/candidate/baseline for each solver. Every one of the 28,800 DET records matched its corrected reference. Each table entry pools 7,160 intervals from two runs (the first 20 records per run are excluded):

| Solver | Mean frame, reference → candidate | P95 frame, reference → candidate | Mean publication interval, reference → candidate |
|---|---:|---:|---:|
| AVBD | 10.157 → 9.332 ms (-8.1%) | 13.276 → 12.035 ms | 11.092 → 10.134 ms |
| TGS | 11.240 → 11.218 ms | 14.081 → 14.134 ms | 12.450 → 12.421 ms |

TGS is unchanged within measurement noise; this is an AVBD optimization. For AVBD's loaded second half alone, mean frame interval falls from 11.862 to 10.827 ms (-8.7%). The GPU-scope table above is also for that second half; the whole-run frame table includes the buildup period. Evidence: `final-results.json`, `final-summary.json` and `final-*.{log,json}` in the evidence directory.

## Collision pipeline review

1. **Broad phase:** the LBVH emits each unordered pair once (`query index < leaf index`). Static-static candidates are discarded by narrow phase. Moving this filter into broad phase preserved replay states but did not establish a useful whole-frame gain in heavy F11, so it was not retained.
2. **Bounding-volume filter:** voxel pairs already reject separating face axes before surface traversal. A trial adding the nine edge-cross axes changed the generated-contact trajectory at step 83 for AVBD and step 94 for TGS, without a clear timing benefit. It was rejected rather than assuming that a tighter geometric filter preserves this approximate detector's existing outputs.
3. **Voxel candidates:** sample cubes are bounded against the cropped target grid before SDF reads; a support-sphere distance check skips unreachable cells; neighbor iteration is clamped to the target grid and empty cells are skipped. Full cell SAT and exposed-face/gradient filtering reject internal faces. Near-surface samples are assigned to one dominant-axis bucket. The deep SDF branch can assign the same sample to multiple directional buckets.
4. **Manifold reduction:** each directional reducer keeps four extreme feature winners, removes repeated voxel identities, and suppresses corners that retreat onto the same point. Most emitted manifolds have fewer than four points. Empty voxel and OBB contact sets already return false before allocation.
5. **Persistence:** voxel matching uses the exact directional key; OBB matching retains its guarded cross-key/swapped fallback. A trial resolving all requested voxel keys in one old-chain walk reproduced both solvers, but its timing gain was within measurement noise and did not justify the added path.

## Captured contact redundancy

The offline tool `tools/audit_contact_redundancy.py` examines existing `BB_DET_DUMP` contact captures. It does not run in the frame loop or add normal-execution readbacks. Exact duplicates require the same unordered body pair, normal, point positions and depths; feature identity is intentionally ignored for geometric comparison. The tool separately counts normals within one degree, which by itself is not proof of redundant support.

| Solver / end step | Bodies | Body pairs | Manifolds | Contacts | Exact duplicate manifolds |
|---|---:|---:|---:|---:|---:|
| AVBD / 1800 | 463 | 1225 | 1670 | 3214 | 0 |
| AVBD / 3000 | 714 | 2128 | 3015 | 5758 | 1 |
| AVBD / 3600 | 838 | 2548 | 3561 | 6835 | 0 |
| TGS / 3600 | 1024 | 1853 | 1960 | 3606 | 0 |

There are no duplicate pair/key allocations or repeated points within a manifold in these captures. At AVBD step 3600, 2,682 of 3,561 manifolds have only one or two points, and no two manifolds of a pair have normals within one degree. This is a sample of the workload, not a proof that duplicates never occur between captures.

The one geometric duplicate is a deeply embedded pair with persistent IDs 699 and 713, keys 92 and 95, four identical points and approximately 0.249 m stored penetration. It is consistent with the deep-gradient branch's multiple-bucket membership. Deleting one copy also changes accumulated penalty stiffness and warm-start state in a finite-iteration solve; this audit does not claim that deleting it is behavior-preserving. Four rows out of 5,758 at that capture are not the sustained bottleneck. No arbitrary contact/manifold cap was introduced.

The TGS capture wrote the full geometry, then the process stalled during `vkDestroyDevice` in the NVIDIA GLX/glcore teardown lock and was terminated after a debugger backtrace. That capture is geometry evidence, not a successful process-exit test. Normal TGS runs without the optional dump are validated separately.

## Rejected solve experiments

- Updating only the lower Hessian triangle preserved states but did not improve timing; the compiler already removes the unused work.
- Compact body lists alone saved too little. Packing 4–32 active bodies per workgroup was slower than one body per workgroup, due to the uneven per-body workload.
- Distributing one body's matrix entries among workgroup lanes was slower and changed numerical results. It was removed.
- Explicit next-link prefetch and grouped old-manifold lookups did not beat the retained implementation reliably.

Only the thread-local motion reuse and stabilization-mode specialization remain in the renderer/solver implementation. Experimental list buffers, new task heads, workgroup changes and tighter collision gates were removed.

## Reproduction

Use `tools/benchmark_sdf_frames.py::run` with the baseline and candidate runtimes, 3600 steps, width 3840 and height 2160. Set `DISPLAY=:0`, `BB_SCENE=11`, `BB_SOLVER=2` (or 3), `BB_DET_STEPS=3600` and `BB_FRACTURE_SPAWN_STEPS=5`; do not set `BB_KILL_Y`. Enable `BB_FRAME_TIMING=1` only for the GPU-scope comparison. The CLI's default F11 recycling fixture uses a different cadence/kill plane, so it is not this workload.

For offline captures, additionally set `BB_DET_DUMP=/absolute/path/capture.txt`, then run:

```sh
python3 tools/audit_contact_redundancy.py /absolute/path/capture.txt.contacts.json
```

Whole-frame comparisons use reversed A/B order, no GPU timing instrumentation, and assert equality of every DET record. The measured intervals between flushed DET records include rendering/publication and the following step; they are not Moonlight presentation timestamps or pure solver time.

## Final validation

All eight CTest targets pass. The eight unprofiled reversed-order A/B runs match all 28,800 DET records against corrected references. Twelve candidate synchronization-validation runs across F5/F6/F7/F9/F10/F11 match another 13,200 records, including 3,600 steps for each F11 solver. Both final F11 runs exit successfully.
