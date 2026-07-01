# Determinism in the beat-box physics engine

This document records what is and isn't reproducible across runs, the **determinism
mode** for when you need cross-launch reproducibility, and the underlying law that
makes full physical determinism impossible for dense piles in this architecture.

## TL;DR

| Property | Status |
|---|---|
| **In-process** reproducibility (run N steps, `reset()`, run N again) | **bitwise-identical** — the GPU kernels themselves are deterministic |
| **Rest-state** cross-launch reproducibility (e.g. scene_3 towers settled) | **bitwise-identical** (PR #10) |
| **Settling-trajectory** cross-launch reproducibility, normal mode | **NO** — diverges after a few hundred steps |
| Settling-trajectory cross-launch reproducibility, **determinism mode** | trajectory stays **locked** — measured ~5/1000 isolated reconverging steps on scene_3 (vs normal mode diverging permanently after ~260), at the cost of penetration depth |

## The root cause

Two processes running the *identical* step sequence diverge during active settling.
We proved (via `BB_DET_INPROC`, and a fixed-data/fixed-order "bounded push" test that
still diverged) that this is **not** a kernel race, **not** FP non-associativity in our
code, and **not** any data value we hash (contact geometry, chain order, coloring,
velocities — all bit-identical cross-launch). The only remaining source is the **GPU
driver's per-launch SPIR-V→ISA codegen**: instruction scheduling / FP grouping differs
each time the driver compiles our shaders, perturbing every floating-point op at the
sub-ULP level. This is inherent to the driver and not fixable from our side (forcing
`floatingPointMode=PRECISE` and clearing the SPIR-V cache did not help).

**Only convergence absorbs it.** A contractive iteration that reaches a *unique fixed
point* lands on the same bits on every launch — the sub-ULP perturbation is below the
fixed-point's basin. This is why the rest state and the AVBD main solve are
deterministic, and why anything non-convergent is not.

## The law (why depenetration cannot be deterministic)

Cross-launch determinism requires **tight Newton-convergence to a unique fixed point.**
Only the AVBD **main velocity solve** has it (per-body 6×6 LDLT Newton, solving the
*vanishing* constraint delta `J·dpose` — as it converges `k·C → 0`, so the result is
bit-stable). Everything that actively **depenetrates** exposes the driver noise:

- **Depenetration of dense piles** solves the full penetration `C0`, which **never
  vanishes** (bodies rest with residual penetration; a wedged pile has *no* configuration
  with all `C0 = 0`). An augmented-Lagrangian multiplier chasing a non-vanishing
  constraint never stabilizes — it *amplifies* the noise. (Measured: adding an AL dual to
  the post-stab made divergence *worse*, not better.)
- **TGS Soft's velocity Gauss-Seidel** converges too loosely in a few sub-steps; the
  residual carries the noise (measured: fully drifts).

**Therefore, in this penalty/velocity-solver architecture, cross-launch determinism and
active depenetration of dense piles are mutually exclusive.** The convergence that
absorbs the driver noise needs a vanishing constraint; depenetration is fundamentally
non-vanishing. A/B tested across five solver configurations (AVBD baseline, post-stab
off, unified-α, post-stab + AL dual, TGS Soft) — only the non-depenetrating convergent
paths are reproducible.

## Determinism mode — `BB_DETERMINISTIC`

Set the environment variable `BB_DETERMINISTIC=1` (see `src/main.cpp`) to enable it.
It skips the non-convergent alpha=0 post-stabilization pass, leaving only the convergent
AVBD main solve. The trajectory then stays **locked** across launches: measured on
scene_3, only ~5 of 1000 steps differ and each reconverges immediately, while the
solve-output hash `cph` is bit-identical *every* step — consistent with debug-readback
artifacts rather than true state divergence. (Normal mode, by contrast, diverges
permanently after ~260 steps.)

**Trade-off:** with no post-stab, dense piles retain deep penetration (bodies visibly
overlap). Use this mode for **regression tests, deterministic replays, and reproducible
test scenes** (e.g. freezing a pathological contact pocket as a repeatable testbed) — not
for visual-quality runs. The normal (default) mode keeps the post-stab and its good
contact resolution, and is not cross-launch reproducible during settling.

## Debug toolkit

- **`BB_DET_STEPS=N`** — run exactly one sim step per render frame (wall-clock ignored),
  self-load scene_3, and print a per-step line of pose/state hashes (`ph rh cph cp2 vhf
  vhi …`), auto-exiting at N. Two launches execute a bit-identical step sequence, so any
  divergence appears at its true first occurrence. Harness:
  `tools/determinism_det.ps1` (runs two processes, reports per-field first-diverge; see docs/TESTING.md).
- **`BB_DET_INPROC=1`** — run N steps, `reset()` to the identical initial state, run N
  again *in the same process*, and compare. Proves whether divergence is process-specific
  (it is) vs. a kernel race (it isn't).
- **Hash gauges** (in `SimConfig`, commutative `InterlockedXor` pose/state hashes):
  `cph`/`crh` = AVBD main-solve pose (post-solve, pre-depenetration — the deterministic
  reference); `ph`/`rh` = end-of-step pose; `cp2` = post-`PRE` (next step's input);
  `vhf`/`vhi` = velocity hashes; `lh` = full contact geometry; `sh` = avbd_state;
  `wh` = chain-walk resolved-partner order; `chash`+`viol` = coloring + validity.
  When `cph` matches cross-launch but `ph` doesn't, the divergence is in the (non-convergent)
  depenetration — exactly the law above.
