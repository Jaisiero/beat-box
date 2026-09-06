"""Run GPU fracture oracles and compare production batching with identical replays.

Requires a Vulkan-capable display and the validation layer; intentionally not a
CPU-only CTest test. Run from the project's runtime directory (build/Release).
"""
import argparse
import os
from pathlib import Path
import re
import subprocess


def run(executable, output, scene, solver, steps, repeat, verify):
    mode = "oracle" if verify else "production"
    name = f"{mode}-{scene}-{solver}-{repeat}"
    env = {k: v for k, v in os.environ.items() if not k.startswith(("BB_", "VK_"))}
    env.update(BB_SCENE=str(scene), BB_SOLVER=str(solver), BB_DET_STEPS=str(steps),
               BB_METRICS_CSV=str(output / (name + ".csv")),
               VK_INSTANCE_LAYERS="VK_LAYER_KHRONOS_validation",
               VK_LAYER_ENABLES="VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT")
    if not verify:
        env["BB_RESPAWN_TIMING"] = "1"
    if verify:
        env.update(BB_CENSUS_VERIFY="1", BB_POOL_VERIFY="1", BB_FRAGMENT_VERIFY="1")
    if scene == 3:
        env["BB_SCENE_FILE"] = str(Path(__file__).resolve().parents[1] /
                                    "tests/scenes/fracture_frame_drop.txt")
    if scene == 11:
        env.update(BB_KILL_Y="0.4", BB_FRACTURE_SPAWN_STEPS="20")
    log = output / (name + ".log")
    with log.open("w", encoding="utf-8") as stream:
        result = subprocess.run([str(executable)], env=env, stdout=stream,
                                stderr=subprocess.STDOUT, timeout=1200)
    text = log.read_text(encoding="utf-8", errors="replace")
    if result.returncode or re.search(r"VUID-|FAILED|MISMATCH", text):
        raise RuntimeError(f"{name} failed; inspect {log}")
    states = [line for line in text.splitlines() if line.startswith("DET ")]
    if len(states) != steps or any(not line.endswith("viol=0") for line in states):
        raise RuntimeError(f"{name}: incomplete replay or solver invariant violation")
    print(f"{name}: {steps} steps passed", flush=True)
    return states


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, default=Path("./beat-box"))
    parser.add_argument("--output", type=Path, default=Path("gpu-fracture-validation"))
    args = parser.parse_args()
    executable = args.executable.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    reference = {}
    for scene, steps, repeat in [(3, 900, 1), (10, 480, 1), (10, 480, 2), (11, 1500, 1)]:
        for solver in (2, 3):
            oracle = run(executable, output, scene, solver, steps, repeat, True)
            production = run(executable, output, scene, solver, steps, repeat, False)
            if oracle != production:
                first = next(i + 1 for i, (a, b) in enumerate(zip(oracle, production)) if a != b)
                raise RuntimeError(f"Scene {scene}, solver {solver}: batch mismatch at step {first}")
            key = (scene, solver)
            if repeat > 1 and reference[key] != production:
                raise RuntimeError(f"Scene {scene}, solver {solver}: repeat mismatch")
            reference[key] = production
    print("All production/oracle replays and repeated F10 states match.")


if __name__ == "__main__":
    main()
