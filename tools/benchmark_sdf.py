"""Fixed-step GPU narrow-phase benchmark. Run on the Linux simulation host.
The application must be closed; this tool neither kills it nor edits shader sources.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import statistics
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--solver", type=int, choices=(2, 3), default=2)
    args = parser.parse_args()
    if args.steps < 2 or args.repeats < 1:
        parser.error("steps >= 2 and repeats >= 1 required")
    runtime, output = args.runtime.resolve(), args.output.resolve()
    executable = runtime / "beat-box"
    fixture = Path(__file__).resolve().parents[1] / "tests/scenes/fracture_frame_drop.txt"
    if not executable.is_file() or not fixture.is_file():
        parser.error("Missing executable or fracture fixture")
    if subprocess.run(["pgrep", "-x", "beat-box"], stdout=subprocess.DEVNULL).returncode == 0:
        parser.error("Close the running beat-box application before benchmarking")
    output.mkdir(parents=True, exist_ok=True)
    env = {k: v for k, v in os.environ.items() if not k.startswith("BB_")}
    env.pop("VK_INSTANCE_LAYERS", None)
    env.pop("VK_LAYER_ENABLES", None)
    env.setdefault("DISPLAY", ":0")
    env.update(BB_SOLVER=str(args.solver), BB_DET_STEPS=str(args.steps), BB_RESPAWN_TIMING="1")
    shader = runtime / "src/shaders/simulation/collision_detection.slang"
    report = dict(solver=args.solver, steps=args.steps, repeats=args.repeats,
                  shader_sha256=hashlib.sha256(shader.read_bytes()).hexdigest(),
                  sdf_build_shader_sha256=hashlib.sha256((runtime / "src/shaders/simulation/voxel_sdf.slang").read_bytes()).hexdigest(),
                  runs=[], scenes={})
    for trial in range(1, args.repeats + 1):
        for scene in (3, 5, 6):
            prefix = output / f"scene-{scene}-run-{trial}"
            metrics = prefix.with_suffix(".csv")
            log = prefix.with_suffix(".log")
            run_env = dict(env, BB_SCENE=str(scene), BB_METRICS_CSV=str(metrics))
            if scene == 3:
                run_env["BB_SCENE_FILE"] = str(fixture)
            with log.open("w") as stream:
                subprocess.run([str(executable)], cwd=runtime, env=run_env, stdout=stream,
                               stderr=subprocess.STDOUT, timeout=180, check=True)
            text = log.read_text(errors="replace")
            if f"DET step={args.steps} " not in text:
                raise RuntimeError(f"Incomplete physics run: {log}")
            times = [float(v) for v in re.findall(r"\[FRACTURE-NP\] gpu_ms=([0-9.eE+-]+)", text)]
            if len(times) != args.steps or not all(math.isfinite(v) and v >= 0 for v in times):
                raise RuntimeError(f"Missing/invalid GPU timestamps: {log}")
            with metrics.open() as stream:
                rows = list(csv.DictReader(stream))
            if len(rows) != args.steps - 1:
                raise RuntimeError(f"Unexpected physics row count: {metrics}")
            builds = [dict(gpu_ms=float(ms), shapes=int(count)) for ms, count in
                      re.findall(r"\[SDF-BUILD\] gpu_ms=([0-9.eE+-]+) shapes=(\d+)", text)]
            if not builds or not all(math.isfinite(b["gpu_ms"]) and b["gpu_ms"] >= 0 for b in builds):
                raise RuntimeError(f"Missing/invalid voxel build timestamps: {log}")
            item = dict(builds=builds, scene=scene, trial=trial, samples=len(times), mean_ms=statistics.mean(times),
                        p95_ms=sorted(times)[math.ceil(.95 * len(times))-1], max_ms=max(times),
                        metrics_sha256=hashlib.sha256(metrics.read_bytes()).hexdigest())
            report["runs"].append(item)
            print(json.dumps(item), flush=True)
    for scene in (3, 5, 6):
        runs = [r for r in report["runs"] if r["scene"] == scene]
        report["scenes"][scene] = dict(median_mean_ms=statistics.median(r["mean_ms"] for r in runs),
                                     repeat_metrics_identical=len({r["metrics_sha256"] for r in runs}) == 1)
    (output / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
