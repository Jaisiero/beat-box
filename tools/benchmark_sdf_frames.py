"""Compare unprofiled 4K SDF replays and publication frame intervals.

Requires DISPLAY and xdotool on the GPU host. Both runtimes must contain the
executable, shaders and includes. Intervals between flushed DET records include
publication, rendering and the next simulation step; they are not display or
Moonlight presentation timestamps. Do not run builds or other GPU tests alongside.
"""
import argparse
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import threading
import time


def stats(values):
    ordered = sorted(values)
    if not ordered:
        return None
    return dict(n=len(values), mean=statistics.mean(values),
                p95=ordered[int(.95 * (len(ordered) - 1))], maximum=ordered[-1],
                over_60hz_budget=sum(v > 1000 / 60 for v in values))


def run(runtime, env, log, steps, width, height):
    process = subprocess.Popen([str(runtime / "beat-box")], cwd=runtime, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    geometry = []
    errors = []

    def resize():
        try:
            for _ in range(600):
                if process.poll() is not None:
                    return
                found = subprocess.run(["xdotool", "search", "--all", "--onlyvisible",
                                        "--pid", str(process.pid), "--name", "Beat Box"],
                                       env=env, capture_output=True, text=True)
                if found.returncode == 0:
                    window = found.stdout.splitlines()[0]
                    subprocess.run(["xdotool", "windowsize", "--sync", window,
                                    str(width), str(height)], env=env, check=True, timeout=20)
                    geometry.append(subprocess.check_output(
                        ["xdotool", "getwindowgeometry", window], env=env, text=True, timeout=20))
                    return
                time.sleep(.1)
        except Exception as error:
            errors.append(str(error))

    thread = threading.Thread(target=resize)
    timer = threading.Timer(1200, process.kill)
    thread.start()
    timer.start()
    lines, states, intervals = [], [], []
    previous, publications = None, 0
    try:
        for line in process.stdout:
            lines.append(line)
            if line.startswith("[FRACTURE] respawn:"):
                publications += 1
            if line.startswith("DET "):
                now = time.monotonic()
                states.append(line)
                if previous is not None and len(states) > 20:
                    intervals.append(dict(step=len(states), ms=(now - previous) * 1000,
                                          publications=publications))
                previous, publications = now, 0
        process.wait()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        timer.cancel()
        thread.join()
    text = "".join(lines)
    log.write_text(text, encoding="utf-8")
    if (process.returncode or len(states) != steps or errors or not geometry or
            f"{width}x{height}" not in geometry[0] or
            re.search(r"VUID-|FAILED|MISMATCH|\[CHAIN\] max=\d+ errors=[1-9]", text) or
            any(not line.rstrip().endswith("viol=0") for line in states)):
        raise RuntimeError(f"Invalid run: {log}; exit={process.returncode}; states={len(states)}/{steps}; resize errors: {errors}")
    log.with_suffix(".json").write_text(json.dumps(intervals), encoding="utf-8")
    return states, dict(all=stats([r["ms"] for r in intervals]),
                        publication_frames=stats([r["ms"] for r in intervals if r["publications"]]),
                        other_frames=stats([r["ms"] for r in intervals if not r["publications"]]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("baseline", "candidate", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--scene", type=int, default=10)
    parser.add_argument("--scene-file", type=Path)
    parser.add_argument("--steps", type=int, default=600)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=2160)
    args = parser.parse_args()
    if args.steps <= 20 or min(args.repeats, args.width, args.height) < 1:
        parser.error("Require more than 20 steps and positive repeats / dimensions")
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for solver in (2, 3):
        reference = None
        for repeat in range(args.repeats):
            order = ("baseline", "candidate") if repeat % 2 == 0 else ("candidate", "baseline")
            for variant in order:
                env = {k: v for k, v in os.environ.items() if not k.startswith(("BB_", "VK_"))}
                env.update(BB_SCENE=str(args.scene), BB_SOLVER=str(solver), BB_DET_STEPS=str(args.steps))
                if args.scene_file:
                    env["BB_SCENE_FILE"] = str(args.scene_file.resolve())
                if args.scene == 11:
                    env.update(BB_KILL_Y=".4", BB_FRACTURE_SPAWN_STEPS="20")
                log = args.output / f"{variant}-{solver}-{repeat}.log"
                states, samples = run(getattr(args, variant).resolve(), env, log, args.steps,
                                      args.width, args.height)
                if reference is None:
                    reference = states
                if states != reference:
                    raise RuntimeError(f"Replay differs: {log}")
                rows.append(dict(scene=args.scene, solver=solver, repeat=repeat,
                                 variant=variant, **samples))
                (args.output / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
                print(f"{variant}, solver {solver}, repeat {repeat}: exact replay", flush=True)


if __name__ == "__main__":
    main()
