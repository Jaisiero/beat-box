"""Compare AS publication runtimes with exact replay checks and paired timing.

Both directories must contain beat-box and its runtime shader/include files.
Run on the GPU host with DISPLAY set. No builds or other simulation processes
should run concurrently. Profile timings include existing completion waits;
process wall time includes startup and rendering, not just simulation.
"""
import argparse
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import time


def summarize(values):
    ordered = sorted(values)
    return dict(n=len(values), mean=statistics.mean(values),
                p95=ordered[int(.95 * (len(ordered) - 1))], maximum=ordered[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scene", type=int, default=10)
    parser.add_argument("--steps", type=int, default=480)
    parser.add_argument("--profile", action="store_true")
    args = parser.parse_args()
    if args.steps < 1 or args.repeats < 1:
        parser.error("Use at least one step and one repeat")
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for solver in (2, 3):
        reference = None
        for repeat in range(args.repeats):
            order = ("baseline", "candidate") if repeat % 2 == 0 else ("candidate", "baseline")
            for variant in order:
                runtime = getattr(args, variant).resolve()
                env = {k: v for k, v in os.environ.items() if not k.startswith(("BB_", "VK_"))}
                env.update(BB_SCENE=str(args.scene), BB_SOLVER=str(solver), BB_DET_STEPS=str(args.steps))
                if args.scene == 11:
                    env.update(BB_KILL_Y="0.4", BB_FRACTURE_SPAWN_STEPS="20")
                if args.profile:
                    env.update(BB_AS_TIMING="1", BB_RESPAWN_TIMING="1")
                start = time.monotonic()
                result = subprocess.run([str(runtime / "beat-box")], cwd=runtime, env=env,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, timeout=1200)
                text = result.stdout
                log = args.output / f"{variant}-{solver}-{repeat}.log"
                log.write_text(text, encoding="utf-8")
                if result.returncode or re.search(r"VUID-|FAILED|MISMATCH", text):
                    raise RuntimeError(f"Run failed: {log}")
                states = [line for line in text.splitlines() if line.startswith("DET ")]
                if len(states) != args.steps or any(not line.endswith("viol=0") for line in states):
                    raise RuntimeError(f"Incomplete or invalid replay: {log}")
                if reference is None:
                    reference = states
                if states != reference:
                    raise RuntimeError(f"Replay changed: {log}")
                samples = {}
                for line in text.splitlines():
                    if not line.startswith(("[AS-PUBLISH]", "[AS-TLAS]", "[RESPAWN-MS]", "[FRACTURE-MS]")):
                        continue
                    tag = line.split("]", 1)[0] + "]"
                    for key, value in re.findall(r"([\w+]+)=([0-9.eE+-]+)", line):
                        samples.setdefault(tag + " " + key, []).append(float(value))
                # AS-PUBLISH includes the initial scene load; exclude it from fracture statistics.
                samples = {k: v[1:] if k.startswith("[AS-PUBLISH]") else v for k, v in samples.items()}
                row = dict(solver=solver, repeat=repeat, variant=variant, profile=args.profile,
                           process_wall_s=time.monotonic()-start,
                           statistics={k: summarize(v) for k, v in samples.items() if v})
                rows.append(row)
                (args.output / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
                print(f"{variant} solver={solver} repeat={repeat}: exact replay", flush=True)


if __name__ == "__main__":
    main()
