"""Paired fracture-publication and steady SDF benchmark; requires a running X server.

Compare only the first fracture: CPU/GPU floating-point differences can change
later split topology, making whole-chain timings unequal-work comparisons.
"""
import argparse
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--baseline", type=Path, required=True)
parser.add_argument("--candidate", type=Path, required=True)
parser.add_argument("--cwd", type=Path, required=True)
parser.add_argument("--fixture", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--trials", type=int, default=10)
parser.add_argument("--skip-steady", action="store_true", help="Measure fracture phases only")
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
env = {k: v for k, v in os.environ.items() if not k.startswith(("BB_", "VK_"))}
env.setdefault("DISPLAY", ":0")
executables = {"cpu": args.baseline.resolve(), "gpu": args.candidate.resolve()}
for label, executable in executables.items():
    with (args.output / f"warm-{label}.log").open("w") as log:
        subprocess.run([str(executable)], cwd=args.cwd, env=dict(env, BB_COMPILE_ONLY="1"),
                       stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
results = []
reference_trace = {}
for mode in (("fracture",) if args.skip_steady else ("fracture", "steady-sdf")):
    for solver in (2, 3):
        for trial in range(args.trials):
            # Alternate order to reduce temperature/clock drift bias.
            for label in (("cpu", "gpu") if trial % 2 == 0 else ("gpu", "cpu")):
                name = f"{mode}-{solver}-{trial}-{label}"
                run_env = dict(env, BB_SCENE="3" if mode == "fracture" else "6",
                               BB_SOLVER=str(solver), BB_DET_STEPS="130" if mode == "fracture" else "600",
                               BB_METRICS_CSV=str((args.output / (name + ".csv")).resolve()))
                if mode == "fracture":
                    run_env.update(BB_SCENE_FILE=str(args.fixture.resolve()), BB_RESPAWN_TIMING="1")
                first_respawn = None
                first_fracture = None
                trace = []
                first_time = last_time = None
                with (args.output / (name + ".log")).open("w") as log:
                    proc = subprocess.Popen([str(executables[label])], cwd=args.cwd, env=run_env,
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    for line in proc.stdout:
                        now = time.monotonic()
                        log.write(line)
                        if line.startswith("DET step="):
                            if first_time is None:
                                first_time = now
                            last_time = now
                            if first_respawn is None:
                                trace.append(line.strip())
                        if line.startswith("[RESPAWN-MS]") and first_respawn is None:
                            first_respawn = {k: float(v) for k, v in re.findall(r"([\w+]+)=([\d.eE+-]+)", line)}
                        if line.startswith("[FRACTURE-MS]") and first_fracture is None:
                            first_fracture = {k: float(v) for k, v in re.findall(r"([\w+]+)=([\d.eE+-]+)", line)}
                    if proc.wait() != 0:
                        raise RuntimeError(f"{name} failed; see its log")
                if mode == "fracture":
                    if first_respawn is None:
                        raise RuntimeError(f"{name}: fixture did not fracture")
                    key = (mode, solver)
                    if key in reference_trace and trace != reference_trace[key]:
                        raise RuntimeError(f"{name}: pre-fracture state differs; timing comparison is invalid")
                    reference_trace[key] = trace
                result = dict(mode=mode, solver=solver, trial=trial, variant=label,
                              first_respawn=first_respawn, first_fracture=first_fracture,
                              steady_frame_ms=(last_time-first_time)*1000/599 if mode == "steady-sdf" else None)
                results.append(result)
                (args.output / "results.json").write_text(json.dumps(results, indent=2))
                print(name, result, flush=True)
for mode in (("fracture",) if args.skip_steady else ("fracture", "steady-sdf")):
    for solver in (2, 3):
        for label in executables:
            runs = [r for r in results if (r["mode"],r["solver"],r["variant"]) == (mode,solver,label)]
            values = [r["first_respawn"]["total"] if mode == "fracture" else r["steady_frame_ms"] for r in runs]
            print(mode, solver, label, "median_ms=", statistics.median(values), "range=", (min(values),max(values)))
