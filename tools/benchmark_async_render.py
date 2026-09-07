"""Compare synchronous/asynchronous scheduling at the same render cap.

Measures CPU frame-loop intervals, not display/Moonlight presentation timestamps.
BB_PACING_TRACE adds no GPU timestamps or deterministic readback. Physics throughput
is reported alongside frame times: rendering stale snapshots is not solver speedup.
Run replay validation separately before interpreting these performance results.
"""
import argparse
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import threading


def summarize(rows):
    if not rows:
        raise RuntimeError("No pacing samples")
    times = sorted(row["wall_ms"] for row in rows)
    return dict(frames=len(rows), steps=sum(row["steps"] for row in rows),
                mean_ms=statistics.mean(times), p95_ms=times[int(.95*(len(times)-1))],
                p99_ms=times[int(.99*(len(times)-1))], max_ms=times[-1],
                over_33ms=sum(t > 33.333 for t in times),
                physics_hz=sum(row["steps"] for row in rows)*1000/sum(times),
                frames_without_completed_step=sum(row["steps"] == 0 for row in rows))


def run(runtime, output, mode, solver, seconds, hz, width, height):
    env = {k:v for k,v in os.environ.items() if not k.startswith(("BB_", "VK_"))}
    env.update(DISPLAY=os.environ.get("DISPLAY", ":0"), BB_SCENE="11", BB_SOLVER=str(solver),
               BB_AUTOSTART="1", BB_RUN_SECONDS=str(seconds), BB_PACING_TRACE="1",
               BB_RENDER_HZ=str(hz), BB_ASYNC_SIM=str(mode), BB_FRACTURE_SPAWN_STEPS="5")
    process = subprocess.Popen([str(runtime/"beat-box")], cwd=runtime, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    resize_errors = []
    def resize():
        try:
            window = subprocess.check_output(["xdotool", "search", "--sync", "--onlyvisible", "--pid", str(process.pid)],
                                             env=env, text=True, timeout=180).splitlines()[0]
            subprocess.run(["xdotool", "windowsize", "--sync", window, str(width), str(height)],
                           env=env, check=True, timeout=20)
            geometry = subprocess.check_output(["xdotool", "getwindowgeometry", window],env=env,text=True)
            if f"{width}x{height}" not in geometry: raise RuntimeError(geometry)
        except Exception as error: resize_errors.append(str(error))
    thread = threading.Thread(target=resize)
    timer = threading.Timer(seconds+240, process.kill)
    thread.start(); timer.start()
    lines, rows = [], []
    elapsed, step = 0.0, 0
    try:
        for line in process.stdout:
            lines.append(line)
            if line.startswith("[FRAME-PHASES]"):
                row = {k:float(v) for k,v in re.findall(r"(\w+)=([0-9.eE+-]+)", line)}
                elapsed += row["wall_ms"]/1000
                step += row["steps"]
                row.update(elapsed_s=elapsed, simulation_step=step)
                rows.append(row)
        process.wait()
    finally:
        if process.poll() is None: process.kill(); process.wait()
        timer.cancel(); thread.join()
    text = "".join(lines)
    output.with_suffix(".log").write_text(text)
    output.with_suffix(".json").write_text(json.dumps(rows))
    if process.returncode or resize_errors or re.search(r"VUID-|SYNC-HAZARD|ASSERT FAILED", text):
        raise RuntimeError(f"Invalid run {output}: exit={process.returncode}, resize={resize_errors}")
    result = dict(mode=mode, solver=solver, steady=summarize([r for r in rows if r["elapsed_s"] >= 10]))
    loaded = [r for r in rows if 1800 <= r["simulation_step"] < 3000]
    if loaded: result["loaded"] = summarize(loaded)
    result["last_step"] = step
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=int, default=70)
    parser.add_argument("--hz", type=int, default=60)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--solvers", type=int, nargs="+", default=[2,3])
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=2160)
    args = parser.parse_args()
    if args.seconds <= 10 or min(args.hz,args.repeats,args.width,args.height) < 1: parser.error("Invalid duration or dimensions")
    args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for solver in args.solvers:
        for repeat in range(args.repeats):
            for mode in ([0,1] if repeat % 2 == 0 else [1,0]):
                name=args.output/f"pacing-{solver}-{repeat}-{mode}"
                print("START",name,flush=True)
                result=run(args.runtime.resolve(),name,mode,solver,args.seconds,args.hz,args.width,args.height)
                results.append(dict(repeat=repeat,**result))
                (args.output/"summary.json").write_text(json.dumps(results,indent=2))
                print("OK",result,flush=True)


if __name__ == "__main__": main()
