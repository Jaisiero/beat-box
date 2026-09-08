#!/usr/bin/env python3
"""Summarize BB_GPU_TIMELINE logs without treating overlapping GPU work as idle time."""
import argparse
import bisect
import json
import re
from pathlib import Path


def analyze(path, top=10, budget_ms=1000.0 / 60.0):
    pattern = re.compile(r'\[GPU-SPAN\] name="([^"]+)" submit=(\d+) begin_tick=(\d+) end_tick=(\d+) period_ns=([\d.]+) ms=([\d.]+)')
    steps, renders, stages = [], [], []
    for line in Path(path).read_text(errors="replace").splitlines():
        stage_match = re.search(r'\[(AVBD|TGS)-STAGES\].*gpu_ticks=(\d+(?::\d+){5})', line)
        if stage_match:
            ticks = [int(t) for t in stage_match[2].split(":")]
            if ticks != sorted(ticks):
                raise ValueError("Reversed solver stage timestamps")
            stages.append(dict(solver=stage_match[1], ticks=ticks))
        match = pattern.search(line)
        if not match:
            continue
        name, submit, begin, end, period, _ = match.groups()
        span = dict(submit=int(submit), begin=int(begin), end=int(end), period=float(period))
        if span["end"] < span["begin"]:
            raise ValueError("Reversed GPU timestamp interval")
        if name == "Simulation step HUD timestamps":
            steps.append(span)
        elif name == "Render GPU HUD timestamps":
            renders.append(span)
    renders.sort(key=lambda span: span["begin"])
    starts = [span["begin"] for span in renders]
    def duration(span):
        # Subtract integer ticks BEFORE conversion: absolute ticks exceed double precision.
        return (span["end"] - span["begin"]) * span["period"] / 1e6
    costs = sorted(map(duration, steps))
    if not costs:
        raise ValueError("No simulation spans; enable BB_GPU_TIMELINE=1")
    stages.sort(key=lambda stage: stage["ticks"][0])
    stage_starts = [stage["ticks"][0] for stage in stages]
    stage_names = ("setup", "prepare", "main", "post", "finalize")
    peaks = []
    for step in sorted(steps, key=duration, reverse=True)[:top]:
        overlaps = []
        for render in renders[:bisect.bisect_left(starts, step["end"])]:
            if render["end"] <= step["begin"]:
                continue
            if render["period"] != step["period"]:
                raise ValueError("Cannot compare different timestamp periods")
            overlaps.append((min(step["end"], render["end"]) - max(step["begin"], render["begin"])) * step["period"] / 1e6)
        peak = dict(submit=step["submit"], step_ms=duration(step), overlapping_render_ms=overlaps)
        # Readback logs can arrive out of order. Match contained GPU intervals,
        # never adjacent lines or rounded absolute floating-point timestamps.
        first = bisect.bisect_left(stage_starts, step["begin"])
        last = bisect.bisect_right(stage_starts, step["end"])
        matches = [stage for stage in stages[first:last] if stage["ticks"][-1] <= step["end"]]
        if len(matches) == 1:
            stage = matches[0]
            peak["solver"] = stage["solver"]
            peak["stages_ms"] = {
                name: (end - begin) * step["period"] / 1e6
                for name, begin, end in zip(stage_names, stage["ticks"], stage["ticks"][1:])
            }
        peaks.append(peak)
    return dict(samples=len(costs), mean_ms=sum(costs)/len(costs),
                budget_ms=budget_ms, over_budget_steps=sum(cost > budget_ms for cost in costs),
                p95_ms=costs[min(len(costs)-1, int(len(costs)*.95))],
                p99_ms=costs[min(len(costs)-1, int(len(costs)*.99))], max_ms=costs[-1],
                peaks=peaks, note="Overlap is concurrent elapsed time, not measured GPU stall time.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--top", type=int, default=10)
    parser.add_argument("--budget-ms", type=float, default=1000.0 / 60.0)
    args = parser.parse_args()
    print(json.dumps({path: analyze(path, args.top, args.budget_ms) for path in args.logs}, indent=2))
