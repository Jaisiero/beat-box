#!/usr/bin/env python3
"""Summarize BB_GPU_TIMELINE logs without treating overlapping GPU work as idle time."""
import argparse
import bisect
import json
import re
from pathlib import Path


def analyze(path, top=10):
    pattern = re.compile(r'\[GPU-SPAN\] name="([^"]+)" submit=(\d+) begin_tick=(\d+) end_tick=(\d+) period_ns=([\d.]+) ms=([\d.]+)')
    steps, renders = [], []
    for line in Path(path).read_text(errors="replace").splitlines():
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
    peaks = []
    for step in sorted(steps, key=duration, reverse=True)[:top]:
        overlaps = []
        for render in renders[:bisect.bisect_left(starts, step["end"])]:
            if render["end"] <= step["begin"]:
                continue
            if render["period"] != step["period"]:
                raise ValueError("Cannot compare different timestamp periods")
            overlaps.append((min(step["end"], render["end"]) - max(step["begin"], render["begin"])) * step["period"] / 1e6)
        peaks.append(dict(submit=step["submit"], step_ms=duration(step), overlapping_render_ms=overlaps))
    return dict(samples=len(costs), mean_ms=sum(costs)/len(costs),
                p95_ms=costs[min(len(costs)-1, int(len(costs)*.95))],
                p99_ms=costs[min(len(costs)-1, int(len(costs)*.99))], max_ms=costs[-1],
                peaks=peaks, note="Overlap is concurrent elapsed time, not measured GPU stall time.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--top", type=int, default=10)
    args = parser.parse_args()
    print(json.dumps({path: analyze(path, args.top) for path in args.logs}, indent=2))
