#!/usr/bin/env python3
"""Correlate optional AVBD workload counters with same-step GPU stages.

Use BB_AVBD_WORK_PROFILE=1 BB_GPU_TIMELINE=1. Counters describe awake endpoint
visits, not unique contact points. Correlation is descriptive, not causal.
"""
import argparse
import json
import math
import re
import statistics
from pathlib import Path

ARRAYS = ("color_bodies", "color_contacts", "color_max_contacts", "color_max_manifolds")
STAGES = ("setup_ms", "prepare_ms", "main_ms", "post_ms", "finalize_ms")
FEATURES = ("awake", "pairs", "manifolds", "colors", "depth", "contact_visits",
            "max_contacts", "max_manifolds", "sum_color_max_contacts", "active_colors", "useful_lane_fraction")


def pearson(xs, ys):
    mx, my = statistics.mean(xs), statistics.mean(ys)
    xx, yy = sum((x-mx)**2 for x in xs), sum((y-my)**2 for y in ys)
    return sum((x-mx)*(y-my) for x, y in zip(xs, ys)) / math.sqrt(xx*yy) if xx and yy else None


def parse(text):
    stages, work = {}, {}
    for line in text.splitlines():
        if line.startswith("[AVBD-STAGES]"):
            fields = dict(re.findall(r"(\w+)=([^ ]+)", line))
            if "gpu_ticks" not in fields:
                continue
            tick = int(fields["gpu_ticks"].split(":")[0])
            if tick in stages:
                raise ValueError("Duplicate stage timestamp")
            stages[tick] = {k: float(fields[k]) for k in STAGES}
        elif line.startswith("[AVBD-WORK]"):
            fields = dict(re.findall(r"(\w+)=([^ ]+)", line))
            arrays = {k: [int(v) for v in fields[k].split(":")] for k in ARRAYS}
            if any(len(v) != 32 for v in arrays.values()):
                raise ValueError("Expected 32 colors")
            row = {k: int(float(fields[k])) for k in ("bodies", "sleeping", "pairs", "manifolds", "colors", "depth")}
            # SimConfig.frame_count can also advance on render-only frames.
            row["frame"] = int(float(fields.get("frame", fields.get("step", "0"))))
            # Keep timestamp integers exact; absolute GPU ticks exceed 2**53.
            tick = int(fields["begin_tick"])
            if tick in work:
                raise ValueError("Duplicate workload timestamp")
            row.update(awake=sum(arrays["color_bodies"]),
                       contact_visits=sum(arrays["color_contacts"]),
                       max_contacts=max(arrays["color_max_contacts"]),
                       max_manifolds=max(arrays["color_max_manifolds"]),
                       sum_color_max_contacts=sum(arrays["color_max_contacts"]))
            row["active_colors"] = sum(n > 0 for n in arrays["color_bodies"])
            # Existing colored shaders use groups of four and scan all body ids.
            # This is a dispatch-lane ratio, not measured hardware occupancy.
            lanes = ((row["bodies"] + 3) // 4) * 4 * row["colors"]
            row["useful_lane_fraction"] = row["awake"] / lanes if lanes else 0.0
            row.update(arrays)
            work[tick] = row
    if not work:
        raise ValueError("No AVBD workload samples")
    if work.keys() - stages.keys():
        raise ValueError("Workload samples lack matching GPU stages")
    rows = []
    for tick, row in sorted(work.items()):
        row["sample"] = len(rows) + 1
        row.update(stages[tick])
        row["stages_ms"] = sum(row[k] for k in STAGES)
        rows.append(row)
    return rows


def summarize(rows):
    slow = sorted(rows, key=lambda r: r["stages_ms"], reverse=True)[:max(1, math.ceil(len(rows)*.01))]
    def means(rs):
        return {k: statistics.mean(r[k] for r in rs) for k in (*FEATURES, *STAGES, "stages_ms")}
    return dict(samples=len(rows), mean=means(rows), slowest_one_percent=means(slow),
                correlations={stage: {k: pearson([r[k] for r in rows], [r[stage] for r in rows])
                                      for k in FEATURES} for stage in ("main_ms", "post_ms", "setup_ms")},
                worst=sorted(rows, key=lambda r: r["stages_ms"], reverse=True)[:5])


def analyze(text):
    rows = parse(text)
    return {name: summarize(rs) for name, rs in
            (("all", rows), ("first1200", [r for r in rows if r["sample"] <= 1200]),
             ("later", [r for r in rows if r["sample"] > 1200])) if rs}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps({str(p): analyze(p.read_text()) for p in args.logs}, indent=2))
