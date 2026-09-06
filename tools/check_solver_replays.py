"""Compare same-step solver replays without treating GPU packing order as identity."""
import argparse
import csv
import json
from pathlib import Path


def canonical_capture(capture):
    bodies = capture["bodies"]
    if len({b["id"] for b in bodies}) != len(bodies):
        raise ValueError("Duplicate persistent body IDs")
    manifolds = []
    for original in capture["manifolds"]:
        m = dict(original)
        m["a"], m["b"] = bodies[m["a"]]["id"], bodies[m["b"]]["id"]
        manifolds.append(m)
    return dict(capture, bodies=sorted(bodies, key=lambda b: b["id"]),
                manifolds=sorted(manifolds, key=lambda m: (m["a"], m["b"], m["key"], json.dumps(m, sort_keys=True))))


def compare(values, paths, label):
    for i in range(1, len(values)):
        if values[i] != values[0]:
            raise ValueError(f"{label} differ: {paths[0]} vs {paths[i]}")
    print(f"{label}: {len(values)} matching replays")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs", nargs="+", type=Path)
    parser.add_argument("--captures", nargs="+", type=Path)
    parser.add_argument("--csv", nargs="+", type=Path)
    parser.add_argument("--sleepers", type=int, default=432)
    args = parser.parse_args()
    if not any((args.logs, args.captures, args.csv)):
        parser.error("Provide logs, captures or CSVs from repeated identical runs")
    if args.logs:
        traces = []
        for path in args.logs:
            text = path.read_text(encoding="utf-8", errors="replace")
            if "DETERMINISTIC mode ON" in text:
                raise ValueError("Post-stabilization-disabled runs cannot validate this regression")
            trace = [line for line in text.splitlines() if line.startswith("DET step=")]
            if not trace:
                raise ValueError(f"No solver checkpoints: {path}")
            traces.append(trace)
        compare(traces, args.logs, "Full checkpoint traces")
    if args.captures:
        compare([canonical_capture(json.loads(p.read_text(encoding="utf-8"))) for p in args.captures], args.captures, "Canonical captures")
    if args.csv:
        compare([p.read_bytes() for p in args.csv], args.csv, "Physics CSVs")
        for path in args.csv:
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            first = next((i for i, r in enumerate(rows) if int(r["sleeping"]) == args.sleepers), None)
            if first is None or any(int(r["sleeping"]) != args.sleepers for r in rows[first:]):
                raise ValueError(f"Full sleep absent or not sustained: {path}")
            print(f"{path}: full sleep from recorded frame {rows[first]['frame']}")


if __name__ == "__main__":
    main()
