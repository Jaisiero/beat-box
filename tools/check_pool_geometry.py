#!/usr/bin/env python3
"""Independent OBB overlap check for BB_DET_DUMP poses from scene 7.

Reconstructs its static floor/walls. IDs are dump-line indices, not GPU body IDs.
Uses no engine contact depths, persistence, sleeping flags or extraction cap.
"""
import argparse
import itertools
import json
import math
from pathlib import Path


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def axes(q):
    length = math.sqrt(dot(q, q))
    if length == 0:
        raise ValueError("Zero quaternion")
    q = tuple(x / length for x in q)
    result = []
    for v in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]:
        u = cross(q[:3], v)
        w = cross(q[:3], u)
        result.append(tuple(v[i] + 2*q[3]*u[i] + 2*w[i] for i in range(3)))
    return result


def box(center, half, quaternion=(0, 0, 0, 1)):
    basis = axes(quaternion)
    extent = tuple(sum(half[j] * abs(basis[j][i]) for j in range(3)) for i in range(3))
    return center, half, basis, extent


def overlap(a, b):
    delta = tuple(b[0][i] - a[0][i] for i in range(3))
    if any(abs(delta[i]) >= a[3][i] + b[3][i] for i in range(3)):
        return 0.0
    candidates = a[2] + b[2] + [cross(u, v) for u in a[2] for v in b[2]]
    minimum = math.inf
    for axis in candidates:
        length = math.sqrt(dot(axis, axis))
        if length < 1e-8:
            continue
        axis = tuple(x / length for x in axis)
        radius = sum(a[1][i] * abs(dot(axis, a[2][i])) for i in range(3))
        radius += sum(b[1][i] * abs(dot(axis, b[2][i])) for i in range(3))
        depth = radius - abs(dot(delta, axis))
        if depth <= 0:
            return 0.0
        minimum = min(minimum, depth)
    return minimum


def self_test():
    a = box((0, 0, 0), (0.5, 0.5, 0.5))
    assert abs(overlap(a, box((0.9, 0, 0), (0.5,)*3)) - 0.1) < 1e-8
    assert overlap(a, box((1.1, 0, 0), (0.5,)*3)) == 0
    q = (0, 0, math.sin(math.pi/8), math.cos(math.pi/8))
    assert abs(overlap(a, box((1.1, 0, 0), (0.5,)*3, q)) - (0.5+math.sqrt(0.5)-1.1)) < 1e-8


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("poses", type=Path, nargs="?")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--max-depth-mm", type=float, default=10.0)
    parser.add_argument("--check", action="store_true", help="Exit with code 2 when geometry exceeds tolerance")
    args = parser.parse_args()
    if not math.isfinite(args.max_depth_mm) or args.max_depth_mm < 0:
        parser.error("--max-depth-mm must be finite and nonnegative")
    if args.self_test:
        self_test()
        print("Geometry oracle self-test passed")
        if args.poses is None:
            return
    if args.poses is None:
        parser.error("Provide a scene 7 pose dump")
    bodies = []
    for line in args.poses.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        values = [float(x) for x in line.split()]
        if len(values) != 11 or not all(math.isfinite(x) for x in values):
            raise ValueError("Expected finite unit-box dump with position/material/quaternion fields")
        bodies.append(box(tuple(values[:3]), (values[3],)*3, tuple(values[7:11])))
    count = len(bodies)
    bodies += [box((0, -50, 0), (50, 50, 50)),
               box((4.25, 5, 14), (0.25, 5, 4.5)),
               box((-4.25, 5, 14), (0.25, 5, 4.5)),
               box((0, 5, 18.25), (4.5, 5, 0.25)),
               box((0, 5, 9.75), (4.5, 5, 0.25))]
    pairs = []
    for i, j in itertools.combinations(range(len(bodies)), 2):
        if i >= count:
            continue
        depth = overlap(bodies[i], bodies[j])
        if depth > 1e-5:
            pairs.append({"a": i, "b": j, "depth_mm": round(depth * 1000, 3)})
    pairs.sort(key=lambda p: p["depth_mm"], reverse=True)
    maximum = pairs[0]["depth_mm"] if pairs else 0
    passed = count == 432 and maximum <= args.max_depth_mm
    print(json.dumps({"dynamic_count": count, "overlapping_pairs": len(pairs),
                      "max_depth_mm": maximum, "passed": passed,
                      "pairs_over_10mm": sum(p["depth_mm"] > 10 for p in pairs),
                      "pairs_over_50mm": sum(p["depth_mm"] > 50 for p in pairs),
                      "deepest_pairs": pairs[:20]}, indent=2))
    if args.check and not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
