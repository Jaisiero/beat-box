"""Measure stored contacts against the captured voxel union, before/after AVBD.
Usage: python tools/analyze_contact_capture.py scene_dump.txt.contacts.json
Requires numpy. Distances are meters; negative is inside solid material.
"""
import json
import sys
import numpy as np


def rotate(q, p):
    q = np.asarray(q)
    return p + 2 * np.cross(q[:3], np.cross(q[:3], p) + q[3] * p)


def surface(shape):
    dims = shape["dims"]
    occupied = set()
    for z in range(dims[2]):
        for y in range(dims[1]):
            for x in range(dims[0]):
                i = x + dims[0] * (y + dims[1] * z)
                if shape["occupancy"][i // 32] & (1 << (i % 32)):
                    occupied.add((x, y, z))
    lo, hi = [], []
    vs = shape["voxel_size"]
    origin = np.array(shape["origin"])
    for c in sorted(occupied):
        for axis in range(3):
            for sign in (-1, 1):
                n = list(c)
                n[axis] += sign
                if tuple(n) in occupied:
                    continue
                a = origin + np.array(c) * vs
                b = a + vs
                a[axis] = b[axis] if sign > 0 else a[axis]
                b[axis] = a[axis]
                lo.append(a)
                hi.append(b)
    return occupied, np.array(lo), np.array(hi)


def analyze(capture):
    shapes = capture["shapes"]
    geometry = {b["shape"]: surface(shapes[b["shape"]-1]) for b in capture["bodies"] if b["shape"]}
    def distance(body, point, start):
        prefix = "start_" if start and body["start_valid"] else ""
        q = np.array(body[prefix + "rotation"])
        q[:3] *= -1
        p = rotate(q, np.array(point) - body[prefix + "position"])
        if not body["shape"]:
            a, b = np.array(body["minimum"]), np.array(body["maximum"])
            d = np.maximum(np.maximum(a-p, p-b), 0)
            return float(np.linalg.norm(d)) if np.any(d) else -float(np.min(np.minimum(p-a,b-p)))
        sh = shapes[body["shape"]-1]
        occupied, lo, hi = geometry[body["shape"]]
        if not len(lo):
            raise ValueError("Referenced shape has no surface")
        closest = np.maximum(lo, np.minimum(hi, p))
        d = float(np.min(np.linalg.norm(closest-p, axis=1)))
        cell = tuple(np.floor((p-sh["origin"])/sh["voxel_size"]).astype(int))
        return -d if cell in occupied else d
    rows = []
    for m in capture["manifolds"]:
        a, b = [capture["bodies"][m[k]] for k in ("a", "b")]
        for c in m["contacts"]:
            before = [distance(x,c["position"],True) for x in (a,b)]
            after = [distance(x,c["position"],False) for x in (a,b)]
            rows.append(dict(ids=[a["id"],b["id"]], key=m["key"], position=c["position"], penetration=c["penetration"], rescue=all(f==0xffffff for f in c["features"]), start_pose_available=[a["start_valid"],b["start_valid"]], before=before, after=after))
    return sorted(rows,key=lambda r:max(r["before"]),reverse=True)


def validate_surfaces(capture):
    errors = []
    for index in sorted({b["shape"] for b in capture["bodies"] if b["shape"]}):
        sh = capture["shapes"][index-1]
        if "surface" not in sh:
            continue  # earlier captures have occupancy only
        occupied, _, _ = surface(sh)
        expected = []
        for c in occupied:
            for d, off in enumerate(((-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1))):
                if tuple(c[i]+off[i] for i in range(3)) not in occupied:
                    expected.append(c[0] | (c[1]<<8) | (c[2]<<16) | (d<<24))
                    break
        if sh["surface_count"] != len(expected) or sh["gpu_surface_count"] != len(expected) or sorted(sh["surface"]) != sorted(expected):
            errors.append(dict(shape=index, expected_count=len(expected), cpu_count=sh["surface_count"], gpu_count=sh["gpu_surface_count"]))
    return errors


if __name__ == "__main__":
    capture=json.load(open(sys.argv[1],encoding="utf-8"))
    rows=analyze(capture)
    errors = validate_surfaces(capture)
    print(json.dumps({"frame":capture["frame"],"bodies":len(capture["bodies"]),"contacts":len(rows),"rescue_contacts":sum(r["rescue"] for r in rows),"surface_errors":errors,"worst":rows[:20]},indent=2))
    raise SystemExit(1 if errors else 0)
