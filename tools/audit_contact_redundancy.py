"""Audit captured contact sets without changing simulation or warm-start state.
Usage: python tools/audit_contact_redundancy.py capture.txt.contacts.json [...]
Geometric duplicates are candidates for investigation, not proof that deleting
rows preserves a finite-iteration penalty solver's stiffness or convergence.
"""
import argparse
import collections
import itertools
import json
import math


def audit(capture):
    pairs = collections.defaultdict(list)
    pair_keys = collections.Counter()
    rows = collections.Counter()
    duplicate_points = 0
    contacts = 0
    for manifold in capture["manifolds"]:
        a, b = manifold["a"], manifold["b"]
        pair = tuple(sorted((a, b)))
        sign = 1 if a <= b else -1
        normal = tuple(sign * x for x in manifold["normal"])
        geometry = tuple(sorted(tuple(c["position"]) + (c["penetration"],)
                                for c in manifold["contacts"]))
        pair_keys[pair + (manifold["key"],)] += 1
        pairs[pair].append((normal, geometry, manifold["key"]))
        seen = set()
        for point in geometry:
            if point in seen:
                duplicate_points += 1
            seen.add(point)
            rows[pair + (normal, point)] += 1
        contacts += len(geometry)
    duplicate_manifolds = 0
    parallel = 0
    examples = []
    for pair, manifolds in pairs.items():
        signatures = collections.Counter((n, g) for n, g, _ in manifolds)
        duplicate_manifolds += sum(n - 1 for n in signatures.values())
        for a, b in itertools.combinations(manifolds, 2):
            dot = sum(x * y for x, y in zip(a[0], b[0]))
            length = math.sqrt(sum(x*x for x in a[0]) * sum(x*x for x in b[0]))
            if length and dot / length > math.cos(math.radians(1)):
                parallel += 1
                shared = set(a[1]) & set(b[1])
                if shared and len(examples) < 10:
                    examples.append(dict(ids=[capture["bodies"][i]["id"] for i in pair],
                                         keys=[a[2], b[2]], shared_points=len(shared)))
    return dict(frame=capture["frame"], solver=capture["solver"],
                bodies=len(capture["bodies"]), pairs=len(pairs),
                manifolds=len(capture["manifolds"]), contacts=contacts,
                contacts_per_manifold=dict(sorted(collections.Counter(len(m["contacts"]) for m in capture["manifolds"]).items())),
                empty_manifolds=sum(not m["contacts"] for m in capture["manifolds"]),
                rescue_contacts=sum(c.get("features") == [0xffffff] * 4 for m in capture["manifolds"] for c in m["contacts"]),
                manifolds_per_pair=dict(sorted(collections.Counter(map(len,pairs.values())).items())),
                duplicate_pair_keys=sum(n-1 for n in pair_keys.values()),
                exact_duplicate_manifolds=duplicate_manifolds,
                exact_duplicate_rows=sum(n-1 for n in rows.values()),
                exact_duplicate_points_within_manifold=duplicate_points,
                parallel_manifold_pairs_1_degree=parallel,
                duplicate_examples=examples)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", nargs="+")
    args = parser.parse_args()
    for name in args.captures:
        with open(name, encoding="utf-8") as stream:
            print(json.dumps(dict(path=name, **audit(json.load(stream))), sort_keys=True))
