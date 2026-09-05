"""Report project-local shader include dependencies (conservative across #if branches)."""
from __future__ import annotations
import argparse
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SEARCH = [ROOT / p for p in ("include", "src/shaders", "src/shaders/path_tracing", "src/shaders/simulation")]

def strip_comments(source):
    return re.sub(r"//[^\n]*|/\*[\s\S]*?\*/", "", source)

def resolve(name, parent=None):
    candidates = ([parent / name] if parent else []) + [p / name for p in SEARCH]
    return next((p.resolve() for p in candidates if p.is_file()), None)

def registrations():
    source = strip_comments((ROOT / "include/defines.hpp").read_text(encoding="utf-8"))
    constants = dict(re.findall(r'(\w+)\s*=\s*"([^"\n]+)"', source))
    pattern = r'\.source\s*=\s*daxa::ShaderFile\{([^}]+)\},\s*\.compile_options\s*=\s*\{\s*\.entry_point\s*=\s*([^,\n}]+)'
    result = []
    for name, entry in re.findall(pattern, source):
        name, entry = name.strip(), entry.strip()
        filename = name.strip('"') if name.startswith('"') else constants[name]
        path = resolve(filename)
        if path is None:
            raise ValueError(f"Missing shader source: {filename}")
        result.append((constants.get(entry, entry.strip('"')), path))
    return result

def dependencies(path, visited=None):
    visited = set() if visited is None else visited
    if path in visited:
        return visited
    visited.add(path)
    source = strip_comments(path.read_text(encoding="utf-8"))
    for name in re.findall(r'^\s*#include\s*[<"]([^>"]+)[>"]', source, re.M):
        child = resolve(name, path.parent)
        # SDK headers (Daxa, GLM, standard library) are outside this project-local report.
        if child is not None:
            dependencies(child, visited)
    return visited

def consumers():
    result = {}
    for entry, path in registrations():
        for dependency in dependencies(path):
            name = dependency.relative_to(ROOT).as_posix()
            result.setdefault(name, []).append(entry)
    return {name: sorted(set(entries)) for name, entries in sorted(result.items())}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    graph = consumers()
    if args.json:
        print(json.dumps(graph, indent=2))
    else:
        for path, entries in graph.items():
            print(f"{len(entries):3}  {path}")
