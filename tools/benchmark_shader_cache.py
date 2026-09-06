"""Measure actual Daxa SPIR-V cache writes after harmless edits to runtime shader copies.

Close Beat Box first. This tool changes only a comment in a runtime copy, restores
its bytes/timestamps in finally, and uses BB_COMPILE_ONLY. Do not point it at sources.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

PROBES = {
    "src/shaders/simulation/collision_detection.slang": 1,
    "src/shaders/simulation/contact_history.slang": 1,
    "src/shaders/simulation/solvers.slang": 9,
    "src/shaders/simulation/passes/avbd_primal.slang": 1,
    "src/shaders/simulation/voxel_fracture.slang": 4,
    "src/shaders/path_tracing/lighting.slang": 7,
}

def snapshot(directory):
    return {str(p.relative_to(directory)): (p.stat().st_mtime_ns, p.stat().st_size)
            for p in directory.rglob("*") if p.is_file()}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    runtime = args.runtime.resolve()
    source = Path(__file__).resolve().parents[1]
    if runtime == source or not (runtime / "beat-box").is_file():
        parser.error("runtime must contain the built Linux beat-box executable, not the source tree")
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    cache = runtime / "spirv_cache"
    env = dict(os.environ, BB_COMPILE_ONLY="1", BB_SCENE="9")
    results = []

    def run(label):
        before = snapshot(cache)
        start = time.perf_counter()
        with (output / (label + ".log")).open("wb") as log:
            completed = subprocess.run([str(runtime / "beat-box")], cwd=runtime, env=env,
                                       stdout=log, stderr=subprocess.STDOUT, timeout=600)
        elapsed = time.perf_counter() - start
        text = (output / (label + ".log")).read_text(errors="replace")
        if completed.returncode or "shader cache warmed; exiting" not in text or "CRITICAL ERROR" in text:
            raise RuntimeError("Compilation failed; inspect " + str(output / (label + ".log")))
        after = snapshot(cache)
        changed = sorted(name for name, stamp in after.items() if before.get(name) != stamp)
        return {"case": label, "seconds": round(elapsed, 3), "cache_writes": len(changed), "files": changed}

    # First run may warm a previously empty cache; it is not the warm-start measurement.
    run("warmup")
    results.append(run("warm"))
    if results[-1]["cache_writes"]:
        raise RuntimeError("Unmodified warm launch rewrites SPIR-V; benchmark is not isolated")
    for index, (relative, expected) in enumerate(PROBES.items()):
        path = runtime / relative
        if path.resolve() == (source / relative).resolve():
            raise RuntimeError("Refusing to edit a source file through a symlink")
        original = path.read_bytes()
        stamp = path.stat()
        try:
            path.write_bytes(original + b"\n// Shader dependency benchmark: no executable change.\n")
            result = run("probe-" + str(index))
            result.update(source=relative, expected_cache_writes=expected)
            results.append(result)
            print(json.dumps(result), flush=True)
        finally:
            path.write_bytes(original)
            os.utime(path, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
    results.append(run("warm-after"))
    (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    failures = [r for r in results if r["cache_writes"] != r.get("expected_cache_writes", 0)]
    if failures:
        raise RuntimeError("Unexpected invalidation counts: " + repr(failures))

if __name__ == "__main__":
    main()
