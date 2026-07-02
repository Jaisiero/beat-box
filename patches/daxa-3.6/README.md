# Daxa 3.6 local patches

`lib/Daxa` is gitignored (cloned by `cmake/deps.cmake`). This patch captures
the local modifications made to the Daxa 3.6 checkout, so the build is
reproducible — **the project does not compile against a clean 3.6 checkout**.

Base: Daxa tag `3.6` (deps.cmake checks out `3.6`).

**The patch is applied automatically at configure time** (idempotently — see the
patch block in `cmake/deps.cmake`). To apply it by hand instead:

```sh
cd lib/Daxa
git checkout 3.6
git apply ../../patches/daxa-3.6/local-changes.patch
```

If you modify `lib/Daxa` further, regenerate the patch so fresh clones pick it up:

```sh
git -C lib/Daxa diff > patches/daxa-3.6/local-changes.patch
```

Contents of `local-changes.patch`:
- `cmake/deps.cmake`: bump bundled Slang `2025.11` -> `2026.10`
  (Slang fix for issue #3870 — coherent loads/stores through pointers).
- `include/daxa/utils/task_graph.hpp`: fix `TInlineTask` move/copy assignment
  operators and a `reinterpret_cast` in `uses()` so the inline-task-with-head
  API used by this project compiles.
- `src/impl_core.hpp`: add the missing newline at end of file (warning fix).
