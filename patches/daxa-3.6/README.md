# Daxa 3.6 local patches

`lib/Daxa` is gitignored (cloned by `cmake/deps.cmake`). These patches capture
the local modifications made to the Daxa 3.6 checkout during the migration, so
the build is reproducible.

Base: Daxa tag `3.6` (deps.cmake checks out `3.6`).

To reconstruct the migration's Daxa state:

```sh
cd lib/Daxa
git checkout 3.6
git apply ../../patches/daxa-3.6/local-changes.patch
```

Contents of `local-changes.patch`:
- `cmake/deps.cmake`: bump bundled Slang `2025.11` -> `2026.10`
  (Slang fix for issue #3870 — coherent loads/stores through pointers).
- `include/daxa/utils/task_graph.hpp`: fix `TInlineTask` move/copy assignment
  operators and a `reinterpret_cast` in `uses()` so the inline-task-with-head
  API used by this project compiles.
