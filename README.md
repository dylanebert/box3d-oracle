# Box3D Oracle

This standalone adapter proves an upstream Box3D source checkout and runs the upstream test executable. It is not a Box3D product fork. The repository contains no Box3D product source and no Shallot expected output.

## Authority

The only active source authority is `https://github.com/erincatto/box3d.git`, `refs/heads/main`. A requested full commit SHA is admitted only when it is reachable from that ref. The initial target is `47d7f7cc7e091142c08d11dc7d2e493c5d34f536`.

## Command

From the Kex root:

```sh
bun projects/box3d-oracle/bin/oracle.ts upstream-test \
  --workspace "$PWD" \
  --sha 47d7f7cc7e091142c08d11dc7d2e493c5d34f536
```

The command fetches only official `main` into a cache under `/tmp`, materializes a fresh detached checkout, verifies its clean index and tree, configures the upstream CMake project in Release/scalar mode (`BOX3D_DISABLE_SIMD=ON`), builds the unmodified `test` target, and runs that executable. It emits a stable JSON receipt at `receipts/upstream-test/<sha>.json`; cache paths, timestamps, and host-specific absolute paths are not receipt inputs.

A nonzero reachability, checkout, configure, build, or upstream-test premise fails the command. The command never falls back to a fork.

## Local Checks

```sh
bun run check
bun run test
```

The tests create temporary fake Git remotes and cover an unreachable commit, a dirty/tree-mismatched checkout, and a failing upstream test executable.
