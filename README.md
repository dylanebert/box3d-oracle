# Box3D Oracle

This standalone adapter proves an upstream Box3D source checkout, compiles public-only adapters, and runs the upstream test executable. It is not a Box3D product fork. The repository contains no Box3D product source and no Shallot expected output.

## Authority

The only active source authority is `https://github.com/erincatto/box3d.git`, `refs/heads/main`. A requested full commit SHA is admitted only when it is reachable from that ref. The initial target is `47d7f7cc7e091142c08d11dc7d2e493c5d34f536`.

## Public Adapter

The O2 corpus is split by public symbol across `adapter/math.c`, `geometry.c`, `distance.c`, `tree.c`, `manifold.c`, `query.c`, and `mover.c`. It covers `b3Add`, `b3Dot`, `b3ComputeCosSin`, `b3ComputeSphereAABB`, `b3ComputeCapsuleMass`, `b3PointToSegmentDistance`, `b3ShapeDistance`, `b3DynamicTree_Query`, `b3CollideSpheres`, `b3RayCastSphere`, `b3SolvePlanes`, and `b3ClipVector`. Each case has a stable logical ID, explicit input and output records, and fixed-width hexadecimal bits. f32, u32, and signed i32 records use `0x` plus eight hexadecimal digits; u64 records use sixteen digits.

Every public adapter compile uses only this repository's include directory and the official checkout's `include/box3d`. The generated depfiles reject upstream `src/` paths. The check also compiles a watched private-include mutation and requires it to fail before generation. Private world hashes, integration phases, contact families, recycle/finalize observations, and joint observations are recorded as deferred O3 membership, not public O2 cases.

## Command

From the Kex root:

```sh
bun projects/box3d-oracle/bin/oracle.ts upstream-test \
  --workspace "$PWD" \
  --sha 47d7f7cc7e091142c08d11dc7d2e493c5d34f536
```

The command fetches only official `main` into a cache under `/tmp`, materializes a fresh detached checkout, verifies its clean index and tree, configures the upstream CMake project in Release/scalar mode (`BOX3D_DISABLE_SIMD=ON`), builds the unmodified `test` target, and runs that executable. It emits a stable JSON receipt at `receipts/upstream-test/<sha>.json`; cache paths, timestamps, and host-specific absolute paths are not receipt inputs.

A nonzero reachability, checkout, configure, build, or upstream-test premise fails the command. The command never falls back to a fork.

Generate an immutable bundle into an empty directory:

```sh
bun projects/box3d-oracle/bin/oracle.ts generate \\
  --workspace "$PWD" \\
  --sha 47d7f7cc7e091142c08d11dc7d2e493c5d34f536 \\
  --output /tmp/box3d-bundle
```

`reproduce --workspace <kex-root> --bundle <bundle>` creates two fresh bundles under distinct `/tmp` directories, checks their byte equality, then compares the result with the committed bundle. Generation refuses a non-empty output and never reads Shallot files or existing golds. The bundle manifest records the official URL, channel, SHA/tree, oracle commit, compiler/CMake options, schema, executable digest, membership, and generated-file digests.

## Local Checks

```sh
bun run check
bun run test
```

The tests create temporary fake Git remotes and cover an unreachable commit, a dirty/tree-mismatched checkout, and a failing upstream test executable.
