# Box3D Oracle

This standalone adapter proves an upstream Box3D source checkout, compiles public-only adapters, and runs the upstream test executable. It is not a Box3D product fork. The repository contains no Box3D product source and no Shallot expected output.

## Authority

The only active source authority is `https://github.com/erincatto/box3d.git`, `refs/heads/main`. A requested full commit SHA is admitted only when it is reachable from that ref. The initial target is `47d7f7cc7e091142c08d11dc7d2e493c5d34f536`.

## Public Adapter

The O2 corpus is split by public symbol across `adapter/math.c`, `geometry.c`, `distance.c`, `tree.c`, `manifold.c`, `query.c`, and `mover.c`. It covers `b3Add`, `b3Dot`, `b3ComputeCosSin`, `b3ComputeSphereAABB`, `b3ComputeCapsuleMass`, `b3PointToSegmentDistance`, `b3ShapeDistance`, `b3DynamicTree_Query`, `b3CollideSpheres`, `b3RayCastSphere`, `b3SolvePlanes`, and `b3ClipVector`. Each case has a stable logical ID, explicit input and output records, and fixed-width hexadecimal bits. f32, u32, and signed i32 records use `0x` plus eight hexadecimal digits; u64 records use sixteen digits.

Every public adapter compile uses only this repository's include directory and the official checkout's `include/box3d`. The generated depfiles reject upstream `src/` paths. The check also compiles a watched private-include mutation and requires it to fail before generation. O3 and O4 use separate immutable bundle lanes. A disposable copy receives only additive `B3_ORACLE_HOOKS` marker patches. The pristine copy remains the public/self-test lane. The v2 adapter calls actual `b3HashWorldState`, `b3IntegrateVelocitiesTask`, `b3IntegratePositionsTask`, `b3FinalizeBodiesTask`, and the contact recycle path through real upstream structs and contexts. The v3 adapter calls public `b3CollideHulls`, world contact getters backed by `b3ComputeMeshManifolds` and `b3UpdateConvexContact`, and all nine public `b3Create*Joint` producers. Scalar and host-relevant SIMD builds each carry their own executable, public-lane executable, patch, link-map, and case digests. There is no joint JSON or copied joint implementation path.

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

`reproduce --workspace <kex-root> --bundle <bundle>` creates two fresh bundles under distinct `/tmp` directories, checks their byte equality, then compares the result with the committed bundle. Generation refuses a non-empty output and never reads Shallot files or existing golds. The bundle manifest records the official URL, channel, SHA/tree, oracle commit, compiler/CMake options, schema, executable digest, membership, patch digest, link-map/`nm` provenance, and generated-file digests. The default `generate` command remains v1 for O2 reproduction; pass `--schema v2` for O3 or `--schema v3` for O4.

## Mover Lane

`generate --schema v7` records the ten cases in `scenarios/mover-v1.json`, independently of the v1–v6 populations. `mover.solve` supplies position, rotation, capsule, velocity, pogo velocity, time step and a `clipVelocity` flag. `adapter/mover_sample.h` transcribes the MIT sample's pogo ray, five-pass collide/solve/cast loop, dynamic-body push and velocity resolution; it does not apply friction, acceleration or gravity. Shapes have default mover behavior and the ignore-shape list is empty. Queries keep the sample's filters.

The lane records position, velocity, pogo velocity, ground flag, final plane count and summed applied linear impulse for each dynamic body as f32 hexadecimal bits. Ground flag and plane count use f32 encodings of their numeric values. Pass count and plane-solver iteration count are additional integer diagnostics. `sentinel-test --schema v7` changes the loop bound from five passes to one and requires the wall's position or velocity vectors to change. `reproduce` supports this lane's byte-equality check.

## Time-of-Impact Lane

`generate --schema v8` records four convex sweep inputs through public `b3TimeOfImpact`: translation, rotation, separation and initial overlap. `adapter/toi.c` emits the supplied point clouds, radii, sweeps and maximum fraction alongside the result. State and iteration counts use signed i32 hexadecimal encodings; the remaining outputs use f32 bits. The lane is independent of v1–v7, uses the pristine pinned source and the public include firewall, and supports byte-identical `reproduce`.

## Local Checks

```sh
bun run check
bun run test
```

The tests create temporary fake Git remotes and cover an unreachable commit, a dirty/tree-mismatched checkout, a failing upstream test executable, outside-marker edits, copied-body fixtures, missing provenance, exact inventory joins, and synthetic inventory additions/removals. `sentinel-test --schema v3` mutates every O4 family hook, including each public joint producer, and requires the corresponding output vector to change.

Extract the cumulative official test inventory and exact coverage join from an empty official checkout:

```sh
bun bin/oracle.ts inventory \
  --workspace <kex-root> \
  --sha 47d7f7cc7e091142c08d11dc7d2e493c5d34f536 \
  --output inventory/current.json \
  --coverage coverage/current.json \
  --diff inventory/update-diff.json
```

The command separately extracts the O6b joint-through-world half, joins it to the immutable O6a artifacts, and emits `inventory/o6b.json` and `coverage/o6b.json` when invoked with `--half o6b`. The cumulative result is ordered by the official 25-suite roster; every case ID is joined exactly once. Coverage rows are executable official-suite registrations, with no Shallot parity claim admitted at inventory time. The update diff reports exact case IDs added, removed, or changed; counts are derived from those records.
