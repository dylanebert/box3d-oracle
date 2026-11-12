# Contributing

```sh
bun install
bun run check
bun run test
```

- Tests that draw randomly rounded runs need a native Verificarlo 2.7.0 build (`VERIFICARLO_PREFIX`, its install prefix) and are skipped by name without one; the Mac runs the rest, a Linux lane host all of them. Verificarlo requires glibc, so it does not build on macOS.
- Building Verificarlo on a lane host: its configure accepts LLVM up to 21.1.2 (use LLVM 20, `LLVM_CONFIG=/usr/lib/llvm-20/bin/llvm-config`), wants gcc as its compiler (`CC=gcc CXX=g++`), and `--without-prism --without-flang`; run `make install-interflop-stdlib` before `make`. Where `libc++abi-20-dev` puts a `cxxabi.h` in LLVM's include directory, give configure an `llvm-config` wrapper pointing at a copy of that directory without it. Compile the adapter with the same LLVM (`VFC_CLANG`), and the native lane with clang (`CC`).
- Every run is seeded from its key (`runSeed`, from `oracleIdentity`), never from a lane's history: a lane is one long-lived process per core that reseeds per request. A result, cached or computed, depends only on its inputs.
- The identity keys on what decides a result, never on paths, hosts or comments: change it only with behavior, and keep the test that a comment-only change leaves it.
- `B3_ORACLE_PROFILE=1` splits a lane's time per request; `stepProfile` sums the client's. A speed change records both, before and after, in its commit.
