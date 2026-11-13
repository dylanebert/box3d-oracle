import { expect, test } from "bun:test";
import { cpSync, mkdirSync, mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { Snapshot } from "../bin/snapshot";
import { stepLaneEnvironment } from "../bin/oracle";
import { tmpdir } from "node:os";
import { join } from "node:path";

test("snapshots key on contents and reject changed executables and backend trees", () => {
  const root = mkdtempSync(join(tmpdir(), "oracle-files-"));
  try {
    mkdirSync(join(root, "lib"));
    const executable = join(root, "clang");
    const backend = join(root, "lib/backend.so");
    writeFileSync(executable, "compiler one");
    writeFileSync(backend, "backend one");
    const first = new Snapshot();
    const compilerKey = first.file(executable);
    const backendKey = first.tree(join(root, "lib"));
    first.assert();
    writeFileSync(backend, "backend two");
    expect(() => first.assert()).toThrow("oracle implementation changed");
    const second = new Snapshot();
    expect(second.tree(join(root, "lib"))).not.toBe(backendKey);
    expect(second.file(executable)).toBe(compilerKey);
    writeFileSync(executable, "compiler two");
    expect(() => second.assert()).toThrow("oracle implementation changed");
    expect(new Snapshot().file(executable)).not.toBe(compilerKey);
  } finally { rmSync(root, { recursive: true, force: true }); }
});

test("lane children receive only the captured allowlist", async () => {
  process.env.ORACLE_AMBIENT_SENTINEL = "must-not-be-inherited";
  try {
    const env = stepLaneEnvironment();
    const child = Bun.spawn(["/usr/bin/env"], { env: { ...env }, stdout: "pipe", stderr: "pipe" });
    const lines = (await new Response(child.stdout).text()).trim().split("\n").sort();
    expect(await child.exited).toBe(0);
    expect(lines).toEqual(Object.entries(env).map(([key, value]) => `${key}=${value}`).sort());
    expect(lines.some((line) => line.startsWith("ORACLE_AMBIENT_SENTINEL="))).toBe(false);
  } finally { delete process.env.ORACLE_AMBIENT_SENTINEL; }
});

test("loader controls changed between identity capture and lane startup fail closed", async () => {
  const script = `
    import { oracleIdentity, openStepOracle } from ${JSON.stringify(join(import.meta.dir, "../bin/step.ts"))};
    const scene = { world: { gravity: [0,-10,0], timeStep: 1/60, subStepCount: 4 }, bodies: [], shapes: [], joints: [] };
    const sha = "336263320469e5e9febfe18697ab562488f7e97a";
    oracleIdentity(sha, scene, { arithmetic: "random", sqrt: "random" });
    process.env.DYLD_INSERT_LIBRARIES = "/not-an-oracle-input.dylib";
    try { const lane = await openStepOracle(sha, scene, undefined, {lanes: 0}); lane.close(); }
    catch (e) { if (String(e).includes("loader control")) process.exit(0); throw e; }
    throw new Error("accepted changed loader environment");
  `;
  const child = Bun.spawn(["bun", "--eval", script], { stdout: "pipe", stderr: "pipe" });
  const err = await new Response(child.stderr).text();
  expect(await child.exited, err).toBe(0);
}, 120_000);

test("runtime dependencies are guarded before lane startup and keyed across processes", async () => {
  const root = mkdtempSync(join(tmpdir(), "oracle-runtime-"));
  try {
    const library = join(root, process.platform === "darwin" ? "dependency.dylib" : "dependency.so");
    const executable = join(root, "lane");
    writeFileSync(join(root, "dependency.c"), "int dependency(void) { return 7; }\n");
    writeFileSync(join(root, "lane.c"), "int dependency(void); int main(void) { return dependency(); }\n");
    const compile = (args: string[]) => {
      const result = Bun.spawnSync([process.env.CC ?? "cc", ...args], { stdout: "pipe", stderr: "pipe" });
      expect(result.exitCode, result.stderr.toString()).toBe(0);
    };
    compile([...(process.platform === "darwin" ? ["-dynamiclib", "-install_name", library] : ["-shared", "-fPIC"]), join(root, "dependency.c"), "-o", library]);
    compile([join(root, "lane.c"), library, "-o", executable]);
    const imports = `import {stepExecutableIdentity, assertStepImplementation} from ${JSON.stringify(join(import.meta.dir, "../bin/oracle.ts"))};`;
    const key = async () => {
      const child = Bun.spawn(["bun", "--eval", `${imports} console.log(stepExecutableIdentity(${JSON.stringify(executable)}));`], { stdout: "pipe", stderr: "pipe" });
      const out = await new Response(child.stdout).text();
      expect(await child.exited, await new Response(child.stderr).text()).toBe(0);
      return out;
    };
    const before = await key();
    const child = Bun.spawn(["bun", "--eval", `${imports}
      import {appendFileSync} from "node:fs";
      stepExecutableIdentity(${JSON.stringify(executable)});
      appendFileSync(${JSON.stringify(library)}, "\\0");
      try { assertStepImplementation(); }
      catch (e) { if (String(e).includes("oracle implementation changed")) process.exit(0); throw e; }
      throw new Error("accepted a changed runtime dependency before startup");
    `], { stdout: "pipe", stderr: "pipe" });
    const err = await new Response(child.stderr).text();
    expect(await child.exited, err).toBe(0);
    expect(await key()).not.toBe(before);
  } finally { rmSync(root, { recursive: true, force: true }); }
}, 120_000);

// Isolate mutations from this test process's imported modules and the working tree.
test("an oracle refuses source synced after its modules loaded", async () => {
  const root = mkdtempSync(join(tmpdir(), "oracle-snapshot-"));
  try {
    for (const name of ["bin", "adapter", "hooks"]) cpSync(join(import.meta.dir, "..", name), join(root, name), { recursive: true });
    const script = `
      import { appendFileSync } from "node:fs";
      import { oracleIdentity } from ${JSON.stringify(join(root, "bin/step.ts"))};
      const scene = { world: { gravity: [0,-10,0], timeStep: 1/60, subStepCount: 4 }, bodies: [], shapes: [], joints: [] };
      appendFileSync(${JSON.stringify(join(root, "bin/stats.ts"))}, "\\nexport const changedAfterImport = 1;\\n");
      try { oracleIdentity("336263320469e5e9febfe18697ab562488f7e97a", scene, { arithmetic: "random", sqrt: "random" }); }
      catch (e) { if (String(e).includes("oracle implementation changed")) process.exit(0); throw e; }
      throw new Error("accepted a changed implementation");
    `;
    const child = Bun.spawn(["bun", "--eval", script], { stdout: "pipe", stderr: "pipe" });
    const err = await new Response(child.stderr).text();
    expect(await child.exited, err).toBe(0);
  } finally { rmSync(root, { recursive: true, force: true }); }
});
