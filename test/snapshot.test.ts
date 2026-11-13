import { expect, test } from "bun:test";
import { cpSync, mkdirSync, mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { Snapshot } from "../bin/snapshot";
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
