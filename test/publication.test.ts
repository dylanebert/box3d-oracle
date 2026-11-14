import { expect, test } from "bun:test";
import { mkdirSync, mkdtempSync, readFileSync, rmSync, symlinkSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { oracleCacheDirectory } from "../bin/oracle";

test("two processes can build the same cold lane in one cache", async () => {
  const root = mkdtempSync(join(tmpdir(), "oracle-publication-"));
  try {
    // Reuse only the upstream object store, never built artifacts.
    symlinkSync(join(oracleCacheDirectory(), "official.git"), join(root, "official.git"));
    mkdirSync(join(root, "step-builds"));
    const script = `import { buildStepOracle } from ${JSON.stringify(join(import.meta.dir, "../bin/oracle.ts"))}; console.log(buildStepOracle("336263320469e5e9febfe18697ab562488f7e97a"));`;
    const children = Array.from({ length: 2 }, () => Bun.spawn(["bun", "--eval", script], {
      env: { ...process.env, BOX3D_ORACLE_CACHE: root, GIT_TRACE: join(root, "clone.trace") }, stdout: "pipe", stderr: "pipe",
    }));
    const results = await Promise.all(children.map(async (child) => ({
      out: await new Response(child.stdout).text(), err: await new Response(child.stderr).text(), exit: await child.exited,
    })));
    for (const result of results) expect(result.exit, result.err).toBe(0);
    expect(results[0].out).toBe(results[1].out);
    // Transport only: neither hardlink nor copy traversal of a live object store.
    const trace = readFileSync(join(root, "clone.trace"), "utf8").split("\n");
    const clones = trace.filter((line) => line.includes("built-in: git clone "));
    expect(clones.length).toBeGreaterThanOrEqual(2);
    for (const clone of clones) expect(clone).toContain("--no-local");
    const checkouts = trace.filter((line) => line.includes("built-in: git fetch --quiet --no-tags ") && line.endsWith("336263320469e5e9febfe18697ab562488f7e97a"));
    expect(checkouts.length).toBeGreaterThanOrEqual(2);
  } finally { rmSync(root, { recursive: true, force: true }); }
}, 120_000);
