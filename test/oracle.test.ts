import { chmodSync, existsSync, mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { spawnSync } from "node:child_process";
import { expect, test } from "bun:test";
import { rejectCopiedBody, verifyDeclaredSymbols, verifyPatchSet, verifyPristine, verifyReachable, verifyTreeMatches, runUpstreamTest } from "../bin/oracle";
import { DECLARED_SYMBOLS, PATCHES } from "../hooks/patches";

function git(cwd: string, ...args: string[]): string {
  const result = spawnSync("git", args, {
    cwd,
    encoding: "utf8",
    env: { ...process.env, GIT_TERMINAL_PROMPT: "0" },
  });
  if (result.status !== 0) throw new Error(`${args.join(" ")}: ${result.stderr || result.stdout || `status=${result.status} signal=${result.signal}`}`);
  return result.stdout.trim();
}

function fakeRemote(): { root: string; remote: string; first: string; unreachable: string } {
  const root = mkdtempSync(join(tmpdir(), "box3d-oracle-fake-"));
  const seed = join(root, "seed");
  const remote = join(root, "remote.git");
  mkdirSync(seed);
  git(seed, "init", "-q", "-b", "main");
  git(seed, "config", "user.name", "oracle-test");
  git(seed, "config", "user.email", "oracle-test@example.invalid");
  writeFileSync(join(seed, "source.txt"), "first\n");
  git(seed, "add", "source.txt");
  git(seed, "commit", "-q", "-m", "first");
  const first = git(seed, "rev-parse", "HEAD");
  git(root, "clone", "--bare", "-q", seed, remote);
  const unreachable = "f".repeat(40);
  return { root, remote, first, unreachable };
}

test("source reachability accepts only the fake remote main ancestry", () => {
  const fixture = fakeRemote();
  try {
    const result = verifyReachable(fixture.remote, fixture.first, join(fixture.root, "cache.git"));
    expect(result.mainSha).toBe(fixture.first);
    expect(() => verifyReachable(fixture.remote, fixture.unreachable, join(fixture.root, "cache.git"))).toThrow(/not present|unreachable from/);
  } finally {
    rmSync(fixture.root, { recursive: true, force: true });
  }
});

test("pristine verification rejects dirty materializations and tree mismatches", () => {
  const root = mkdtempSync(join(tmpdir(), "box3d-oracle-pristine-"));
  try {
    git(root, "init", "-q", "-b", "main");
    git(root, "config", "user.name", "oracle-test");
    git(root, "config", "user.email", "oracle-test@example.invalid");
    writeFileSync(join(root, "source.txt"), "clean\n");
    git(root, "add", "source.txt");
    git(root, "commit", "-q", "-m", "source");
    const sha = git(root, "rev-parse", "HEAD");
    writeFileSync(join(root, ".git", "HEAD"), `${sha}\n`);
    expect(() => verifyPristine(root, sha)).not.toThrow();
    writeFileSync(join(root, "source.txt"), "dirty\n");
    expect(() => verifyPristine(root, sha)).toThrow(/dirty/);
    git(root, "add", "source.txt");
    expect(() => verifyTreeMatches(root, sha)).toThrow(/tree mismatch/);
  } finally {
    rmSync(root, { recursive: true, force: true });
  }
});

test("marker patch verifier rejects an outside-marker edit", () => {
  const fixture = PATCHES[0];
  const original = fixture.anchor + "tail\\n";
  const patched = fixture.addition + "changed\\n";
  expect(() => verifyPatchSet(original, patched, [fixture])).toThrow(/outside named/);
});

test("token detector rejects a substantive copied upstream body fixture", () => {
  const body = "static void copied(void) { int i = 0; i += 1; i += 2; i += 3; i += 4; i += 5; i += 6; i += 7; i += 8; i += 9; return; }";
  expect(() => rejectCopiedBody([body], [body])).toThrow(/copied upstream/);
});

test("declared provenance rejects a missing or wrong symbol", () => {
  const source = mkdtempSync(join(tmpdir(), "box3d-oracle-provenance-"));
  try {
    mkdirSync(join(source, "src"), { recursive: true });
    for (const declared of DECLARED_SYMBOLS) {
      const path = join(source, declared.file);
      writeFileSync(path, `${existsSync(path) ? readFileSync(path, "utf8") : ""} ${declared.symbol}`);
    }
    expect(() => verifyDeclaredSymbols(source, "b3HashWorldState")).toThrow(/missing from nm/);
    expect(() => verifyDeclaredSymbols(source, "b3HashWorldState b3WrongBody", [{ symbol: "b3WrongBody", file: "src/recording.c", vector: "whitebox.wrong.v2" }])).toThrow(/missing from source/);
  } finally {
    rmSync(source, { recursive: true, force: true });
  }
});

test("scenario membership is an exact 53-name join with owned setup and observation schedules", () => {
  const corpus = JSON.parse(readFileSync(join(import.meta.dir, "..", "scenarios", "v1.json"), "utf8")) as { scenarios: Array<{ id: string; name: string; setup?: unknown; actions?: unknown; observations?: unknown; whiteBoxObservations?: unknown }> };
  expect(corpus.scenarios).toHaveLength(53);
  expect(new Set(corpus.scenarios.map((item) => item.id)).size).toBe(53);
  expect(new Set(corpus.scenarios.map((item) => item.name)).size).toBe(53);
  for (const item of corpus.scenarios) {
    expect(item.setup).toBeDefined();
    expect(item.actions).toBeDefined();
    expect(item.observations).toBeDefined();
    expect(item.whiteBoxObservations).toBeDefined();
  }
});

test("scenario mutation is named at the shared gravity input and is not an expected-value edit", () => {
  const source = readFileSync(join(import.meta.dir, "..", "scenarios", "v1.json"), "utf8");
  expect(source).toContain('"gravity"');
  expect(source).toContain('"whiteBoxObservations"');
  expect(source).not.toContain('"expected"');
});

test("upstream test evidence preserves a failed executable premise", () => {
  const root = mkdtempSync(join(tmpdir(), "box3d-oracle-test-"));
  try {
    const executable = join(root, "test-executable");
    writeFileSync(executable, "#!/bin/sh\nprintf 'upstream failure\\n'\nexit 7\n");
    chmodSync(executable, 0o755);
    const result = runUpstreamTest(executable, root);
    expect(result.exit).toBe(7);
    expect(result.stdout).toContain("upstream failure");
  } finally {
    rmSync(root, { recursive: true, force: true });
  }
});
