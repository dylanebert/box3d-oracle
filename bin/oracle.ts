#!/usr/bin/env bun
import { createHash } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, rmSync, statSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join, relative, resolve } from "node:path";
import { spawnSync } from "node:child_process";

export const OFFICIAL_SOURCE_URL = "https://github.com/erincatto/box3d.git";
export const OFFICIAL_SOURCE_REF = "refs/heads/main";
export const INITIAL_SHA = "47d7f7cc7e091142c08d11dc7d2e493c5d34f536";
const FULL_SHA = /^[0-9a-f]{40}$/;

type CommandResult = {
  status: number;
  stdout: string;
  stderr: string;
};

export class OracleError extends Error {}

function command(program: string, args: string[], cwd?: string): CommandResult {
  const result = spawnSync(program, args, {
    cwd,
    encoding: "utf8",
    stdio: ["ignore", "pipe", "pipe"],
  });
  if (result.error) throw new OracleError(`${program} failed to start: ${result.error.message}`);
  return {
    status: result.status ?? 1,
    stdout: result.stdout ?? "",
    stderr: result.stderr ?? "",
  };
}

function checked(program: string, args: string[], cwd?: string): CommandResult {
  const result = command(program, args, cwd);
  if (result.status !== 0) {
    const detail = `${result.stdout}${result.stderr}`.trim();
    throw new OracleError(`${program} ${args.join(" ")} failed (exit ${result.status})${detail ? `: ${detail}` : ""}`);
  }
  return result;
}

function digest(value: string | Uint8Array): string {
  return createHash("sha256").update(value).digest("hex");
}

function requireFullSha(sha: string): void {
  if (!FULL_SHA.test(sha)) throw new OracleError(`--sha must be a 40-character lowercase full commit SHA: ${sha}`);
}

export function verifyTreeMatches(repo: string, expectedSha: string): void {
  const expectedTree = checked("git", ["rev-parse", `${expectedSha}^{tree}`], repo).stdout.trim();
  const actualTree = checked("git", ["write-tree"], repo).stdout.trim();
  if (actualTree !== expectedTree) {
    throw new OracleError(`source tree mismatch: expected ${expectedTree}, materialized ${actualTree}`);
  }
}

export function verifyClean(repo: string): void {
  const status = checked("git", ["status", "--porcelain=v1", "--untracked-files=all"], repo).stdout;
  if (status !== "") throw new OracleError(`source checkout is dirty:\n${status}`);
}

export function verifyPristine(repo: string, expectedSha: string): { commit: string; tree: string } {
  requireFullSha(expectedSha);
  const head = checked("git", ["rev-parse", "HEAD"], repo).stdout.trim();
  if (head !== expectedSha) throw new OracleError(`source checkout is not detached at requested SHA: ${head}`);
  const symbolic = command("git", ["symbolic-ref", "--quiet", "--short", "HEAD"], repo);
  if (symbolic.status === 0 && symbolic.stdout.trim() !== "") {
    throw new OracleError(`source checkout is attached to ${symbolic.stdout.trim()}`);
  }
  verifyClean(repo);
  verifyTreeMatches(repo, expectedSha);
  return { commit: head, tree: checked("git", ["rev-parse", `${expectedSha}^{tree}`], repo).stdout.trim() };
}

function ensureOfficialRemote(cache: string, sourceUrl: string): void {
  mkdirSync(dirname(cache), { recursive: true });
  if (!existsSync(join(cache, "HEAD"))) {
    checked("git", ["init", "--bare", cache]);
  }
  const configured = command("git", ["config", "--get", "remote.official.url"], cache);
  if (configured.status === 0 && configured.stdout.trim() !== sourceUrl) {
    throw new OracleError(`cache remote is not the official source: ${configured.stdout.trim()}`);
  }
  if (configured.status !== 0) checked("git", ["remote", "add", "official", sourceUrl], cache);
  checked("git", ["fetch", "--no-tags", "official", `+${OFFICIAL_SOURCE_REF}:refs/remotes/official/main`], cache);
}

export function verifyReachable(remote: string, sha: string, cache: string): { mainSha: string } {
  requireFullSha(sha);
  ensureOfficialRemote(cache, remote);
  const mainSha = checked("git", ["rev-parse", "refs/remotes/official/main"], cache).stdout.trim();
  const object = command("git", ["cat-file", "-e", `${sha}^{commit}`], cache);
  if (object.status !== 0) throw new OracleError(`requested SHA is not present in official main history: ${sha}`);
  const ancestor = command("git", ["merge-base", "--is-ancestor", sha, "refs/remotes/official/main"], cache);
  if (ancestor.status !== 0) {
    throw new OracleError(`requested SHA is unreachable from ${OFFICIAL_SOURCE_REF}: ${sha}`);
  }
  return { mainSha };
}

export function materializePristine(cache: string, sha: string): string {
  const root = join(cache, "checkouts", sha);
  rmSync(root, { recursive: true, force: true });
  mkdirSync(dirname(root), { recursive: true });
  checked("git", ["clone", "--quiet", "--no-checkout", cache, root]);
  checked("git", ["checkout", "--quiet", "--detach", sha], root);
  return root;
}

export function runUpstreamTest(executable: string, cwd: string): { exit: number; stdout: string; stderr: string } {
  const result = command(executable, [], cwd);
  return { exit: result.status, stdout: result.stdout, stderr: result.stderr };
}

function executablePath(build: string): string {
  const candidates = [join(build, "bin", "test"), join(build, "test")];
  for (const candidate of candidates) {
    if (existsSync(candidate) && statSync(candidate).isFile()) return candidate;
  }
  throw new OracleError(`upstream test executable was not produced; checked ${candidates.join(", ")}`);
}

function cmakeGenerator(): string | null {
  const ninja = command("ninja", ["--version"]);
  return ninja.status === 0 ? "Ninja" : null;
}

function buildAndTest(source: string, build: string): {
  configure: string[];
  buildCommand: string[];
  generator: string;
  configureEvidence: string;
  buildEvidence: string;
  test: { command: string[]; executable: string; executableSha256: string; exit: number; stdoutSha256: string; stderrSha256: string };
} {
  mkdirSync(build, { recursive: true });
  const generator = cmakeGenerator();
  const configure = [
    "-S", source, "-B", build,
    ...(generator ? ["-G", generator] : []),
    "-DCMAKE_BUILD_TYPE=Release",
    "-DBOX3D_DISABLE_SIMD=ON",
    "-DBOX3D_SAMPLES=OFF",
    "-DBOX3D_BENCHMARKS=OFF",
    "-DBOX3D_DOCS=OFF",
    "-DBOX3D_UNIT_TESTS=ON",
    "-DBOX3D_VALIDATE=ON",
  ];
  const configured = checked("cmake", configure);
  const buildCommand = ["--build", build, "--target", "test"];
  const built = checked("cmake", buildCommand);
  const executable = executablePath(build);
  const result = runUpstreamTest(executable, source);
  const test = {
    command: [relative(source, executable)],
    executable: relative(source, executable),
    executableSha256: digest(readFileSync(executable)),
    exit: result.exit,
    stdoutSha256: digest(result.stdout),
    stderrSha256: digest(result.stderr),
  };
  if (result.exit !== 0) {
    throw new OracleError(`unmodified upstream test failed (exit ${result.exit})${result.stdout || result.stderr ? `:\n${result.stdout}${result.stderr}` : ""}`);
  }
  return {
    configure,
    buildCommand,
    generator: generator ?? "CMake default",
    configureEvidence: digest(`${configured.stdout}\n${configured.stderr}`),
    buildEvidence: digest(`${built.stdout}\n${built.stderr}`),
    test,
  };
}

function writeReceipt(workspace: string, receipt: Record<string, unknown>, requestedSha: string): string {
  const path = join(workspace, "projects", "box3d-oracle", "receipts", "upstream-test", `${requestedSha}.json`);
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, `${JSON.stringify(receipt, null, 2)}\n`);
  return path;
}

export function upstreamTest(workspace: string, sha: string): { receiptPath: string; receipt: Record<string, unknown> } {
  requireFullSha(sha);
  const root = resolve(workspace);
  if (!existsSync(root)) throw new OracleError(`workspace does not exist: ${root}`);
  const cache = resolve(process.env.BOX3D_ORACLE_CACHE ?? join(tmpdir(), "box3d-oracle-cache"));
  const remoteCache = join(cache, "official.git");
  const reachability = verifyReachable(OFFICIAL_SOURCE_URL, sha, remoteCache);
  const source = materializePristine(remoteCache, sha);
  const before = verifyPristine(source, sha);
  const build = join(cache, "builds", sha);
  rmSync(build, { recursive: true, force: true });
  const evidence = buildAndTest(source, build);
  const after = verifyPristine(source, sha);
  if (before.tree !== after.tree) throw new OracleError("source tree changed during configure, build, or test");
  const receipt: Record<string, unknown> = {
    schema: 1,
    source: {
      url: OFFICIAL_SOURCE_URL,
      ref: OFFICIAL_SOURCE_REF,
      requestedSha: sha,
      mainSha: reachability.mainSha,
      reachable: true,
      checkout: "detached",
      commit: after.commit,
      tree: after.tree,
      clean: true,
    },
    range: {
      channel: OFFICIAL_SOURCE_REF,
      lowerBound: sha,
      upperBound: reachability.mainSha,
      relation: "lowerBound is an ancestor of upperBound",
    },
    build: {
      profile: "Release",
      scalar: true,
      options: evidence.configure.filter((value) => value.startsWith("-D")),
      generator: evidence.generator,
      configureEvidenceSha256: evidence.configureEvidence,
      buildCommand: evidence.buildCommand,
      buildEvidenceSha256: evidence.buildEvidence,
    },
    test: evidence.test,
  };
  return { receiptPath: writeReceipt(root, receipt, sha), receipt };
}

function parseArgs(args: string[]): { workspace: string; sha: string } {
  if (args[0] !== "upstream-test") throw new OracleError("usage: oracle.ts upstream-test --workspace <kex-root> --sha <full-sha>");
  let workspace = "";
  let sha = "";
  for (let index = 1; index < args.length; index += 1) {
    const flag = args[index];
    const value = args[index + 1];
    if ((flag === "--workspace" || flag === "--sha") && value) {
      if (flag === "--workspace") workspace = value;
      else sha = value;
      index += 1;
    } else {
      throw new OracleError(`unknown or incomplete argument: ${flag}`);
    }
  }
  if (!workspace || !sha) throw new OracleError("usage: oracle.ts upstream-test --workspace <kex-root> --sha <full-sha>");
  return { workspace, sha };
}

if (import.meta.main) {
  try {
    const { workspace, sha } = parseArgs(process.argv.slice(2));
    const result = upstreamTest(workspace, sha);
    console.log(`upstream-test: PASS ${sha}`);
    console.log(`receipt: ${relative(resolve(workspace), result.receiptPath)}`);
    console.log(JSON.stringify(result.receipt, null, 2));
  } catch (error) {
    console.error(`upstream-test: FAIL ${error instanceof Error ? error.message : String(error)}`);
    process.exit(1);
  }
}
