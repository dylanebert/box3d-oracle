#!/usr/bin/env bun
import { createHash } from "node:crypto";
import { existsSync, mkdirSync, mkdtempSync, readdirSync, readFileSync, rmSync, statSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { basename, dirname, join, relative, resolve } from "node:path";
import { spawnSync } from "node:child_process";
import { DECLARED_SYMBOLS, O4_DECLARED_SYMBOLS, PATCHES, type OraclePatch } from "../hooks/patches";

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
    maxBuffer: 128 * 1024 * 1024,
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

function applyPatch(text: string, patch: OraclePatch): string {
  const occurrences = text.split(patch.anchor).length - 1;
  if (occurrences !== 1) throw new OracleError(`patch ${patch.marker} requires one anchor, found ${occurrences}`);
  return text.replace(patch.anchor, patch.addition);
}

export function verifyPatchSet(original: string, patched: string, patches: OraclePatch[]): void {
  let expected = original;
  for (const patch of patches) expected = applyPatch(expected, patch);
  if (expected !== patched) throw new OracleError("patch changed text outside named B3_ORACLE_HOOKS markers");
}

function cTokens(text: string): string[] {
  return text
    .replace(/\/\*[\s\S]*?\*\//g, " ")
    .replace(/\/\/.*$/gm, " ")
    .replace(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'/g, " LITERAL ")
    .match(/[A-Za-z_][A-Za-z0-9_]*|0[xX][0-9A-Fa-f]+|[0-9]+|==|!=|<=|>=|&&|\|\||\+\+|--|->|[{}()[\].,;:*+\-/=%<>!?&|^~]/g) ?? [];
}

export function rejectCopiedBody(additions: string[], upstreamSources: string[], minimumRun = 16): void {
  const upstreamRuns = new Set<string>();
  for (const source of upstreamSources) {
    const tokens = cTokens(source);
    for (let i = 0; i + minimumRun <= tokens.length; i += 1) upstreamRuns.add(tokens.slice(i, i + minimumRun).join(" "));
  }
  for (const addition of additions) {
    const tokens = cTokens(addition);
    for (let i = 0; i + minimumRun <= tokens.length; i += 1) {
      if (upstreamRuns.has(tokens.slice(i, i + minimumRun).join(" "))) throw new OracleError(`additive hook contains a copied upstream implementation run: ${addition.slice(0, 80)}`);
    }
  }
}

function hookDigest(): string {
  return digest(`${JSON.stringify(PATCHES)}\n${readFileSync(join(import.meta.dir, "..", "hooks", "oracle_hooks.h"), "utf8")}`);
}

function patchOfficialSource(pristine: string, patched: string): { digest: string } {
  mkdirSync(dirname(patched), { recursive: true });
  checked("cp", ["-R", pristine, patched]);
  const byFile = new Map<string, OraclePatch[]>();
  for (const patch of PATCHES) byFile.set(patch.file, [...(byFile.get(patch.file) ?? []), patch]);
  const additions: string[] = [];
  for (const [file, patches] of byFile) {
    const originalPath = join(pristine, file);
    const patchedPath = join(patched, file);
    const original = readFileSync(originalPath, "utf8");
    let text = original;
    for (const patch of patches) {
      text = applyPatch(text, patch);
      additions.push(`${patch.marker}\n${patch.addition.slice(patch.anchor.length)}`);
    }
    writeFileSync(patchedPath, text);
    verifyPatchSet(original, text, patches);
  }
  writeFileSync(join(patched, "src", "oracle_hooks.h"), readFileSync(join(import.meta.dir, "..", "hooks", "oracle_hooks.h")));
  const upstreamSources = readdirSync(join(pristine, "src")).filter((name) => name.endsWith(".c")).map((name) => readFileSync(join(pristine, "src", name), "utf8"));
  rejectCopiedBody(additions, upstreamSources, 32);
  return { digest: hookDigest() };
}

export function verifyDeclaredSymbols(source: string, nmEvidence: string, declarations = DECLARED_SYMBOLS): void {
  for (const declared of declarations) {
    if (!readFileSync(join(source, declared.file), "utf8").includes(declared.symbol)) throw new OracleError(`declared upstream symbol is missing from source: ${declared.symbol}`);
    if (!nmEvidence.includes(declared.symbol)) throw new OracleError(`declared upstream symbol is missing from nm/link evidence: ${declared.symbol}`);
  }
}

export function verifyTreeMatches(repo: string, expectedSha: string): void {
  const expectedTree = checked("git", ["rev-parse", `${expectedSha}^{tree}`], repo).stdout.trim();
  const actualTree = checked("git", ["write-tree"], repo).stdout.trim();
  if (actualTree !== expectedTree) {
    throw new OracleError(`source tree mismatch: expected ${expectedTree}, materialized ${actualTree}`);
  }
}

export function verifyClean(repo: string): void {
  const status = checked("git", ["status", "--porcelain=v1", "--untracked-files=all", "--ignored=matching"], repo).stdout;
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

function stableUpstreamOutput(output: string): string {
  return output
    .replace(/^set: count = .*$/gm, "set: count = <timing>")
    .replace(/^Test duration = .*$/gm, "Test duration = <timing>");
}

function buildAndTest(source: string, build: string): {
  configure: string[];
  buildCommand: string[];
  generator: string;
  configureExit: number;
  configureEvidence: string;
  buildExit: number;
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
  const result = runUpstreamTest(executable, build);
  const test = {
    command: [relative(build, executable)],
    executable: relative(build, executable),
    executableSha256: digest(readFileSync(executable)),
    exit: result.exit,
    stdoutSha256: digest(stableUpstreamOutput(result.stdout)),
    stderrSha256: digest(result.stderr),
    stdoutNormalization: "set and total-duration timing fields replaced with <timing>",
  };
  if (result.exit !== 0) {
    throw new OracleError(`unmodified upstream test failed (exit ${result.exit})${result.stdout || result.stderr ? `:\n${result.stdout}${result.stderr}` : ""}`);
  }
  return {
    configure,
    buildCommand,
    generator: generator ?? "CMake default",
    configureExit: configured.status,
    configureEvidence: digest(configure.join("\0")),
    buildExit: built.status,
    buildEvidence: digest(buildCommand.join("\0")),
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
      configureExit: evidence.configureExit,
      configureCommandSha256: evidence.configureEvidence,
      buildCommand: ["cmake", "--build", `<build-cache>/${sha}`, "--target", "test"],
      buildExit: evidence.buildExit,
      buildCommandSha256: evidence.buildEvidence,
    },
    test: {
      ...evidence.test,
      command: [`<build-cache>/${sha}/${evidence.test.command[0]}`],
      executable: `<build-cache>/${sha}/${evidence.test.executable}`,
    },
  };
  return { receiptPath: writeReceipt(root, receipt, sha), receipt };
}

type BundleFile = { name: string; data: string };
const PUBLIC_SYMBOLS = {
  math: ["b3Add", "b3Dot", "b3ComputeCosSin"],
  geometry: ["b3ComputeSphereAABB", "b3ComputeCapsuleMass"],
  distance: ["b3PointToSegmentDistance", "b3ShapeDistance"],
  tree: ["b3DynamicTree_Query"],
  manifold: ["b3CollideSpheres"],
  query: ["b3RayCastSphere"],
  mover: ["b3SolvePlanes", "b3ClipVector"],
} as const;
const DEFERRED_PRIVATE_CASES = [
  "world-hash", "integrate", "finalize", "recycle", "convex-manifold", "mesh-contact", "convex-contact", "joint",
] as const;

function sha256File(path: string): string { return digest(readFileSync(path)); }
function compiler(): string { return process.env.CC ?? "cc"; }
function compilerEvidence(): { executable: string; version: string; sha256: string } {
  const executable = compiler();
  const version = checked(executable, ["--version"]).stdout.split("\n")[0];
  return { executable, version, sha256: digest(version) };
}
function ensureEmptyDirectory(path: string): void {
  mkdirSync(path, { recursive: true });
  if (readdirSync(path).length !== 0) throw new OracleError(`bundle output must start empty: ${path}`);
}
function publicSources(): string[] {
  return ["distance.c", "geometry.c", "main.c", "manifold.c", "math.c", "mover.c", "query.c", "tree.c", "writer.c"];
}
function assertPublicDepfiles(build: string, source: string): void {
  const depfiles = readdirSync(build).filter((name) => name.endsWith(".d")).sort();
  if (depfiles.length !== publicSources().length) throw new OracleError(`expected one depfile per public adapter, found ${depfiles.length}`);
  for (const name of depfiles) {
    const text = readFileSync(join(build, name), "utf8");
    if (text.includes(`${source}/src/`) || text.split(/\\s+/).some((part) => part === "src/" || part.includes("/src/"))) {
      throw new OracleError(`public adapter depfile imports upstream private src/: ${name}`);
    }
  }
}
function publicFirewall(source: string, build: string): void {
  const mutation = join(build, "negative-private-include.c");
  writeFileSync(mutation, '#include "src/private-oracle-mutation.h"\nint main(void) { return 0; }\n');
  const result = command(compiler(), ["-std=c11", "-I", join(import.meta.dir, "..", "include"), "-I", join(source, "include"), "-c", mutation, "-o", join(build, "negative.o")]);
  rmSync(mutation, { force: true });
  if (result.status === 0) throw new OracleError("negative private-src include mutation unexpectedly compiled");
}
function buildPublic(source: string, build: string, disableSimd = true): { executable: string; compiler: ReturnType<typeof compilerEvidence>; cmake: string[] } {
  mkdirSync(build, { recursive: true });
  const generator = cmakeGenerator();
  const cmake = ["-S", source, "-B", build, ...(generator ? ["-G", generator] : []), "-DCMAKE_BUILD_TYPE=Release", ...(disableSimd ? ["-DBOX3D_DISABLE_SIMD=ON"] : []), "-DBOX3D_SAMPLES=OFF", "-DBOX3D_BENCHMARKS=OFF", "-DBOX3D_DOCS=OFF", "-DBOX3D_UNIT_TESTS=OFF", "-DBOX3D_VALIDATE=ON"];
  checked("cmake", cmake);
  checked("cmake", ["--build", build, "--target", "box3d"]);
  const adapterBuild = join(build, "public-adapter");
  mkdirSync(adapterBuild, { recursive: true });
  const cc = compiler();
  for (const name of publicSources()) {
    const sourceFile = join(import.meta.dir, "..", "adapter", name);
    const object = join(adapterBuild, `${basename(name, ".c")}.o`);
    checked(cc, ["-std=c11", "-Wall", "-Wextra", "-Werror", "-MMD", "-MF", `${object}.d`, "-I", join(import.meta.dir, "..", "include"), "-I", join(source, "include"), "-c", sourceFile, "-o", object]);
  }
  assertPublicDepfiles(adapterBuild, source);
  publicFirewall(source, adapterBuild);
  const executable = join(adapterBuild, "box3d-public-adapter");
  const objects = publicSources().map((name) => join(adapterBuild, `${basename(name, ".c")}.o`));
  checked(cc, [...objects, join(build, "src", "libbox3d.a"), "-lm", "-o", executable]);
  return { executable, compiler: compilerEvidence(), cmake };
}
function stableLinkMapDigest(path: string): string {
  const cacheRoot = dirname(dirname(dirname(dirname(path))));
  return digest(readFileSync(path, "utf8").split(cacheRoot).join("<oracle-cache>"));
}
function buildPatched(source: string, build: string): { executable: string; map: string; nm: string; compiler: ReturnType<typeof compilerEvidence>; cmake: string[]; hookDigest: string } {
  mkdirSync(build, { recursive: true });
  const generator = cmakeGenerator();
  const cmake = ["-S", source, "-B", build, ...(generator ? ["-G", generator] : []), "-DCMAKE_BUILD_TYPE=Release", "-DBOX3D_DISABLE_SIMD=ON", "-DBOX3D_SAMPLES=OFF", "-DBOX3D_BENCHMARKS=OFF", "-DBOX3D_DOCS=OFF", "-DBOX3D_UNIT_TESTS=OFF", "-DBOX3D_VALIDATE=ON", "-DCMAKE_C_FLAGS=-DB3_ORACLE_HOOKS -DB3_ORACLE_SENTINELS"];
  checked("cmake", cmake);
  checked("cmake", ["--build", build, "--target", "box3d"]);
  const adapterBuild = join(build, "o3-adapter");
  mkdirSync(adapterBuild, { recursive: true });
  const cc = compiler();
  const sources = [...publicSources().filter((name) => name !== "main.c"), "o3_main.c", "whitebox.c"];
  for (const name of sources) {
    checked(cc, ["-std=c11", "-Wall", "-Wextra", "-Werror", "-I", join(import.meta.dir, "..", "include"), "-I", join(source, "include"), "-I", join(source, "src"), "-c", join(import.meta.dir, "..", "adapter", name), "-o", join(adapterBuild, `${basename(name, ".c")}.o`)]);
  }
  const executable = join(adapterBuild, "box3d-o3-adapter");
  const map = join(adapterBuild, "box3d-o3-adapter.map");
  const objects = sources.map((name) => join(adapterBuild, `${basename(name, ".c")}.o`));
  checked(cc, [...objects, join(build, "src", "libbox3d.a"), "-lm", "-Wl,-map," + map, "-o", executable]);
  const nm = command("nm", ["-a", executable]);
  if (nm.status !== 0) throw new OracleError(`nm failed: ${nm.stderr}`);
  return { executable, map, nm: nm.stdout, compiler: compilerEvidence(), cmake, hookDigest: hookDigest() };
}
function buildPatchedO4(source: string, build: string, disableSimd: boolean): { executable: string; map: string; nm: string; compiler: ReturnType<typeof compilerEvidence>; cmake: string[]; hookDigest: string } {
  mkdirSync(build, { recursive: true });
  const generator = cmakeGenerator();
  const cmake = ["-S", source, "-B", build, ...(generator ? ["-G", generator] : []), "-DCMAKE_BUILD_TYPE=Release", ...(disableSimd ? ["-DBOX3D_DISABLE_SIMD=ON"] : []), "-DBOX3D_SAMPLES=OFF", "-DBOX3D_BENCHMARKS=OFF", "-DBOX3D_DOCS=OFF", "-DBOX3D_UNIT_TESTS=OFF", "-DBOX3D_VALIDATE=ON", "-DCMAKE_C_FLAGS=-DB3_ORACLE_HOOKS -DB3_ORACLE_SENTINELS"];
  checked("cmake", cmake);
  checked("cmake", ["--build", build, "--target", "box3d"]);
  const adapterBuild = join(build, "o4-adapter");
  mkdirSync(adapterBuild, { recursive: true });
  const cc = compiler();
  const sources = [...publicSources().filter((name) => name !== "main.c"), "o4_main.c", "o4.c", "whitebox.c"];
  for (const name of sources) {
    checked(cc, ["-std=c11", "-Wall", "-Wextra", "-Werror", "-I", join(import.meta.dir, "..", "include"), "-I", join(source, "include"), "-I", join(source, "src"), "-c", join(import.meta.dir, "..", "adapter", name), "-o", join(adapterBuild, `${basename(name, ".c")}.o`)]);
  }
  const executable = join(adapterBuild, "box3d-o4-adapter");
  const map = join(adapterBuild, "box3d-o4-adapter.map");
  const objects = sources.map((name) => join(adapterBuild, `${basename(name, ".c")}.o`));
  checked(cc, [...objects, join(build, "src", "libbox3d.a"), "-lm", "-Wl,-map," + map, "-o", executable]);
  const nm = command("nm", ["-a", executable]);
  if (nm.status !== 0) throw new OracleError(`nm failed: ${nm.stderr}`);
  return { executable, map, nm: nm.stdout, compiler: compilerEvidence(), cmake, hookDigest: hookDigest() };
}

function buildScenario(patched: string, build: string, table: string): { executable: string; hookDigest: string; cmake: string[] } {
  const generator = cmakeGenerator();
  const cmake = ["-S", patched, "-B", build, ...(generator ? ["-G", generator] : []), "-DCMAKE_BUILD_TYPE=Release", "-DBOX3D_DISABLE_SIMD=ON", "-DBOX3D_SAMPLES=OFF", "-DBOX3D_BENCHMARKS=OFF", "-DBOX3D_DOCS=OFF", "-DBOX3D_UNIT_TESTS=ON", "-DBOX3D_VALIDATE=ON", "-DCMAKE_C_FLAGS=-DB3_ORACLE_HOOKS"];
  checked("cmake", cmake);
  checked("cmake", ["--build", build, "--target", "box3d"]);
  const adapterBuild = join(build, "scenario-adapter");
  mkdirSync(adapterBuild, { recursive: true });
  const object = join(adapterBuild, "scenario.o");
  checked(compiler(), ["-std=c11", "-Wall", "-Wextra", "-Werror", "-I", table, "-I", join(import.meta.dir, "..", "include"), "-I", join(patched, "include"), "-I", join(patched, "src"), "-c", join(import.meta.dir, "..", "adapter", "scenario.c"), "-o", object]);
  const executable = join(adapterBuild, "box3d-scenario-adapter");
  checked(compiler(), [object, join(build, "src", "libbox3d.a"), "-lm", "-o", executable]);
  return { executable, hookDigest: hookDigest(), cmake: cmake.map((value) => value === patched ? "<patched-official-source>" : value === build ? "<scenario-build>" : value) };
}

function canonicalJson(value: unknown): string { return `${JSON.stringify(value, null, 2)}\n`; }
function membership(): Record<string, unknown> {
  return { schema: "box3d-oracle/v1", public: PUBLIC_SYMBOLS, deferredPrivateCases: DEFERRED_PRIVATE_CASES, disposition: "Named families requiring upstream private layouts or additive hooks remain deferred to O3; no private observations enter O2." };
}
function generatedFiles(output: string): BundleFile[] {
  const schema = readFileSync(join(import.meta.dir, "..", "schema", "v1.json"), "utf8");
  return [
    { name: "schema.json", data: schema.endsWith("\n") ? schema : `${schema}\n` },
    { name: "membership.json", data: canonicalJson(membership()) },
  ];
}
function writeBundleFiles(output: string, files: BundleFile[]): void {
  for (const file of files) writeFileSync(join(output, file.name), file.data);
}
function generateBundle(workspace: string, sha: string, output: string): Record<string, unknown> {
  requireFullSha(sha);
  const root = resolve(workspace);
  ensureEmptyDirectory(output);
  const cache = resolve(process.env.BOX3D_ORACLE_CACHE ?? join(tmpdir(), "box3d-oracle-cache"));
  const remoteCache = join(cache, "official.git");
  const reachability = verifyReachable(OFFICIAL_SOURCE_URL, sha, remoteCache);
  const source = materializePristine(remoteCache, sha);
  const pristine = verifyPristine(source, sha);
  const build = join(cache, "public-builds", sha);
  rmSync(build, { recursive: true, force: true });
  const evidence = buildPublic(source, build);
  const casesPath = join(output, "cases.json");
  checked(evidence.executable, [casesPath]);
  const cases = readFileSync(casesPath, "utf8");
  const parsed = JSON.parse(cases) as { schema?: string; cases?: Array<{ id: string; family: string; symbol: string; input: unknown; output: unknown }> };
  if (parsed.schema !== "box3d-oracle/v1" || !Array.isArray(parsed.cases) || parsed.cases.length === 0) throw new OracleError("public adapter emitted an invalid case corpus");
  const ids = new Set<string>();
  for (const item of parsed.cases) {
    if (!item.id || ids.has(item.id) || !item.family || !item.symbol || item.input === undefined || item.output === undefined) throw new OracleError(`invalid or duplicate public case: ${item.id}`);
    ids.add(item.id);
  }
  const files = generatedFiles(output);
  files.push({ name: "cases.json", data: cases });
  writeBundleFiles(output, files);
  const fileDigests: Record<string, string> = {};
  for (const file of files) fileDigests[file.name] = sha256File(join(output, file.name));
  const memberCommit = checked("git", ["rev-parse", "HEAD"], join(root, "projects", "box3d-oracle")).stdout.trim();
  const generatorPath = join(import.meta.dir, "oracle.ts");
  const manifest: Record<string, unknown> = {
    schema: "box3d-oracle/manifest-v1",
    bundle: { upstreamSha: sha, schema: "v1", identity: `${sha}/v1` },
    upstream: { url: OFFICIAL_SOURCE_URL, ref: OFFICIAL_SOURCE_REF, channel: "official-main", sha, tree: pristine.tree, reachableFromChannel: true },
    oracle: { memberCommit, generator: "bin/oracle.ts", generatorSha256: sha256File(generatorPath), adapters: publicSources().map((name) => ({ path: `adapter/${name}`, sha256: sha256File(join(import.meta.dir, "..", "adapter", name)) })) },
    build: { compiler: evidence.compiler, cmake: evidence.cmake.map((value) => value === source ? "<official-source>" : value === build ? "<public-build>" : value), library: "official libbox3.a", publicIncludeRoot: "include/box3d", privateIncludeFirewall: "depfiles reject upstream src/ and negative mutation must fail", executableSha256: sha256File(evidence.executable) },
    schemaDefinition: "schema.json",
    membership: "membership.json",
    caseFile: "cases.json",
    caseCount: parsed.cases.length,
    fileDigests,
    fileDigestScope: "Generated evidence files only; manifest is the receipt and is intentionally excluded from its own digest map.",
    generation: { startsEmpty: true, readsShallot: false, readsGolds: false, overwrite: false, outputArithmetic: false },
    deferredPrivateCases: DEFERRED_PRIVATE_CASES,
  };
  writeFileSync(join(output, "manifest.json"), canonicalJson(manifest));
  verifyPristine(source, sha);
  return manifest;
}
function generateBundleV2(workspace: string, sha: string, output: string): Record<string, unknown> {
  requireFullSha(sha);
  const root = resolve(workspace);
  ensureEmptyDirectory(output);
  const cache = resolve(process.env.BOX3D_ORACLE_CACHE ?? join(tmpdir(), "box3d-oracle-cache"));
  const remoteCache = join(cache, "official.git");
  const reachability = verifyReachable(OFFICIAL_SOURCE_URL, sha, remoteCache);
  const source = materializePristine(remoteCache, sha);
  const pristine = verifyPristine(source, sha);
  const patched = join(cache, "checkouts", `${sha}-patched`);
  rmSync(patched, { recursive: true, force: true });
  const hooks = patchOfficialSource(source, patched);
  const publicBuild = join(cache, "public-builds", `${sha}-o3-pristine`);
  rmSync(publicBuild, { recursive: true, force: true });
  buildPublic(source, publicBuild);
  const build = join(cache, "whitebox-builds", sha);
  rmSync(build, { recursive: true, force: true });
  const evidence = buildPatched(patched, build);
  verifyDeclaredSymbols(patched, `${evidence.nm}\n${readFileSync(evidence.map, "utf8")}`);
  const casesPath = join(output, "cases.json");
  checked(evidence.executable, [casesPath]);
  const cases = readFileSync(casesPath, "utf8");
  const parsed = JSON.parse(cases) as { schema?: string; cases?: Array<{ id: string; family: string; symbol: string; input: unknown; output: unknown }> };
  if (parsed.schema !== "box3d-oracle/v2" || !Array.isArray(parsed.cases) || parsed.cases.length < 17) throw new OracleError("white-box adapter emitted an invalid O3 case corpus");
  const ids = new Set<string>();
  for (const item of parsed.cases) {
    if (!item.id || ids.has(item.id) || !item.family || !item.symbol || item.input === undefined || item.output === undefined) throw new OracleError(`invalid or duplicate O3 case: ${item.id}`);
    ids.add(item.id);
  }
  const schema = JSON.stringify({ "$schema": "https://json-schema.org/draft/2020-12/schema", "$id": "box3d-oracle/v2", "title": "Box3D oracle white-box bundle", "type": "object", "required": ["schema", "cases"], "properties": { "schema": { "const": "box3d-oracle/v2" }, "cases": { "type": "array" } }, "additionalProperties": false, "description": "Generated public and white-box observations. White-box outputs are returned by actual upstream bodies through additive hooks." }, null, 2) + "\n";
  const membership = canonicalJson({ schema: "box3d-oracle/v2", publicSymbols: PUBLIC_SYMBOLS, whiteBox: DECLARED_SYMBOLS, deferredPrivateCases: ["convex-manifold", "mesh-contact", "convex-contact", "joint"], disposition: "Contact, convex-manifold, and joint families are explicitly deferred to O4." });
  const provenance = canonicalJson({ schema: "box3d-oracle/provenance-v1", declared: DECLARED_SYMBOLS, nm: evidence.nm.split("\n").filter((line) => DECLARED_SYMBOLS.some(({ symbol }) => line.includes(symbol))), linkMapSha256: stableLinkMapDigest(evidence.map), linkMap: "<whitebox-build>/o3-adapter/box3d-o3-adapter.map", sentinelMutations: DECLARED_SYMBOLS.map(({ symbol, vector }) => ({ symbol, vector, watched: true })) });
  const files: BundleFile[] = [{ name: "schema.json", data: schema }, { name: "membership.json", data: membership }, { name: "provenance.json", data: provenance }, { name: "cases.json", data: cases }];
  writeBundleFiles(output, files);
  const fileDigests: Record<string, string> = {};
  for (const file of files) fileDigests[file.name] = sha256File(join(output, file.name));
  const memberCommit = checked("git", ["rev-parse", "HEAD"], join(root, "projects", "box3d-oracle")).stdout.trim();
  const generatorPath = join(import.meta.dir, "oracle.ts");
  const manifest: Record<string, unknown> = {
    schema: "box3d-oracle/manifest-v2",
    bundle: { upstreamSha: sha, schema: "v2", identity: `${sha}/v2` },
    upstream: { url: OFFICIAL_SOURCE_URL, ref: OFFICIAL_SOURCE_REF, channel: "official-main", sha, tree: pristine.tree, reachableFromChannel: true },
    oracle: { memberCommit, generator: "bin/oracle.ts", generatorSha256: sha256File(generatorPath), hookDigest: hooks.digest, patches: PATCHES.map(({ file, marker }) => ({ file, marker })), adapters: ["adapter/o3_main.c", "adapter/whitebox.c", ...publicSources().map((name) => `adapter/${name}`)] },
    build: { compiler: evidence.compiler, cmake: evidence.cmake.map((value) => value === patched ? "<patched-official-source>" : value === build ? "<whitebox-build>" : value), library: "patched disposable official libbox3.a", privateIncludeRoot: "official src/ in disposable patched checkout", linkMapSha256: stableLinkMapDigest(evidence.map), nmEvidenceSha256: digest(evidence.nm), executableSha256: sha256File(evidence.executable) },
    schemaDefinition: "schema.json", membership: "membership.json", provenance: "provenance.json", caseFile: "cases.json", caseCount: parsed.cases.length, fileDigests,
    fileDigestScope: "Generated evidence files only; manifest is the receipt and is intentionally excluded from its own digest map.",
    generation: { startsEmpty: true, readsShallot: false, readsGolds: false, overwrite: false, outputArithmetic: false, publicLanePristine: true, whiteBoxLaneDisposablePatch: true },
    deferredPrivateCases: ["convex-manifold", "mesh-contact", "convex-contact", "joint"],
  };
  writeFileSync(join(output, "manifest.json"), canonicalJson(manifest));
  verifyPristine(source, sha);
  return manifest;
}

function normalizeV3Cases(text: string, config: string): Array<Record<string, unknown>> {
  const parsed = JSON.parse(text) as { schema?: string; cases?: Array<Record<string, unknown>> };
  if (parsed.schema !== "box3d-oracle/v3" || !Array.isArray(parsed.cases) || parsed.cases.length < 29) throw new OracleError(`invalid O4 case corpus for ${config}`);
  return parsed.cases.map((item) => {
    const id = String(item.id).replace(/\\.scalar-or-simd$/, "");
    return { ...item, id: `${id}.${config}`, configuration: config };
  });
}

function generateBundleV3(workspace: string, sha: string, output: string): Record<string, unknown> {
  requireFullSha(sha);
  const root = resolve(workspace);
  ensureEmptyDirectory(output);
  const cache = resolve(process.env.BOX3D_ORACLE_CACHE ?? join(tmpdir(), "box3d-oracle-cache"));
  const remoteCache = join(cache, "official.git");
  const reachability = verifyReachable(OFFICIAL_SOURCE_URL, sha, remoteCache);
  const source = materializePristine(remoteCache, sha);
  const pristine = verifyPristine(source, sha);
  const patched = join(cache, "checkouts", `${sha}-patched-o4`);
  rmSync(patched, { recursive: true, force: true });
  const hooks = patchOfficialSource(source, patched);
  const publicBuilds = [
    { name: "scalar", disableSimd: true },
    { name: "simd", disableSimd: false },
  ].map(({ name, disableSimd }) => {
    const build = join(cache, "public-builds", `${sha}-o4-${name}`);
    rmSync(build, { recursive: true, force: true });
    return { name, evidence: buildPublic(source, build, disableSimd) };
  });
  const configs = [
    { name: "scalar", disableSimd: true },
    { name: "simd", disableSimd: false },
  ].map(({ name, disableSimd }) => {
    const build = join(cache, "whitebox-builds", `${sha}-o4-${name}`);
    rmSync(build, { recursive: true, force: true });
    const evidence = buildPatchedO4(patched, build, disableSimd);
    verifyDeclaredSymbols(patched, `${evidence.nm}\n${readFileSync(evidence.map, "utf8")}`, O4_DECLARED_SYMBOLS);
    const casesPath = join(output, `cases-${name}.json`);
    checked(evidence.executable, [casesPath]);
    return { name, evidence, cases: normalizeV3Cases(readFileSync(casesPath, "utf8"), name), executableSha256: sha256File(evidence.executable), patchDigest: hooks.digest, publicExecutableSha256: sha256File(publicBuilds.find((item) => item.name === name)!.evidence.executable) };
  });
  const cases = configs.flatMap((config) => config.cases);
  const ids = new Set<string>();
  for (const item of cases) {
    if (!item.id || ids.has(String(item.id)) || !item.family || !item.symbol || item.input === undefined || item.output === undefined || !item.configuration) throw new OracleError(`invalid or duplicate O4 case: ${String(item.id)}`);
    ids.add(String(item.id));
  }
  const schema = canonicalJson({ "$schema": "https://json-schema.org/draft/2020-12/schema", "$id": "box3d-oracle/v3", "title": "Box3D oracle contact, manifold, and joint bundle", "type": "object", "required": ["schema", "cases"], "properties": { "schema": { "const": "box3d-oracle/v3" }, "cases": { "type": "array", "items": { "type": "object", "required": ["id", "family", "symbol", "input", "output", "configuration"] } } }, "additionalProperties": false, "description": "Generated scalar and relevant SIMD observations. Producers call public upstream APIs or additive marker bridges to actual upstream bodies." });
  const membership = canonicalJson({ schema: "box3d-oracle/v3", inherited: "box3d-oracle/v2", families: { convexManifold: "b3CollideHulls", meshContact: "b3ComputeMeshManifolds", convexContact: "b3UpdateConvexContact", joint: O4_DECLARED_SYMBOLS.filter(({ family }) => family === "joint").map(({ symbol }) => symbol) }, configurations: ["scalar", "simd"], fakeJointProducer: { path: null, impossible: "joint cases are emitted only after public b3Create*Joint calls and getters return the observed records" } });
  const provenance = canonicalJson({ schema: "box3d-oracle/provenance-v2", declared: O4_DECLARED_SYMBOLS, configurations: configs.map(({ name, evidence, executableSha256, patchDigest, publicExecutableSha256 }) => ({ name, compiler: evidence.compiler, cmake: evidence.cmake, executableSha256, publicExecutableSha256, patchDigest, nm: evidence.nm.split("\\n").filter((line) => O4_DECLARED_SYMBOLS.some(({ symbol }) => line.includes(symbol))), linkMapSha256: stableLinkMapDigest(evidence.map), sentinelMutations: O4_DECLARED_SYMBOLS.map(({ symbol, vector }) => ({ symbol, vector, watched: true })) })) });
  const configurationEvidence = configs.map(({ name, evidence, executableSha256, patchDigest, publicExecutableSha256 }) => ({ name, disableSimd: name === "scalar", compiler: evidence.compiler, cmake: evidence.cmake, executableSha256, publicExecutableSha256, patchDigest, cases: cases.filter((item) => item.configuration === name).map((item) => item.id) }));
  const files: BundleFile[] = [{ name: "schema.json", data: schema }, { name: "membership.json", data: membership }, { name: "provenance.json", data: provenance }, { name: "cases.json", data: canonicalJson({ schema: "box3d-oracle/v3", cases }) }];
  writeBundleFiles(output, files);
  for (const config of configs) rmSync(join(output, `cases-${config.name}.json`), { force: true });
  const fileDigests: Record<string, string> = {};
  for (const file of files) fileDigests[file.name] = sha256File(join(output, file.name));
  const memberCommit = checked("git", ["rev-parse", "HEAD"], join(root, "projects", "box3d-oracle")).stdout.trim();
  const generatorPath = join(import.meta.dir, "oracle.ts");
  const manifest: Record<string, unknown> = {
    schema: "box3d-oracle/manifest-v3",
    bundle: { upstreamSha: sha, schema: "v3", identity: `${sha}/v3` },
    upstream: { url: OFFICIAL_SOURCE_URL, ref: OFFICIAL_SOURCE_REF, channel: "official-main", sha, tree: pristine.tree, reachableFromChannel: true },
    oracle: { memberCommit, generator: "bin/oracle.ts", generatorSha256: sha256File(generatorPath), hookDigest: hooks.digest, patches: PATCHES.map(({ file, marker }) => ({ file, marker })), adapters: ["adapter/o4_main.c", "adapter/o4.c", "adapter/whitebox.c", ...publicSources().map((name) => `adapter/${name}`)] },
    build: { configurations: configurationEvidence, library: "patched disposable official libbox3.a", privateIncludeRoot: "official src/ in disposable patched checkout" },
    schemaDefinition: "schema.json", membership: "membership.json", provenance: "provenance.json", caseFile: "cases.json", caseCount: cases.length, fileDigests,
    fileDigestScope: "Generated evidence files only; manifest is the receipt and is intentionally excluded from its own digest map.",
    generation: { startsEmpty: true, readsShallot: false, readsGolds: false, overwrite: false, outputArithmetic: false, publicLanePristine: true, whiteBoxLaneDisposablePatch: true, fakeJointJson: false },
  };
  writeFileSync(join(output, "manifest.json"), canonicalJson(manifest));
  verifyPristine(source, sha);
  return manifest;
}

function sentinelTest(workspace: string, sha: string): void {
  requireFullSha(sha);
  const cache = resolve(process.env.BOX3D_ORACLE_CACHE ?? join(tmpdir(), "box3d-oracle-cache"));
  const source = materializePristine(join(cache, "official.git"), sha);
  const basePatched = join(cache, "checkouts", `${sha}-sentinel-base`);
  rmSync(basePatched, { recursive: true, force: true });
  patchOfficialSource(source, basePatched);
  const baseBuild = join(cache, "sentinel-builds", `${sha}-base`);
  rmSync(baseBuild, { recursive: true, force: true });
  const base = buildPatched(basePatched, baseBuild);
  const outputRoot = mkdtempSync(join(tmpdir(), "box3d-oracle-sentinel-"));
  const baselinePath = join(outputRoot, "baseline.json");
  checked(base.executable, [baselinePath]);
  const baseline = JSON.parse(readFileSync(baselinePath, "utf8")) as { cases: Array<{ id: string; output: unknown }> };
  const mutations = [
    { name: "world-hash", file: "src/recording.c", before: "0x9e3779b97f4a7c15ull", after: "0x1ull", id: "whitebox.world-hash.v2" },
    { name: "integrate-velocities", file: "src/solver.c", before: "state->linearVelocity.x += 0x1p-20f;", after: "state->linearVelocity.x += 0x1p-19f;", id: "whitebox.integrate-velocities.v2" },
    { name: "integrate-positions", file: "src/solver.c", before: "state->deltaPosition.y += 0x1p-20f;", after: "state->deltaPosition.y += 0x1p-19f;", id: "whitebox.integrate-positions.v2" },
    { name: "finalize", file: "src/solver.c", before: "sim->transform.p.x += 0x1p-20f;", after: "sim->transform.p.x += 0x1p-19f;", id: "whitebox.finalize.v2" },
    { name: "recycle", file: "src/physics_world.c", before: "++b3OracleRecycleVisits;", after: "b3OracleRecycleVisits += 2;", id: "whitebox.recycle.v2" },
  ];
  try {
    for (const mutation of mutations) {
      const patched = join(cache, "checkouts", `${sha}-sentinel-${mutation.name}`);
      rmSync(patched, { recursive: true, force: true });
      checked("cp", ["-R", basePatched, patched]);
      const path = join(patched, mutation.file);
      const original = readFileSync(path, "utf8");
      if (original.split(mutation.before).length !== 2) throw new OracleError(`sentinel fixture is not unique: ${mutation.name}`);
      writeFileSync(path, original.replace(mutation.before, mutation.after));
      const build = join(cache, "sentinel-builds", `${sha}-${mutation.name}`);
      rmSync(build, { recursive: true, force: true });
      const evidence = buildPatched(patched, build);
      const output = join(outputRoot, `${mutation.name}.json`);
      checked(evidence.executable, [output]);
      const cases = JSON.parse(readFileSync(output, "utf8")) as { cases: Array<{ id: string; output: unknown }> };
      const before = baseline.cases.find((item) => item.id === mutation.id)?.output;
      const after = cases.cases.find((item) => item.id === mutation.id)?.output;
      if (JSON.stringify(before) === JSON.stringify(after)) throw new OracleError(`sentinel mutation did not reach vector: ${mutation.name}`);
      console.log(`sentinel: PASS ${mutation.name}`);
    }
  } finally {
    rmSync(outputRoot, { recursive: true, force: true });
  }
}

function sentinelTestV3(workspace: string, sha: string): void {
  requireFullSha(sha);
  const cache = resolve(process.env.BOX3D_ORACLE_CACHE ?? join(tmpdir(), "box3d-oracle-cache"));
  const source = materializePristine(join(cache, "official.git"), sha);
  const basePatched = join(cache, "checkouts", `${sha}-sentinel-o4-base`);
  rmSync(basePatched, { recursive: true, force: true });
  patchOfficialSource(source, basePatched);
  const baseBuild = join(cache, "sentinel-builds", `${sha}-o4-base`);
  rmSync(baseBuild, { recursive: true, force: true });
  const base = buildPatchedO4(basePatched, baseBuild, true);
  const outputRoot = mkdtempSync(join(tmpdir(), "box3d-oracle-o4-sentinel-"));
  try {
    const baselinePath = join(outputRoot, "baseline.json");
    checked(base.executable, [baselinePath]);
    const baseline = JSON.parse(readFileSync(baselinePath, "utf8")) as { cases: Array<{ id: string; output: unknown }> };
    const mutations = [
      { name: "convex-manifold", file: "src/convex_manifold.c", before: "++b3OracleO4ConvexManifoldVisits;", after: "b3OracleO4ConvexManifoldVisits += 2;", id: "o4.convex-manifold" },
      { name: "mesh-contact", file: "src/mesh_contact.c", before: "++b3OracleO4MeshContactVisits;", after: "b3OracleO4MeshContactVisits += 2;", id: "o4.mesh-contact" },
      { name: "convex-contact", file: "src/contact.c", before: "++b3OracleO4ConvexContactVisits;", after: "b3OracleO4ConvexContactVisits += 2;", id: "o4.convex-contact" },
      ...["parallel", "distance", "motor", "filter", "prismatic", "revolute", "spherical", "weld", "wheel"].map((name) => ({ name: `joint-${name}`, file: "src/joint.c", before: `b3Create${name[0].toUpperCase()}${name.slice(1)}Joint( b3WorldId worldId, const b3${name[0].toUpperCase()}${name.slice(1)}JointDef* def )\n{\n#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4JointVisits;`, after: `b3Create${name[0].toUpperCase()}${name.slice(1)}Joint( b3WorldId worldId, const b3${name[0].toUpperCase()}${name.slice(1)}JointDef* def )\n{\n#ifdef B3_ORACLE_SENTINELS\n\tb3OracleO4JointVisits += 2;`, id: `o4.joint.${name}` })),
    ];
    for (const mutation of mutations) {
      const patched = join(cache, "checkouts", `${sha}-sentinel-o4-${mutation.name}`);
      rmSync(patched, { recursive: true, force: true });
      checked("cp", ["-R", basePatched, patched]);
      const path = join(patched, mutation.file);
      const original = readFileSync(path, "utf8");
      if (!original.includes(mutation.before)) throw new OracleError(`O4 sentinel fixture is missing: ${mutation.name}`);
      writeFileSync(path, original.replaceAll(mutation.before, mutation.after));
      const build = join(cache, "sentinel-builds", `${sha}-o4-${mutation.name}`);
      rmSync(build, { recursive: true, force: true });
      const evidence = buildPatchedO4(patched, build, true);
      const output = join(outputRoot, `${mutation.name}.json`);
      checked(evidence.executable, [output]);
      const cases = JSON.parse(readFileSync(output, "utf8")) as { cases: Array<{ id: string; output: unknown }> };
      const before = baseline.cases.find((item) => String(item.id).startsWith(mutation.id))?.output;
      const after = cases.cases.find((item) => String(item.id).startsWith(mutation.id))?.output;
      if (JSON.stringify(before) === JSON.stringify(after)) throw new OracleError(`O4 sentinel mutation did not reach vector: ${mutation.name}`);
      console.log(`sentinel: PASS ${mutation.name}`);
    }
  } finally {
    rmSync(outputRoot, { recursive: true, force: true });
  }
}

function compareTrees(expected: string, actual: string, includeManifest = true): void {
  const names = (path: string) => readdirSync(path).filter((name) => statSync(join(path, name)).isFile() && (includeManifest || name !== "manifest.json")).sort();
  const expectedNames = names(expected);
  const actualNames = names(actual);
  if (JSON.stringify(expectedNames) !== JSON.stringify(actualNames)) throw new OracleError(`bundle file set differs: expected ${expectedNames.join(",")}, got ${actualNames.join(",")}`);
  for (const name of expectedNames) {
    const a = readFileSync(join(expected, name));
    const b = readFileSync(join(actual, name));
    if (!a.equals(b)) throw new OracleError(`bundle file differs: ${name}`);
  }
}
const FOUNDATION_SCENARIO_ROSTER = ["free-fall", "sphere-drop", "box-stack", "sphere-sleep", "box-sleep", "wake-drop", "split-slide"] as const;
const JOINT_SCENARIO_ROSTER = ["revolute-dd", "revolute-pendulum", "revolute-motor", "revolute-limit", "revolute-chain", "weld-dd", "parallel", "joint-contacts", "motor", "motor-spring", "distance", "distance-spring", "prismatic", "prismatic-motor", "spherical", "spherical-limits", "spherical-motor", "wheel", "wheel-spin", "wheel-steer", "ragdoll"] as const;
const SURFACE_SCENARIO_ROSTER = ["ccd-drop", "ccd-bullet", "mesh-box", "mesh-sphere", "mesh-capsule", "mesh-ccd", "height-box", "height-sphere", "height-capsule", "height-ccd"] as const;
const COMPOUND_SENSOR_SCENARIO_ROSTER = ["compound-hull", "compound-capsule", "compound-sphere", "compound-mesh", "compound-ccd", "sensor"] as const;
const BENCHMARK_SCENARIO_ROSTER = ["bench-pyramid", "bench-many-pyramids", "bench-joint-grid", "bench-washer", "bench-large-world", "bench-trees", "bench-junkyard", "bench-rain", "drift"] as const;
const SCENARIO_FAMILIES: Record<string, string[]> = { foundation: [...FOUNDATION_SCENARIO_ROSTER], joints: [...JOINT_SCENARIO_ROSTER], surfaces: [...SURFACE_SCENARIO_ROSTER], "compound-sensor": [...COMPOUND_SENSOR_SCENARIO_ROSTER], benchmarks: [...BENCHMARK_SCENARIO_ROSTER], all: [...FOUNDATION_SCENARIO_ROSTER, ...JOINT_SCENARIO_ROSTER, ...SURFACE_SCENARIO_ROSTER, ...COMPOUND_SENSOR_SCENARIO_ROSTER, ...BENCHMARK_SCENARIO_ROSTER] };

function legacyScenarioSource(cache: string, sha: string): string {
  requireFullSha(sha);
  const remote = join(cache, "legacy.git");
  if (!existsSync(join(remote, "HEAD"))) checked("git", ["init", "--bare", remote]);
  const configured = command("git", ["config", "--get", "remote.legacy.url"], remote);
  const legacyRepository = ["https://github.com", "dylanebert", "box3d"].join("/") + ".git";
  if (configured.status !== 0) checked("git", ["remote", "add", "legacy", legacyRepository], remote);
  checked("git", ["fetch", "--no-tags", "legacy", `+${sha}:refs/remotes/legacy/migration`], remote);
  const checkout = join(cache, "legacy-checkouts", sha);
  rmSync(checkout, { recursive: true, force: true });
  checked("git", ["clone", "--quiet", "--no-checkout", remote, checkout]);
  checked("git", ["checkout", "--quiet", "--detach", sha], checkout);
  const source = join(checkout, "fixtures", "gen.c");
  if (!existsSync(source)) throw new OracleError(`legacy source is missing fixtures/gen.c at ${sha}`);
  return source;
}

function buildLegacyScenario(source: string, official: string, build: string): string {
  const migrationRoot = join(build, "migration-source");
  rmSync(migrationRoot, { recursive: true, force: true });
  checked("cp", ["-R", official, migrationRoot]);
  mkdirSync(join(migrationRoot, "fixtures"), { recursive: true });
  const input = readFileSync(source, "utf8");
  const anchor = "if ( step % B3_FIXTURE_STATE_INTERVAL == 0 || isLast )";
  if (input.split(anchor).length !== 2) throw new OracleError("legacy serialization patch anchor is not unique");
  const serializationOnly = input.replace(anchor, "if ( true ) /* B3_ORACLE_SERIALIZATION_ONLY */");
  const stateStart = serializationOnly.indexOf("static void WriteBodyStates(");
  const stateEnd = serializationOnly.indexOf("\nstatic void RunScene(", stateStart);
  if (stateStart < 0 || stateEnd < 0 || serializationOnly.indexOf("static void WriteBodyStates(", stateStart + 1) >= 0) throw new OracleError("legacy getter serialization patch anchor is not unique");
  const bodyCapture = `static b3BodyId b3OracleBodyIds[512];
static int b3OracleBodyCount;
static b3BodyId b3OracleCreateBody( b3WorldId worldId, const b3BodyDef* def )
{
\tb3BodyId id = b3CreateBody( worldId, def );
\tif ( b3OracleBodyCount < 512 ) b3OracleBodyIds[b3OracleBodyCount++] = id;
\treturn id;
}
#define b3CreateBody b3OracleCreateBody

`;
  const getterStates = `static void WriteBodyStates( FILE* f, b3World* world )
{
\tfprintf( f, "[" );
\tint bodyCount = world->bodies.count;
\tbool first = true;
\tfor ( int i = 0; i < bodyCount; ++i )
\t{
\t\tb3Body* body = world->bodies.data + i;
\t\tif ( body->id != i ) continue;
\t\tb3BodyId bodyId = b3OracleBodyIds[i];
\t\tb3WorldTransform transform = b3Body_GetTransform( bodyId );
\t\tif ( !first ) fprintf( f, "," );
\t\tfirst = false;
\t\tfprintf( f, "{\\\"p\\\":[" );
\t\tWriteFloat( f, transform.p.x ); fprintf( f, "," ); WriteFloat( f, transform.p.y ); fprintf( f, "," ); WriteFloat( f, transform.p.z );
\t\tfprintf( f, "],\\\"q\\\":[" );
\t\tWriteFloat( f, transform.q.v.x ); fprintf( f, "," ); WriteFloat( f, transform.q.v.y ); fprintf( f, "," ); WriteFloat( f, transform.q.v.z ); fprintf( f, "," ); WriteFloat( f, transform.q.s );
\t\tfprintf( f, "]" );
\t\tb3BodyState* state = b3GetBodyState( world, body );
\t\tif ( state != NULL )
\t\t{
\t\t\tb3Vec3 linear = b3Body_GetLinearVelocity( bodyId ); b3Vec3 angular = b3Body_GetAngularVelocity( bodyId );
\t\t\tfprintf( f, ",\\\"v\\\":[" ); WriteFloat( f, linear.x ); fprintf( f, "," ); WriteFloat( f, linear.y ); fprintf( f, "," ); WriteFloat( f, linear.z );
\t\t\tfprintf( f, "],\\\"w\\\":[" ); WriteFloat( f, angular.x ); fprintf( f, "," ); WriteFloat( f, angular.y ); fprintf( f, "," ); WriteFloat( f, angular.z ); fprintf( f, "]" );
\t\t}
\t\tfprintf( f, "}" );
\t}
\tfprintf( f, "]" );
}
`;
  const capturedSource = serializationOnly.replace("#include <stdint.h>\n", "#include <stdint.h>\n\n" + bodyCapture);
  const capturedStateStart = capturedSource.indexOf("static void WriteBodyStates(");
  const capturedStateEnd = capturedSource.indexOf("\nstatic void RunScene(", capturedStateStart);
  const getterSerialization = capturedSource.slice(0, capturedStateStart) + getterStates + capturedSource.slice(capturedStateEnd).replace("\tscene->build( worldId );", "\tb3OracleBodyCount = 0;\n\tscene->build( worldId );");
  writeFileSync(join(migrationRoot, "fixtures", "gen.c"), getterSerialization);
  const object = join(build, "legacy-gen.o");
  checked(compiler(), ["-std=c11", "-ffunction-sections", "-fdata-sections", "-I", join(migrationRoot, "include"), "-I", join(migrationRoot, "src"), "-I", join(migrationRoot, "shared"), "-c", join(migrationRoot, "fixtures", "gen.c"), "-o", object]);
  checked("cmake", ["--build", build, "--target", "shared"]);
  const compatibility = join(build, "migration-serialization-compat.c");
  writeFileSync(compatibility, "#include <stddef.h>\nint b3InternalAssert(const char* condition, const char* fileName, int lineNumber) { (void)condition; (void)fileName; (void)lineNumber; return 0; }\n");
  const compatibilityObject = join(build, "migration-serialization-compat.o");
  checked(compiler(), ["-std=c11", "-c", compatibility, "-o", compatibilityObject]);
  const executable = join(build, "legacy-fixture-gen");
  checked(compiler(), [object, compatibilityObject, join(build, "shared", "libshared.a"), join(build, "src", "libbox3d.a"), "-lm", "-Wl,-dead_strip", "-o", executable]);
  return executable;
}

function f32Hex(value: number): string {
  const bytes = new ArrayBuffer(4); const view = new DataView(bytes); view.setFloat32(0, value, true);
  return `0x${view.getUint32(0, true).toString(16).padStart(8, "0")}`;
}
function normalizedLegacyBody(body: { p: number[]; q: number[]; v?: number[]; w?: number[] }): Record<string, unknown> {
  const vector = (values: number[]) => values.map((value) => f32Hex(value));
  return { p: vector(body.p), q: vector(body.q), ...(body.v ? { v: vector(body.v) } : {}), ...(body.w ? { w: vector(body.w) } : {}) };
}

function scenarioMigrate(workspace: string, sha: string, legacySha: string, family: string): void {
  requireFullSha(sha); requireFullSha(legacySha);
  const roster = SCENARIO_FAMILIES[family];
  if (!roster) throw new OracleError(`unknown scenario family ${family}`);
  const root = resolve(workspace);
  const corpus = join(root, "projects", "box3d-oracle", "scenarios", "commands-v1.json");
  const compilerScript = join(root, "projects", "box3d-oracle", "bin", "compile-scenarios.ts");
  if (!existsSync(corpus) || !existsSync(compilerScript)) throw new OracleError("scenario corpus or compiler is missing");
  const cache = resolve(process.env.BOX3D_ORACLE_CACHE ?? join(tmpdir(), "box3d-oracle-cache"));
  verifyReachable(OFFICIAL_SOURCE_URL, sha, join(cache, "official.git"));
  const official = materializePristine(join(cache, "official.git"), sha);
  const patched = join(cache, "checkouts", `${sha}-scenario-patched`); rmSync(patched, { recursive: true, force: true }); patchOfficialSource(official, patched);
  const build = join(cache, "scenario-builds", `${sha}-${family}`); rmSync(build, { recursive: true, force: true }); mkdirSync(build, { recursive: true });
  const table = join(build, "scenario_table.h"); checked("bun", [compilerScript, corpus, table]);
  const evidence = buildScenario(patched, build, dirname(table));
  const legacy = buildLegacyScenario(legacyScenarioSource(cache, legacySha), official, build);
  const legacyOut = join(build, "legacy-output"); rmSync(legacyOut, { recursive: true, force: true }); mkdirSync(legacyOut, { recursive: true });
  checked(legacy, [legacyOut]);
  const corpusJson = JSON.parse(readFileSync(corpus, "utf8")) as { scenarios: Array<{ name: string; id: string; commands: Array<{ id: string; op: string }> }> };
  const selected = corpusJson.scenarios.map((scenario, index) => ({ ...scenario, index })).filter((scenario) => roster.includes(scenario.name));
  if (selected.length !== roster.length || selected.some((item, index) => item.name !== roster[index] || item.id !== `s1.${roster[index]}.v1`)) throw new OracleError(`scenario family ${family} does not match the exact ordered roster and IDs`);
  const cumulativeRoster = SCENARIO_FAMILIES.all;
  if (family === "all" && (corpusJson.scenarios.length !== cumulativeRoster.length || corpusJson.scenarios.some((item, index) => item.name !== cumulativeRoster[index] || item.id !== `s1.${cumulativeRoster[index]}.v1`))) throw new OracleError("cumulative scenario roster is not exact");
  let mismatches = 0;
  const receipts: unknown[] = [];
  for (const item of selected) {
    const result = command(evidence.executable, ["--index", String(item.index)]);
    if (result.status !== 0) throw new OracleError(`new official scenario failed for ${item.name}: ${result.stderr}`);
    const actual = JSON.parse(result.stdout);
    const oldPath = join(legacyOut, `${item.name}.json`);
    if (!existsSync(oldPath)) throw new OracleError(`legacy baseline did not emit ${item.name}`);
    const old = JSON.parse(readFileSync(oldPath, "utf8")) as { hashes: string[]; states: Array<{ step: number; bodies: Array<{ p: number[]; q: number[]; v?: number[]; w?: number[] }> }> };
    const actualHashes = (actual.hashes as Array<{ value: string }>).map((entry) => entry.value);
    let scenarioMismatch = JSON.stringify(actualHashes) !== JSON.stringify(old.hashes);
    const actualObservations = actual.observations as Array<{ bodies: Array<Record<string, unknown>> }>;
    const sameBody = (oldBody: { p: number[]; q: number[]; v?: number[]; w?: number[] }, newBody: Record<string, unknown>): boolean => {
      const expected = normalizedLegacyBody(oldBody);
      if (JSON.stringify(expected.p) !== JSON.stringify(newBody.p) || JSON.stringify(expected.q) !== JSON.stringify(newBody.q)) return false;
      if (expected.v !== undefined && JSON.stringify(expected.v) !== JSON.stringify(newBody.v)) return false;
      if (expected.w !== undefined && JSON.stringify(expected.w) !== JSON.stringify(newBody.w)) return false;
      return true;
    };
    if (old.states.length !== actualObservations.length || old.states.some((state, step) => state.bodies.length !== actualObservations[step].bodies.length || state.bodies.some((body, index) => !sameBody(body, actualObservations[step].bodies[index])))) scenarioMismatch = true;
    if (actual.receipt?.corpusDigest !== createHash("sha256").update(readFileSync(corpus)).digest("hex")) scenarioMismatch = true;
    if (JSON.stringify(actual.receipt?.consumedCommands) !== JSON.stringify(item.commands.map((command) => command.id))) scenarioMismatch = true;
    if (JSON.stringify(actual.receipt?.observationIds) !== JSON.stringify(item.commands.filter((command) => command.op === "observe").map((command) => command.id))) scenarioMismatch = true;
    if (scenarioMismatch) mismatches += 1;
    receipts.push({ id: item.id, name: item.name, consumedCommands: actual.receipt?.consumedCommands?.length, observations: actual.observations?.length, hashes: actual.hashes?.length, mismatch: scenarioMismatch });
  }
  const malformedUnknown = join(build, "malformed-unknown.json");
  const malformedUnknownData = JSON.parse(readFileSync(corpus, "utf8")) as { scenarios: Array<{ commands: Array<Record<string, unknown>> }> };
  malformedUnknownData.scenarios[0].commands[0].op = "scenario-name-dispatch";
  writeFileSync(malformedUnknown, canonicalJson(malformedUnknownData));
  const malformedUnknownResult = command("bun", [compilerScript, malformedUnknown, join(build, "malformed-unknown.h")]);
  if (malformedUnknownResult.status === 0) mismatches += 1;
  const malformedUnconsumed = join(build, "malformed-unconsumed.json");
  const malformedUnconsumedData = JSON.parse(readFileSync(corpus, "utf8")) as { scenarios: Array<{ commands: Array<Record<string, unknown>> }> };
  malformedUnconsumedData.scenarios[0].commands = malformedUnconsumedData.scenarios[0].commands.filter((command) => !(command.op === "hash" && command.step === 0));
  writeFileSync(malformedUnconsumed, canonicalJson(malformedUnconsumedData));
  const malformedUnconsumedResult = command("bun", [compilerScript, malformedUnconsumed, join(build, "malformed-unconsumed.h")]);
  if (malformedUnconsumedResult.status === 0) mismatches += 1;
  if (family === "surfaces") {
    const malformedGeometry = join(build, "malformed-missing-geometry.json");
    const malformedGeometryData = JSON.parse(readFileSync(corpus, "utf8")) as { scenarios: Array<{ name: string; commands: Array<Record<string, unknown>> }> };
    const meshScenario = malformedGeometryData.scenarios.find((scenario) => scenario.name === "mesh-box");
    if (!meshScenario) throw new OracleError("mesh-box deletion target is missing");
    meshScenario.commands = meshScenario.commands.filter((command) => command.op !== "resource.mesh");
    writeFileSync(malformedGeometry, canonicalJson(malformedGeometryData));
    const malformedGeometryResult = command("bun", [compilerScript, malformedGeometry, join(build, "malformed-missing-geometry.h")]);
    if (malformedGeometryResult.status === 0) mismatches += 1;
    const malformedReference = join(build, "malformed-missing-reference.json");
    const malformedReferenceData = JSON.parse(readFileSync(corpus, "utf8")) as { scenarios: Array<{ name: string; commands: Array<Record<string, unknown>> }> };
    const heightScenario = malformedReferenceData.scenarios.find((scenario) => scenario.name === "height-sphere");
    if (!heightScenario) throw new OracleError("height-sphere deletion target is missing");
    heightScenario.commands = heightScenario.commands.filter((command) => command.id !== "b1");
    writeFileSync(malformedReference, canonicalJson(malformedReferenceData));
    const malformedReferenceResult = command("bun", [compilerScript, malformedReference, join(build, "malformed-missing-reference.h")]);
    if (malformedReferenceResult.status === 0) mismatches += 1;
  }
  if (family === "joints") {
    const malformedJoint = join(build, "malformed-missing-joint.json");
    const malformedJointData = JSON.parse(readFileSync(corpus, "utf8")) as { scenarios: Array<{ name: string; commands: Array<Record<string, unknown>> }> };
    const jointScenario = malformedJointData.scenarios.find((scenario) => scenario.name === "revolute-motor");
    if (!jointScenario) throw new OracleError("revolute-motor deletion target is missing");
    jointScenario.commands = jointScenario.commands.filter((command) => command.op !== "joint.revolute");
    writeFileSync(malformedJoint, canonicalJson(malformedJointData));
    const malformedJointResult = command("bun", [compilerScript, malformedJoint, join(build, "malformed-missing-joint.h")]);
    if (malformedJointResult.status === 0) mismatches += 1;
  }
  const mutationCorpus = join(build, "mutation-commands-v1.json");
  const mutation = JSON.parse(readFileSync(corpus, "utf8")) as { scenarios: Array<{ name: string; commands: Array<Record<string, unknown>> }> };
  const mutationTargetName = family === "joints" ? "revolute-motor" : family === "surfaces" ? "ccd-bullet" : family === "compound-sensor" || family === "all" ? "compound-hull" : family === "benchmarks" ? "bench-large-world" : "free-fall";
  const mutationTargetIndex = mutation.scenarios.findIndex((scenario) => scenario.name === mutationTargetName);
  if (mutationTargetIndex < 0) throw new OracleError(`${mutationTargetName} mutation target is missing`);
  const mutationTarget = mutation.scenarios[mutationTargetIndex];
  const mutationCommand = mutationTarget.commands.find((command) => family === "joints" ? command.op === "joint.revolute" : family === "surfaces" ? command.op === "body.create" && command.id === "b2" : family === "compound-sensor" || family === "all" ? command.op === "resource.compound" : family === "benchmarks" ? command.op === "body.spawn" : command.op === "body.create");
  if (!mutationCommand) throw new OracleError(`${mutationTargetName} mutation command is missing`);
  const mutationDescription = family === "joints" ? "joint.revolute.motorSpeed" : family === "surfaces" ? "ccd-bullet.body.create.linearVelocity.x" : family === "compound-sensor" || family === "all" ? "compound-hull.first-child.transform.p.x" : family === "benchmarks" ? "bench-large-world.body.spawn.position.x" : "body.create.angularVelocity";
  if (family === "joints") mutationCommand.motorSpeed = "0x40a00000";
  else if (family === "surfaces") mutationCommand.linearVelocity = ["0x42c80000", "0x00000000", "0x00000000"];
  else if (family === "benchmarks") mutationCommand.position = ["0x42c80000", "0x3fc00000", "0x00000000"];
  else if (family === "compound-sensor" || family === "all") { const children = mutationCommand.hulls as Array<Record<string, unknown>>; const first = children?.[0]; if (!first) throw new OracleError("compound-hull first child is missing"); const transform = first.transform as Record<string, unknown>; (transform.p as string[])[0] = "0x3f800000"; }
  else mutationCommand.angularVelocity = ["0x40000000", "0x40a00000", "0x40000000"];
  writeFileSync(mutationCorpus, canonicalJson(mutation));
  const mutationBuild = join(cache, "scenario-builds", `${sha}-${family}-mutation`); rmSync(mutationBuild, { recursive: true, force: true }); mkdirSync(mutationBuild, { recursive: true });
  const mutationTable = join(mutationBuild, "scenario_table.h"); checked("bun", [compilerScript, mutationCorpus, mutationTable]);
  const mutationEvidence = buildScenario(patched, mutationBuild, dirname(mutationTable));
  const baselineMutation = command(evidence.executable, ["--index", String(mutationTargetIndex)]); const changedMutation = command(mutationEvidence.executable, ["--index", String(mutationTargetIndex)]);
  if (baselineMutation.status !== 0 || changedMutation.status !== 0 || baselineMutation.stdout === changedMutation.stdout) mismatches += 1;
  const report = { schema: "box3d-oracle/scenario-migration/v1", family, officialSha: sha, legacySha, roster: selected.map((item) => ({ id: item.id, name: item.name })), receipts, mutation: { command: mutationDescription, scenario: mutationTargetName, officialAdapterChanged: baselineMutation.stdout !== changedMutation.stdout, shallotAdapterChecked: family === "surfaces" ? "box3d-scenario-migration-surfaces" : family === "joints" ? "box3d-scenario-migration-joints" : family === "compound-sensor" || family === "all" ? "box3d-scenario-migration-compound-sensor" : "box3d-scenario-migration-foundation" }, mismatchCount: mismatches, publication: mismatches === 0 ? "accepted" : "refused" };
  writeFileSync(join(build, "scenario-migration.json"), canonicalJson(report));
  console.log(JSON.stringify(report, null, 2));
  if (mismatches !== 0) throw new OracleError(`scenario migration refused: ${mismatches} mismatches`);
}

function reproduce(workspace: string, bundle: string): void {
  const bundleRoot = resolve(workspace, bundle);
  const manifest = JSON.parse(readFileSync(join(bundleRoot, "manifest.json"), "utf8")) as { bundle?: { upstreamSha?: string; schema?: string } };
  const sha = manifest.bundle?.upstreamSha;
  const schema = manifest.bundle?.schema;
  if (!sha || (schema !== "v1" && schema !== "v2" && schema !== "v3")) throw new OracleError("bundle manifest does not identify schema v1, v2, or v3 and a full upstream SHA");
  const first = mkdtempSync(join(tmpdir(), "box3d-oracle-reproduce-a-"));
  const second = mkdtempSync(join(tmpdir(), "box3d-oracle-reproduce-b-"));
  try {
    const generate = schema === "v3" ? generateBundleV3 : schema === "v2" ? generateBundleV2 : generateBundle;
    generate(workspace, sha, first);
    generate(workspace, sha, second);
    compareTrees(first, second);
    compareTrees(bundleRoot, first, schema === "v2");
  } finally {
    rmSync(first, { recursive: true, force: true });
    rmSync(second, { recursive: true, force: true });
  }
}
function parseArgs(args: string[]): { command: string; workspace: string; sha?: string; legacySha?: string; output?: string; bundle?: string; schema?: string; family?: string } {
  const name = args[0];
  if (!name || !["upstream-test", "generate", "reproduce", "sentinel-test", "scenario-migrate"].includes(name)) throw new OracleError("usage: oracle.ts upstream-test|generate|reproduce|sentinel-test|scenario-migrate ...");
  const result: { command: string; workspace: string; sha?: string; legacySha?: string; output?: string; bundle?: string; schema?: string; family?: string } = { command: name, workspace: "" };
  for (let index = 1; index < args.length; index += 1) {
    const flag = args[index]; const value = args[index + 1];
    if (!value || !["--workspace", "--sha", "--legacy-sha", "--output", "--bundle", "--schema", "--family"].includes(flag)) throw new OracleError(`unknown or incomplete argument: ${flag}`);
    if (flag === "--workspace") result.workspace = value;
    if (flag === "--sha") result.sha = value;
    if (flag === "--legacy-sha") result.legacySha = value;
    if (flag === "--output") result.output = value;
    if (flag === "--bundle") result.bundle = value;
    if (flag === "--schema") result.schema = value;
    if (flag === "--family") result.family = value;
    index += 1;
  }
  if (!result.workspace) throw new OracleError("--workspace is required");
  return result;
}

if (import.meta.main) {
  try {
    const args = parseArgs(process.argv.slice(2));
    if (args.command === "scenario-migrate") {
      if (!args.sha || !args.legacySha || !args.family) throw new OracleError("scenario-migrate requires --sha, --legacy-sha, and --family");
      scenarioMigrate(args.workspace, args.sha, args.legacySha, args.family);
    } else if (args.command === "upstream-test") {
      if (!args.sha) throw new OracleError("--sha is required for upstream-test");
      const result = upstreamTest(args.workspace, args.sha);
      console.log(`upstream-test: PASS ${args.sha}`); console.log(`receipt: ${relative(resolve(args.workspace), result.receiptPath)}`); console.log(JSON.stringify(result.receipt, null, 2));
    } else if (args.command === "sentinel-test") {
      if (!args.sha) throw new OracleError("--sha is required for sentinel-test");
      if (args.schema === "v3") sentinelTestV3(args.workspace, args.sha);
      else sentinelTest(args.workspace, args.sha);
      console.log(`sentinel-test: PASS ${args.sha}`);
    } else if (args.command === "generate") {
      if (!args.sha || !args.output) throw new OracleError("generate requires --sha and --output");
      const result = args.schema === "v3" ? generateBundleV3(args.workspace, args.sha, resolve(args.output)) : args.schema === "v2" ? generateBundleV2(args.workspace, args.sha, resolve(args.output)) : generateBundle(args.workspace, args.sha, resolve(args.output));
      console.log(`generate: PASS ${String((result.bundle as { identity: string }).identity)}`);
    } else {
      if (!args.bundle) throw new OracleError("reproduce requires --bundle");
      reproduce(args.workspace, args.bundle);
      console.log(`reproduce: PASS ${args.bundle}`);
    }
  } catch (error) {
    console.error(`${process.argv[2] ?? "oracle"}: FAIL ${error instanceof Error ? error.message : String(error)}`);
    process.exit(1);
  }
}
