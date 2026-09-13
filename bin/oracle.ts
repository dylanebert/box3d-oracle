#!/usr/bin/env bun
import { createHash } from "node:crypto";
import { existsSync, mkdirSync, mkdtempSync, readdirSync, readFileSync, rmSync, statSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { basename, dirname, join, relative, resolve } from "node:path";
import { spawnSync } from "node:child_process";
import { DECLARED_SYMBOLS, PATCHES, type OraclePatch } from "../hooks/patches";

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

export function verifyDeclaredSymbols(source: string, nmEvidence: string): void {
  for (const declared of DECLARED_SYMBOLS) {
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
function buildPublic(source: string, build: string): { executable: string; compiler: ReturnType<typeof compilerEvidence>; cmake: string[] } {
  mkdirSync(build, { recursive: true });
  const generator = cmakeGenerator();
  const cmake = ["-S", source, "-B", build, ...(generator ? ["-G", generator] : []), "-DCMAKE_BUILD_TYPE=Release", "-DBOX3D_DISABLE_SIMD=ON", "-DBOX3D_SAMPLES=OFF", "-DBOX3D_BENCHMARKS=OFF", "-DBOX3D_DOCS=OFF", "-DBOX3D_UNIT_TESTS=OFF", "-DBOX3D_VALIDATE=ON"];
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
  const provenance = canonicalJson({ schema: "box3d-oracle/provenance-v1", declared: DECLARED_SYMBOLS, nm: evidence.nm.split("\n").filter((line) => DECLARED_SYMBOLS.some(({ symbol }) => line.includes(symbol))), linkMapSha256: sha256File(evidence.map), linkMap: relative(root, evidence.map), sentinelMutations: DECLARED_SYMBOLS.map(({ symbol, vector }) => ({ symbol, vector, watched: true })) });
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
    build: { compiler: evidence.compiler, cmake: evidence.cmake.map((value) => value === patched ? "<patched-official-source>" : value === build ? "<whitebox-build>" : value), library: "patched disposable official libbox3.a", privateIncludeRoot: "official src/ in disposable patched checkout", linkMapSha256: sha256File(evidence.map), nmEvidenceSha256: digest(evidence.nm), executableSha256: sha256File(evidence.executable) },
    schemaDefinition: "schema.json", membership: "membership.json", provenance: "provenance.json", caseFile: "cases.json", caseCount: parsed.cases.length, fileDigests,
    fileDigestScope: "Generated evidence files only; manifest is the receipt and is intentionally excluded from its own digest map.",
    generation: { startsEmpty: true, readsShallot: false, readsGolds: false, overwrite: false, outputArithmetic: false, publicLanePristine: true, whiteBoxLaneDisposablePatch: true },
    deferredPrivateCases: ["convex-manifold", "mesh-contact", "convex-contact", "joint"],
  };
  writeFileSync(join(output, "manifest.json"), canonicalJson(manifest));
  verifyPristine(source, sha);
  return manifest;
}

function compareTrees(expected: string, actual: string): void {
  const names = (path: string) => readdirSync(path).filter((name) => statSync(join(path, name)).isFile()).sort();
  const expectedNames = names(expected);
  const actualNames = names(actual);
  if (JSON.stringify(expectedNames) !== JSON.stringify(actualNames)) throw new OracleError(`bundle file set differs: expected ${expectedNames.join(",")}, got ${actualNames.join(",")}`);
  for (const name of expectedNames) {
    const a = readFileSync(join(expected, name));
    const b = readFileSync(join(actual, name));
    if (!a.equals(b)) throw new OracleError(`bundle file differs: ${name}`);
  }
}
function reproduce(workspace: string, bundle: string): void {
  const bundleRoot = resolve(workspace, bundle);
  const manifest = JSON.parse(readFileSync(join(bundleRoot, "manifest.json"), "utf8")) as { bundle?: { upstreamSha?: string; schema?: string } };
  const sha = manifest.bundle?.upstreamSha;
  const schema = manifest.bundle?.schema;
  if (!sha || (schema !== "v1" && schema !== "v2")) throw new OracleError("bundle manifest does not identify schema v1 or v2 and a full upstream SHA");
  const first = mkdtempSync(join(tmpdir(), "box3d-oracle-reproduce-a-"));
  const second = mkdtempSync(join(tmpdir(), "box3d-oracle-reproduce-b-"));
  try {
    const generate = schema === "v2" ? generateBundleV2 : generateBundle;
    generate(workspace, sha, first);
    generate(workspace, sha, second);
    compareTrees(first, second);
    compareTrees(bundleRoot, first);
  } finally {
    rmSync(first, { recursive: true, force: true });
    rmSync(second, { recursive: true, force: true });
  }
}
function parseArgs(args: string[]): { command: string; workspace: string; sha?: string; output?: string; bundle?: string; schema?: string } {
  const name = args[0];
  if (!name || !["upstream-test", "generate", "reproduce"].includes(name)) throw new OracleError("usage: oracle.ts upstream-test|generate|reproduce ...");
  const result: { command: string; workspace: string; sha?: string; output?: string; bundle?: string; schema?: string } = { command: name, workspace: "" };
  for (let index = 1; index < args.length; index += 1) {
    const flag = args[index]; const value = args[index + 1];
    if (!value || !["--workspace", "--sha", "--output", "--bundle", "--schema"].includes(flag)) throw new OracleError(`unknown or incomplete argument: ${flag}`);
    if (flag === "--workspace") result.workspace = value;
    if (flag === "--sha") result.sha = value;
    if (flag === "--output") result.output = value;
    if (flag === "--bundle") result.bundle = value;
    if (flag === "--schema") result.schema = value;
    index += 1;
  }
  if (!result.workspace) throw new OracleError("--workspace is required");
  return result;
}

if (import.meta.main) {
  try {
    const args = parseArgs(process.argv.slice(2));
    if (args.command === "upstream-test") {
      if (!args.sha) throw new OracleError("--sha is required for upstream-test");
      const result = upstreamTest(args.workspace, args.sha);
      console.log(`upstream-test: PASS ${args.sha}`); console.log(`receipt: ${relative(resolve(args.workspace), result.receiptPath)}`); console.log(JSON.stringify(result.receipt, null, 2));
    } else if (args.command === "generate") {
      if (!args.sha || !args.output) throw new OracleError("generate requires --sha and --output");
      const result = args.schema === "v2" ? generateBundleV2(args.workspace, args.sha, resolve(args.output)) : generateBundle(args.workspace, args.sha, resolve(args.output));
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
