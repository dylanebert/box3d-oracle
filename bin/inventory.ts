import { createHash } from "node:crypto";
import { existsSync, readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";

export const OFFICIAL_SOURCE_URL = "https://github.com/erincatto/box3d.git";
export const OFFICIAL_SOURCE_REF = "refs/heads/main";
export const O6A_SUITE_FILES = [
  "test_allocator.c",
  "test_bitset.c",
  "test_body.c",
  "test_body_query.c",
  "test_collision.c",
  "test_compound.c",
  "test_container.c",
  "test_determinism.c",
  "test_distance.c",
  "test_hash.c",
  "test_height_field.c",
  "test_hull.c",
  "test_id.c",
] as const;

export const ALL_SUITE_FILES = [
  ...O6A_SUITE_FILES,
  "test_joint.c",
  "test_large_world.c",
  "test_manifold.c",
  "test_math.c",
  "test_mesh.c",
  "test_mover.c",
  "test_name_cache.c",
  "test_recording.c",
  "test_sat.c",
  "test_shape.c",
  "test_table.c",
  "test_world.c",
] as const;

const SUITE_SYMBOLS: Record<string, string> = {
  "test_allocator.c": "AllocatorTest",
  "test_bitset.c": "BitTest",
  "test_body.c": "BodyTest",
  "test_body_query.c": "BodyQueryTest",
  "test_collision.c": "CollisionTest",
  "test_compound.c": "CompoundTest",
  "test_container.c": "ContainerTest",
  "test_determinism.c": "DeterminismTest",
  "test_distance.c": "DistanceTest",
  "test_hash.c": "HashTest",
  "test_height_field.c": "HeightFieldTest",
  "test_hull.c": "HullTest",
  "test_id.c": "IdTest",
  "test_joint.c": "JointTest",
  "test_large_world.c": "LargeWorldTest",
  "test_manifold.c": "ManifoldTest",
  "test_math.c": "MathTest",
  "test_mesh.c": "MeshTest",
  "test_mover.c": "MoverTest",
  "test_name_cache.c": "NameCacheTest",
  "test_recording.c": "RecordingTest",
  "test_sat.c": "SeparatingAxisTest",
  "test_shape.c": "ShapeTest",
  "test_table.c": "TableTest",
  "test_world.c": "WorldTest",
};

export const CASE_STATUSES = ["equivalent", "broader-equivalent", "expected-difference", "not-applicable"] as const;
export type CaseStatus = (typeof CASE_STATUSES)[number];

export class InventoryError extends Error {}

export type RegistrationEvidence = {
  kind: "subtest" | "direct";
  macro: "RUN_SUBTEST" | "MAYBE_RUN_TEST";
  file: string;
  line: number;
  text: string;
  fingerprint: string;
};

export type InventoryCase = {
  id: string;
  suiteFile: string;
  suiteSymbol: string;
  caseSymbol: string;
  sourceSymbol: string;
  registration: RegistrationEvidence;
  source: {
    line: number;
    endLine: number;
    fingerprint: string;
  };
  suiteBlobSha256: string;
};

export type InventorySuite = {
  file: string;
  symbol: string;
  registration: RegistrationEvidence;
  blobSha256: string;
  cases: InventoryCase[];
};

export type Inventory = {
  schema: "box3d-oracle/inventory/v1";
  source: { url: string; ref: string; sha: string; tree: string };
  population: { half: "o6a"; suiteFiles: string[]; suiteCount: number; caseCount: number };
  main: { file: "main.c"; blobSha256: string; registrations: Array<{ symbol: string; line: number; text: string; fingerprint: string }> };
  suites: InventorySuite[];
  cases: InventoryCase[];
};

export type CoverageRow = {
  id: string;
  status: CaseStatus;
  reason: string;
  execution: { command: string; filter: string; executable: "official-upstream-test" };
};

export type Coverage = {
  schema: "box3d-oracle/coverage/v1";
  inventorySchema: "box3d-oracle/inventory/v1";
  source: { sha: string; tree: string };
  population: { half: "o6a"; suiteCount: number; caseCount: number };
  cases: CoverageRow[];
};

function sha256(value: string): string {
  return createHash("sha256").update(value).digest("hex");
}

function stripComments(text: string): string {
  let result = "";
  let index = 0;
  let state: "code" | "line" | "block" | "string" | "char" = "code";
  while (index < text.length) {
    const current = text[index];
    const next = text[index + 1];
    if (state === "line") {
      if (current === "\n") {
        result += "\n";
        state = "code";
      } else result += " ";
      index += 1;
      continue;
    }
    if (state === "block") {
      if (current === "*" && next === "/") {
        result += "  ";
        index += 2;
        state = "code";
      } else {
        result += current === "\n" ? "\n" : " ";
        index += 1;
      }
      continue;
    }
    if (state === "string" || state === "char") {
      result += current;
      if (current === "\\" && index + 1 < text.length) {
        result += text[index + 1];
        index += 2;
      } else {
        if ((state === "string" && current === '"') || (state === "char" && current === "'")) state = "code";
        index += 1;
      }
      continue;
    }
    if (current === "/" && next === "/") {
      result += "  ";
      index += 2;
      state = "line";
    } else if (current === "/" && next === "*") {
      result += "  ";
      index += 2;
      state = "block";
    } else if (current === '"') {
      result += current;
      index += 1;
      state = "string";
    } else if (current === "'") {
      result += current;
      index += 1;
      state = "char";
    } else {
      result += current;
      index += 1;
    }
  }
  return result;
}

function lineNumber(text: string, offset: number): number {
  return text.slice(0, offset).split("\n").length;
}

function lineText(text: string, offset: number): { line: number; text: string } {
  const start = text.lastIndexOf("\n", offset - 1) + 1;
  const end = text.indexOf("\n", offset);
  return { line: lineNumber(text, offset), text: text.slice(start, end < 0 ? text.length : end).trim() };
}

function matchingBrace(text: string, open: number): number {
  let depth = 0;
  let state: "code" | "line" | "block" | "string" | "char" = "code";
  for (let index = open; index < text.length; index += 1) {
    const current = text[index];
    const next = text[index + 1];
    if (state === "line") {
      if (current === "\n") state = "code";
      continue;
    }
    if (state === "block") {
      if (current === "*" && next === "/") {
        index += 1;
        state = "code";
      }
      continue;
    }
    if (state === "string" || state === "char") {
      if (current === "\\") index += 1;
      else if ((state === "string" && current === '"') || (state === "char" && current === "'")) state = "code";
      continue;
    }
    if (current === "/" && next === "/") {
      index += 1;
      state = "line";
    } else if (current === "/" && next === "*") {
      index += 1;
      state = "block";
    } else if (current === '"') state = "string";
    else if (current === "'") state = "char";
    else if (current === "{") depth += 1;
    else if (current === "}" && --depth === 0) return index;
  }
  throw new InventoryError("unbalanced function body");
}

function functionSource(text: string, symbol: string): { start: number; end: number; line: number; endLine: number } {
  const stripped = stripComments(text);
  const pattern = new RegExp(`(?:^|\\n)[ \\t]*(?:(?:static|inline)\\s+)*(?:int|void|bool)[ \\t]+${symbol}\\s*\\([^)]*\\)[ \\t\\r\\n]*\\{`, "g");
  const matches = [...stripped.matchAll(pattern)];
  if (matches.length !== 1) throw new InventoryError(`expected one source definition for ${symbol}, found ${matches.length}`);
  const start = matches[0].index! + (stripped[matches[0].index!] === "\n" ? 1 : 0);
  const open = stripped.indexOf("{", start);
  const end = matchingBrace(text, open) + 1;
  return { start, end, line: lineNumber(text, start), endLine: lineNumber(text, end - 1) };
}

function macroCalls(text: string, macro: string): Array<{ offset: number; symbol: string; line: number; text: string }> {
  const stripped = stripComments(text);
  const pattern = new RegExp(`\\b${macro}\\s*\\(\\s*([A-Za-z_][A-Za-z0-9_]*)\\s*\\)` , "g");
  return [...stripped.matchAll(pattern)].map((match) => {
    const offset = match.index!;
    const row = lineText(text, offset);
    return { offset, symbol: match[1], line: row.line, text: row.text };
  });
}

function rejectUnknownRegistrationForms(text: string, file: string, allowRunnerMacro = false): void {
  const stripped = stripComments(text);
  const unknown = [...stripped.matchAll(/\b((?:RUN|REGISTER|ADD)_[A-Z][A-Z0-9_]*)\s*\(/g)].filter((match) => match[1] !== "RUN_SUBTEST" && (!allowRunnerMacro || match[1] !== "RUN_TEST"));
  if (unknown.length > 0) throw new InventoryError(`unknown registration form ${unknown[0][1]} in ${file}`);
}

function evidence(kind: "subtest" | "direct", macro: "RUN_SUBTEST" | "MAYBE_RUN_TEST", file: string, line: number, text: string): RegistrationEvidence {
  return { kind, macro, file, line, text, fingerprint: sha256(`${file}:${line}:${text}`) };
}

export function extractSuite(suiteFile: string, source: string, suiteSymbol: string, mainRegistration?: { line: number; text: string }): InventorySuite {
  rejectUnknownRegistrationForms(source, suiteFile);
  const registrations = macroCalls(source, "RUN_SUBTEST");
  const blobSha256 = sha256(source);
  const caseRegistrations = registrations.length > 0
    ? registrations.map((registration) => ({ symbol: registration.symbol, registration: evidence("subtest", "RUN_SUBTEST", suiteFile, registration.line, registration.text) }))
    : [{ symbol: suiteSymbol, registration: evidence("direct", "MAYBE_RUN_TEST", "main.c", mainRegistration?.line ?? 0, mainRegistration?.text ?? `MAYBE_RUN_TEST( ${suiteSymbol} );`) }];
  const cases = caseRegistrations.map(({ symbol, registration }) => {
    const body = functionSource(source, symbol);
    const caseSource = source.slice(body.start, body.end);
    return {
      id: `${suiteFile}::${symbol}`,
      suiteFile,
      suiteSymbol,
      caseSymbol: symbol,
      sourceSymbol: symbol,
      registration,
      source: { line: body.line, endLine: body.endLine, fingerprint: sha256(caseSource) },
      suiteBlobSha256: blobSha256,
    } satisfies InventoryCase;
  });
  return {
    file: suiteFile,
    symbol: suiteSymbol,
    registration: evidence("direct", "MAYBE_RUN_TEST", "main.c", mainRegistration?.line ?? 0, mainRegistration?.text ?? `MAYBE_RUN_TEST( ${suiteSymbol} );`),
    blobSha256,
    cases,
  };
}

function mainRegistrations(source: string): Array<{ symbol: string; line: number; text: string; fingerprint: string }> {
  rejectUnknownRegistrationForms(source, "main.c", true);
  const registrations = macroCalls(source, "MAYBE_RUN_TEST");
  if (registrations.length === 0) throw new InventoryError("main.c has no MAYBE_RUN_TEST registrations");
  return registrations.map((registration) => ({ symbol: registration.symbol, line: registration.line, text: registration.text, fingerprint: sha256(`main.c:${registration.line}:${registration.text}`) }));
}

export function extractInventoryFromSources(sourceRoot: string, sha: string, tree: string, suiteFiles: readonly string[] = O6A_SUITE_FILES): Inventory {
  const testRoot = join(sourceRoot, "test");
  const mainPath = join(testRoot, "main.c");
  if (!existsSync(mainPath)) throw new InventoryError(`official source is missing test/main.c: ${sourceRoot}`);
  const mainSource = readFileSync(mainPath, "utf8");
  const registrations = mainRegistrations(mainSource);
  const bySymbol = new Map(registrations.map((registration) => [registration.symbol, registration]));
  const suites = suiteFiles.map((file) => {
    const symbol = SUITE_SYMBOLS[file];
    if (!symbol) throw new InventoryError(`unknown suite selection: ${file}`);
    const mainRegistration = bySymbol.get(symbol);
    if (!mainRegistration) throw new InventoryError(`suite ${file} is not registered by main.c as ${symbol}`);
    const path = join(testRoot, file);
    if (!existsSync(path)) throw new InventoryError(`registered suite source is missing: ${file}`);
    return extractSuite(file, readFileSync(path, "utf8"), symbol, mainRegistration);
  });
  const cases = suites.flatMap((suite) => suite.cases);
  const ids = new Set<string>();
  for (const item of cases) {
    if (ids.has(item.id)) throw new InventoryError(`duplicate inventory ID: ${item.id}`);
    ids.add(item.id);
  }
  return {
    schema: "box3d-oracle/inventory/v1",
    source: { url: OFFICIAL_SOURCE_URL, ref: OFFICIAL_SOURCE_REF, sha, tree },
    population: { half: "o6a", suiteFiles: suites.map((suite) => suite.file), suiteCount: suites.length, caseCount: cases.length },
    main: { file: "main.c", blobSha256: sha256(mainSource), registrations },
    suites,
    cases,
  };
}

export function classifyInventory(inventory: Inventory): Coverage {
  const cases = inventory.cases.map((item) => ({
    id: item.id,
    status: "not-applicable" as const,
    reason: "No Shallot parity claim is admitted for this upstream case by O6a; the exact case remains runnable through its official suite registration.",
    execution: {
      command: `test ${item.suiteSymbol}`,
      filter: item.suiteSymbol,
      executable: "official-upstream-test" as const,
    },
  }));
  return {
    schema: "box3d-oracle/coverage/v1",
    inventorySchema: "box3d-oracle/inventory/v1",
    source: { sha: inventory.source.sha, tree: inventory.source.tree },
    population: { half: "o6a", suiteCount: inventory.population.suiteCount, caseCount: cases.length },
    cases,
  };
}

export function joinInventoryCoverage(inventory: Inventory, coverage: Coverage): { schema: "box3d-oracle/inventory-coverage/v1"; source: Inventory["source"]; population: Inventory["population"]; cases: Array<InventoryCase & { status: CaseStatus; reason: string; execution: CoverageRow["execution"] }> } {
  if (coverage.schema !== "box3d-oracle/coverage/v1" || coverage.inventorySchema !== inventory.schema) throw new InventoryError("coverage schema does not match the inventory");
  if (coverage.source.sha !== inventory.source.sha || coverage.source.tree !== inventory.source.tree) throw new InventoryError("coverage source does not match the inventory source");
  const inventoryIds = new Set(inventory.cases.map((item) => item.id));
  const seen = new Set<string>();
  for (const row of coverage.cases) {
    if (!row.id || row.id.includes("*") || !row.id.includes("::")) throw new InventoryError(`coverage row is not an exact case ID: ${row.id}`);
    if (!inventoryIds.has(row.id)) throw new InventoryError(`coverage row is not in the generated inventory: ${row.id}`);
    if (seen.has(row.id)) throw new InventoryError(`duplicate coverage row: ${row.id}`);
    if (!CASE_STATUSES.includes(row.status)) throw new InventoryError(`unknown coverage status for ${row.id}: ${row.status}`);
    if (!row.reason.trim()) throw new InventoryError(`coverage row has no reason: ${row.id}`);
    if (row.status === "not-applicable" && (!row.execution?.command || !row.execution.filter || row.execution.executable !== "official-upstream-test")) throw new InventoryError(`not-applicable row is not executable: ${row.id}`);
    seen.add(row.id);
  }
  if (seen.size !== inventoryIds.size || inventory.cases.some((item) => !seen.has(item.id))) throw new InventoryError("inventory and coverage do not form an exact case join");
  const rows = new Map(coverage.cases.map((row) => [row.id, row]));
  return {
    schema: "box3d-oracle/inventory-coverage/v1",
    source: inventory.source,
    population: inventory.population,
    cases: inventory.cases.map((item) => {
      const row = rows.get(item.id)!;
      return { ...item, status: row.status, reason: row.reason, execution: row.execution };
    }),
  };
}

export function writeInventoryArtifacts(inventory: Inventory, inventoryPath: string, coveragePath: string): void {
  const coverage = classifyInventory(inventory);
  joinInventoryCoverage(inventory, coverage);
  writeFileSync(inventoryPath, `${JSON.stringify(inventory, null, 2)}\n`);
  writeFileSync(coveragePath, `${JSON.stringify(coverage, null, 2)}\n`);
}
