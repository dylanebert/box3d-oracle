#!/usr/bin/env bun
import { existsSync, readFileSync } from "node:fs";
import { join } from "node:path";
import { DECLARED_SYMBOLS, O4_DECLARED_SYMBOLS, PATCHES } from "../hooks/patches";

const required = ["README.md", "package.json", "bin/oracle.ts", "bin/check.ts", "bin/compile-scenarios.ts", "test/oracle.test.ts", "include/box3d_oracle_adapter.h", "adapter/main.c", "adapter/math.c", "adapter/geometry.c", "adapter/distance.c", "adapter/tree.c", "adapter/manifold.c", "adapter/query.c", "adapter/mover.c", "adapter/writer.c", "adapter/o3_main.c", "adapter/o4_main.c", "adapter/o4.c", "adapter/whitebox.c", "adapter/scenario.c", "hooks/patches.ts", "hooks/oracle_hooks.h", "schema/v1.json", "schema/scenario-command-v1.json", "scenarios/commands-v1.json"];
for (const path of required) {
  if (!existsSync(join(import.meta.dir, "..", path))) throw new Error(`missing required path: ${path}`);
}
const packageJson = JSON.parse(readFileSync(join(import.meta.dir, "..", "package.json"), "utf8")) as {
  scripts?: Record<string, string>;
};
for (const script of ["check", "test", "upstream-test"]) {
  if (!packageJson.scripts?.[script]) throw new Error(`package.json is missing ${script}`);
}
const oracle = readFileSync(join(import.meta.dir, "oracle.ts"), "utf8");
if (!oracle.includes("https://github.com/erincatto/box3d.git")) throw new Error("official source authority is not explicit");
if (oracle.includes("dylanebert/box3d")) throw new Error("fork source is present");
if (oracle.includes("shallot/src") || oracle.includes("gold.json") || oracle.includes("expected output")) throw new Error("generation imports Shallot or expected-output material");
for (const source of ["adapter/math.c", "adapter/geometry.c", "adapter/distance.c", "adapter/tree.c", "adapter/manifold.c", "adapter/query.c", "adapter/mover.c"]) {
  const text = readFileSync(join(import.meta.dir, "..", source), "utf8");
  if (text.includes("#include \"src/") || text.includes("#include <src/")) throw new Error(`private upstream include in ${source}`);
}
const schema = JSON.parse(readFileSync(join(import.meta.dir, "..", "schema/v1.json"), "utf8")) as { $id?: string; description?: string };
if (schema.$id !== "box3d-oracle/v1" || !schema.description?.includes("fixed-width")) throw new Error("integer-preserving schema is incomplete");
const scenarioSchema = JSON.parse(readFileSync(join(import.meta.dir, "..", "schema/scenario-command-v1.json"), "utf8")) as { $id?: string };
if (scenarioSchema.$id !== "box3d-oracle/scenario-command/v1") throw new Error("scenario command schema is incomplete");
const scenarioCorpus = JSON.parse(readFileSync(join(import.meta.dir, "..", "scenarios/commands-v1.json"), "utf8")) as { schema?: string; scenarios?: Array<{ id: string; name: string; commands: Array<Record<string, unknown>> }> };
const foundationRoster = ["free-fall", "sphere-drop", "box-stack", "sphere-sleep", "box-sleep", "wake-drop", "split-slide"];
const jointsRoster = ["revolute-dd", "revolute-pendulum", "revolute-motor", "revolute-limit", "revolute-chain", "weld-dd", "parallel", "joint-contacts", "motor", "motor-spring", "distance", "distance-spring", "prismatic", "prismatic-motor", "spherical", "spherical-limits", "spherical-motor", "wheel", "wheel-spin", "wheel-steer", "ragdoll"];
const scenarioRoster = [...foundationRoster, ...jointsRoster];
if (scenarioCorpus.schema !== "box3d-oracle/scenario-command/v1" || JSON.stringify(scenarioCorpus.scenarios?.map((scenario) => scenario.name)) !== JSON.stringify(scenarioRoster) || scenarioCorpus.scenarios?.some((scenario) => scenario.id !== `s1.${scenario.name}.v1`)) throw new Error("scenario corpus membership is not the exact cumulative O5b roster/ID join");
if (scenarioCorpus.scenarios?.filter((scenario) => jointsRoster.includes(scenario.name)).some((scenario) => !Array.isArray((scenario as { requiredJointIds?: unknown }).requiredJointIds))) throw new Error("joint scenarios lack required joint references");
if (JSON.stringify(scenarioCorpus).includes("legacyBuilder") || JSON.stringify(scenarioCorpus).includes("setupKind") || JSON.stringify(scenarioCorpus).includes("hashes") || JSON.stringify(scenarioCorpus).includes("states")) throw new Error("scenario corpus contains forbidden expected-output or name-dispatch data");
const scenarioAdapter = readFileSync(join(import.meta.dir, "..", "adapter/scenario.c"), "utf8");
if (scenarioAdapter.includes("strcmp(record->name") || scenarioAdapter.includes("legacyBuilder") || scenarioAdapter.includes("setupKind")) throw new Error("scenario C adapter contains forbidden name dispatch");
if (!oracle.includes("scenarioMigrate") || !oracle.includes("serializationOnly") || !oracle.includes("legacySha")) throw new Error("migration-only scenario runner is incomplete");
if (PATCHES.length < 20 || PATCHES.some((patch) => !patch.marker.startsWith("recording.") && !patch.marker.startsWith("solver.") && !patch.marker.startsWith("physics.") && !patch.marker.startsWith("o4."))) throw new Error("O3/O4 hook markers are incomplete");
if (DECLARED_SYMBOLS.length !== 5 || DECLARED_SYMBOLS.some((declared) => !declared.vector.endsWith(".v2"))) throw new Error("O3 declared provenance is incomplete");
if (O4_DECLARED_SYMBOLS.length !== 12 || O4_DECLARED_SYMBOLS.some((declared) => !declared.vector.startsWith("o4."))) throw new Error("O4 declared provenance is incomplete");
if (!oracle.includes("generateBundleV3") || !oracle.includes("sentinelTestV3") || !oracle.includes("fakeJointJson")) throw new Error("O4 configuration and sentinel lanes are incomplete");
if (!oracle.includes("rejectCopiedBody") || !oracle.includes("verifyPatchSet") || !oracle.includes("B3_ORACLE_SENTINELS")) throw new Error("O3 verifier lanes are incomplete");
console.log("box3d-oracle check: PASS");
