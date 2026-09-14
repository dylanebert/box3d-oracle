#!/usr/bin/env bun
import { existsSync, readFileSync } from "node:fs";
import { join } from "node:path";
import { DECLARED_SYMBOLS, O4_DECLARED_SYMBOLS, PATCHES } from "../hooks/patches";
import { ALL_SUITE_FILES, O6A_SUITE_FILES, O6B_SUITE_FILES, diffInventories, joinInventoryCoverage, mergeCoverage, mergeInventories, type Coverage, type Inventory, type InventoryUpdateDiff } from "./inventory";

const required = ["README.md", "package.json", "bin/oracle.ts", "bin/inventory.ts", "bin/check.ts", "bin/compile-scenarios.ts", "test/oracle.test.ts", "test/fixtures/inventory/direct-registration.c", "test/fixtures/inventory/malformed-registration.c", "include/box3d_oracle_adapter.h", "adapter/main.c", "adapter/math.c", "adapter/geometry.c", "adapter/distance.c", "adapter/tree.c", "adapter/manifold.c", "adapter/query.c", "adapter/mover.c", "adapter/writer.c", "adapter/o3_main.c", "adapter/o4_main.c", "adapter/o4.c", "adapter/whitebox.c", "adapter/scenario.c", "hooks/patches.ts", "hooks/oracle_hooks.h", "schema/v1.json", "schema/v6.json", "schema/scenario-command-v1.json", "schema/inventory-v1.json", "schema/coverage-v1.json", "schema/inventory-update-diff-v1.json", "scenarios/commands-v1.json", "inventory/o6a.json", "coverage/o6a.json", "inventory/o6b.json", "coverage/o6b.json", "inventory/current.json", "coverage/current.json", "inventory/update-diff.json"];
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
const surfacesRoster = ["ccd-drop", "ccd-bullet", "mesh-box", "mesh-sphere", "mesh-capsule", "mesh-ccd", "height-box", "height-sphere", "height-capsule", "height-ccd"];
const compoundSensorRoster = ["compound-hull", "compound-capsule", "compound-sphere", "compound-mesh", "compound-ccd", "sensor"];
const benchmarkRoster = ["bench-pyramid", "bench-many-pyramids", "bench-joint-grid", "bench-washer", "bench-large-world", "bench-trees", "bench-junkyard", "bench-rain", "drift"];
const scenarioRoster = [...foundationRoster, ...jointsRoster, ...surfacesRoster, ...compoundSensorRoster, ...benchmarkRoster];
if (scenarioCorpus.schema !== "box3d-oracle/scenario-command/v1" || JSON.stringify(scenarioCorpus.scenarios?.map((scenario) => scenario.name)) !== JSON.stringify(scenarioRoster) || scenarioCorpus.scenarios?.some((scenario) => scenario.id !== `s1.${scenario.name}.v1`)) throw new Error("scenario corpus membership is not the exact cumulative O5d roster/ID join");
if (scenarioCorpus.scenarios?.filter((scenario) => surfacesRoster.includes(scenario.name)).some((scenario) => !scenario.commands.some((command) => command.op === "resource.mesh" || command.op === "resource.height-field" || command.op === "body.create"))) throw new Error("surface scenarios lack explicit geometry/body commands");
if (scenarioCorpus.scenarios?.filter((scenario) => jointsRoster.includes(scenario.name)).some((scenario) => !Array.isArray((scenario as { requiredJointIds?: unknown }).requiredJointIds))) throw new Error("joint scenarios lack required joint references");
if (scenarioCorpus.scenarios?.filter((scenario) => compoundSensorRoster.includes(scenario.name)).some((scenario) => !scenario.commands.some((command) => command.op === "resource.compound" || command.op === "sensor.events"))) throw new Error("compound-sensor scenarios lack explicit compound or event commands");
if (scenarioCorpus.scenarios?.filter((scenario) => benchmarkRoster.includes(scenario.name)).some((scenario) => !scenario.commands.some((command) => ["body.spawn", "body.apply-mass", "body.set-velocity", "body.target-transform", "resource.hull", "joint.filter"].includes(String(command.op)) || command.op === "world.create"))) throw new Error("benchmark scenarios lack explicit scheduled or geometry commands");
if (scenarioCorpus.scenarios?.find((scenario) => scenario.name === "sensor") && !Array.isArray((scenarioCorpus.scenarios.find((scenario) => scenario.name === "sensor") as { requiredSensorEventIds?: unknown }).requiredSensorEventIds)) throw new Error("sensor event schedule is not explicit");
if (JSON.stringify(scenarioCorpus).includes("legacyBuilder") || JSON.stringify(scenarioCorpus).includes("setupKind") || JSON.stringify(scenarioCorpus).includes("hashes") || JSON.stringify(scenarioCorpus).includes("states")) throw new Error("scenario corpus contains forbidden expected-output or name-dispatch data");
const scenarioAdapter = readFileSync(join(import.meta.dir, "..", "adapter/scenario.c"), "utf8");
if (scenarioAdapter.includes("strcmp(record->name") || scenarioAdapter.includes("legacyBuilder") || scenarioAdapter.includes("setupKind")) throw new Error("scenario C adapter contains forbidden name dispatch");
if (!scenarioAdapter.includes("SCENARIO_MESH_RESOURCE") || !scenarioAdapter.includes("SCENARIO_HEIGHT_RESOURCE") || !scenarioAdapter.includes("SCENARIO_COMPOUND_RESOURCE") || !scenarioAdapter.includes("SCENARIO_SENSOR_EVENTS") || !scenarioAdapter.includes("SCENARIO_SPAWN") || !scenarioAdapter.includes("SCENARIO_TARGET_TRANSFORM") || !scenarioAdapter.includes("SCENARIO_HULL_RESOURCE") || !scenarioAdapter.includes("isBullet")) throw new Error("surface and compound-sensor commands are not interpreted");
if (!oracle.includes("scenarioMigrate") || !oracle.includes("serializationOnly") || !oracle.includes("legacySha")) throw new Error("migration-only scenario runner is incomplete");
if (PATCHES.length < 20 || PATCHES.some((patch) => !patch.marker.startsWith("recording.") && !patch.marker.startsWith("solver.") && !patch.marker.startsWith("physics.") && !patch.marker.startsWith("o4."))) throw new Error("O3/O4 hook markers are incomplete");
if (DECLARED_SYMBOLS.length !== 5 || DECLARED_SYMBOLS.some((declared) => !declared.vector.endsWith(".v2"))) throw new Error("O3 declared provenance is incomplete");
if (O4_DECLARED_SYMBOLS.length !== 12 || O4_DECLARED_SYMBOLS.some((declared) => !declared.vector.startsWith("o4."))) throw new Error("O4 declared provenance is incomplete");
const inventoryO6a = JSON.parse(readFileSync(join(import.meta.dir, "..", "inventory/o6a.json"), "utf8")) as Inventory;
const coverageO6a = JSON.parse(readFileSync(join(import.meta.dir, "..", "coverage/o6a.json"), "utf8")) as Coverage;
const inventoryO6b = JSON.parse(readFileSync(join(import.meta.dir, "..", "inventory/o6b.json"), "utf8")) as Inventory;
const coverageO6b = JSON.parse(readFileSync(join(import.meta.dir, "..", "coverage/o6b.json"), "utf8")) as Coverage;
const inventory = JSON.parse(readFileSync(join(import.meta.dir, "..", "inventory/current.json"), "utf8")) as Inventory;
const coverage = JSON.parse(readFileSync(join(import.meta.dir, "..", "coverage/current.json"), "utf8")) as Coverage;
const update = JSON.parse(readFileSync(join(import.meta.dir, "..", "inventory/update-diff.json"), "utf8")) as InventoryUpdateDiff;
if (JSON.stringify(inventoryO6a.population.suiteFiles) !== JSON.stringify(O6A_SUITE_FILES)) throw new Error("O6a inventory suite selection is not allocator-through-id");
if (JSON.stringify(inventoryO6b.population.suiteFiles) !== JSON.stringify(O6B_SUITE_FILES)) throw new Error("O6b inventory suite selection is not joint-through-world");
if (JSON.stringify(inventory.population.suiteFiles) !== JSON.stringify(ALL_SUITE_FILES)) throw new Error("cumulative inventory suite selection is not the exact official roster");
for (const [label, item] of [["O6a", inventoryO6a], ["O6b", inventoryO6b], ["current", inventory]] as const) {
  if (item.population.suiteCount !== item.suites.length || item.population.caseCount !== item.cases.length) throw new Error(`${label} inventory population counts are not derived from its records`);
}
for (const [label, item, expected] of [["O6a", coverageO6a, inventoryO6a], ["O6b", coverageO6b, inventoryO6b], ["current", coverage, inventory]] as const) {
  if (item.population.caseCount !== item.cases.length || item.population.suiteCount !== expected.population.suiteCount) throw new Error(`${label} coverage population counts are not derived from its records`);
}
const joinedO6a = joinInventoryCoverage(inventoryO6a, coverageO6a);
const joinedO6b = joinInventoryCoverage(inventoryO6b, coverageO6b);
const joined = joinInventoryCoverage(inventory, coverage);
if (joinedO6a.cases.length !== inventoryO6a.cases.length || joinedO6b.cases.length !== inventoryO6b.cases.length || joined.cases.length !== inventory.cases.length || joined.cases.some((item) => !item.id.includes("::"))) throw new Error("inventory coverage join is incomplete");
const merged = mergeInventories(inventoryO6a, inventoryO6b);
if (JSON.stringify(merged) !== JSON.stringify(inventory)) throw new Error("current inventory is not the deterministic O6a/O6b join");
if (JSON.stringify(mergeCoverage(coverageO6a, coverageO6b, inventory)) !== JSON.stringify(coverage)) throw new Error("current coverage is not the deterministic O6a/O6b join");
if (JSON.stringify(diffInventories(inventoryO6a, inventory)) !== JSON.stringify(update)) throw new Error("inventory update diff is not derived from the two inventories");
if (update.added.length !== inventoryO6b.cases.length || update.removed.length !== 0 || update.changed.length !== 0 || update.summary.added !== update.added.length || update.summary.removed !== update.removed.length || update.summary.changed !== update.changed.length) throw new Error("inventory update diff does not expose the O6b additions and removals exactly");
if (!oracle.includes("generateBundleV3") || !oracle.includes("generateBundleV6") || !oracle.includes("V6_SCENARIO_SCHEMA") || !oracle.includes("sentinelTestV3") || !oracle.includes("fakeJointJson")) throw new Error("O4/O5f configuration and sentinel lanes are incomplete");
if (!oracle.includes("rejectCopiedBody") || !oracle.includes("verifyPatchSet") || !oracle.includes("B3_ORACLE_SENTINELS")) throw new Error("O3 verifier lanes are incomplete");
console.log("box3d-oracle check: PASS");
