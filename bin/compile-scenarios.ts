#!/usr/bin/env bun
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";

type Command = Record<string, unknown>;
type Scenario = { id: string; name: string; commands: Command[]; stepCount: number };
const [input, output] = Bun.argv.slice(2);
if (!input || !output) throw new Error("usage: compile-scenarios.ts <commands-v1.json> <temporary-header.h>");
const bytes = readFileSync(input);
const corpus = JSON.parse(bytes.toString()) as { schema?: string; corpusVersion?: number; scenarios?: Scenario[] };
if (corpus.schema !== "box3d-oracle/scenario-command/v1" || corpus.corpusVersion !== 1 || !Array.isArray(corpus.scenarios)) throw new Error("invalid scenario command corpus");
const opCode: Record<string, number> = { "world.create": 1, "body.create": 2, "resource.box": 3, "resource.sphere": 4, "shape.create": 5, step: 6, observe: 7, hash: 8 };
const names = new Set<string>();
const f32 = (value: unknown): string => {
  if (typeof value !== "string" || !/^0x[0-9a-f]{8}$/.test(value)) throw new Error(`f32 input is not fixed-width hex: ${String(value)}`);
  return value;
};
const words = (values: unknown[], count: number): string[] => {
  if (!Array.isArray(values) || values.length !== count) throw new Error(`expected ${count} f32 words`);
  return values.map(f32);
};
const cString = (value: string): string => JSON.stringify(value);
const arrays: string[] = [];
const records: string[] = [];
for (let si = 0; si < corpus.scenarios.length; si++) {
  const scenario = corpus.scenarios[si];
  if (!scenario.id || !scenario.name || !Array.isArray(scenario.commands) || !Number.isInteger(scenario.stepCount)) throw new Error(`invalid scenario at ${si}`);
  if (names.has(scenario.name) || names.has(scenario.id)) throw new Error(`duplicate scenario ${scenario.name}`);
  names.add(scenario.name); names.add(scenario.id);
  const ids = new Set<string>();
  const rows: string[] = [];
  for (const command of scenario.commands) {
    const op = String(command.op ?? "");
    if (!(op in opCode)) throw new Error(`unknown command op ${op}`);
    const id = String(command.id ?? "");
    if (!id || ids.has(id)) throw new Error(`duplicate or missing command id ${id} in ${scenario.name}`);
    ids.add(id);
    const values = new Array<string>(12).fill("0x00000000");
    let a = -1, b = -1, step = -1, bodyCount = 0, kind = 0;
    let flag0 = 0, flag1 = 0;
    const bodies = new Array<number>(8).fill(-1);
    if (op === "world.create") { words(command.gravity as unknown[], 3).forEach((x, i) => values[i] = x); flag0 = command.enableSleep === true ? 1 : 0; flag1 = command.enableContinuous === true ? 1 : 0; }
    if (op === "body.create") {
      a = Number(String(command.id).slice(1));
      const type = command.type === "static" ? 0 : command.type === "kinematic" ? 1 : command.type === "dynamic" ? 2 : -1;
      if (type < 0) throw new Error(`unknown body type in ${scenario.name}`);
      kind = type; words(command.position as unknown[], 3).forEach((x, i) => values[i] = x); words(command.linearVelocity as unknown[], 3).forEach((x, i) => values[3 + i] = x); words(command.angularVelocity as unknown[], 3).forEach((x, i) => values[6 + i] = x); if (command.angularDamping !== undefined) values[9] = f32(command.angularDamping);
    }
    if (op === "resource.box") { a = Number(String(command.id).slice(1)); words(command.halfExtents as unknown[], 3).forEach((x, i) => values[i] = x); kind = 1; }
    if (op === "resource.sphere") { a = Number(String(command.id).slice(1)); values[0] = f32(command.radius); kind = 2; }
    if (op === "shape.create") { a = Number(String(command.id).slice(1)); b = Number(String(command.body).slice(1)); const r = Number(String(command.resource).slice(1)); if (!Number.isInteger(b) || !Number.isInteger(r)) throw new Error(`invalid shape reference in ${scenario.name}`); values[0] = `0x${r.toString(16).padStart(8, "0")}`; kind = command.kind === "box" ? 1 : command.kind === "sphere" ? 2 : -1; if (kind < 0) throw new Error(`unknown shape kind in ${scenario.name}`); }
    if (op === "step") { step = Number(id.slice(5)); values[0] = f32(command.timeStep); a = Number(command.subStepCount); }
    if (op === "observe") { step = Number(command.step); const list = command.bodies; if (!Array.isArray(list) || list.length > 8) throw new Error(`invalid observation list in ${scenario.name}`); for (const item of list) { const n = Number(String(item).slice(1)); if (!Number.isInteger(n)) throw new Error(`invalid observed body ${String(item)}`); bodies[bodyCount++] = n; } }
    if (op === "hash") step = Number(command.step);
    if ((op === "step" || op === "observe" || op === "hash") && (step < 0 || step >= scenario.stepCount)) throw new Error(`invalid step in ${scenario.name}`);
    rows.push(`  { ${opCode[op]}, ${cString(id)}, ${a}, ${b}, ${kind}, ${step}, ${bodyCount}, ${flag0}, ${flag1}, { ${values.join(", ")} }, { ${bodies.join(", ")} } },`);
  }
  arrays.push(`static const ScenarioCommand scenario_commands_${si}[] = {\n${rows.join("\n")}\n};`);
  records.push(`  { ${cString(scenario.id)}, ${cString(scenario.name)}, scenario_commands_${si}, ${rows.length} },`);
  if (!scenario.commands.some((c) => c.op === "world.create")) throw new Error(`scenario ${scenario.name} has no world.create`);
  for (let step = 0; step < scenario.stepCount; step++) if (!scenario.commands.some((c) => c.op === "observe" && c.step === step) || !scenario.commands.some((c) => c.op === "hash" && c.step === step)) throw new Error(`scenario ${scenario.name} has incomplete observation schedule at ${step}`);
}
const digest = createHash("sha256").update(bytes).digest("hex");
const header = `#ifndef BOX3D_SCENARIO_TABLE_V1_H\n#define BOX3D_SCENARIO_TABLE_V1_H\n#include <stdint.h>\n#define SCENARIO_CORPUS_DIGEST \"${digest}\"\nenum ScenarioOp { SCENARIO_WORLD=1, SCENARIO_BODY=2, SCENARIO_BOX_RESOURCE=3, SCENARIO_SPHERE_RESOURCE=4, SCENARIO_SHAPE=5, SCENARIO_STEP=6, SCENARIO_OBSERVE=7, SCENARIO_HASH=8 };\ntypedef struct ScenarioCommand { int op; const char* id; int a; int b; int kind; int step; int bodyCount; int flag0; int flag1; uint32_t values[12]; int bodies[8]; } ScenarioCommand;\ntypedef struct ScenarioRecord { const char* id; const char* name; const ScenarioCommand* commands; int commandCount; } ScenarioRecord;\n${arrays.join("\n")}\nstatic const ScenarioRecord scenario_records[] = {\n${records.join("\n")}\n};\nstatic const int scenario_record_count = ${corpus.scenarios.length};\n#endif\n`;
writeFileSync(output, header);
console.log(JSON.stringify({ schema: corpus.schema, digest, scenarios: corpus.scenarios.map((s) => ({ id: s.id, name: s.name, commands: s.commands.length })) }));
