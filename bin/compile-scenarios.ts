#!/usr/bin/env bun
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";

type Command = Record<string, unknown>;
type Scenario = { id: string; name: string; commands: Command[]; stepCount: number; requiredJointIds?: string[] };
type Corpus = { schema?: string; corpusVersion?: number; scenarios?: Scenario[] };
const [input, output] = Bun.argv.slice(2);
if (!input || !output) throw new Error("usage: compile-scenarios.ts <commands-v1.json> <temporary-header.h>");
const bytes = readFileSync(input);
const corpus = JSON.parse(bytes.toString()) as Corpus;
if (corpus.schema !== "box3d-oracle/scenario-command/v1" || corpus.corpusVersion !== 1 || !Array.isArray(corpus.scenarios)) throw new Error("invalid scenario command corpus");

const opCode: Record<string, number> = {
  "world.create": 1, "body.create": 2, "resource.box": 3, "resource.sphere": 4, "resource.capsule": 5,
  "resource.mesh": 6, "resource.height-field": 7, "shape.create": 8, "joint.revolute": 9, "joint.weld": 10,
  "joint.parallel": 11, "joint.motor": 12, "joint.distance": 13, "joint.prismatic": 14,
  "joint.spherical": 15, "joint.wheel": 16, step: 17, observe: 18, hash: 19,
};
const jointKinds: Record<string, number> = { revolute: 1, weld: 2, parallel: 3, motor: 4, distance: 5, prismatic: 6, spherical: 7, wheel: 8 };
const jointFields = ["targetAngle", "hertz", "dampingRatio", "lowerAngle", "upperAngle", "maxMotorTorque", "motorSpeed", "maxTorque", "maxVelocityForce", "maxVelocityTorque", "linearHertz", "linearDampingRatio", "maxSpringForce", "angularHertz", "angularDampingRatio", "maxSpringTorque", "length", "minLength", "maxLength", "maxMotorForce", "targetTranslation", "lowerTranslation", "upperTranslation", "coneAngle", "lowerTwistAngle", "upperTwistAngle", "maxSteeringTorque", "targetSteeringAngle", "lowerSteeringLimit", "upperSteeringLimit", "maxSpinTorque", "spinSpeed", "suspensionHertz", "suspensionDampingRatio", "lowerSuspensionLimit", "upperSuspensionLimit", "steeringHertz", "steeringDampingRatio", "linearVelocity", "angularVelocity", "motorVelocity"];
const boolFields = ["enableSpring", "enableLimit", "enableMotor", "enableConeLimit", "enableTwistLimit", "enableSuspensionSpring", "enableSuspensionLimit", "enableSpinMotor", "enableSteering", "enableSteeringLimit"];
const fieldBit = (name: string): string => { const i = jointFields.indexOf(name); if (i < 0) throw new Error(`unknown joint field ${name}`); return `UINT64_C(1) << ${i}`; };
const boolBit = (name: string): string => { const i = boolFields.indexOf(name); if (i < 0) throw new Error(`unknown joint boolean field ${name}`); return `UINT64_C(1) << ${48 + i}`; };
const f32 = (value: unknown): string => { if (typeof value !== "string" || !/^0x[0-9a-f]{8}$/.test(value)) throw new Error(`f32 input is not fixed-width hex: ${String(value)}`); return value; };
const words = (values: unknown[], count: number): string[] => { if (!Array.isArray(values) || values.length !== count) throw new Error(`expected ${count} f32 words`); return values.map(f32); };
const ints = (values: unknown[], count: number): number[] => { if (!Array.isArray(values) || values.length !== count || values.some((x) => !Number.isInteger(x) || (x as number) < 0)) throw new Error(`expected ${count} non-negative integer values`); return values as number[]; };
const cString = (value: string): string => JSON.stringify(value);
const numberId = (value: unknown, prefix: string, scene: string): number => { const match = new RegExp(`^${prefix}([0-9]+)$`).exec(String(value)); if (!match) throw new Error(`invalid ${prefix} id ${String(value)} in ${scene}`); return Number(match[1]); };
const arrays: string[] = [];
const records: string[] = [];
const names = new Set<string>();
for (let si = 0; si < corpus.scenarios.length; si++) {
  const scenario = corpus.scenarios[si];
  if (!scenario.id || !scenario.name || !Array.isArray(scenario.commands) || !Number.isInteger(scenario.stepCount)) throw new Error(`invalid scenario at ${si}`);
  if (names.has(scenario.name) || names.has(scenario.id)) throw new Error(`duplicate scenario ${scenario.name}`);
  names.add(scenario.name); names.add(scenario.id);
  const ids = new Set<string>(); const definedBodies = new Set<number>(); const definedResources = new Set<number>(); const definedJoints = new Set<string>();
  const rows: string[] = [];
  for (let ci = 0; ci < scenario.commands.length; ci++) {
    const command = scenario.commands[ci];
    const op = String(command.op ?? ""); if (!(op in opCode)) throw new Error(`unknown command op ${op}`);
    const id = String(command.id ?? ""); if (!id || ids.has(id)) throw new Error(`duplicate or missing command id ${id} in ${scenario.name}`); ids.add(id);
    const values = new Array<string>(68).fill("0x00000000"); const bodies = new Array<number>(32).fill(-1);
    let a = -1, b = -1, kind = 0, step = -1, bodyCount = 0, substeps = 0; let flags = "UINT64_C(0)";
    let meshPointer = "NULL", heightPointer = "NULL";
    if (op === "world.create") {
      words(command.gravity as unknown[], 3).forEach((x, i) => values[i] = x);
      if (command.enableSleep === true) flags += " | UINT64_C(1) << 62";
      if (command.enableContinuous === true) flags += " | UINT64_C(1) << 63";
    } else if (op === "body.create") {
      a = numberId(id, "b", scenario.name); if (definedBodies.has(a)) throw new Error(`duplicate body ${id}`); definedBodies.add(a);
      kind = command.type === "static" ? 0 : command.type === "kinematic" ? 1 : command.type === "dynamic" ? 2 : -1;
      if (kind < 0) throw new Error(`unknown body type in ${scenario.name}`);
      words(command.position as unknown[], 3).forEach((x, i) => values[i] = x);
      words((command.rotation ?? ["0x00000000", "0x00000000", "0x00000000", "0x3f800000"]) as unknown[], 4).forEach((x, i) => values[3 + i] = x);
      words(command.linearVelocity as unknown[], 3).forEach((x, i) => values[7 + i] = x);
      words(command.angularVelocity as unknown[], 3).forEach((x, i) => values[10 + i] = x);
      values[13] = f32(command.linearDamping ?? "0x00000000"); values[14] = f32(command.angularDamping ?? "0x00000000");
      if (command.isBullet === true) flags += " | UINT64_C(1) << 61";
    } else if (op === "resource.box" || op === "resource.sphere" || op === "resource.capsule" || op === "resource.mesh" || op === "resource.height-field") {
      a = numberId(id, "r", scenario.name); if (definedResources.has(a)) throw new Error(`duplicate resource ${id}`); definedResources.add(a);
      if (op === "resource.box") { kind = 1; words(command.halfExtents as unknown[], 3).forEach((x, i) => values[i] = x); }
      else if (op === "resource.sphere") { kind = 2; values[0] = f32(command.radius); }
      else if (op === "resource.capsule") { kind = 3; words(command.center1 as unknown[], 3).forEach((x, i) => values[i] = x); words(command.center2 as unknown[], 3).forEach((x, i) => values[3 + i] = x); values[6] = f32(command.radius); }
      else if (op === "resource.mesh") {
        kind = 4; const vertices = command.vertices as unknown[]; const indices = ints(command.indices as unknown[], Number(command.triangleCount) * 3);
        if (!Array.isArray(vertices) || vertices.length !== Number(command.vertexCount) * 3) throw new Error(`mesh vertex count mismatch in ${scenario.name}`);
        const name = `scenario_mesh_${si}_${ci}`;
        const vertexWords = words(vertices, vertices.length);
        arrays.push(`static const uint32_t ${name}_vertices[] = { ${vertexWords.join(", ")} };`, `static const int32_t ${name}_indices[] = { ${indices.join(", ")} };`, `static const ScenarioMeshData ${name} = { ${name}_vertices, ${Number(command.vertexCount)}, ${name}_indices, ${Number(command.triangleCount)}, ${command.useMedianSplit === true ? 1 : 0}, ${command.identifyEdges === true ? 1 : 0} };`);
        meshPointer = `&${name}`;
      } else {
        kind = 5; const samples = command.samples as unknown[]; const countX = Number(command.countX); const countZ = Number(command.countZ); const cellCount = (countX - 1) * (countZ - 1);
        if (!Array.isArray(samples) || samples.length !== countX * countZ) throw new Error(`height-field sample count mismatch in ${scenario.name}`);
        const materials = command.materialIndices === undefined ? new Array(cellCount).fill(0) : ints(command.materialIndices as unknown[], cellCount);
        const name = `scenario_height_${si}_${ci}`;
        arrays.push(`static const uint32_t ${name}_samples[] = { ${words(samples, samples.length).join(", ")} };`, `static const uint8_t ${name}_materials[] = { ${materials.join(", ")} };`, `static const ScenarioHeightFieldData ${name} = { ${name}_samples, ${name}_materials, ${countX}, ${countZ}, ${f32(command.scaleX)}, ${f32(command.scaleY)}, ${f32(command.scaleZ)}, ${f32(command.globalMinimumHeight)}, ${f32(command.globalMaximumHeight)}, ${command.clockwiseWinding === true ? 1 : 0} };`);
        heightPointer = `&${name}`;
      }
    } else if (op === "shape.create") {
      const shapeMatch = /^s(?:h)?([0-9]+)$/.exec(id); if (!shapeMatch) throw new Error(`invalid shape id ${id} in ${scenario.name}`);
      a = Number(shapeMatch[1]); b = numberId(command.body, "b", scenario.name); const r = numberId(command.resource, "r", scenario.name);
      if (!definedBodies.has(b) || !definedResources.has(r)) throw new Error(`shape reference before definition in ${scenario.name}`);
      kind = command.kind === "box" ? 1 : command.kind === "sphere" ? 2 : command.kind === "capsule" ? 3 : command.kind === "mesh" ? 4 : command.kind === "height-field" ? 5 : -1;
      if (kind < 0) throw new Error(`unknown shape kind in ${scenario.name}`); values[0] = `0x${r.toString(16).padStart(8, "0")}`; values[1] = f32(command.rollingResistance ?? "0x00000000"); values[2] = `0x${Number(command.groupIndex ?? 0).toString(16).padStart(8, "0")}`;
      words((command.scale ?? ["0x3f800000", "0x3f800000", "0x3f800000"]) as unknown[], 3).forEach((x, i) => values[3 + i] = x);
    } else if (op.startsWith("joint.")) {
      const type = op.slice(6); kind = jointKinds[type]; a = numberId(command.bodyA, "b", scenario.name); b = numberId(command.bodyB, "b", scenario.name); if (!definedBodies.has(a) || !definedBodies.has(b)) throw new Error(`joint ${id} references body before definition in ${scenario.name}`); definedJoints.add(id);
      const fa = command.localFrameA as Record<string, unknown>; const fb = command.localFrameB as Record<string, unknown>; words(fa.p as unknown[], 3).forEach((x, i) => values[i] = x); words(fa.q as unknown[], 4).forEach((x, i) => values[3 + i] = x); words(fb.p as unknown[], 3).forEach((x, i) => values[7 + i] = x); words(fb.q as unknown[], 4).forEach((x, i) => values[10 + i] = x);
      for (const [name, value] of Object.entries(command)) { if (["op", "id", "bodyA", "bodyB", "localFrameA", "localFrameB"].includes(name)) continue; if (typeof value === "boolean") { if (value) flags += ` | ${boolBit(name)}`; } else if (Array.isArray(value)) { words(value, 3).forEach((x, i) => values[56 + ["linearVelocity", "angularVelocity", "motorVelocity"].indexOf(name) * 3 + i] = x); flags += ` | ${fieldBit(name)}`; } else { values[14 + jointFields.indexOf(name)] = f32(value); flags += ` | ${fieldBit(name)}`; } }
    } else if (op === "step") { step = Number(id.slice(5)); if (!Number.isInteger(step)) throw new Error(`invalid step id ${id}`); values[0] = f32(command.timeStep); substeps = Number(command.subStepCount); if (!Number.isInteger(substeps) || substeps < 1) throw new Error(`invalid substep count in ${scenario.name}`); }
    else if (op === "observe") { step = Number(command.step); if (!Number.isInteger(step)) throw new Error(`invalid observation step in ${scenario.name}`); const list = command.bodies; if (!Array.isArray(list) || list.length > 32) throw new Error(`invalid observation list in ${scenario.name}`); for (const item of list) { const body = numberId(item, "b", scenario.name); if (!definedBodies.has(body)) throw new Error(`observation references body before definition in ${scenario.name}`); bodies[bodyCount++] = body; }
    else if (op === "hash") { step = Number(command.step); }
    if (["step", "observe", "hash"].includes(op) && (step < 0 || step >= scenario.stepCount)) throw new Error(`invalid step in ${scenario.name}`);
    rows.push(`  { ${opCode[op]}, ${cString(id)}, ${a}, ${b}, ${kind}, ${step}, ${bodyCount}, ${substeps}, ${flags}, { ${values.join(", ")} }, { ${bodies.join(", ")} }, ${meshPointer}, ${heightPointer} },`);
  }
  const required = scenario.requiredJointIds ?? [];
  if (new Set(required).size !== required.length || definedJoints.size !== required.length || required.some((id) => !definedJoints.has(id))) throw new Error(`required joint reference validation failed in ${scenario.name}`);
  arrays.push(`static const ScenarioCommand scenario_commands_${si}[] = {\n${rows.join("\n")}\n};`);
  records.push(`  { ${cString(scenario.id)}, ${cString(scenario.name)}, scenario_commands_${si}, ${rows.length}, ${required.length} },`);
  if (!scenario.commands.some((c) => c.op === "world.create")) throw new Error(`scenario ${scenario.name} has no world.create`);
  for (let step = 0; step < scenario.stepCount; step++) if (!scenario.commands.some((c) => c.op === "observe" && c.step === step) || !scenario.commands.some((c) => c.op === "hash" && c.step === step)) throw new Error(`incomplete observation schedule at ${scenario.name}:${step}`);
}
const digest = createHash("sha256").update(bytes).digest("hex");
const header = `#ifndef BOX3D_SCENARIO_TABLE_V1_H\n#define BOX3D_SCENARIO_TABLE_V1_H\n#include <stdint.h>\n#include <stddef.h>\n#define SCENARIO_CORPUS_DIGEST "${digest}"\ntypedef struct ScenarioMeshData { const uint32_t* vertices; int vertexCount; const int32_t* indices; int triangleCount; int useMedianSplit; int identifyEdges; } ScenarioMeshData;\ntypedef struct ScenarioHeightFieldData { const uint32_t* samples; const uint8_t* materials; int countX; int countZ; uint32_t scaleX; uint32_t scaleY; uint32_t scaleZ; uint32_t minHeight; uint32_t maxHeight; int clockwiseWinding; } ScenarioHeightFieldData;\nenum ScenarioOp { SCENARIO_WORLD=1, SCENARIO_BODY=2, SCENARIO_BOX_RESOURCE=3, SCENARIO_SPHERE_RESOURCE=4, SCENARIO_CAPSULE_RESOURCE=5, SCENARIO_MESH_RESOURCE=6, SCENARIO_HEIGHT_RESOURCE=7, SCENARIO_SHAPE=8, SCENARIO_REVOLUTE=9, SCENARIO_WELD=10, SCENARIO_PARALLEL=11, SCENARIO_MOTOR=12, SCENARIO_DISTANCE=13, SCENARIO_PRISMATIC=14, SCENARIO_SPHERICAL=15, SCENARIO_WHEEL=16, SCENARIO_STEP=17, SCENARIO_OBSERVE=18, SCENARIO_HASH=19 };\ntypedef struct ScenarioCommand { int op; const char* id; int a; int b; int kind; int step; int bodyCount; int substeps; uint64_t flags; uint32_t values[68]; int bodies[32]; const ScenarioMeshData* mesh; const ScenarioHeightFieldData* heightField; } ScenarioCommand;\ntypedef struct ScenarioRecord { const char* id; const char* name; const ScenarioCommand* commands; int commandCount; int requiredJointCount; } ScenarioRecord;\n${arrays.join("\n")}\nstatic const ScenarioRecord scenario_records[] = {\n${records.join("\n")}\n};\nstatic const int scenario_record_count = ${corpus.scenarios.length};\n#endif\n`;
writeFileSync(output, header);
console.log(JSON.stringify({ schema: corpus.schema, digest, scenarios: corpus.scenarios.map((s) => ({ id: s.id, name: s.name, commands: s.commands.length, joints: (s.requiredJointIds ?? []).length })) }));
