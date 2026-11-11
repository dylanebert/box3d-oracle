import { afterAll, beforeAll, expect, test } from "bun:test";
import { entityOf, nextState, occurrenceKey, OVERFLOW_COLOR, openStepOracle, perturb, perturbScene, SPREAD_SCALE, unstableDecisions, type StepOracle, type StepResult, type StepScene, type StepState } from "../bin/step";

// The pinned official Box3D the step lane is held at; building it fetches official main once into the cache.
const SHA = "336263320469e5e9febfe18697ab562488f7e97a";
const STEPS = 240;

const turn = (angle: number): [number, number, number, number] => [0, 0, Math.sin(angle / 2), Math.cos(angle / 2)];
// A ground, a jointed chain of capsules and spheres falling into a pile, a sphere dropped on the chain
// and a bouncing sphere, so
// a rollout has contacts that begin, touch between dynamic bodies, stop and are destroyed, freed
// contact ids reused, and revolute, spherical, prismatic and filter joints warm starting.
const scene: StepScene = {
  world: { gravity: [0, -10, 0], timeStep: 1 / 60, subStepCount: 4 },
  bodies: [
    { type: "static", position: [0, -1, 0], rotation: [0, 0, 0, 1] },
    { type: "dynamic", position: [0, 0.6, 0], rotation: turn(0.1) },
    { type: "dynamic", position: [0.7, 0.9, 0.05], rotation: turn(-0.4) },
    { type: "dynamic", position: [1.3, 1.3, -0.05], rotation: turn(0.9) },
    { type: "dynamic", position: [0.2, 1.6, 0.1], rotation: [0, 0, 0, 1] },
    { type: "dynamic", position: [-1, 2.5, 0], rotation: [0, 0, 0, 1], angularDamping: 0.1 },
    { type: "dynamic", position: [1.3, 0.95, -0.05], rotation: turn(Math.PI / 2) },
    { type: "dynamic", position: [-0.1, 1.1, 0.02], rotation: [0, 0, 0, 1] },
    { type: "dynamic", position: [2, 0.7, 0.3], rotation: [0.2, 0.3, 0.5, Math.sqrt(1 - 0.38)] },
  ],
  shapes: [
    { body: 0, kind: "box", halfExtents: [20, 1, 20], friction: 0.7 },
    { body: 1, kind: "capsule", center1: [-0.3, 0, 0], center2: [0.3, 0, 0], radius: 0.1, rollingResistance: 0.2 },
    { body: 2, kind: "capsule", center1: [-0.25, 0, 0], center2: [0.25, 0, 0], radius: 0.12 },
    { body: 3, kind: "sphere", center: [0, 0, 0], radius: 0.15, density: 500 },
    { body: 4, kind: "sphere", center: [0.05, 0, 0], radius: 0.2 },
    { body: 5, kind: "sphere", center: [0, 0, 0], radius: 0.1, restitution: 0.8 },
    { body: 6, kind: "capsule", center1: [0, -0.1, 0], center2: [0, 0.1, 0], radius: 0.08 },
    { body: 7, kind: "sphere", center: [0, 0, 0], radius: 0.12, friction: 0.4 },
    { body: 8, kind: "capsule", center1: [-0.25, 0.02, 0], center2: [0.25, -0.02, 0.03], radius: 0.07, rollingResistance: 0.1 },
  ],
  joints: [
    { kind: "revolute", bodyA: 1, bodyB: 2, localFrameA: { p: [0.35, 0, 0], q: [0, 0, 0, 1] }, localFrameB: { p: [-0.35, 0, 0], q: [0, 0, 0, 1] }, enableSpring: true, hertz: 2, dampingRatio: 0.5, enableLimit: true, lowerAngle: -0.6, upperAngle: 0.4, enableMotor: true, maxMotorTorque: 1 },
    { kind: "spherical", bodyA: 2, bodyB: 3, localFrameA: { p: [0.3, 0, 0], q: [0, 0, 0, 1] }, localFrameB: { p: [-0.25, 0, 0], q: [0, 0, 0, 1] }, enableSpring: true, hertz: 1, dampingRatio: 0.7, enableConeLimit: true, coneAngle: 0.5, enableTwistLimit: true, lowerTwistAngle: -0.3, upperTwistAngle: 0.3, enableMotor: true, maxMotorTorque: 0.5 },
    { kind: "prismatic", bodyA: 3, bodyB: 6, localFrameA: { p: [-0.35 * Math.sin(0.9), -0.35 * Math.cos(0.9), 0], q: turn(Math.PI / 2 - 0.9) }, enableLimit: true, lowerTranslation: -0.2, upperTranslation: 0.3, enableSpring: true, hertz: 3, dampingRatio: 0.3 },
    { kind: "filter", bodyA: 1, bodyB: 4 },
  ],
};

let oracle: StepOracle;
let native: StepResult[];
beforeAll(async () => {
  oracle = await openStepOracle(SHA, scene);
  native = await oracle.rollout(STEPS);
}, 600_000);
afterAll(() => oracle?.close());

test("a chain of loaded steps reproduces an uninterrupted upstream rollout bit for bit", async () => {
  const kinds = new Set<string>();
  let state = nextState(oracle.initial);
  for (let step = 0; step < STEPS; step++) {
    const result = await oracle.step(state);
    expect(result).toEqual(native[step]);
    for (const c of result.contacts) if (c.touching) kinds.add(c.color < 20 ? "dynamic" : "static");
    state = nextState(result);
  }
  // The rollout exercises what the load carries: both contact kinds, destroyed and reused ids.
  const pairs = new Map<number, Set<string>>();
  for (const r of native) for (const c of r.contacts) pairs.set(c.id, (pairs.get(c.id) ?? new Set()).add(`${c.shapeA}:${c.shapeB}`));
  expect([...kinds].sort()).toEqual(["dynamic", "static"]);
  expect(native.some((r) => r.pool.free.length > 0)).toBe(true);
  expect([...pairs.values()].some((p) => p.size > 1)).toBe(true);
  expect(native.some((r) => r.joints.some((j) => j.impulses.some((x) => x !== 0)))).toBe(true);
}, 120_000);

// Sentinel: every loaded field reaches the step. Each mutation of the state round trip must change
// what upstream computes from it, not only the echo of the mutated field.
test("each loaded field changes the step it feeds", async () => {
  const pressed = (dynamic: boolean) => (c: StepState["contacts"][number]) => c.touching && (c.shapeA !== 0) === dynamic && c.points[0].normalImpulse > 0;
  const index = native.findIndex((r) => { const s = nextState(r); return s.contacts.some(pressed(true)) && s.contacts.some(pressed(false)); });
  expect(index).toBeGreaterThan(0);
  const base = nextState(native[index]);
  const touching = base.contacts.findIndex(pressed(true));
  const grounded = base.contacts.findIndex(pressed(false));
  const reference = await oracle.step(base);
  const dynamics = (r: StepResult) => JSON.stringify([r.bodies, r.contacts.map((c) => [c.id, c.touching, c.manifold?.points.map((p) => p.normalImpulse)])]);
  const mutations: [string, (s: StepState) => void][] = [
    ["body origin", (s) => { s.bodies[1].origin[1] += 1e-3; }],
    ["body center", (s) => { s.bodies[1].center[1] += 1e-3; }],
    ["body rotation", (s) => { s.bodies[2].rotation = turn(0.2); }],
    ["body linear velocity", (s) => { s.bodies[3].linearVelocity[0] += 0.1; }],
    ["body angular velocity", (s) => { s.bodies[3].angularVelocity[2] += 0.1; }],
    ["body force", (s) => { s.bodies[1].force = [5, 0, 0]; }],
    ["body torque", (s) => { s.bodies[1].torque = [0, 0, 1]; }],
    ...[0, 1, 2].flatMap((j) => base.joints[j].map((_, k) => [`joint ${scene.joints[j].kind} impulse ${k}`, (s: StepState) => { s.joints[j][k] += 0.05; }] as [string, (s: StepState) => void])),
    ["fat AABB", (s) => { s.fatAabbs[5].lower = [-20, -20, -20]; s.fatAabbs[5].upper = [20, 20, 20]; }],
    ["contact normal impulse", (s) => { s.contacts[touching].points[0].normalImpulse *= 2; }],
    ["contact feature id", (s) => { s.contacts[grounded].points[0].featureId ^= 0x10000; }],
    ["contact friction impulse", (s) => { s.contacts[grounded].frictionImpulse[0] += 0.05; }],
    ["contact twist impulse", (s) => { s.contacts[grounded].twistImpulse += 0.01; }],
    ["contact rolling impulse", (s) => { s.contacts[grounded].rollingImpulse[2] += 0.01; }],
    ["contact color", (s) => { s.contacts[touching].color = OVERFLOW_COLOR; }],
    ["contact touching", (s) => { Object.assign(s.contacts[touching], { touching: false, color: -1, points: [] }); }],
  ];
  const unchanged: string[] = [];
  for (const [name, mutate] of mutations) {
    const state = structuredClone(base);
    mutate(state);
    let result: StepResult | undefined;
    try {
      result = await oracle.step(state);
    } catch (error) {
      throw new Error(`${name}: ${error}`);
    }
    if (dynamics(result) === dynamics(reference)) unchanged.push(name);
  }
  expect(unchanged).toEqual([]);
  // A cold convex cache changes the simplex upstream's GJK ends on, which the next step reads; in
  // this scene it leaves bodies and manifolds as they are.
  const caches = (r: StepResult) => r.contacts.map((c) => c.cache);
  let coldChanges = 0;
  for (let k = 1; k < STEPS; k++) {
    const cold = nextState(native[k - 1]);
    for (const c of cold.contacts) c.cache = [0, 0, 0, 0];
    if (JSON.stringify(caches(await oracle.step(cold))) !== JSON.stringify(caches(native[k]))) coldChanges++;
  }
  expect(coldChanges).toBeGreaterThan(0);
  // The contact id pool feeds the ids new contacts take.
  const fresh = native.findIndex((r, k) => k > 0 && r.contacts.some((c) => !native[k - 1].contacts.some((p) => p.id === c.id)));
  expect(fresh).toBeGreaterThan(0);
  const before = nextState(native[fresh - 1]);
  const shifted = structuredClone(before);
  // Upstream reuses the last freed id first.
  shifted.pool.free.push(shifted.pool.next, shifted.pool.next + 1);
  shifted.pool.next += 2;
  const ids = (r: StepResult) => r.contacts.map((c) => c.id);
  expect(ids(await oracle.step(shifted))).not.toEqual(ids(await oracle.step(before)));
}, 120_000);

const word = (bits: number) => new Float32Array(new Uint32Array([bits]).buffer)[0];
const ulp32 = (x: number) => {
  const v = new Float32Array([Math.abs(x)]);
  const u = new Uint32Array(v.buffer);
  u[0] += 1;
  return v[0] - Math.fround(Math.abs(x));
};

// Sentinel: the hook at b3SegmentDistance's parallel test reports a decision within rounding of its
// threshold, and the other side it forces is upstream's own arithmetic.
test("an ill-conditioned segment decision is reported and its other side forced", async () => {
  // Human.c's torso capsules in the first body's frame at the ragdoll's first step: a*e and b*b
  // round to the same float, so denom is zero where a contracted a*e - b*b is 2.07e-11.
  const p1: [number, number, number] = [word(0x3d75c28f), 0, word(0xbd5612c7)];
  const q1: [number, number, number] = [word(0xbd75c28f), 0, word(0xbd5612c7)];
  const p2: [number, number, number] = [word(0x3de1479c), word(0xbd9d27a0), word(0xbea1f19a)];
  const q2: [number, number, number] = [word(0xbde147c0), word(0xbd9d274e), word(0xbea1f19c)];
  const near = await oracle.segment(p1, q1, p2, q2);
  expect(near.illConditioned).toEqual([{ kind: "segment", id: -1, quantity: 0, bound: expect.any(Number), side: false }]);
  expect(near.illConditioned[0].bound).toBeGreaterThan(2.07e-11);
  expect(near.fraction1).toBe(0);
  const other = await oracle.segment(p1, q1, p2, q2, true);
  expect(other.fraction1).toBe(1);
}, 60_000);

test("a decision outside the bound is not reported, and one ten bounds away would be", async () => {
  const at = (angle: number) => oracle.segment([0, 0, 0], [1, 0, 0], [0, 1, 0], [Math.cos(angle), 1 + Math.sin(angle), 0]);
  const inside = await at(1e-4);
  expect(inside.illConditioned.length).toBe(1);
  // sin^2 of the angle against ulp(a*e) + ulp(b*b), with a = e = 1 and b = cos(angle).
  const angle = 7e-4;
  const b = Math.fround(Math.cos(angle));
  const denom = Math.fround(1 - Math.fround(b * b));
  const bound = ulp32(1) + ulp32(Math.fround(b * b));
  expect(denom).toBeGreaterThan(bound);
  expect(denom).toBeLessThan(10 * bound);
  const outside = await at(angle);
  expect(outside.illConditioned).toEqual([]);
  expect(outside.near.map((d) => d.kind)).toEqual(["segment"]);
}, 60_000);

const unitQuat = (x: number, y: number, z: number): number[] => [x, y, z, Math.sqrt(1 - x * x - y * y - z * z)];
// Each decision hook on operands where only the deciding quantity moves: within its bound it is
// reported and its other side taken when flipped; at five bounds it is only near, which a bound
// widened tenfold would report.
const U = 2 ** -24;
const sum = (xs: number[]) => xs.reduce((a, b) => a + b, 0);
const absDot = (a: number[], b: number[]) => sum(a.map((x, i) => Math.abs(x * b[i])));
// Leading-order estimates of each bound: unit roundoff times the magnitudes the quantity sums.
const estimates: Record<string, (o: number[]) => number> = {
  rolling: (o) => { const imp = o.slice(5, 8).map(Math.abs); const dw = [0, 1, 2].map((k) => Math.abs(o[14 + k]) + Math.abs(o[17 + k])); const m = o.slice(8, 14).map(Math.abs); const rows = [[m[0], m[1], m[2]], [m[1], m[3], m[4]], [m[2], m[4], m[5]]]; return U * (sum(imp.map((x) => x * x)) + sum(imp.map((x, k) => 2 * x * (absDot(rows[k], dw) + Math.abs(o[2 + k])))) + EPS); },
  separation: (o) => U * (absDot(o.slice(1, 4), o.slice(4, 7).map((x, i) => Math.abs(x) + Math.abs(o[7 + i]))) + absDot(o.slice(24, 27), o.slice(18, 21)) + absDot(o.slice(27, 30), o.slice(21, 24)) + Math.abs(o[30]) + Math.abs(o[31]) + sum(o.slice(18, 24).map(Math.abs))),
  limit: (o) => { const [x, y, z, w] = o.slice(19, 23); const twist = o[0] !== 4; const r = twist ? Math.hypot(z, w) : 1; return U * ((twist ? 2 * (Math.abs(z) + Math.abs(w)) / (r * r) : 4) + Math.abs(o[2])); },
  translation: (o) => 0.4 * U * (absDot(o.slice(37, 40), o.slice(34, 37)) + sum(o.slice(4, 25).map(Math.abs)) + Math.abs(o[2])),
};
const EPS = 2 ** -23;
// A rolling impulse of length near sqrt(FLT_EPSILON) against a zero cap, as a contact that stops pressing.
const rollingOperands = (q: number) => [Math.fround(EPS + q), 0, 2e-4, 1e-4, -1e-4, 1e-4, 7e-5, -3.2e-4, 0.01, 0.001, -0.002, 0.02, 0.003, 0.015, 0.5, -0.2, 0.1, -0.3, 0.4, 0.05];
const hooks: [string, "separation" | "rolling" | "limit" | "translation", (q: number) => number[], (q: number, value: number) => boolean][] = [
  ["rolling", "rolling", rollingOperands, (q, mask) => (mask === 1) === Math.fround(EPS + q) > EPS],
  ["separation", "separation", (q) => [q, 0, 1, 0, 0, 0, 0, 1e-3, -2e-3, 5e-4, 0, 0, 0, 1, ...unitQuat(0.01, 0.002, 0), 0.1, -0.05, 0.02, -0.03, 0.12, 0.01, 0, 1, 0, 0.0199, 0.9998, 0.002, 2e-3, -0.01], (q, mask) => (mask === 1) === q > 0],
  ...([[0, "revolute-lower"], [2, "twist-lower"], [4, "cone"]] as const).map(([index, name]) => [name, "limit", (q: number) => [index, q, -0.5, ...unitQuat(0.003, -0.001, 0.002), ...unitQuat(-0.002, 0.001, 0.004), ...unitQuat(0.1, 0.2, 0.05), ...unitQuat(-0.3, 0.1, 0.2), ...unitQuat(0.05, 0.3, -0.24)], (q: number, c: number) => (c > 0) === q > 0] as [string, "limit", (q: number) => number[], (q: number, value: number) => boolean]),
  ["prismatic-lower", "translation", (q) => [5, q, -1.8, 0.9, 1e-3, 2e-3, 0, -3e-4, 1e-3, 0, 0.2, 1, 0, 0, 0.5, 0, -0.1, -0.4, 0.05, 0, 1, 0, 0, -0.5, 0, ...unitQuat(0.001, 0, 0.002), ...unitQuat(0, 0.003, 0), 1, 0, 0, 1, 0.002, -0.001, 0.3, 1.6, 0.02], (q, c) => (c > 0) === q > 0],
  ["prismatic-lower-speculative", "translation", (q) => [5, Math.fround(0.9 + q), -1.8, 0.9, 1e-3, 2e-3, 0, -3e-4, 1e-3, 0, 0.2, 1, 0, 0, 0.5, 0, -0.1, -0.4, 0.05, 0, 1, 0, 0, -0.5, 0, ...unitQuat(0.001, 0, 0.002), ...unitQuat(0, 0.003, 0), 1, 0, 0, 1, 0.002, -0.001, 0.3, 1.6, 0.02], (q, c) => (c < 0.9) === Math.fround(0.9 + q) < 0.9],
];
for (const [name, kind, operands, sameSide] of hooks) {
  test(`${name}: a decision within its forward error bound is reported and flips; five bounds away is only near`, async () => {
    const probe = await oracle.decide(kind, operands(0));
    expect(probe.illConditioned.map((d) => d.kind)).toEqual([name]);
    const bound = probe.illConditioned[0].bound;
    const estimate = estimates[kind](operands(0)) + (name.endsWith("speculative") ? 0.9 * U : 0);
    expect(bound).toBeGreaterThan(estimate);
    expect(bound).toBeLessThan(10 * estimate);
    const inside = Math.fround(0.5 * bound);
    const kept = await oracle.decide(kind, operands(inside));
    expect(kept.illConditioned.map((d) => d.kind)).toEqual([name]);
    expect(sameSide(inside, kept.value)).toBe(true);
    const flipped = await oracle.decide(kind, operands(inside), true);
    expect(sameSide(inside, flipped.value)).toBe(false);
    const outside = await oracle.decide(kind, operands(Math.fround(5 * bound)), true);
    expect(outside.illConditioned).toEqual([]);
    expect(outside.near.map((d) => d.kind)).toEqual([name]);
    expect(sameSide(Math.fround(5 * bound), outside.value)).toBe(true);
  }, 60_000);
}

// Sentinel: the separation hook sits in upstream's contact push, and its other side changes the step.
test("flipping a contact's ill-conditioned separation changes upstream's step", async () => {
  const index = native.findIndex((r) => r.illConditioned.some((d) => d.kind === "separation"));
  expect(index).toBeGreaterThan(0);
  const state = nextState(native[index - 1]);
  const decision = native[index].illConditioned.find((d) => d.kind === "separation")!;
  const flipped = await oracle.step({ ...state, flip: [{ entity: "contact", id: decision.id }] });
  expect(JSON.stringify(flipped.bodies)).not.toEqual(JSON.stringify(native[index].bodies));
}, 60_000);

// Sentinel: the stage hook keeps the state after the stage it names, within the step.
test("a stop keeps bodies and joints after its stage", async () => {
  const state = nextState(native[100]);
  const atVelocities = await oracle.step({ ...state, stop: { substep: 0, stage: "velocities" } });
  const atPositions = await oracle.step({ ...state, stop: { substep: 0, stage: "positions" } });
  const full = await oracle.step(state);
  expect(atVelocities.bodies).toEqual(full.bodies);
  const moving = atPositions.stage!.bodies.findIndex((b) => b !== null && b.deltaPosition.some((x) => x !== 0));
  expect(moving).toBeGreaterThan(0);
  expect(atVelocities.stage!.bodies[moving]!.deltaPosition).toEqual([0, 0, 0]);
  expect(atPositions.stage!.joints.length).toBe(scene.joints.length);
}, 60_000);

test("a spread perturbs every continuous input by at most 2^-23 and measures upstream's response", async () => {
  const state = nextState(native[120]);
  const a = perturb(state, 3), b = perturb(state, 3), c = perturb(state, 4);
  expect(a).toEqual(b);
  expect(a).not.toEqual(c);
  const flat = (s: StepState) => [...s.bodies.flatMap((x) => [...x.origin, ...x.center, ...x.rotation, ...x.linearVelocity, ...x.angularVelocity]), ...s.joints.flat(), ...s.contacts.flatMap((x) => [...x.points.map((p) => p.normalImpulse), ...x.frictionImpulse, x.twistImpulse, ...x.rollingImpulse])];
  const before = flat(state), after = flat(a);
  expect(after.every((x, i) => Math.abs(x - before[i]) <= 2 * SPREAD_SCALE * Math.abs(before[i]))).toBe(true);
  expect(after.some((x, i) => x !== before[i])).toBe(true);
  expect(a.contacts.map((x) => [x.id, x.color, x.points.map((p) => p.featureId), x.cache])).toEqual(state.contacts.map((x) => [x.id, x.color, x.points.map((p) => p.featureId), x.cache]));
  const { result, spread, spreadAll } = await oracle.stepWithSpread(state);
  expect(result).toEqual(await oracle.step(state));
  for (const [key, value] of spread) expect(spreadAll.get(key)!).toBeGreaterThanOrEqual(value);
  expect(spread.get("bodies.0.center.1")).toBe(0);
  expect(Math.max(...[0, 1, 2].map((k) => spread.get(`bodies.3.linearVelocity.${k}`)!))).toBeGreaterThan(0);
}, 60_000);

test("a spread's runs also perturb every shape's continuous geometry by at most 2^-23, nothing discrete", async () => {
  const a = perturbScene(scene, 3), b = perturbScene(scene, 3), c = perturbScene(scene, 4);
  expect(a).toEqual(b);
  expect(a).not.toEqual(c);
  const geometry = (x: StepScene) => x.shapes.flatMap((s) => (s.kind === "capsule" ? [...s.center1, ...s.center2, s.radius] : s.kind === "sphere" ? [...s.center, s.radius] : [...s.halfExtents]));
  const before = geometry(scene), after = geometry(a);
  expect(after.every((x, i) => Math.abs(x - before[i]) <= 2 * SPREAD_SCALE * Math.abs(before[i]))).toBe(true);
  expect(after.some((x, i) => x !== before[i])).toBe(true);
  const rest = (x: StepScene) => ({ ...x, shapes: x.shapes.map(({ body, kind, density, friction, restitution, rollingResistance }) => ({ body, kind, density, friction, restitution, rollingResistance })) });
  expect(rest(a)).toEqual(rest(scene));
  // The state's and the geometry's draws are independent: the first geometry value is not scaled as
  // the first state value of the same seed.
  const state = nextState(native[120]);
  const scaleState = perturb(state, 3).bodies[0].origin[1] / state.bodies[0].origin[1];
  const scaleGeometry = (a.shapes[0] as { halfExtents: number[] }).halfExtents[0] / 20;
  expect(scaleState).not.toBe(scaleGeometry);
  // A dynamic body's response to geometry alone: the same state on a perturbed scene moves it.
  const lane = await openStepOracle(SHA, a);
  try {
    const moved = await lane.step(state);
    const base = await oracle.step(state);
    expect(moved.bodies.some((x, i) => x.linearVelocity.some((v, k) => v !== base.bodies[i].linearVelocity[k]))).toBe(true);
  } finally {
    lane.close();
  }
}, 60_000);

// Sentinel: every hooked decision reports its closest approach, a decision is unstable when ten times
// its margin's spread over the perturbed runs (or its local bound) reaches its threshold, and a flip
// with that window changes upstream's step.
test("decisions unstable by their spread are reported and flip within their window", async () => {
  let checked = 0;
  for (let k = 1; k < STEPS && checked < 3; k++) {
    const state = nextState(native[k - 1]);
    const { result, unstable } = await oracle.stepWithSpread(state);
    expect(result.decisions.length).toBeGreaterThan(0);
    for (const ill of result.illConditioned) expect(unstable.some((u) => u.kind === ill.kind && u.id === ill.id)).toBe(true);
    for (const u of unstable) expect(u.otherSideRuns).toBeLessThanOrEqual(8);
    const spreadOnly = unstable.find((u) => u.window > u.bound);
    if (!spreadOnly) continue;
    const flipped = await oracle.step({ ...state, flip: [entityOf(spreadOnly)] });
    expect(JSON.stringify(flipped.bodies)).not.toEqual(JSON.stringify(result.bodies));
    checked++;
  }
  expect(checked).toBeGreaterThan(0);
}, 120_000);

// r-rl-humanoid-3's cases. An entity's closest occurrence moves from A to B while every occurrence keeps
// its side: no flip. A manifold that loses its second point in a run: its occurrences match by feature id,
// not visit order, so nothing crosses and the lost ones are unmatched. An occurrence that crosses: a flip.
test("a run takes a decision's other side only when a matched occurrence crosses", () => {
  const d = (substep: number, point: number, quantity: number, bound = 2e-8) => ({ kind: "separation" as const, id: 7, substep, stage: 1, point, quantity, threshold: 0, bound });
  const baseline = [d(1, 0, -1e-8), d(1, 65537, 1.01e-8)];
  expect(unstableDecisions(baseline, [[d(1, 0, -1.02e-8), d(1, 65537, 1.01e-8)]]).unstable.map((u) => u.otherSideRuns)).toEqual([0]);
  const crossed = unstableDecisions(baseline, [[d(1, 0, 1e-9), d(1, 65537, 1.01e-8)]]);
  expect(crossed.unstable.map((u) => [u.otherSideRuns, u.margin])).toEqual([[1, -1e-8]]);
  // The reviewer's probe: two points over four substeps, the run keeping only the first.
  const two = [1, 2, 3, 4].flatMap((s) => [d(s, 0, -1e-8, 1e-12), d(s, 65537, 1e-8, 1e-12)]);
  const one = [1, 2, 3, 4].map((s) => d(s, 0, -1e-8, 1e-12));
  const lost = unstableDecisions(two, [one]);
  expect(lost.unstable).toEqual([]);
  expect(lost.unmatched).toBe(4);
  // An occurrence is not made unstable by another's larger spread.
  const tight = [d(1, 0, -1e-8, 1e-12), d(1, 65537, 1e-3, 1e-12)];
  expect(unstableDecisions(tight, [[d(1, 0, -1.0002e-8, 1e-12), d(1, 65537, 1.001e-3, 1e-12)]]).unstable).toEqual([]);
});

test("occurrence keys are structural: unique in a step, and equal between a step and its rerun", async () => {
  for (const k of [20, 100, 200]) {
    const state = nextState(native[k]);
    const a = await oracle.step(state), b = await oracle.step(state);
    expect(a.decisions.length).toBeGreaterThan(0);
    const keys = (r: StepResult) => r.decisions.map(occurrenceKey);
    expect(new Set(keys(a)).size).toBe(keys(a).length);
    expect(keys(b)).toEqual(keys(a));
    expect(a.decisions.some((x) => x.point >= 0) && a.decisions.some((x) => x.substep > 0)).toBe(true);
  }
}, 60_000);
