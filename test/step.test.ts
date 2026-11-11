import { afterAll, beforeAll, expect, test } from "bun:test";
import { nextState, OVERFLOW_COLOR, openStepOracle, type StepOracle, type StepResult, type StepScene, type StepState } from "../bin/step";

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
  expect(near.illConditioned).toEqual([{ contact: -1, denom: 0, bound: expect.any(Number), side: false }]);
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
  expect((await at(angle)).illConditioned).toEqual([]);
}, 60_000);
