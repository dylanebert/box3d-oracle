// Client for the step lane (adapter/step.c): a scene once, then one b3World_Step per request from a
// complete loaded state. Numbers cross as float32 bits; an omitted optional field is upstream's default.
import { buildStepOracle } from "./oracle";

export type Vec3 = [number, number, number];
/** x, y, z, w. */
export type Quat = [number, number, number, number];
export interface Frame { p: Vec3; q: Quat }

/** The lane's world runs without sleep, continuous collision and contact recycling. */
export interface StepWorld {
  gravity: Vec3;
  restitutionThreshold?: number;
  restitutionIterations?: number;
  contactHertz?: number;
  contactDampingRatio?: number;
  contactSpeed?: number;
  maximumLinearSpeed?: number;
  isotropicInertiaTolerance?: number;
  /** MuJoCo's friction mixing: pairs with a shape of this material take the larger friction, others none. */
  maxFrictionMaterial?: bigint;
  timeStep: number;
  subStepCount: number;
}
export interface StepBodyDef {
  type: "static" | "kinematic" | "dynamic";
  position: Vec3;
  rotation: Quat;
  linearDamping?: number;
  angularDamping?: number;
  gravityScale?: number;
  allowFastRotation?: boolean;
}
export type StepShapeDef = {
  body: number;
  density?: number;
  friction?: number;
  restitution?: number;
  rollingResistance?: number;
  groupIndex?: number;
  categoryBits?: bigint;
  maskBits?: bigint;
  userMaterialId?: bigint;
} & ({ kind: "capsule"; center1: Vec3; center2: Vec3; radius: number } | { kind: "sphere"; center: Vec3; radius: number } | { kind: "box"; halfExtents: Vec3 });
type JointBase = { bodyA: number; bodyB: number; localFrameA?: Frame; localFrameB?: Frame; collideConnected?: boolean; constraintHertz?: number; constraintDampingRatio?: number };
export type StepJointDef = JointBase & (
  | { kind: "revolute"; enableSpring?: boolean; hertz?: number; dampingRatio?: number; targetAngle?: number; enableLimit?: boolean; lowerAngle?: number; upperAngle?: number; enableMotor?: boolean; maxMotorTorque?: number; motorSpeed?: number }
  | { kind: "spherical"; enableSpring?: boolean; hertz?: number; dampingRatio?: number; targetRotation?: Quat; enableConeLimit?: boolean; coneAngle?: number; enableTwistLimit?: boolean; lowerTwistAngle?: number; upperTwistAngle?: number; enableMotor?: boolean; maxMotorTorque?: number; motorVelocity?: Vec3 }
  | { kind: "prismatic"; enableSpring?: boolean; hertz?: number; dampingRatio?: number; targetTranslation?: number; enableLimit?: boolean; lowerTranslation?: number; upperTranslation?: number; enableMotor?: boolean; maxMotorForce?: number; motorSpeed?: number }
  | { kind: "filter" }
);
/** Body, shape and joint indices are creation order, which are Box3D's ids. */
export interface StepScene { world: StepWorld; bodies: StepBodyDef[]; shapes: StepShapeDef[]; joints: StepJointDef[] }

/** The warm-start impulses each joint kind carries, in protocol order (b3JointSim fields). */
export const JOINT_IMPULSES = {
  revolute: ["linearImpulse.x", "linearImpulse.y", "linearImpulse.z", "perpImpulse.x", "perpImpulse.y", "springImpulse", "motorImpulse", "lowerImpulse", "upperImpulse"],
  spherical: ["linearImpulse.x", "linearImpulse.y", "linearImpulse.z", "springImpulse.x", "springImpulse.y", "springImpulse.z", "motorImpulse.x", "motorImpulse.y", "motorImpulse.z", "lowerTwistImpulse", "upperTwistImpulse", "swingImpulse"],
  prismatic: ["perpImpulse.x", "perpImpulse.y", "angularImpulse.x", "angularImpulse.y", "angularImpulse.z", "springImpulse", "motorImpulse", "lowerImpulse", "upperImpulse"],
  filter: [],
} as const;

/** 32-bit words in a contact's convex cache, sizeof(b3ContactCache) / 4 at 3362633. */
export const CACHE_WORDS = 4;

/** Box3D's overflow color; a non-touching contact has color -1. */
export const OVERFLOW_COLOR = 23;

/** `origin` is b3BodySim.transform.p, which upstream stores beside the center of mass. */
export interface StepBodyState { origin: Vec3; center: Vec3; rotation: Quat; linearVelocity: Vec3; angularVelocity: Vec3; force?: Vec3; torque?: Vec3 }
export interface StepContactState {
  id: number;
  shapeA: number;
  shapeB: number;
  touching: boolean;
  color: number;
  points: { featureId: number; normalImpulse: number }[];
  frictionImpulse: Vec3;
  twistImpulse: number;
  rollingImpulse: Vec3;
  /** The contact's b3ContactCache as raw 32-bit words (`CACHE_WORDS` of them): the simplex cache that
   * warm-starts b3ShapeDistance for a hull against a sphere or capsule. All zero for a new contact. */
  cache: number[];
}
export interface Aabb { lower: Vec3; upper: Vec3 }
/** Everything a step reads beyond the scene: contacts ascend by id and with the free ids cover the pool. */
export interface StepState {
  bodies: StepBodyState[];
  joints: number[][];
  fatAabbs: Aabb[];
  pool: { next: number; free: number[] };
  contacts: StepContactState[];
  /** Contacts and joints that take the other side of their ill-conditioned decisions this step. */
  flip?: Entity[];
  /** A stage hook: keep bodies' and joints' state after this stage of this substep (0-based) and
   * graph color (omitted for any), returned as the result's `stage`. */
  stop?: { substep: number; stage: SolverStage; color?: number };
}

/** b3SolverStageType's per-substep stages. */
export const SOLVER_STAGES = { velocities: 3, warm: 4, solve: 5, positions: 6, relax: 7 } as const;
export type SolverStage = keyof typeof SOLVER_STAGES;
export interface StageBody { linearVelocity: Vec3; angularVelocity: Vec3; deltaPosition: Vec3; deltaRotation: Quat }

/** A flipped entity's decisions within max(local bound, `window`) of their threshold take the other side. */
export interface Entity { entity: "contact" | "joint"; id: number; window?: number }
/** A hooked decision's closest approach to its threshold in a step, per kind and entity. */
export interface Decision { kind: DecisionKind; id: number; quantity: number; threshold: number; bound: number }
/** The discrete tests the step lane bounds: b3SegmentDistance's parallel test, contact push's
 * speculative switch on separation, joint limits' speculative switch, and a prismatic limit's
 * speculative distance, beyond which it is not solved, and contact relax's rolling resistance clamp. */
export type DecisionKind = "segment" | "separation" | "revolute-lower" | "revolute-upper" | "twist-lower" | "twist-upper" | "cone" | "prismatic-lower" | "prismatic-upper" | "prismatic-lower-speculative" | "prismatic-upper-speculative" | "rolling";
/** A decision whose deciding quantity upstream computed within `bound` (its forward error bound from
 * the site's operands) of the threshold, where a port's legal rounding may land on either side.
 * `id` is a contact's for segment and separation, else a joint's; `side` is upstream's. */
export interface IllConditioned { kind: DecisionKind; id: number; quantity: number; bound: number; side: boolean }
export const entityOf = (d: { kind: DecisionKind; id: number; window?: number }): Entity => ({ entity: d.kind === "segment" || d.kind === "separation" || d.kind === "rolling" ? "contact" : "joint", id: d.id, window: d.window });

export interface StepManifoldPoint { featureId: number; anchorA: Vec3; anchorB: Vec3; separation: number; normalImpulse: number; totalNormalImpulse: number }
export interface StepResultContact {
  id: number;
  shapeA: number;
  shapeB: number;
  touching: boolean;
  color: number;
  cache: number[];
  /** With the contact's mixed materials, which upstream refreshes with every manifold. */
  manifold?: { normal: Vec3; points: StepManifoldPoint[]; frictionImpulse: Vec3; twistImpulse: number; rollingImpulse: Vec3; friction: number; restitution: number; rollingResistance: number };
}
export interface StepResult {
  bodies: { origin: Vec3; center: Vec3; rotation: Quat; linearVelocity: Vec3; angularVelocity: Vec3 }[];
  joints: { color: number; impulses: number[] }[];
  fatAabbs: Aabb[];
  pool: { next: number; free: number[] };
  contacts: StepResultContact[];
  illConditioned: IllConditioned[];
  /** Decisions within ten bounds of their threshold but outside one, for tests of the bounds. */
  near: IllConditioned[];
  /** Every hooked decision's closest approach to its threshold. */
  decisions: Decision[];
  /** With `stop`: awake bodies' state (null for others) and joints' impulses after that stage. */
  stage?: { bodies: (StageBody | null)[]; joints: number[][] };
}
export interface StepMass { invMass: number; localCenter: Vec3; invInertiaLocal: [Vec3, Vec3, Vec3] }

const bits = new DataView(new ArrayBuffer(4));
function f(value: number): string {
  if (!Number.isFinite(value)) throw new Error(`non-finite step input: ${value}`);
  bits.setFloat32(0, value);
  return bits.getUint32(0).toString(16).padStart(8, "0");
}
function unf(word: string): number {
  bits.setUint32(0, Number.parseInt(word, 16));
  return bits.getFloat32(0);
}
const opt = (value: number | undefined) => (value === undefined ? "-" : f(value));
const optBool = (value: boolean | undefined) => (value === undefined ? "-" : value ? "1" : "0");
const optInt = (value: number | undefined) => (value === undefined ? "-" : String(value));
const optU64 = (value: bigint | undefined) => (value === undefined ? "-" : value.toString(16));
const vec = (values: readonly number[]) => values.map(f).join(" ");
const IDENTITY: Frame = { p: [0, 0, 0], q: [0, 0, 0, 1] };
const BODY_TYPES = { static: 0, kinematic: 1, dynamic: 2 } as const;
const SHAPE_KINDS = { capsule: 0, sphere: 1, box: 2 } as const;
const JOINT_KINDS = { revolute: 0, spherical: 1, prismatic: 2, filter: 3 } as const;

export function encodeScene(scene: StepScene): string {
  const w = scene.world;
  const lines = [
    `scene ${scene.bodies.length} ${scene.shapes.length} ${scene.joints.length}`,
    `world ${vec(w.gravity)} ${opt(w.restitutionThreshold)} ${optInt(w.restitutionIterations)} ${opt(w.contactHertz)} ${opt(w.contactDampingRatio)} ${opt(w.contactSpeed)} ${opt(w.maximumLinearSpeed)} ${opt(w.isotropicInertiaTolerance)} ${optU64(w.maxFrictionMaterial)} ${f(w.timeStep)} ${w.subStepCount}`,
  ];
  for (const b of scene.bodies) lines.push(`body ${BODY_TYPES[b.type]} ${vec(b.position)} ${vec(b.rotation)} ${opt(b.linearDamping)} ${opt(b.angularDamping)} ${opt(b.gravityScale)} ${optBool(b.allowFastRotation)}`);
  for (const s of scene.shapes) {
    const geometry = s.kind === "capsule" ? `${vec(s.center1)} ${vec(s.center2)} ${f(s.radius)}` : s.kind === "sphere" ? `${vec(s.center)} ${f(s.radius)}` : vec(s.halfExtents);
    lines.push(`shape ${s.body} ${SHAPE_KINDS[s.kind]} ${geometry} ${opt(s.density)} ${opt(s.friction)} ${opt(s.restitution)} ${opt(s.rollingResistance)} ${optInt(s.groupIndex)} ${optU64(s.categoryBits)} ${optU64(s.maskBits)} ${optU64(s.userMaterialId)}`);
  }
  for (const j of scene.joints) {
    const a = j.localFrameA ?? IDENTITY, b = j.localFrameB ?? IDENTITY;
    let params = "";
    if (j.kind === "revolute") params = [optBool(j.enableSpring), opt(j.hertz), opt(j.dampingRatio), opt(j.targetAngle), optBool(j.enableLimit), opt(j.lowerAngle), opt(j.upperAngle), optBool(j.enableMotor), opt(j.maxMotorTorque), opt(j.motorSpeed)].join(" ");
    else if (j.kind === "spherical") params = [optBool(j.enableSpring), opt(j.hertz), opt(j.dampingRatio), j.targetRotation ? vec(j.targetRotation) : "-", optBool(j.enableConeLimit), opt(j.coneAngle), optBool(j.enableTwistLimit), opt(j.lowerTwistAngle), opt(j.upperTwistAngle), optBool(j.enableMotor), opt(j.maxMotorTorque), j.motorVelocity ? vec(j.motorVelocity) : "-"].join(" ");
    else if (j.kind === "prismatic") params = [optBool(j.enableSpring), opt(j.hertz), opt(j.dampingRatio), opt(j.targetTranslation), optBool(j.enableLimit), opt(j.lowerTranslation), opt(j.upperTranslation), optBool(j.enableMotor), opt(j.maxMotorForce), opt(j.motorSpeed)].join(" ");
    lines.push(`joint ${JOINT_KINDS[j.kind]} ${j.bodyA} ${j.bodyB} ${vec(a.p)} ${vec(a.q)} ${vec(b.p)} ${vec(b.q)} ${optBool(j.collideConnected)} ${opt(j.constraintHertz)} ${opt(j.constraintDampingRatio)} ${params}`.trimEnd());
  }
  lines.push("end");
  return `${lines.join("\n")}\n`;
}

export function encodeState(scene: StepScene, state: StepState): string {
  if (state.bodies.length !== scene.bodies.length || state.joints.length !== scene.joints.length || state.fatAabbs.length !== scene.shapes.length) throw new Error("step state does not match its scene's counts");
  const lines = ["step"];
  const zero: Vec3 = [0, 0, 0];
  for (const b of state.bodies) lines.push(`${vec(b.origin)} ${vec(b.center)} ${vec(b.rotation)} ${vec(b.linearVelocity)} ${vec(b.angularVelocity)} ${vec(b.force ?? zero)} ${vec(b.torque ?? zero)}`);
  state.joints.forEach((impulses, i) => {
    if (impulses.length !== JOINT_IMPULSES[scene.joints[i].kind].length) throw new Error(`joint ${i} carries ${impulses.length} impulses, not ${JOINT_IMPULSES[scene.joints[i].kind].length}`);
    lines.push(vec(impulses));
  });
  for (const a of state.fatAabbs) lines.push(`${vec(a.lower)} ${vec(a.upper)}`);
  lines.push(`pool ${state.pool.next} ${state.pool.free.length} ${state.pool.free.join(" ")}`.trimEnd());
  lines.push(`contacts ${state.contacts.length}`);
  for (const c of state.contacts) {
    const points = c.points.map((p) => `${p.featureId >>> 0} ${f(p.normalImpulse)}`).join(" ");
    if (c.cache.length !== CACHE_WORDS) throw new Error(`contact ${c.id} cache carries ${c.cache.length} words, not ${CACHE_WORDS}`);
    const cache = c.cache.map((word) => (word >>> 0).toString(16).padStart(8, "0")).join(" ");
    lines.push(`${c.id} ${c.shapeA} ${c.shapeB} ${c.touching ? 1 : 0} ${c.color} ${c.points.length} ${points} ${vec(c.frictionImpulse)} ${f(c.twistImpulse)} ${vec(c.rollingImpulse)} ${cache}`.replace(/ +/g, " "));
  }
  const flip = state.flip ?? [];
  lines.push(`flip ${flip.length} ${flip.map((e) => `${e.entity} ${e.id} ${f(e.window ?? 0)}`).join(" ")}`.trimEnd());
  lines.push(state.stop ? `stop ${state.stop.substep} ${SOLVER_STAGES[state.stop.stage]} ${state.stop.color ?? -1}` : "stop -1 0 -1");
  lines.push("end");
  return `${lines.join("\n")}\n`;
}

/** Parses one response, the lines between a step request and `end`. */
export function decodeResult(lines: string[]): StepResult {
  const result: StepResult = { bodies: [], joints: [], fatAabbs: [], pool: { next: 0, free: [] }, contacts: [], illConditioned: [], near: [], decisions: [] };
  for (const line of lines) {
    const [tag, ...words] = line.split(" ");
    let i = 0;
    const num = () => unf(words[i++]);
    const int = () => Number.parseInt(words[i++], 10);
    const v3 = (): Vec3 => [num(), num(), num()];
    if (tag === "body") result.bodies.push({ origin: v3(), center: v3(), rotation: [num(), num(), num(), num()], linearVelocity: v3(), angularVelocity: v3() });
    else if (tag === "joint") { const color = int(); result.joints.push({ color, impulses: words.slice(1).map(unf) }); }
    else if (tag === "decision") result.decisions.push({ kind: words[i++] as DecisionKind, id: int(), quantity: num(), threshold: num(), bound: num() });
    else if (tag === "ill" || tag === "near") (tag === "ill" ? result.illConditioned : result.near).push({ kind: words[i++] as DecisionKind, id: int(), quantity: num(), bound: num(), side: int() === 1 });
    else if (tag === "stagebody" || tag === "stagejoint") {
      result.stage ??= { bodies: [], joints: [] };
      const index = int();
      if (tag === "stagejoint") result.stage.joints[index] = words.slice(1).map(unf);
      else result.stage.bodies[index] = words.length > 1 ? { linearVelocity: v3(), angularVelocity: v3(), deltaPosition: v3(), deltaRotation: [num(), num(), num(), num()] } : null;
    } else if (tag === "fat") result.fatAabbs.push({ lower: v3(), upper: v3() });
    else if (tag === "pool") { const next = int(); const count = int(); result.pool = { next, free: words.slice(2, 2 + count).map(Number) }; }
    else if (tag === "contact") {
      const id = int(), shapeA = int(), shapeB = int(), touching = int() === 1, color = int();
      const cache = Array.from({ length: CACHE_WORDS }, () => Number.parseInt(words[i++], 16));
      const manifoldCount = int();
      if (manifoldCount > 1) throw new Error("the step lane carries convex contacts, one manifold each");
      let manifold: StepResultContact["manifold"];
      if (manifoldCount === 1) {
        const normal = v3();
        const points = Array.from({ length: int() }, () => ({ featureId: Number(words[i++]), anchorA: v3(), anchorB: v3(), separation: num(), normalImpulse: num(), totalNormalImpulse: num() }));
        manifold = { normal, points, frictionImpulse: v3(), twistImpulse: num(), rollingImpulse: v3(), friction: num(), restitution: num(), rollingResistance: num() };
      }
      result.contacts.push({ id, shapeA, shapeB, touching, color, cache, manifold });
    } else throw new Error(`unexpected step oracle line: ${line}`);
  }
  // Upstream visits a color's constraints in an order a rebuilt world need not share.
  const order = (a: IllConditioned, b: IllConditioned) => a.kind.localeCompare(b.kind) || a.id - b.id || a.quantity - b.quantity || a.bound - b.bound;
  result.illConditioned.sort(order);
  result.decisions.sort((a, b) => a.kind.localeCompare(b.kind) || a.id - b.id);
  result.near.sort(order);
  return result;
}

/** The state a result leaves for the next step, with that step's applied forces and torques. */
export function nextState(result: StepResult, applied: { force?: Vec3; torque?: Vec3 }[] = []): StepState {
  const free = new Set(result.pool.free);
  if (result.contacts.length + free.size !== result.pool.next) throw new Error("result contacts and free ids do not cover the pool");
  return {
    bodies: result.bodies.map((b, i) => ({ origin: b.origin, center: b.center, rotation: b.rotation, linearVelocity: b.linearVelocity, angularVelocity: b.angularVelocity, ...applied[i] })),
    joints: result.joints.map((j) => j.impulses),
    fatAabbs: result.fatAabbs,
    pool: result.pool,
    contacts: result.contacts.map((c) => ({
      id: c.id,
      shapeA: c.shapeA,
      shapeB: c.shapeB,
      touching: c.touching,
      color: c.color,
      points: c.manifold?.points.map((p) => ({ featureId: p.featureId, normalImpulse: p.normalImpulse })) ?? [],
      frictionImpulse: c.manifold?.frictionImpulse ?? [0, 0, 0],
      twistImpulse: c.manifold?.twistImpulse ?? 0,
      rollingImpulse: c.manifold?.rollingImpulse ?? [0, 0, 0],
      cache: c.cache,
    })),
  };
}

/** Backward error analysis (Higham): a backward-stable step is the exact step of an input perturbed by
 * about an ulp, so the spread a port may legitimately show is upstream's response to such inputs. The
 * spread runs upstream's step from the state with every continuous input scaled by 1 + u 2^-23, u
 * uniform in [-1, 1) and independent per value, in SPREAD_DIRECTIONS seeded directions fixed here. */
export const SPREAD_DIRECTIONS = 8;
export const SPREAD_SCALE = 2 ** -23;

function random(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** The state with every continuous input (body origins, centers, rotations, velocities, forces and
 * torques, joint impulses, contacts' normal, friction, twist and rolling impulses) perturbed by
 * relative amounts of up to 2^-23 from `seed`. Ids, colors, feature ids, caches, fat AABBs and the
 * pool are discrete or broadphase state and stay as they are. */
export function perturb(state: StepState, seed: number): StepState {
  const next = random(seed);
  const p = (x: number) => Math.fround(x * (1 + (2 * next() - 1) * SPREAD_SCALE));
  const v = (xs: readonly number[]) => xs.map(p);
  return {
    ...state,
    bodies: state.bodies.map((b) => ({ origin: v(b.origin) as Vec3, center: v(b.center) as Vec3, rotation: v(b.rotation) as Quat, linearVelocity: v(b.linearVelocity) as Vec3, angularVelocity: v(b.angularVelocity) as Vec3, force: v(b.force ?? [0, 0, 0]) as Vec3, torque: v(b.torque ?? [0, 0, 0]) as Vec3 })),
    joints: state.joints.map(v),
    contacts: state.contacts.map((c) => ({ ...c, points: c.points.map((q) => ({ featureId: q.featureId, normalImpulse: p(q.normalImpulse) })), frictionImpulse: v(c.frictionImpulse) as Vec3, twistImpulse: p(c.twistImpulse), rollingImpulse: v(c.rollingImpulse) as Vec3 })),
  };
}

/** A result's continuous values by key: `bodies.<i>.<field>.<k>`, `joints.<i>.impulses.<k>`,
 * `fatAabbs.<s>.<lower|upper>.<k>` and `contacts.<id>.manifold...` for a touching contact. */
export function continuousValues(result: StepResult): Map<string, number> {
  const values = new Map<string, number>();
  const put = (key: string, xs: readonly number[]) => xs.forEach((x, k) => values.set(`${key}.${k}`, x));
  result.bodies.forEach((b, i) => {
    for (const field of ["origin", "center", "rotation", "linearVelocity", "angularVelocity"] as const) put(`bodies.${i}.${field}`, b[field]);
  });
  result.joints.forEach((j, i) => put(`joints.${i}.impulses`, j.impulses));
  result.fatAabbs.forEach((a, s) => { put(`fatAabbs.${s}.lower`, a.lower); put(`fatAabbs.${s}.upper`, a.upper); });
  for (const c of result.contacts) {
    const m = c.manifold;
    if (!m) continue;
    const key = `contacts.${c.id}.manifold`;
    put(`${key}.normal`, m.normal);
    m.points.forEach((q, k) => {
      put(`${key}.points.${k}.anchorA`, q.anchorA);
      put(`${key}.points.${k}.anchorB`, q.anchorB);
      for (const field of ["separation", "normalImpulse", "totalNormalImpulse"] as const) values.set(`${key}.points.${k}.${field}`, q[field]);
    });
    put(`${key}.frictionImpulse`, m.frictionImpulse);
    put(`${key}.rollingImpulse`, m.rollingImpulse);
    for (const field of ["twistImpulse", "friction", "restitution", "rollingResistance"] as const) values.set(`${key}.${field}`, m[field]);
  }
  return values;
}

/** Upstream's discrete outcome, which a perturbation within rounding should not change. */
const discrete = (r: StepResult) => JSON.stringify([r.contacts.map((c) => [c.id, c.shapeA, c.shapeB, c.touching, c.color, c.manifold?.points.map((q) => q.featureId)]), r.pool, r.joints.map((j) => j.color)]);

/** A decision ill-conditioned at this state: SPREAD_FACTOR times the spread of its margin over the
 * perturbed runs, or its local bound, reaches its threshold. `window` is the larger of the two. */
export interface Unstable { kind: DecisionKind; id: number; margin: number; bound: number; spread: number; window: number; otherSideRuns: number }
/** The factor on a spread, fixed with SPREAD_DIRECTIONS before any port was held to it. */
export const SPREAD_FACTOR = 10;
/** `spreadAll` is the spread over every perturbed run, those whose discrete outcome changed included. */
export interface Spread { result: StepResult; spread: Map<string, number>; spreadAll: Map<string, number>; discreteChanges: number; unstable: Unstable[] }

export interface StepOracle {
  /** Box3D's mass properties for every body, from the scene's shapes. */
  mass: StepMass[];
  /** The scene as built, before any step. */
  initial: StepResult;
  step(state: StepState): Promise<StepResult>;
  /** The step and its spread: per continuous value, the largest change over the SPREAD_DIRECTIONS
   * perturbed runs whose discrete outcome equals the step's (and over all of them, `spreadAll`), how many
   * runs' did not, and the hooked decisions ill-conditioned by their own margin's spread or local bound,
   * with how many runs took their other side. */
  stepWithSpread(state: StepState): Promise<Spread>;
  /** The scene stepped without reloading, each step's result: what a complete load must reproduce. */
  rollout(steps: number): Promise<StepResult[]>;
  /** One decision hook on supplied operands, as `decide <kind> ...` reads them (adapter/step.c): the
   * value the test then sees (a mask lane for separation, c for limits) and its report. */
  decide(kind: "separation" | "rolling" | "limit" | "translation", operands: number[], flip?: boolean): Promise<{ value: number; illConditioned: IllConditioned[]; near: IllConditioned[] }>;
  /** One upstream b3SegmentDistance, optionally taking the other side of an ill-conditioned decision. */
  segment(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3, flip?: boolean): Promise<{ fraction1: number; fraction2: number; illConditioned: IllConditioned[]; near: IllConditioned[] }>;
  close(): void;
}

/** Builds (once, cached) and starts the step lane for an official Box3D SHA with one scene. */
export async function openStepOracle(sha: string, scene: StepScene): Promise<StepOracle> {
  const executable = buildStepOracle(sha);
  const child = Bun.spawn([executable], { stdin: "pipe", stdout: "pipe", stderr: "inherit" });
  const reader = child.stdout.getReader();
  const decoder = new TextDecoder();
  let buffered = "";
  const pending: string[] = [];
  const line = async (): Promise<string> => {
    while (pending.length === 0) {
      const { value, done } = await reader.read();
      if (done) throw new Error(`step oracle exited (${await child.exited})`);
      buffered += decoder.decode(value, { stream: true });
      const parts = buffered.split("\n");
      buffered = parts.pop()!;
      pending.push(...parts);
    }
    const next = pending.shift()!;
    if (next.startsWith("error ")) throw new Error(`step oracle: ${next.slice(6)}`);
    return next;
  };
  const until = async (end: string) => {
    const lines: string[] = [];
    for (let next = await line(); next !== end; next = await line()) lines.push(next);
    return lines;
  };
  child.stdin.write(encodeScene(scene));
  child.stdin.flush();
  const ready = await until("ready");
  const initial = decodeResult(ready.filter((text) => !text.startsWith("mass ")));
  const mass = ready.filter((text) => text.startsWith("mass ")).map((text) => {
    const words = text.split(" ").slice(1).map(unf);
    const v = (k: number): Vec3 => [words[k], words[k + 1], words[k + 2]];
    return { invMass: words[0], localCenter: v(1), invInertiaLocal: [v(4), v(7), v(10)] as [Vec3, Vec3, Vec3] };
  });
  let queue: Promise<unknown> = Promise.resolve();
  const exchange = <T>(request: string, read: () => Promise<T>): Promise<T> => {
    const run = queue.then(async () => {
      child.stdin.write(request);
      child.stdin.flush();
      return read();
    });
    queue = run.catch(() => undefined);
    return run;
  };
  return {
    mass,
    initial,
    step: (state) => exchange(encodeState(scene, state), async () => decodeResult(await until("end"))),
    async stepWithSpread(state) {
      const run = (s: StepState) => exchange(encodeState(scene, s), async () => decodeResult(await until("end")));
      const result = await run(state);
      const base = continuousValues(result);
      const spread = new Map<string, number>();
      const key = (d: Decision) => `${d.kind} ${d.id}`;
      const margins = new Map(result.decisions.map((d) => [key(d), { decision: d, margin: d.quantity - d.threshold, spread: 0, otherSideRuns: 0 }]));
      const spreadAll = new Map<string, number>();
      let discreteChanges = 0;
      for (let seed = 1; seed <= SPREAD_DIRECTIONS; seed++) {
        const other = await run(perturb(state, seed));
        for (const d of other.decisions) {
          const m = margins.get(key(d));
          if (!m) continue;
          m.spread = Math.max(m.spread, Math.abs(d.quantity - d.threshold - m.margin));
          if (d.quantity - d.threshold > 0 !== m.margin > 0) m.otherSideRuns++;
        }
        for (const [key, x] of continuousValues(other)) {
          const b = base.get(key);
          if (b !== undefined) spreadAll.set(key, Math.max(spreadAll.get(key) ?? 0, Math.abs(x - b)));
        }
        if (discrete(other) !== discrete(result)) {
          discreteChanges++;
          continue;
        }
        for (const [key, x] of continuousValues(other)) {
          const b = base.get(key);
          if (b !== undefined) spread.set(key, Math.max(spread.get(key) ?? 0, Math.abs(x - b)));
        }
      }
      const unstable: Unstable[] = [];
      for (const { decision, margin, spread: s, otherSideRuns } of margins.values()) {
        const window = Math.max(decision.bound, SPREAD_FACTOR * s);
        if (Math.abs(margin) <= window) unstable.push({ kind: decision.kind, id: decision.id, margin, bound: decision.bound, spread: s, window, otherSideRuns });
      }
      return { result, spread, spreadAll, discreteChanges, unstable };
    },
    decide: (kind, operands, flip = false) =>
      exchange(`decide ${kind} ${kind === "separation" || kind === "rolling" ? vec(operands) : `${operands[0]} ${vec(operands.slice(1))}`} ${flip ? 1 : 0}\n`, async () => {
        const lines = await until("end");
        const decisions = decodeResult(lines.slice(0, -1));
        return { value: unf(lines.at(-1)!.split(" ")[1]), illConditioned: decisions.illConditioned, near: decisions.near };
      }),
    segment: (p1, q1, p2, q2, flip = false) =>
      exchange(`segment ${flip ? 1 : 0} ${vec([...p1, ...q1, ...p2, ...q2])}\n`, async () => {
        const lines = await until("end");
        const words = lines.at(-1)!.split(" ");
        const decisions = decodeResult(lines.slice(0, -1));
        return { fraction1: unf(words[1]), fraction2: unf(words[2]), illConditioned: decisions.illConditioned, near: decisions.near };
      }),
    rollout: (steps) =>
      exchange(`rollout ${steps}\n`, async () => {
        const results: StepResult[] = [];
        for (let k = 0; k < steps; k++) results.push(decodeResult(await until("end")));
        return results;
      }),
    close() {
      child.stdin.end();
      child.kill();
    },
  };
}
