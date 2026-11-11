// Client for the step lane (adapter/step.c): a scene once, then one b3World_Step per request from a
// complete loaded state. Numbers cross as float32 bits; an omitted optional field is upstream's default.
import { buildStepOracle, buildStepOracleMca } from "./oracle";
import { ANDERSON_DARLING_05, andersonDarling, chiSquareQuantile, normalQuantile } from "./stats";

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
  /** A stage hook: before this stage of this substep and color, the listed bodies' velocities and
   * substep deltas and the listed contacts' constraint lanes (CONTACT_LANES) take these values; a
   * `stop` then also returns those contacts' lanes after its stage. */
  inject?: { substep: number; stage: SolverStage; color: number; bodies: ({ index: number } & StageBody)[]; contacts: { id: number; lanes: number[] }[] };
}
/** A wide convex contact's lanes, in the order inject and a stage's `contacts` carry them. */
export const CONTACT_LANES = ["normalImpulse0", "normalImpulse1", "totalNormalImpulse0", "totalNormalImpulse1", "frictionImpulseX", "frictionImpulseY", "twistImpulse", "rollingImpulseX", "rollingImpulseY", "rollingImpulseZ", "baseSeparation0", "baseSeparation1",
  // Prepared once per step: the manifold's frame, friction centers and masses, each point's anchors,
  // normal mass and lever arm, and the constraint's body masses and friction.
  "normalX", "normalY", "normalZ", "tangent1X", "tangent1Y", "tangent1Z", "tangent2X", "tangent2Y", "tangent2Z", "centerAX", "centerAY", "centerAZ", "centerBX", "centerBY", "centerBZ", "twistMass", "tangentMassXX", "tangentMassXY", "tangentMassYY", "tangentVelocity1", "tangentVelocity2",
  ...[0, 1].flatMap((k) => ["anchorAX", "anchorAY", "anchorAZ", "anchorBX", "anchorBY", "anchorBZ", "normalMass", "leverArm"].map((f) => `${f}${k}`)),
  "invMassA", "invMassB", "invIAXX", "invIAXY", "invIAXZ", "invIAYY", "invIAYZ", "invIAZZ", "invIBXX", "invIBXY", "invIBXZ", "invIBYY", "invIBYZ", "invIBZZ", "friction"] as const;

/** b3SolverStageType's per-substep stages. */
export const SOLVER_STAGES = { velocities: 3, warm: 4, solve: 5, positions: 6, relax: 7 } as const;
export type SolverStage = keyof typeof SOLVER_STAGES;
export interface StageBody { linearVelocity: Vec3; angularVelocity: Vec3; deltaPosition: Vec3; deltaRotation: Quat }

/** A flipped entity's decisions within max(local bound, `window`) of their threshold take the other side. */
export interface Entity { entity: "contact" | "joint"; id: number; window?: number }
/** One occurrence of a hooked decision in a step, keyed by its place in upstream's loop: kind and
 * entity, substep (0 before the solver's first), stage (0 collide, 1 the biased solve or push, 2 relax)
 * and manifold point by feature id (-1 for a decision per contact or joint; a limit's side is its kind). */
export interface Decision { kind: DecisionKind; id: number; substep: number; stage: number; point: number; quantity: number; threshold: number; bound: number }
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
  /** Every occurrence of a hooked decision. */
  decisions: Decision[];
  /** With `stop`: awake bodies' state (null for others) and joints' impulses after that stage. */
  stage?: { bodies: (StageBody | null)[]; joints: number[][]; contacts: Record<number, number[]> };
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
  const inject = state.inject;
  if (inject) {
    lines.push(`inject ${inject.substep} ${SOLVER_STAGES[inject.stage]} ${inject.color} ${inject.bodies.length}`);
    for (const b of inject.bodies) lines.push(`${b.index} ${vec(b.linearVelocity)} ${vec(b.angularVelocity)} ${vec(b.deltaPosition)} ${vec(b.deltaRotation)}`);
    lines.push(`${inject.contacts.length}`);
    for (const c of inject.contacts) {
      if (c.lanes.length !== CONTACT_LANES.length) throw new Error(`contact ${c.id} carries ${c.lanes.length} lanes, not ${CONTACT_LANES.length}`);
      lines.push(`${c.id} ${vec(c.lanes)}`);
    }
  } else lines.push("inject -1 0 -1 0 0");
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
    else if (tag === "decision") result.decisions.push({ kind: words[i++] as DecisionKind, id: int(), substep: int(), stage: int(), point: int(), quantity: num(), threshold: num(), bound: num() });
    else if (tag === "ill" || tag === "near") (tag === "ill" ? result.illConditioned : result.near).push({ kind: words[i++] as DecisionKind, id: int(), quantity: num(), bound: num(), side: int() === 1 });
    else if (tag === "stagebody" || tag === "stagejoint" || tag === "stagecontact") {
      result.stage ??= { bodies: [], joints: [], contacts: {} };
      const index = int();
      if (tag === "stagecontact") result.stage.contacts[index] = words.slice(1).map(unf);
      else if (tag === "stagejoint") result.stage.joints[index] = words.slice(1).map(unf);
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
  result.decisions.sort((a, b) => a.kind.localeCompare(b.kind) || a.id - b.id || a.substep - b.substep || a.stage - b.stage || a.point - b.point);
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

/** The spread is Monte Carlo arithmetic (Stott Parker): upstream's step run MCA_RUNS times with every
 * float operation of Box3D's source randomly rounded through Verificarlo's MCA backend in random
 * rounding mode at binary32's own 24-bit virtual precision, which rounds each inexact result to one of
 * its two neighbouring floats with probability by distance and leaves exact results exact. sqrtf, the
 * one libm call whose result Box3D's step rounds (remainderf and nextafterf are exact), is randomly
 * rounded the same way by a link-time wrapper (adapter/mca_wrap.c).
 *
 * Bounds follow Sohier et al., "Confidence Intervals for Stochastic Arithmetic" (arXiv 1807.09655),
 * with the round-to-nearest step as reference, Z = run - step. Under the Centered Normality Hypothesis
 * (section 4.1) a quantity's tolerance is |mean Z| + k sigma, sigma the sample deviation of the n runs
 * kept inflated by its one-sided chi-square bound at MCA_CONFIDENCE, sqrt((n - 1) / chi2_{1-c}(n - 1)),
 * and k = familyQuantile(N), the normal quantile at 1 - FAMILY_ALPHA / (2N) for the N quantities a caller
 * compares. Where Anderson-Darling rejects normality at 0.05 (ANDERSON_DARLING_05), or fewer than eight
 * runs are kept, the quantity falls back to the distribution-free (Bernoulli, section 5) range bound,
 * sohierBound of its largest change. Every constant is fixed here before any scene ran.
 *
 * What each bound guarantees is per quantity, as the paper's intervals are: a normal quantity's sigma
 * bound holds at confidence 0.95 and, with k for one quantity, a fresh run at probability 0.95; a
 * fallback quantity's range bound holds a fresh run at probability 0.99 with confidence 0.95 (Table 3).
 * Nothing here is a family-wise guarantee over a scene's schedule: the Bonferroni k on the normal
 * quantities is margin, the fallback quantities carry only their individual bound, and the quantities'
 * independence is not established. Further, the chi-square inflation is per quantity (its 0.95 is not
 * shared across the family), the mean term |mean Z| carries no confidence allowance, and Anderson-Darling
 * not rejecting normality does not certify the tails k reaches into. What the claim cannot guarantee, a
 * null run measures: shallot-gpu-physics's qualify-oracle holds independentRun's results, upstream's
 * rounding alone, through each full schedule and counts its reds. */
export const MCA_CONFIDENCE = 0.95;
export const FAMILY_ALPHA = 0.05;
/** 299 = ceil(ln 0.05 / ln 0.99): Table 3's count for the fallback at probability 0.99, and a row of
 * Table 4, where the chi-square inflation of a deviation is 1.07. */
export const MCA_RUNS = Math.ceil(Math.log(1 - MCA_CONFIDENCE) / Math.log(0.99));
export const MCA_BACKEND = "libinterflop_mca.so --mode=rr --precision-binary32=24 --precision-binary64=53";
/** Processes the runs are spread over, lane k seeded k + 1, run r on lane r mod MCA_LANES: a fixed
 * request sequence draws the same runs. */
export const MCA_LANES = 12;
/** The seed of the one lane `independentRun` draws from, outside the bounds' 1..MCA_LANES. */
export const INDEPENDENT_SEED = 1_000_001;

/** The paper's Bernoulli significance bound 2^-s, s the largest integer with every run's change
 * |Z_i| < 2^-s: the smallest power of two above the largest change, zero when none changed. */
export function sohierBound(largest: number): number {
  if (largest === 0) return 0;
  let bound = 2 ** (Math.floor(Math.log2(largest)) + 1);
  while (bound <= largest) bound *= 2;
  return bound;
}

/** The normal quantile a tolerance multiplies sigma by for `quantities` compared at once. */
export const familyQuantile = (quantities: number) => normalQuantile(1 - FAMILY_ALPHA / (2 * Math.max(1, quantities)));

const inflation = new Map<number, number>();
/** sqrt((n - 1) / chi2_{1-c}(n - 1)): the one-sided upper confidence factor on a deviation from n runs. */
export function deviationInflation(n: number): number {
  let f = inflation.get(n);
  if (f === undefined) inflation.set(n, (f = Math.sqrt((n - 1) / chiSquareQuantile(1 - MCA_CONFIDENCE, n - 1))));
  return f;
}

/** A quantity's spread: under normality |mean| + k sigma; otherwise `range`. `tested` is false for a
 * quantity that no run changed or too few runs kept to test; `normal` is whether the CNH bound applies. */
export interface Bound { mean: number; sigma: number; range: number; normal: boolean; tested: boolean }
export const tolerance = (b: Bound, k: number) => (b.normal ? b.mean + k * b.sigma : b.range);
const MIN_KEPT = 8;

/** The bound of a quantity from its runs' changes: one component per sample row (a scalar's change,
 * a vector's change, a rotation's rotation vector), `largest` the norm or angle the range bounds. */
export function bound(changes: readonly (readonly number[])[], largest: readonly number[]): Bound {
  const n = changes.length;
  const range = sohierBound(Math.max(0, ...largest));
  if (n < MIN_KEPT) return { mean: 0, sigma: 0, range, normal: false, tested: false };
  let variance = 0, tested = false, normal = true;
  const mean = changes[0].map((_, c) => changes.reduce((sum, z) => sum + z[c], 0) / n);
  for (let c = 0; c < mean.length; c++) {
    const column = changes.map((z) => z[c]);
    variance += column.reduce((sum, z) => sum + (z - mean[c]) ** 2, 0) / (n - 1);
    const a = andersonDarling(column);
    if (a === undefined) continue;
    tested = true;
    if (a > ANDERSON_DARLING_05) normal = false;
  }
  return { mean: Math.hypot(...mean), sigma: Math.sqrt(variance) * deviationInflation(n), range, normal, tested };
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

/** A quantity compared by its own geometry: a vector by its Euclidean norm, a rotation by its angle.
 * `members` are its continuous value keys in order; a scalar is not listed. */
export interface Quantity { key: string; members: string[]; rotation: boolean }

/** The scene's vector and rotation quantities in a result: bodies' origin, center and velocities (and
 * rotation), fat AABB corners, joints' vector impulses by JOINT_IMPULSES's names, contacts' normal,
 * anchors, friction and rolling impulses. */
export function quantities(scene: StepScene, result: StepResult): Quantity[] {
  const out: Quantity[] = [];
  const vec = (key: string, n = 3, rotation = false) => out.push({ key, members: Array.from({ length: n }, (_, k) => `${key}.${k}`), rotation });
  result.bodies.forEach((_, i) => {
    for (const field of ["origin", "center", "linearVelocity", "angularVelocity"]) vec(`bodies.${i}.${field}`);
    vec(`bodies.${i}.rotation`, 4, true);
  });
  result.fatAabbs.forEach((_, s) => { vec(`fatAabbs.${s}.lower`); vec(`fatAabbs.${s}.upper`); });
  result.joints.forEach((_, i) => {
    const names: readonly string[] = JOINT_IMPULSES[scene.joints[i].kind];
    const groups = new Map<string, string[]>();
    names.forEach((name, k) => {
      const dot = name.indexOf(".");
      if (dot < 0) return;
      const base = name.slice(0, dot);
      groups.set(base, [...(groups.get(base) ?? []), `joints.${i}.impulses.${k}`]);
    });
    for (const [base, members] of groups) out.push({ key: `joints.${i}.impulses.${base}`, members, rotation: false });
  });
  for (const c of result.contacts) {
    const m = c.manifold;
    if (!m) continue;
    const key = `contacts.${c.id}.manifold`;
    vec(`${key}.normal`);
    m.points.forEach((_, k) => { vec(`${key}.points.${k}.anchorA`); vec(`${key}.points.${k}.anchorB`); });
    vec(`${key}.frictionImpulse`);
    vec(`${key}.rollingImpulse`);
  }
  return out;
}

/** The angle between two unit quaternions, either sign, accurate near zero. */
export function rotationAngle(a: readonly number[], b: readonly number[]): number {
  const s = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] < 0 ? -1 : 1;
  const d = Math.hypot(...a.map((x, k) => x - s * b[k]));
  const t = Math.hypot(...a.map((x, k) => x + s * b[k]));
  return 4 * Math.atan2(d, t);
}

/** Upstream's discrete outcome, which rounding should not change. */
const discrete = (r: StepResult) => JSON.stringify([r.contacts.map((c) => [c.id, c.shapeA, c.shapeB, c.touching, c.color, c.manifold?.points.map((q) => q.featureId)]), r.pool, r.joints.map((j) => j.color)]);

/** A decision ill-conditioned at this state, per kind and entity: for some occurrence, its margin's
 * tolerance over the randomly rounded runs (`bound` at quantile k), or its local bound, reaches its
 * threshold. `margin`, `bound`, `spread` and `window` are the closest such occurrence's;
 * `otherSideRuns` counts the runs in which some occurrence of it ended on the other side of its
 * threshold from the baseline's. */
export interface Unstable { kind: DecisionKind; id: number; margin: number; bound: number; spread: number; window: number; otherSideRuns: number }

/** An occurrence's key: its place in upstream's loop, never its visit order. */
export const occurrenceKey = (d: Decision) => `${d.kind} ${d.id} ${d.substep} ${d.stage} ${d.point}`;

/** The unstable decisions of a baseline step given its randomly rounded runs, occurrence by occurrence.
 * Only occurrences whose keys exist in both the baseline and a run are compared; `unmatched` counts
 * those present in only one, which never count as the other side and never feed a spread. */
export function unstableDecisions(baseline: Decision[], runs: Decision[][], k: number): { unstable: Unstable[]; unmatched: number } {
  const key = occurrenceKey;
  const occurrences = new Map(baseline.map((d) => [key(d), { decision: d, margin: d.quantity - d.threshold, changes: [] as number[] }]));
  if (occurrences.size !== baseline.length) throw new Error("decision occurrences do not have unique keys");
  const otherSide = new Map<string, number>();
  let unmatched = 0;
  for (const run of runs) {
    const crossed = new Set<string>();
    const seen = new Set<string>();
    for (const d of run) {
      const o = occurrences.get(key(d));
      if (!o) {
        unmatched++;
        continue;
      }
      seen.add(key(d));
      const margin = d.quantity - d.threshold;
      o.changes.push(margin - o.margin);
      if (margin > 0 !== o.margin > 0) crossed.add(`${d.kind} ${d.id}`);
    }
    for (const entity of crossed) otherSide.set(entity, (otherSide.get(entity) ?? 0) + 1);
    unmatched += occurrences.size - seen.size;
  }
  const unstable = new Map<string, Unstable>();
  for (const { decision, margin, changes } of occurrences.values()) {
    const spread = tolerance(bound(changes.map((z) => [z]), changes.map(Math.abs)), k);
    const window = Math.max(decision.bound, spread);
    if (Math.abs(margin) > window) continue;
    const entity = `${decision.kind} ${decision.id}`;
    const prior = unstable.get(entity);
    if (prior && Math.abs(prior.margin) <= Math.abs(margin)) {
      prior.window = Math.max(prior.window, window);
      continue;
    }
    unstable.set(entity, { kind: decision.kind, id: decision.id, margin, bound: decision.bound, spread, window: Math.max(window, prior?.window ?? 0), otherSideRuns: otherSide.get(entity) ?? 0 });
  }
  return { unstable: [...unstable.values()], unmatched };
}
/** A quantity's bound by key: `values` per continuous value, `vectors` per vector or rotation quantity
 * (`quantities`), a vector by its change, a rotation by its rotation vector from the step's. */
export interface Bounds { values: Map<string, Bound>; vectors: Map<string, Bound> }
/** `kept` bounds each quantity over the runs whose discrete outcome equals the step's, `runsKept` of
 * them; `all()` over every run, computed on demand. */
export interface Spread { result: StepResult; kept: Bounds; all: () => Bounds; runsKept: number; discreteChanges: number; unstable: Unstable[]; unmatched: number }

/** A rotation's change as a rotation vector, 2 vec(conj(b) a) on the shorter arc. */
function rotationVector(a: readonly number[], b: readonly number[]): number[] {
  const [ax, ay, az, aw] = a, [bx, by, bz, bw] = b;
  const w = bw * aw + bx * ax + by * ay + bz * az;
  const v = [bw * ax - bx * aw - by * az + bz * ay, bw * ay - by * aw - bz * ax + bx * az, bw * az - bz * aw - bx * ay + by * ax];
  return v.map((x) => 2 * (w < 0 ? -x : x));
}

export interface StepOracle {
  /** Box3D's mass properties for every body, from the scene's shapes. */
  mass: StepMass[];
  /** The scene as built, before any step. */
  initial: StepResult;
  step(state: StepState): Promise<StepResult>;
  /** The step and its spread: each quantity's bound over the MCA_RUNS randomly rounded runs whose
   * discrete outcome equals the step's (and over all of them), how many runs' did not, and the hooked
   * decisions ill-conditioned by their margin's tolerance at quantile `k` or their local bound, with how
   * many runs took their other side. */
  stepWithSpread(state: StepState, k: number): Promise<Spread>;
  /** Upstream's step from the state MCA_RUNS times under random rounding, in run order. */
  randomlyRounded(state: StepState): Promise<StepResult[]>;
  /** One randomly rounded step from a lane seeded outside the runs' range (INDEPENDENT_SEED), which no
   * bound saw: a stand-in for a port whose only difference is rounding, for a null run. */
  independentRun(state: StepState): Promise<StepResult>;
  /** sqrtf(x) as the step calls it: here, and on each randomly rounded lane in turn, MCA_RUNS times. */
  sqrtf(x: number): Promise<{ nearest: number; runs: number[] }>;
  /** The scene stepped without reloading, each step's result: what a complete load must reproduce. */
  rollout(steps: number): Promise<StepResult[]>;
  /** One decision hook on supplied operands, as `decide <kind> ...` reads them (adapter/step.c): the
   * value the test then sees (a mask lane for separation, c for limits) and its report. */
  decide(kind: "separation" | "rolling" | "limit" | "translation", operands: number[], flip?: boolean): Promise<{ value: number; illConditioned: IllConditioned[]; near: IllConditioned[] }>;
  /** One upstream b3SegmentDistance, optionally taking the other side of an ill-conditioned decision. */
  segment(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3, flip?: boolean): Promise<{ fraction1: number; fraction2: number; illConditioned: IllConditioned[]; near: IllConditioned[] }>;
  close(): void;
}

/** Which of the randomly rounded lanes' arithmetic and sqrtf round randomly; a test may round either
 * to nearest. */
export interface Rounding { arithmetic: "random" | "nearest"; sqrt: "random" | "nearest" }

/** Builds (once, cached) and starts the step lane for an official Box3D SHA with one scene. The
 * round-to-nearest step runs here; the randomly rounded runs run Verificarlo's build in Docker. */
export async function openStepOracle(sha: string, scene: StepScene, rounding: Rounding = { arithmetic: "random", sqrt: "random" }): Promise<StepOracle> {
  const executable = buildStepOracle(sha);
  const start = async (argv: string[]) => {
    const child = Bun.spawn(argv, { stdin: "pipe", stdout: "pipe", stderr: "inherit" });
    const reader = child.stdout.getReader();
    const decoder = new TextDecoder();
    let buffered = "";
    let pending: string[] = [];
    let cursor = 0;
    const line = async (): Promise<string> => {
      while (cursor === pending.length) {
        const { value, done } = await reader.read();
        if (done) throw new Error(`step oracle exited (${await child.exited})`);
        buffered += decoder.decode(value, { stream: true });
        pending = buffered.split("\n");
        buffered = pending.pop()!;
        cursor = 0;
      }
      const next = pending[cursor++];
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
    const close = () => {
      child.stdin.end();
      child.kill();
    };
    return { ready, until, exchange, close };
  };
  const main = await start([executable]);
  const { until, exchange } = main;
  const ready = main.ready;
  const initial = decodeResult(ready.filter((text) => !text.startsWith("mass ")));
  const mass = ready.filter((text) => text.startsWith("mass ")).map((text) => {
    const words = text.split(" ").slice(1).map(unf);
    const v = (k: number): Vec3 => [words[k], words[k + 1], words[k + 2]];
    return { invMass: words[0], localCenter: v(1), invInertiaLocal: [v(4), v(7), v(10)] as [Vec3, Vec3, Vec3] };
  });
  // The randomly rounded lanes, started at the first spread.
  let stochastic: Awaited<ReturnType<typeof start>>[] | undefined;
  let independent: Awaited<ReturnType<typeof start>> | undefined;
  const startLane = (seed: number) => {
    const tag = buildStepOracleMca(sha);
    const backend = rounding.arithmetic === "random" ? MCA_BACKEND : "libinterflop_mca.so --mode=ieee";
    const env = ["-e", `VFC_BACKENDS=${backend} --seed=${seed}`, "-e", `B3_ORACLE_SQRT=${rounding.sqrt === "random" ? `random:${seed}` : "nearest"}`, "-e", "VFC_BACKENDS_SILENT_LOAD=True", "-e", "VFC_BACKENDS_LOGGER=False"];
    return start(["docker", "run", "--rm", "-i", "--network=none", ...env, tag, "box3d-step-oracle"]);
  };
  const lanes = async () => (stochastic ??= await Promise.all(Array.from({ length: MCA_LANES }, (_, k) => startLane(k + 1))));
  const send = (request: string, lane = main) => lane.exchange(request, async () => decodeResult(await lane.until("end")));
  const run = (s: StepState, lane = main) => send(encodeState(scene, s), lane);
  const randomlyRounded = async (state: StepState) => {
    const all = await lanes();
    const request = encodeState(scene, state);
    return Promise.all(Array.from({ length: MCA_RUNS }, (_, r) => send(request, all[r % MCA_LANES])));
  };
  return {
    mass,
    initial,
    step: (state) => run(state),
    randomlyRounded,
    async independentRun(state) {
      return run(state, (independent ??= await startLane(INDEPENDENT_SEED)));
    },
    async sqrtf(x) {
      const ask = (lane: typeof main) => lane.exchange(`sqrtf ${f(x)}\n`, async () => unf((await lane.until("end"))[0].split(" ")[1]));
      const all = await lanes();
      const [nearest, ...runs] = await Promise.all([ask(main), ...Array.from({ length: MCA_RUNS }, (_, r) => ask(all[r % MCA_LANES]))]);
      return { nearest, runs };
    },
    async stepWithSpread(state, k) {
      const [result, others] = await Promise.all([run(state), randomlyRounded(state)]);
      const base = continuousValues(result);
      const groups = quantities(scene, result);
      const kept = others.filter((other) => discrete(other) === discrete(result));
      const bounds = (set: StepResult[]): Bounds => {
        const values = new Map<string, Bound>();
        const vectors = new Map<string, Bound>();
        const runs = set.map(continuousValues);
        for (const [key, b] of base) {
          const z = runs.map((v) => v.get(key)).filter((x) => x !== undefined).map((x) => [x! - b]);
          values.set(key, bound(z.length === set.length ? z : [], z.map((x) => Math.abs(x[0]))));
        }
        for (const g of groups) {
          const b = g.members.map((m) => base.get(m)!);
          const xs = runs.map((v) => g.members.map((m) => v.get(m))).filter((x) => x.every((y) => y !== undefined)) as number[][];
          const changes = xs.map((x) => (g.rotation ? rotationVector(x, b) : x.map((y, c) => y - b[c])));
          const largest = xs.map((x, i) => (g.rotation ? rotationAngle(x, b) : Math.hypot(...changes[i])));
          vectors.set(g.key, bound(xs.length === set.length ? changes : [], largest));
        }
        return { values, vectors };
      };
      const { unstable, unmatched } = unstableDecisions(result.decisions, others.map((o) => o.decisions), k);
      return { result, kept: bounds(kept), all: () => bounds(others), runsKept: kept.length, discreteChanges: others.length - kept.length, unstable, unmatched };
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
      main.close();
      for (const lane of stochastic ?? []) lane.close();
      independent?.close();
    },
  };
}
