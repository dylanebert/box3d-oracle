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
  /** Contacts that take the other side of an ill-conditioned decision this step. */
  flip?: number[];
}

/** A decision upstream computed within `bound` of its threshold, where a port's legal rounding
 * (a contracted a*e - b*b in b3SegmentDistance) may land on either side. `side` is upstream's. */
export interface IllConditioned { contact: number; denom: number; bound: number; side: boolean }

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
  lines.push(`flip ${flip.length} ${flip.join(" ")}`.trimEnd());
  lines.push("end");
  return `${lines.join("\n")}\n`;
}

/** Parses one response, the lines between a step request and `end`. */
export function decodeResult(lines: string[]): StepResult {
  const result: StepResult = { bodies: [], joints: [], fatAabbs: [], pool: { next: 0, free: [] }, contacts: [], illConditioned: [] };
  for (const line of lines) {
    const [tag, ...words] = line.split(" ");
    let i = 0;
    const num = () => unf(words[i++]);
    const int = () => Number.parseInt(words[i++], 10);
    const v3 = (): Vec3 => [num(), num(), num()];
    if (tag === "body") result.bodies.push({ origin: v3(), center: v3(), rotation: [num(), num(), num(), num()], linearVelocity: v3(), angularVelocity: v3() });
    else if (tag === "joint") { const color = int(); result.joints.push({ color, impulses: words.slice(1).map(unf) }); }
    else if (tag === "ill") result.illConditioned.push({ contact: int(), denom: num(), bound: num(), side: int() === 1 });
    else if (tag === "fat") result.fatAabbs.push({ lower: v3(), upper: v3() });
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

export interface StepOracle {
  /** Box3D's mass properties for every body, from the scene's shapes. */
  mass: StepMass[];
  /** The scene as built, before any step. */
  initial: StepResult;
  step(state: StepState): Promise<StepResult>;
  /** The scene stepped without reloading, each step's result: what a complete load must reproduce. */
  rollout(steps: number): Promise<StepResult[]>;
  /** One upstream b3SegmentDistance, optionally taking the other side of an ill-conditioned decision. */
  segment(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3, flip?: boolean): Promise<{ fraction1: number; fraction2: number; illConditioned: IllConditioned[] }>;
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
    segment: (p1, q1, p2, q2, flip = false) =>
      exchange(`segment ${flip ? 1 : 0} ${vec([...p1, ...q1, ...p2, ...q2])}\n`, async () => {
        const lines = await until("end");
        const words = lines.at(-1)!.split(" ");
        return { fraction1: unf(words[1]), fraction2: unf(words[2]), illConditioned: decodeResult(lines.slice(0, -1)).illConditioned };
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
