// Step oracle: loads the state one b3World_Step reads into upstream structs, steps once and
// writes the state after it. Protocol: whitespace tokens on stdin and stdout, floats as eight
// hexadecimal digits of their bits, "-" for an upstream default. See bin/step.ts.
#define _POSIX_C_SOURCE 200809L
#include <float.h>
#include <math.h>

#include "bitset.h"
#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "island.h"
#include "joint.h"
#include "oracle_hooks.h"
#include "physics_world.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ITEMS 1024

typedef struct Scene
{
	b3WorldDef world;
	float timeStep;
	int subStepCount;
	int bodyCount, shapeCount, jointCount;
	b3BodyDef bodies[MAX_ITEMS];
	b3ShapeDef shapeDefs[MAX_ITEMS];
	int shapeBody[MAX_ITEMS], shapeKind[MAX_ITEMS];
	b3Capsule capsules[MAX_ITEMS];
	b3Sphere spheres[MAX_ITEMS];
	b3BoxHull boxes[MAX_ITEMS];
	int jointKind[MAX_ITEMS];
	b3RevoluteJointDef revolute[MAX_ITEMS];
	b3SphericalJointDef spherical[MAX_ITEMS];
	b3PrismaticJointDef prismatic[MAX_ITEMS];
	b3FilterJointDef filter[MAX_ITEMS];
	b3JointDef* base[MAX_ITEMS];
	int jointBodyA[MAX_ITEMS], jointBodyB[MAX_ITEMS];
} Scene;

enum { JOINT_REVOLUTE, JOINT_SPHERICAL, JOINT_PRISMATIC, JOINT_FILTER };
enum { SHAPE_CAPSULE, SHAPE_SPHERE, SHAPE_BOX };

static Scene scene;
static FILE* out;
static uint64_t maxFrictionMaterial;
static float maxFrictionAgainstMaterial(float frictionA, uint64_t materialA, float frictionB, uint64_t materialB)
{
	return materialA == maxFrictionMaterial || materialB == maxFrictionMaterial ? b3MaxFloat(frictionA, frictionB) : 0.0f;
}
static char token[256];

static void fail(const char* message)
{
	fprintf(out, "error %s\n", message);
	fflush(out);
	exit(3);
}

static const char* next(void)
{
	if (scanf("%255s", token) != 1) fail("unexpected end of input");
	return token;
}
static int isDefault(void) { return strcmp(token, "-") == 0; }
static void expect(const char* word)
{
	if (strcmp(next(), word) != 0) fail(word);
}
static float asFloat(const char* text)
{
	char* end = NULL;
	uint32_t bits = (uint32_t)strtoul(text, &end, 16);
	if (end == text || *end || strlen(text) != 8) fail("float is not eight hexadecimal digits");
	float value;
	memcpy(&value, &bits, sizeof(value));
	return value;
}
static float readFloat(void) { return asFloat(next()); }
static int readInt(void)
{
	char* end = NULL;
	long value = strtol(next(), &end, 10);
	if (end == token || *end) fail("integer expected");
	return (int)value;
}
static void optFloat(float* field) { next(); if (!isDefault()) *field = asFloat(token); }
static void optBool(bool* field) { next(); if (!isDefault()) *field = strcmp(token, "1") == 0; }
static void optU64(uint64_t* field)
{
	next();
	if (isDefault()) return;
	char* end = NULL;
	*field = strtoull(token, &end, 16);
	if (end == token || *end) fail("u64 expected");
}
static b3Vec3 readVec3(void)
{
	b3Vec3 v;
	v.x = readFloat(); v.y = readFloat(); v.z = readFloat();
	return v;
}
static b3Quat readQuat(void)
{
	b3Quat q;
	q.v = readVec3(); q.s = readFloat();
	return q;
}
static b3Transform readTransform(void)
{
	b3Transform t;
	t.p = readVec3(); t.q = readQuat();
	return t;
}

static void putFloat(float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	fprintf(out, " %08x", bits);
}
static void putVec3(b3Vec3 v) { putFloat(v.x); putFloat(v.y); putFloat(v.z); }
static void putQuat(b3Quat q) { putVec3(q.v); putFloat(q.s); }

// world gx gy gz restitutionThreshold restitutionIterations contactHertz contactDampingRatio contactSpeed
//   maximumLinearSpeed isotropicInertiaTolerance maxFrictionMaterial timeStep subStepCount
static void readWorld(void)
{
	expect("world");
	b3WorldDef* d = &scene.world;
	*d = b3DefaultWorldDef();
	d->gravity = readVec3();
	optFloat(&d->restitutionThreshold);
	next(); if (!isDefault()) d->restitutionIterations = atoi(token);
	optFloat(&d->contactHertz);
	optFloat(&d->contactDampingRatio);
	optFloat(&d->contactSpeed);
	optFloat(&d->maximumLinearSpeed);
	optFloat(&d->isotropicInertiaTolerance);
	// MuJoCo's mixing for scenes translated from it: the larger friction against one material,
	// none between other shapes (frictionless condim-1 geoms).
	next();
	if (!isDefault())
	{
		char* end = NULL;
		maxFrictionMaterial = strtoull(token, &end, 16);
		if (end == token || *end) fail("u64 expected");
		d->frictionCallback = maxFrictionAgainstMaterial;
	}
	// Sleep, continuous collision and contact recycling read state the lane does not carry: sleep
	// times and islands, sweeps, and contacts' cached relative poses.
	d->enableSleep = false;
	d->enableContinuous = false;
	d->workerCount = 1;
	scene.timeStep = readFloat();
	scene.subStepCount = readInt();
}

// body type px py pz qx qy qz qw linearDamping angularDamping gravityScale allowFastRotation
static void readBody(int i)
{
	expect("body");
	b3BodyDef* d = &scene.bodies[i];
	*d = b3DefaultBodyDef();
	d->type = (b3BodyType)readInt();
	b3Vec3 p = readVec3();
	d->position = (b3Pos){ p.x, p.y, p.z };
	d->rotation = readQuat();
	optFloat(&d->linearDamping);
	optFloat(&d->angularDamping);
	optFloat(&d->gravityScale);
	optBool(&d->allowFastRotation);
	d->enableSleep = false;
}

// shape body kind geometry... density friction restitution rollingResistance groupIndex categoryBits maskBits
//   userMaterialId
//   capsule: c1(3) c2(3) radius; sphere: center(3) radius; box: halfExtents(3)
static void readShape(int i)
{
	expect("shape");
	scene.shapeBody[i] = readInt();
	if (scene.shapeBody[i] < 0 || scene.shapeBody[i] >= scene.bodyCount) fail("shape body out of range");
	scene.shapeKind[i] = readInt();
	switch (scene.shapeKind[i])
	{
		case SHAPE_CAPSULE:
			scene.capsules[i].center1 = readVec3();
			scene.capsules[i].center2 = readVec3();
			scene.capsules[i].radius = readFloat();
			break;
		case SHAPE_SPHERE:
			scene.spheres[i].center = readVec3();
			scene.spheres[i].radius = readFloat();
			break;
		case SHAPE_BOX:
		{
			b3Vec3 h = readVec3();
			scene.boxes[i] = b3MakeBoxHull(h.x, h.y, h.z);
			break;
		}
		default: fail("unknown shape kind");
	}
	b3ShapeDef* d = &scene.shapeDefs[i];
	*d = b3DefaultShapeDef();
	optFloat(&d->density);
	optFloat(&d->baseMaterial.friction);
	optFloat(&d->baseMaterial.restitution);
	optFloat(&d->baseMaterial.rollingResistance);
	next(); if (!isDefault()) d->filter.groupIndex = atoi(token);
	optU64(&d->filter.categoryBits);
	optU64(&d->filter.maskBits);
	optU64(&d->baseMaterial.userMaterialId);
}

// joint kind bodyA bodyB frameA(7) frameB(7) collideConnected constraintHertz constraintDampingRatio params...
static void readJoint(int i)
{
	expect("joint");
	int kind = readInt();
	scene.jointKind[i] = kind;
	b3JointDef* base;
	switch (kind)
	{
		case JOINT_REVOLUTE: scene.revolute[i] = b3DefaultRevoluteJointDef(); base = &scene.revolute[i].base; break;
		case JOINT_SPHERICAL: scene.spherical[i] = b3DefaultSphericalJointDef(); base = &scene.spherical[i].base; break;
		case JOINT_PRISMATIC: scene.prismatic[i] = b3DefaultPrismaticJointDef(); base = &scene.prismatic[i].base; break;
		case JOINT_FILTER: scene.filter[i] = b3DefaultFilterJointDef(); base = &scene.filter[i].base; break;
		default: fail("unknown joint kind"); return;
	}
	scene.base[i] = base;
	scene.jointBodyA[i] = readInt();
	scene.jointBodyB[i] = readInt();
	if (scene.jointBodyA[i] < 0 || scene.jointBodyA[i] >= scene.bodyCount || scene.jointBodyB[i] < 0 || scene.jointBodyB[i] >= scene.bodyCount) fail("joint body out of range");
	base->localFrameA = readTransform();
	base->localFrameB = readTransform();
	optBool(&base->collideConnected);
	optFloat(&base->constraintHertz);
	optFloat(&base->constraintDampingRatio);
	if (kind == JOINT_REVOLUTE)
	{
		b3RevoluteJointDef* d = &scene.revolute[i];
		optBool(&d->enableSpring); optFloat(&d->hertz); optFloat(&d->dampingRatio); optFloat(&d->targetAngle);
		optBool(&d->enableLimit); optFloat(&d->lowerAngle); optFloat(&d->upperAngle);
		optBool(&d->enableMotor); optFloat(&d->maxMotorTorque); optFloat(&d->motorSpeed);
	}
	else if (kind == JOINT_SPHERICAL)
	{
		b3SphericalJointDef* d = &scene.spherical[i];
		optBool(&d->enableSpring); optFloat(&d->hertz); optFloat(&d->dampingRatio);
		next(); if (!isDefault()) { b3Quat q; q.v.x = asFloat(token); q.v.y = readFloat(); q.v.z = readFloat(); q.s = readFloat(); d->targetRotation = q; }
		optBool(&d->enableConeLimit); optFloat(&d->coneAngle);
		optBool(&d->enableTwistLimit); optFloat(&d->lowerTwistAngle); optFloat(&d->upperTwistAngle);
		optBool(&d->enableMotor); optFloat(&d->maxMotorTorque);
		next(); if (!isDefault()) { b3Vec3 v; v.x = asFloat(token); v.y = readFloat(); v.z = readFloat(); d->motorVelocity = v; }
	}
	else if (kind == JOINT_PRISMATIC)
	{
		b3PrismaticJointDef* d = &scene.prismatic[i];
		optBool(&d->enableSpring); optFloat(&d->hertz); optFloat(&d->dampingRatio); optFloat(&d->targetTranslation);
		optBool(&d->enableLimit); optFloat(&d->lowerTranslation); optFloat(&d->upperTranslation);
		optBool(&d->enableMotor); optFloat(&d->maxMotorForce); optFloat(&d->motorSpeed);
	}
}

static void readScene(void)
{
	expect("scene");
	scene.bodyCount = readInt();
	scene.shapeCount = readInt();
	scene.jointCount = readInt();
	if (scene.bodyCount < 1 || scene.bodyCount > MAX_ITEMS || scene.shapeCount < 0 || scene.shapeCount > MAX_ITEMS || scene.jointCount < 0 || scene.jointCount > MAX_ITEMS) fail("scene counts out of range");
	readWorld();
	for (int i = 0; i < scene.bodyCount; ++i) readBody(i);
	for (int i = 0; i < scene.shapeCount; ++i) readShape(i);
	for (int i = 0; i < scene.jointCount; ++i) readJoint(i);
	expect("end");
}

static b3BodyId bodyIds[MAX_ITEMS];

// Ill-conditioned decisions. A discrete test whose deciding quantity upstream computes within
// rounding of its threshold may land on either side in a port that rounds differently. Each hook
// bounds the quantity's forward error from the operands at its site (Higham's analysis: a sum of n
// products errs by at most gamma_n times the sum of their magnitudes), reports a decision within the
// bound with the entity it belongs to, and takes the other side for an entity listed in `flip`. A
// decision within ten bounds but outside one is reported as near, for tests of the bound.
enum { KIND_SEGMENT, KIND_SEPARATION, KIND_REVOLUTE_LOWER, KIND_REVOLUTE_UPPER, KIND_TWIST_LOWER, KIND_TWIST_UPPER,
	   KIND_CONE, KIND_PRISMATIC_LOWER, KIND_PRISMATIC_UPPER, KIND_COUNT };
static const char* kindNames[KIND_COUNT] = { "segment", "separation", "revolute-lower", "revolute-upper", "twist-lower",
											 "twist-upper", "cone", "prismatic-lower", "prismatic-upper" };
typedef struct Decision
{
	int kind, id;
	float quantity, bound;
	int side, near;
} Decision;
static Decision decisions[4 * MAX_ITEMS];
static int decisionCount;
static int flipContacts[MAX_ITEMS], flipContactCount;
static int flipJoints[MAX_ITEMS], flipJointCount;

static const double unit = 0x1p-24;
static double gam(int n) { return n * unit / (1.0 - n * unit); }
static float ulp(float x)
{
	x = fabsf(x);
	return nextafterf(x, INFINITY) - x;
}
static bool listed(const int* ids, int count, int id)
{
	for (int k = 0; k < count; ++k)
		if (ids[k] == id) return true;
	return false;
}
// Reports the decision if within its bound (or ten of them, as near); returns whether to flip it.
static bool decide(int kind, int id, double quantity, double threshold, double bound, bool flipped)
{
	double margin = fabs(quantity - threshold);
	if (margin > 10.0 * bound) return false;
	bool ill = margin <= bound;
	if (decisionCount < 4 * MAX_ITEMS) decisions[decisionCount++] = (Decision){ kind, id, (float)quantity, (float)bound, quantity > threshold, !ill };
	return ill && flipped;
}

// b3SegmentDistance's denom = a*e - b*b > threshold. A contraction rounds one product fewer, moving
// denom by at most half an ulp of either product; ulp(a*e) + ulp(b*b) bounds that.
static bool segmentDecision(float ae, float bb, float denom, float threshold)
{
	return decide(KIND_SEGMENT, b3OracleContactId, denom, threshold, ulp(ae) + ulp(bb), listed(flipContacts, flipContactCount, b3OracleContactId));
}

typedef struct V { double x, y, z; } V;
static V vabs(b3Vec3 v) { return (V){ fabs(v.x), fabs(v.y), fabs(v.z) }; }
static V vadd(V a, V b) { return (V){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static V vscale(double s, V a) { return (V){ s * a.x, s * a.y, s * a.z }; }
static double vdot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
// |a| x |b| with every term's magnitude: bounds each component of a cross product's terms.
static V crossAbs(V a, V b) { return (V){ a.y * b.z + a.z * b.y, a.z * b.x + a.x * b.z, a.x * b.y + a.y * b.x }; }
// Componentwise error bound of b3RotateVector or b3InvRotateVector(q, v), v + 2 cross(q.v, cross(q.v, v) +- q.s v),
// for exact operands: each component is a sum of at most six rounded steps over these magnitudes.
static V rotateError(b3Quat q, b3Vec3 v)
{
	V qv = vabs(q.v), av = vabs(v);
	V t2 = vadd(crossAbs(qv, av), vscale(fabs(q.s), av));
	return vscale(gam(6), vadd(av, vscale(2.0, crossAbs(qv, t2))));
}

// Contact push's s = (dot(n, dpB - dpA) + (dot(nB, rB) - dot(nA, rA))) + baseSeparation, nX = b3InvRotateVector(dqX, n).
static void separationDecision(b3Contact* const* contacts, int width, const void* s, void* separated, const void* normal,
							   const void* dpA, const void* dpB, const void* dqA, const void* dqB, const void* rA,
							   const void* rB, const void* normalA, const void* normalB, const void* normalSeparation,
							   const void* baseSeparation, const void* normalMass)
{
	const float* M = normalMass;
	const float* S = s;
	float* mask = separated;
	const float *N = normal, *PA = dpA, *PB = dpB, *QA = dqA, *QB = dqB, *RA = rA, *RB = rB, *NA = normalA, *NB = normalB,
				*NS = normalSeparation, *BS = baseSeparation;
	for (int lane = 0; lane < width; ++lane)
	{
		if (contacts[lane] == NULL || M[lane] == 0.0f) continue;
#define LANE3( P ) ( (b3Vec3){ P[lane], P[width + lane], P[2 * width + lane] } )
#define LANEQ( P ) ( (b3Quat){ { P[lane], P[width + lane], P[2 * width + lane] }, P[3 * width + lane] } )
		b3Vec3 n = LANE3(N), pa = LANE3(PA), pb = LANE3(PB), ra = LANE3(RA), rb = LANE3(RB), na = LANE3(NA), nb = LANE3(NB);
		b3Quat qa = LANEQ(QA), qb = LANEQ(QB);
#undef LANE3
#undef LANEQ
		V an = vabs(n);
		double errNormal = gam(4) * vdot(an, vadd(vabs(pa), vabs(pb)));
		double errA = gam(3) * vdot(vabs(na), vabs(ra)) + vdot(rotateError(qa, n), vabs(ra));
		double errB = gam(3) * vdot(vabs(nb), vabs(rb)) + vdot(rotateError(qb, n), vabs(rb));
		double dA = fabs(na.x * ra.x + na.y * ra.y + na.z * ra.z), dB = fabs(nb.x * rb.x + nb.y * rb.y + nb.z * rb.z);
		double bound = errNormal + errA + errB + gam(3) * (fabs(NS[lane]) + dA + dB + fabs(BS[lane]));
		int id = contacts[lane]->contactId;
		if (decide(KIND_SEPARATION, id, S[lane], 0.0, bound, listed(flipContacts, flipContactCount, id)))
			mask[lane] = mask[lane] != 0.0f ? 0.0f : 1.0f;
	}
}

typedef struct Q { double x, y, z, w; } Q;
static Q qabs(b3Quat q) { return (Q){ fabs(q.v.x), fabs(q.v.y), fabs(q.v.z), fabs(q.s) }; }
// |a| (x) |b| for a quaternion product, every term's magnitude.
static Q qmulAbs(Q a, Q b)
{
	return (Q){ a.w * b.x + a.x * b.w + a.y * b.z + a.z * b.y, a.w * b.y + a.y * b.w + a.z * b.x + a.x * b.z,
				a.w * b.z + a.z * b.w + a.x * b.y + a.y * b.x, a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z };
}
static Q qadd(Q a, Q b) { return (Q){ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
static Q qscale(double s, Q a) { return (Q){ s * a.x, s * a.y, s * a.z, s * a.w }; }

// A revolute or spherical limit's c from relQ = b3InvMulQuat(dqA * frameA, dqB * frameB): each product
// component sums four rounded terms; the twist angle is 2 atan2(z, w) and the swing 2 atan2(|xy|, |zw|),
// whose error follows from the components' through atan2's derivative, plus b3Atan2's own rounding.
static float limitDecision(int jointId, int limit, bool useBias, float c, float limitValue, b3Quat dqA, b3Quat dqB,
						   b3Quat frameA, b3Quat frameB, b3Quat relQ)
{
	if (!useBias) return c; // without bias both sides of c > 0 meet at c = 0
	Q qa = qmulAbs(qabs(dqA), qabs(frameA)), qb = qmulAbs(qabs(dqB), qabs(frameB));
	Q ea = qscale(gam(4), qa), eb = qscale(gam(4), qb);
	Q er = qadd(qscale(gam(4), qmulAbs(qa, qb)), qadd(qmulAbs(ea, qb), qmulAbs(qa, eb)));
	double x = relQ.v.x, y = relQ.v.y, z = relQ.v.z, w = relQ.s, angle, errAngle;
	if (limit == 4)
	{
		double r1 = sqrt(z * z + w * w), r2 = sqrt(x * x + y * y);
		double e1 = (r1 > 0 ? (fabs(z) * er.z + fabs(w) * er.w) / r1 : er.z + er.w) + gam(3) * r1;
		double e2 = (r2 > 0 ? (fabs(x) * er.x + fabs(y) * er.y) / r2 : er.x + er.y) + gam(3) * r2;
		angle = 2.0 * atan2(r2, r1);
		errAngle = 2.0 * (r1 * e2 + r2 * e1) / (r1 * r1 + r2 * r2);
	}
	else
	{
		angle = 2.0 * atan2(fabs(z), fabs(w));
		errAngle = 2.0 * (fabs(w) * er.z + fabs(z) * er.w) / (z * z + w * w);
	}
	double bound = errAngle + gam(12) * fabs(angle) + gam(1) * (fabs(angle) + fabs(limitValue));
	int kind = (int[]){ KIND_REVOLUTE_LOWER, KIND_REVOLUTE_UPPER, KIND_TWIST_LOWER, KIND_TWIST_UPPER, KIND_CONE }[limit];
	if (!decide(kind, jointId, c, 0.0, bound, listed(flipJoints, flipJointCount, jointId))) return c;
	// The other side, at the threshold.
	return c > 0.0f ? -0.0f : FLT_TRUE_MIN;
}

// A prismatic limit's C from translation = dot(d, axis), d = ((dcB - dcA) + deltaCenter) + (rB - rA),
// rX = b3RotateVector(dqX, frameX.p), axis = b3RotateVector(dqA, localAxis).
static float translationDecision(int jointId, int limit, bool useBias, float C, float limitValue, b3Vec3 dcA, b3Vec3 dcB,
								 b3Vec3 deltaCenter, b3Vec3 rA, b3Vec3 rB, b3Vec3 frameA, b3Vec3 frameB, b3Quat dqA,
								 b3Quat dqB, b3Vec3 localAxis, b3Vec3 axis, b3Vec3 d)
{
	if (!useBias) return C;
	V ed = vadd(vscale(gam(3), vadd(vadd(vadd(vabs(dcA), vabs(dcB)), vabs(deltaCenter)), vadd(vabs(rA), vabs(rB)))),
				vadd(rotateError(dqA, frameA), rotateError(dqB, frameB)));
	V ea = rotateError(dqA, localAxis);
	double translation = d.x * axis.x + d.y * axis.y + d.z * axis.z;
	double bound = gam(3) * vdot(vabs(d), vabs(axis)) + vdot(ed, vabs(axis)) + vdot(vabs(d), ea) + gam(1) * (fabs(translation) + fabs(limitValue));
	int kind = limit == 5 ? KIND_PRISMATIC_LOWER : KIND_PRISMATIC_UPPER;
	if (!decide(kind, jointId, C, 0.0, bound, listed(flipJoints, flipJointCount, jointId))) return C;
	return C > 0.0f ? -0.0f : FLT_TRUE_MIN;
}

// ill kind id quantity bound side, or near ... for one within ten bounds.
static void writeIll(void)
{
	for (int k = 0; k < decisionCount; ++k)
	{
		Decision* d = decisions + k;
		fprintf(out, "%s %s %d", d->near ? "near" : "ill", kindNames[d->kind], d->id);
		putFloat(d->quantity);
		putFloat(d->bound);
		fprintf(out, " %d\n", d->side);
	}
	decisionCount = 0;
}

// The scene in creation order, so body, shape and joint ids are scene indices and joint colors are
// the ones Box3D assigns at creation, before any contact exists.
static b3WorldId buildWorld(void)
{
	b3WorldId worldId = b3CreateWorld(&scene.world);
	b3World_SetContactRecycleDistance(worldId, 0.0f);
	for (int i = 0; i < scene.bodyCount; ++i) bodyIds[i] = b3CreateBody(worldId, &scene.bodies[i]);
	for (int i = 0; i < scene.shapeCount; ++i)
	{
		b3BodyId body = bodyIds[scene.shapeBody[i]];
		const b3ShapeDef* d = &scene.shapeDefs[i];
		if (scene.shapeKind[i] == SHAPE_CAPSULE) b3CreateCapsuleShape(body, d, &scene.capsules[i]);
		else if (scene.shapeKind[i] == SHAPE_SPHERE) b3CreateSphereShape(body, d, &scene.spheres[i]);
		else b3CreateHullShape(body, d, &scene.boxes[i].base);
	}
	for (int i = 0; i < scene.jointCount; ++i)
	{
		scene.base[i]->bodyIdA = bodyIds[scene.jointBodyA[i]];
		scene.base[i]->bodyIdB = bodyIds[scene.jointBodyB[i]];
		switch (scene.jointKind[i])
		{
			case JOINT_REVOLUTE: b3CreateRevoluteJoint(worldId, &scene.revolute[i]); break;
			case JOINT_SPHERICAL: b3CreateSphericalJoint(worldId, &scene.spherical[i]); break;
			case JOINT_PRISMATIC: b3CreatePrismaticJoint(worldId, &scene.prismatic[i]); break;
			default: b3CreateFilterJoint(worldId, &scene.filter[i]); break;
		}
	}
	return worldId;
}

static void writeResult(b3World* world);

static void writeMass(void)
{
	b3WorldId worldId = buildWorld();
	b3World* world = b3GetWorldFromId(worldId);
	writeResult(world);
	for (int i = 0; i < scene.bodyCount; ++i)
	{
		b3BodySim* sim = b3GetBodySim(world, b3Array_Get(world->bodies, i));
		fprintf(out, "mass");
		putFloat(sim->invMass);
		putVec3(sim->localCenter);
		putVec3(sim->invInertiaLocal.cx); putVec3(sim->invInertiaLocal.cy); putVec3(sim->invInertiaLocal.cz);
		fprintf(out, "\n");
	}
	b3DestroyWorld(worldId);
}

// The impulses a joint's warm start reads, by kind, in the order the protocol carries them.
static int jointFloats(b3JointSim* sim, float** out)
{
	int n = 0;
	switch (sim->type)
	{
		case b3_revoluteJoint:
		{
			b3RevoluteJoint* j = &sim->revoluteJoint;
			out[n++] = &j->linearImpulse.x; out[n++] = &j->linearImpulse.y; out[n++] = &j->linearImpulse.z;
			out[n++] = &j->perpImpulse.x; out[n++] = &j->perpImpulse.y;
			out[n++] = &j->springImpulse; out[n++] = &j->motorImpulse; out[n++] = &j->lowerImpulse; out[n++] = &j->upperImpulse;
			break;
		}
		case b3_sphericalJoint:
		{
			b3SphericalJoint* j = &sim->sphericalJoint;
			out[n++] = &j->linearImpulse.x; out[n++] = &j->linearImpulse.y; out[n++] = &j->linearImpulse.z;
			out[n++] = &j->springImpulse.x; out[n++] = &j->springImpulse.y; out[n++] = &j->springImpulse.z;
			out[n++] = &j->motorImpulse.x; out[n++] = &j->motorImpulse.y; out[n++] = &j->motorImpulse.z;
			out[n++] = &j->lowerTwistImpulse; out[n++] = &j->upperTwistImpulse; out[n++] = &j->swingImpulse;
			break;
		}
		case b3_prismaticJoint:
		{
			b3PrismaticJoint* j = &sim->prismaticJoint;
			out[n++] = &j->perpImpulse.x; out[n++] = &j->perpImpulse.y;
			out[n++] = &j->angularImpulse.x; out[n++] = &j->angularImpulse.y; out[n++] = &j->angularImpulse.z;
			out[n++] = &j->springImpulse; out[n++] = &j->motorImpulse; out[n++] = &j->lowerImpulse; out[n++] = &j->upperImpulse;
			break;
		}
		case b3_filterJoint: break;
		default: fail("unsupported joint type");
	}
	return n;
}

// A contact's convex cache (b3ContactCache: the simplex cache warm-starting b3ShapeDistance for a hull
// against a sphere or capsule, or the SAT cache for two hulls) as its raw 32-bit words.
#define CACHE_WORDS ( (int)( sizeof( b3ContactCache ) / sizeof( uint32_t ) ) )
_Static_assert( sizeof( b3ContactCache ) == 16, "bin/step.ts CACHE_WORDS carries four words" );
static void readCache(b3ContactCache* cache)
{
	uint32_t words[CACHE_WORDS];
	for (int k = 0; k < CACHE_WORDS; ++k)
	{
		char* end = NULL;
		words[k] = (uint32_t)strtoul(next(), &end, 16);
		if (end == token || *end || strlen(token) != 8) fail("cache word is not eight hexadecimal digits");
	}
	memcpy(cache, words, sizeof(words));
}
static void putCache(const b3ContactCache* cache)
{
	uint32_t words[CACHE_WORDS];
	memcpy(words, cache, sizeof(words));
	for (int k = 0; k < CACHE_WORDS; ++k) fprintf(out, " %08x", words[k]);
}

static void writeResult(b3World* world)
{
	for (int i = 0; i < scene.bodyCount; ++i)
	{
		b3Body* body = b3Array_Get(world->bodies, i);
		b3BodySim* sim = b3GetBodySim(world, body);
		b3BodyState* state = b3GetBodyState(world, body);
		fprintf(out, "body");
		putVec3((b3Vec3){ (float)sim->transform.p.x, (float)sim->transform.p.y, (float)sim->transform.p.z });
		putVec3((b3Vec3){ (float)sim->center.x, (float)sim->center.y, (float)sim->center.z });
		putQuat(sim->transform.q);
		putVec3(state ? state->linearVelocity : b3Vec3_zero);
		putVec3(state ? state->angularVelocity : b3Vec3_zero);
		fprintf(out, "\n");
	}
	for (int i = 0; i < scene.jointCount; ++i)
	{
		b3Joint* joint = b3Array_Get(world->joints, i);
		float* fields[16];
		int n = jointFloats(b3GetJointSim(world, joint), fields);
		fprintf(out, "joint %d", joint->colorIndex);
		for (int k = 0; k < n; ++k) putFloat(*fields[k]);
		fprintf(out, "\n");
	}
	for (int i = 0; i < scene.shapeCount; ++i)
	{
		fprintf(out, "fat");
		putVec3(world->fatAABBs.data[i].lowerBound);
		putVec3(world->fatAABBs.data[i].upperBound);
		fprintf(out, "\n");
	}
	fprintf(out, "pool %d %d", world->contactIdPool.nextIndex, world->contactIdPool.freeArray.count);
	for (int k = 0; k < world->contactIdPool.freeArray.count; ++k) fprintf(out, " %d", world->contactIdPool.freeArray.data[k]);
	fprintf(out, "\n");
	for (int i = 0; i < world->contacts.count; ++i)
	{
		b3Contact* contact = world->contacts.data + i;
		if (contact->contactId != i) continue;
		int touching = (contact->flags & b3_contactTouchingFlag) != 0;
		fprintf(out, "contact %d %d %d %d %d", i, contact->shapeIdA, contact->shapeIdB, touching, contact->colorIndex);
		putCache(&contact->convexContact.cache);
		fprintf(out, " %d", contact->manifoldCount);
		for (int m = 0; m < contact->manifoldCount; ++m)
		{
			b3Manifold* manifold = contact->manifolds + m;
			putVec3(manifold->normal);
			fprintf(out, " %d", manifold->pointCount);
			for (int p = 0; p < manifold->pointCount; ++p)
			{
				b3ManifoldPoint* mp = manifold->points + p;
				fprintf(out, " %u", mp->featureId);
				putVec3(mp->anchorA); putVec3(mp->anchorB);
				putFloat(mp->separation); putFloat(mp->normalImpulse); putFloat(mp->totalNormalImpulse);
			}
			putVec3(manifold->frictionImpulse); putFloat(manifold->twistImpulse); putVec3(manifold->rollingImpulse);
			// Mixed materials are refreshed with every manifold and stale without one.
			putFloat(contact->friction); putFloat(contact->restitution); putFloat(contact->rollingResistance);
		}
		fprintf(out, "\n");
	}
}

static b3Contact emptySlot(void)
{
	b3Contact slot = { 0 };
	slot.contactId = B3_NULL_INDEX;
	slot.setIndex = B3_NULL_INDEX;
	slot.colorIndex = B3_NULL_INDEX;
	slot.localIndex = B3_NULL_INDEX;
	return slot;
}

// b3AddContactToGraph puts a contact in the first free color in its search order; occupying the
// colors that order visits before the loaded one makes it place the contact there.
static void addToColor(b3World* world, b3Contact* contact, int wanted)
{
	b3ConstraintGraph* graph = &world->constraintGraph;
	int bodyA = contact->edges[0].bodyId, bodyB = contact->edges[1].bodyId;
	bool dynamicA = b3Array_Get(world->bodies, bodyA)->type == b3_dynamicBody;
	bool dynamicB = b3Array_Get(world->bodies, bodyB)->type == b3_dynamicBody;
	int blocked[B3_GRAPH_COLOR_COUNT];
	int blockedCount = 0;
	int marker = dynamicA ? bodyA : bodyB;
	if (dynamicA && dynamicB)
	{
		if (wanted != B3_OVERFLOW_INDEX && (wanted < 0 || wanted >= B3_DYNAMIC_COLOR_COUNT)) fail("dynamic contact color out of range");
		int end = wanted == B3_OVERFLOW_INDEX ? B3_DYNAMIC_COLOR_COUNT : wanted;
		for (int i = 0; i < end; ++i)
			if (!b3GetBit(&graph->colors[i].bodySet, bodyA) && !b3GetBit(&graph->colors[i].bodySet, bodyB)) blocked[blockedCount++] = i;
	}
	else
	{
		if (wanted != B3_OVERFLOW_INDEX && (wanted < 1 || wanted >= B3_OVERFLOW_INDEX)) fail("static contact color out of range");
		int end = wanted == B3_OVERFLOW_INDEX ? 0 : wanted;
		for (int i = B3_OVERFLOW_INDEX - 1; i > end; --i)
			if (!b3GetBit(&graph->colors[i].bodySet, marker)) blocked[blockedCount++] = i;
	}
	for (int k = 0; k < blockedCount; ++k) b3SetBitGrow(&graph->colors[blocked[k]].bodySet, marker);
	b3AddContactToGraph(world, contact);
	for (int k = 0; k < blockedCount; ++k) b3ClearBit(&graph->colors[blocked[k]].bodySet, marker);
	if (contact->colorIndex != wanted) fail("loaded color already holds one of the contact's bodies");
}

// A stage hook to localize a red within a step: after stage `stopType` of color `stopColor` (-1 for
// any) in substep `stopSubstep`, every body's velocities and substep deltas and every joint's
// impulses are kept, and emitted with the step's result.
static int stopSubstep = -1, stopType, stopColor, substep;
static b3World* stageWorld;
static char* stageLines;
static size_t stageLength;
static FILE* stageOut;

static void stageHook(int type, int color, b3StepContext* context)
{
	if (type == b3_stageIntegrateVelocities) substep += 1;
	if (stopSubstep < 0 || substep - 1 != stopSubstep || type != stopType || (stopColor >= 0 && color != stopColor)) return;
	if (stageOut != NULL) fclose(stageOut);
	stageOut = open_memstream(&stageLines, &stageLength);
	FILE* saved = out;
	out = stageOut;
	for (int i = 0; i < scene.bodyCount; ++i)
	{
		b3Body* body = b3Array_Get(stageWorld->bodies, i);
		fprintf(out, "stagebody %d", i);
		if (body->setIndex == b3_awakeSet)
		{
			b3BodyState* state = context->states + body->localIndex;
			putVec3(state->linearVelocity);
			putVec3(state->angularVelocity);
			putVec3(state->deltaPosition);
			putQuat(state->deltaRotation);
		}
		fprintf(out, "\n");
	}
	for (int i = 0; i < scene.jointCount; ++i)
	{
		float* fields[16];
		int n = jointFloats(b3GetJointSim(stageWorld, b3Array_Get(stageWorld->joints, i)), fields);
		fprintf(out, "stagejoint %d", i);
		for (int k = 0; k < n; ++k) putFloat(*fields[k]);
		fprintf(out, "\n");
	}
	fflush(out);
	out = saved;
}

// step
// bodies: ox oy oz cx cy cz qx qy qz qw vx vy vz wx wy wz fx fy fz tx ty tz, per body
// joints: the impulses of jointFloats, per joint
// fat: lx ly lz ux uy uz, per shape
// pool nextIndex freeCount free...
// contacts count, then per contact: id shapeA shapeB touching color pointCount
//   (featureId normalImpulse) per point, frictionImpulse(3) twistImpulse rollingImpulse(3) cache words
// flip count (contact|joint id)... (entities that take the other side of their ill-conditioned decisions)
// stop substep stageType color (substep -1 for none; see stageHook)
static void loadAndStep(void)
{
	b3WorldId worldId = buildWorld();
	b3World* world = b3GetWorldFromId(worldId);

	for (int i = 0; i < scene.bodyCount; ++i)
	{
		b3Vec3 o = readVec3();
		b3Vec3 c = readVec3();
		b3Quat q = readQuat();
		b3Vec3 v = readVec3(), w = readVec3(), force = readVec3(), torque = readVec3();
		b3Body* body = b3Array_Get(world->bodies, i);
		if (body->type == b3_staticBody) continue;
		b3BodySim* sim = b3GetBodySim(world, body);
		// The origin is state: b3FinalizeBodiesTask derives it from the center of mass once, with
		// the library's rounding, and every later read takes the stored value.
		b3Pos center = { c.x, c.y, c.z };
		b3Body_SetTransform(bodyIds[i], (b3Pos){ o.x, o.y, o.z }, q);
		sim->center = center;
		sim->center0 = center;
		b3Body_SetLinearVelocity(bodyIds[i], v);
		b3Body_SetAngularVelocity(bodyIds[i], w);
		b3Body_ApplyForceToCenter(bodyIds[i], force, false);
		b3Body_ApplyTorque(bodyIds[i], torque, false);
	}

	for (int i = 0; i < scene.jointCount; ++i)
	{
		float* fields[16];
		int n = jointFloats(b3GetJointSim(world, b3Array_Get(world->joints, i)), fields);
		for (int k = 0; k < n; ++k) *fields[k] = readFloat();
	}

	for (int i = 0; i < scene.shapeCount; ++i)
	{
		b3AABB fat;
		fat.lowerBound = readVec3();
		fat.upperBound = readVec3();
		b3Shape* shape = b3Array_Get(world->shapes, i);
		b3AABB* current = world->fatAABBs.data + i;
		if (memcmp(current, &fat, sizeof(fat)) == 0) continue;
		*current = fat;
		b3BroadPhase_MoveProxy(&world->broadPhase, shape->proxyKey, fat);
	}

	expect("pool");
	int nextIndex = readInt();
	int freeCount = readInt();
	if (nextIndex < 0 || freeCount < 0 || freeCount > nextIndex) fail("contact id pool out of range");
	int freeIds[MAX_ITEMS];
	if (freeCount > MAX_ITEMS) fail("too many free contact ids");
	for (int k = 0; k < freeCount; ++k) freeIds[k] = readInt();

	expect("contacts");
	int contactCount = readInt();
	if (contactCount + freeCount != nextIndex) fail("live and free contact ids do not cover the pool");
	int previous = -1;
	for (int k = 0; k < contactCount; ++k)
	{
		int id = readInt(), shapeA = readInt(), shapeB = readInt(), touching = readInt(), color = readInt(), pointCount = readInt();
		if (id <= previous || id >= nextIndex) fail("contact ids must ascend within the pool");
		if (shapeA < 0 || shapeA >= scene.shapeCount || shapeB < 0 || shapeB >= scene.shapeCount) fail("contact shape out of range");
		if ((touching != 0) != (pointCount > 0) || pointCount > B3_MAX_MANIFOLD_POINTS) fail("a contact touches exactly when it has points");
		previous = id;
		while (world->contacts.count < id) b3Array_Push(world->contacts, emptySlot());
		world->contactIdPool.nextIndex = id;
		b3CreateContact(world, b3Array_Get(world->shapes, shapeA), b3Array_Get(world->shapes, shapeB), 0);
		if (world->contacts.count != id + 1) fail("upstream created no contact for the pair");
		b3Contact* contact = b3Array_Get(world->contacts, id);
		uint32_t features[B3_MAX_MANIFOLD_POINTS];
		float impulses[B3_MAX_MANIFOLD_POINTS];
		for (int p = 0; p < pointCount; ++p)
		{
			char* end = NULL;
			features[p] = (uint32_t)strtoul(next(), &end, 10);
			if (end == token || *end) fail("feature id expected");
			impulses[p] = readFloat();
		}
		b3Vec3 friction = readVec3();
		float twist = readFloat();
		b3Vec3 rolling = readVec3();
		readCache(&contact->convexContact.cache);
		if (!touching)
		{
			if (color != B3_NULL_INDEX) fail("a non-touching contact has no color");
			continue;
		}
		contact->manifolds = b3AllocateManifolds(world, 1);
		contact->manifoldCount = 1;
		b3Manifold* manifold = contact->manifolds;
		memset(manifold, 0, sizeof(*manifold));
		manifold->pointCount = pointCount;
		for (int p = 0; p < pointCount; ++p)
		{
			manifold->points[p].featureId = features[p];
			manifold->points[p].normalImpulse = impulses[p];
		}
		manifold->frictionImpulse = friction;
		manifold->twistImpulse = twist;
		manifold->rollingImpulse = rolling;
		// As b3Collide does when a contact starts touching.
		contact->flags |= b3_simTouchingFlag | b3_contactTouchingFlag;
		b3LinkContact(world, contact);
		int nonTouchingIndex = contact->localIndex;
		addToColor(world, contact, color);
		b3OracleRemoveNonTouchingContact(world, b3_awakeSet, nonTouchingIndex);
	}
	while (world->contacts.count < nextIndex) b3Array_Push(world->contacts, emptySlot());
	world->contactIdPool.nextIndex = nextIndex;
	for (int k = 0; k < freeCount; ++k)
	{
		int id = freeIds[k];
		if (id < 0 || id >= nextIndex || world->contacts.data[id].contactId != B3_NULL_INDEX) fail("free contact id is live or out of range");
		b3Array_Push(world->contactIdPool.freeArray, id);
	}
	expect("flip");
	int flipCount = readInt();
	if (flipCount < 0 || flipCount > MAX_ITEMS) fail("flip count out of range");
	flipContactCount = flipJointCount = 0;
	for (int k = 0; k < flipCount; ++k)
	{
		next();
		bool joint = strcmp(token, "joint") == 0;
		if (!joint && strcmp(token, "contact") != 0) fail("flip entity is contact or joint");
		int id = readInt();
		if (joint) flipJoints[flipJointCount++] = id;
		else flipContacts[flipContactCount++] = id;
	}
	expect("stop");
	stopSubstep = readInt();
	stopType = readInt();
	stopColor = readInt();
	expect("end");

	decisionCount = 0;
	substep = 0;
	stageWorld = world;
	b3World_Step(worldId, scene.timeStep, scene.subStepCount);
	flipContactCount = flipJointCount = 0;
	stopSubstep = -1;
	if (stageOut != NULL)
	{
		fclose(stageOut);
		stageOut = NULL;
		fwrite(stageLines, 1, stageLength, out);
		free(stageLines);
		stageLines = NULL;
	}
	writeIll();

	writeResult(world);
	fprintf(out, "end\n");
	fflush(out);
	b3DestroyWorld(worldId);
}

// rollout n: the scene stepped n times without reloading, each step's state as a step response.
static void rollout(void)
{
	int steps = readInt();
	b3WorldId worldId = buildWorld();
	b3World* world = b3GetWorldFromId(worldId);
	for (int k = 0; k < steps; ++k)
	{
		b3World_Step(worldId, scene.timeStep, scene.subStepCount);
		writeIll();
		writeResult(world);
		fprintf(out, "end\n");
	}
	fflush(out);
	b3DestroyWorld(worldId);
}

// segment flip p1 q1 p2 q2: one b3SegmentDistance, its fractions and any ill-conditioned decision.
static void segment(void)
{
	flipContactCount = readInt() != 0 ? 1 : 0;
	flipContacts[0] = -1;
	b3Vec3 p1 = readVec3(), q1 = readVec3(), p2 = readVec3(), q2 = readVec3();
	b3OracleContactId = -1;
	b3SegmentDistanceResult result = b3SegmentDistance(p1, q1, p2, q2);
	flipContactCount = 0;
	writeIll();
	fprintf(out, "segment");
	putFloat(result.fraction1);
	putFloat(result.fraction2);
	fprintf(out, "\nend\n");
	fflush(out);
}

// decide kind operands... flip: one decision hook on supplied operands, for tests of its bound.
//   separation s n dpA dpB dqA dqB rA rB nA nB normalSeparation baseSeparation
//   limit index(0-4) c limitValue dqA dqB frameA frameB relQ
//   translation index(5-6) C limitValue dcA dcB deltaCenter rA rB frameA frameB dqA dqB localAxis axis d
static void decideCommand(void)
{
	next();
	char kind[32];
	snprintf(kind, sizeof(kind), "%s", token);
	float result = 0.0f;
	if (strcmp(kind, "separation") == 0)
	{
		float sv = readFloat();
		b3Vec3 n = readVec3(), pa = readVec3(), pb = readVec3();
		b3Quat qa = readQuat(), qb = readQuat();
		b3Vec3 ra = readVec3(), rb = readVec3(), na = readVec3(), nb = readVec3();
		float ns = readFloat(), base = readFloat(), mass = 1.0f;
		flipContactCount = readInt() != 0 ? 1 : 0;
		flipContacts[0] = -1;
		static b3Contact probe;
		probe.contactId = -1;
		b3Contact* lanes[1] = { &probe };
		float mask = sv > 0.0f ? 1.0f : 0.0f;
		separationDecision(lanes, 1, &sv, &mask, &n, &pa, &pb, &qa, &qb, &ra, &rb, &na, &nb, &ns, &base, &mass);
		result = mask;
	}
	else if (strcmp(kind, "limit") == 0)
	{
		int index = readInt();
		float c = readFloat(), limitValue = readFloat();
		b3Quat qa = readQuat(), qb = readQuat(), fa = readQuat(), fb = readQuat(), rel = readQuat();
		flipJointCount = readInt() != 0 ? 1 : 0;
		flipJoints[0] = -1;
		result = limitDecision(-1, index, true, c, limitValue, qa, qb, fa, fb, rel);
	}
	else if (strcmp(kind, "translation") == 0)
	{
		int index = readInt();
		float c = readFloat(), limitValue = readFloat();
		b3Vec3 dca = readVec3(), dcb = readVec3(), dc = readVec3(), ra = readVec3(), rb = readVec3(), fa = readVec3(), fb = readVec3();
		b3Quat qa = readQuat(), qb = readQuat();
		b3Vec3 local = readVec3(), axis = readVec3(), d = readVec3();
		flipJointCount = readInt() != 0 ? 1 : 0;
		flipJoints[0] = -1;
		result = translationDecision(-1, index, true, c, limitValue, dca, dcb, dc, ra, rb, fa, fb, qa, qb, local, axis, d);
	}
	else fail("decide kind is separation, limit or translation");
	flipContactCount = flipJointCount = 0;
	writeIll();
	fprintf(out, "decided");
	putFloat(result);
	fprintf(out, "\nend\n");
	fflush(out);
}

int main(void)
{
	out = stdout;
	b3OracleSegmentDecision = segmentDecision;
	b3OracleSeparationDecision = separationDecision;
	b3OracleLimitDecision = limitDecision;
	b3OracleTranslationDecision = translationDecision;
	b3OracleStageHook = stageHook;
	readScene();
	writeMass();
	fprintf(out, "ready\n");
	fflush(out);
	while (scanf("%255s", token) == 1)
	{
		if (strcmp(token, "step") == 0) loadAndStep();
		else if (strcmp(token, "rollout") == 0) rollout();
		else if (strcmp(token, "segment") == 0) segment();
		else if (strcmp(token, "decide") == 0) decideCommand();
		else fail("step, rollout, segment or decide expected");
	}
	return 0;
}
