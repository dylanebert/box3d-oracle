// Step oracle: loads the state one b3World_Step reads into upstream structs, steps once and
// writes the state after it. Protocol: whitespace tokens on stdin and stdout, floats as eight
// hexadecimal digits of their bits, "-" for an upstream default. See bin/step.ts.
#include "bitset.h"
#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "island.h"
#include "joint.h"
#include "oracle_hooks.h"
#include "physics_world.h"
#include "shape.h"
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
static char token[256];

static void fail(const char* message)
{
	printf("error %s\n", message);
	fflush(stdout);
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
	printf(" %08x", bits);
}
static void putVec3(b3Vec3 v) { putFloat(v.x); putFloat(v.y); putFloat(v.z); }
static void putQuat(b3Quat q) { putVec3(q.v); putFloat(q.s); }

// world gx gy gz restitutionThreshold restitutionIterations contactHertz contactDampingRatio contactSpeed
//   maximumLinearSpeed isotropicInertiaTolerance timeStep subStepCount
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
		printf("mass");
		putFloat(sim->invMass);
		putVec3(sim->localCenter);
		putVec3(sim->invInertiaLocal.cx); putVec3(sim->invInertiaLocal.cy); putVec3(sim->invInertiaLocal.cz);
		printf("\n");
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

static void writeResult(b3World* world)
{
	for (int i = 0; i < scene.bodyCount; ++i)
	{
		b3Body* body = b3Array_Get(world->bodies, i);
		b3BodySim* sim = b3GetBodySim(world, body);
		b3BodyState* state = b3GetBodyState(world, body);
		printf("body");
		putVec3((b3Vec3){ (float)sim->transform.p.x, (float)sim->transform.p.y, (float)sim->transform.p.z });
		putVec3((b3Vec3){ (float)sim->center.x, (float)sim->center.y, (float)sim->center.z });
		putQuat(sim->transform.q);
		putVec3(state ? state->linearVelocity : b3Vec3_zero);
		putVec3(state ? state->angularVelocity : b3Vec3_zero);
		printf("\n");
	}
	for (int i = 0; i < scene.jointCount; ++i)
	{
		b3Joint* joint = b3Array_Get(world->joints, i);
		float* fields[16];
		int n = jointFloats(b3GetJointSim(world, joint), fields);
		printf("joint %d", joint->colorIndex);
		for (int k = 0; k < n; ++k) putFloat(*fields[k]);
		printf("\n");
	}
	for (int i = 0; i < scene.shapeCount; ++i)
	{
		printf("fat");
		putVec3(world->fatAABBs.data[i].lowerBound);
		putVec3(world->fatAABBs.data[i].upperBound);
		printf("\n");
	}
	printf("pool %d %d", world->contactIdPool.nextIndex, world->contactIdPool.freeArray.count);
	for (int k = 0; k < world->contactIdPool.freeArray.count; ++k) printf(" %d", world->contactIdPool.freeArray.data[k]);
	printf("\n");
	for (int i = 0; i < world->contacts.count; ++i)
	{
		b3Contact* contact = world->contacts.data + i;
		if (contact->contactId != i) continue;
		int touching = (contact->flags & b3_contactTouchingFlag) != 0;
		printf("contact %d %d %d %d %d %d", i, contact->shapeIdA, contact->shapeIdB, touching, contact->colorIndex, contact->manifoldCount);
		for (int m = 0; m < contact->manifoldCount; ++m)
		{
			b3Manifold* manifold = contact->manifolds + m;
			putVec3(manifold->normal);
			printf(" %d", manifold->pointCount);
			for (int p = 0; p < manifold->pointCount; ++p)
			{
				b3ManifoldPoint* mp = manifold->points + p;
				printf(" %u", mp->featureId);
				putVec3(mp->anchorA); putVec3(mp->anchorB);
				putFloat(mp->separation); putFloat(mp->normalImpulse); putFloat(mp->totalNormalImpulse);
			}
			putVec3(manifold->frictionImpulse); putFloat(manifold->twistImpulse); putVec3(manifold->rollingImpulse);
			// Mixed materials are refreshed with every manifold and stale without one.
			putFloat(contact->friction); putFloat(contact->restitution); putFloat(contact->rollingResistance);
		}
		printf("\n");
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

// step
// bodies: cx cy cz qx qy qz qw vx vy vz wx wy wz fx fy fz tx ty tz, per body
// joints: the impulses of jointFloats, per joint
// fat: lx ly lz ux uy uz, per shape
// pool nextIndex freeCount free...
// contacts count, then per contact: id shapeA shapeB touching color pointCount
//   (featureId normalImpulse) per point, frictionImpulse(3) twistImpulse rollingImpulse(3)
static void loadAndStep(void)
{
	b3WorldId worldId = buildWorld();
	b3World* world = b3GetWorldFromId(worldId);

	for (int i = 0; i < scene.bodyCount; ++i)
	{
		b3Vec3 c = readVec3();
		b3Quat q = readQuat();
		b3Vec3 v = readVec3(), w = readVec3(), force = readVec3(), torque = readVec3();
		b3Body* body = b3Array_Get(world->bodies, i);
		if (body->type == b3_staticBody) continue;
		b3BodySim* sim = b3GetBodySim(world, body);
		// The origin as b3FinalizeBodiesTask derives it from the center of mass.
		b3Pos center = { c.x, c.y, c.z };
		b3Pos origin = b3OffsetPos(center, b3Neg(b3RotateVector(q, sim->localCenter)));
		b3Body_SetTransform(bodyIds[i], origin, q);
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
	expect("end");

	b3World_Step(worldId, scene.timeStep, scene.subStepCount);

	writeResult(world);
	printf("end\n");
	fflush(stdout);
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
		writeResult(world);
		printf("end\n");
	}
	fflush(stdout);
	b3DestroyWorld(worldId);
}

int main(void)
{
	readScene();
	writeMass();
	printf("ready\n");
	fflush(stdout);
	while (scanf("%255s", token) == 1)
	{
		if (strcmp(token, "step") == 0) loadAndStep();
		else if (strcmp(token, "rollout") == 0) rollout();
		else fail("step or rollout expected");
	}
	return 0;
}
