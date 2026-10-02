// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Transcription of samples/mover.cpp lines 46-80 (PlaneResultFcn),
// 140-267 (SolveMove: pogo ray through velocity clip), and mover.h's capacity 8.
// Friction, acceleration, gravity and drawing are omitted. Case supplies state.
// No shape user data is authored: maxPush=FLT_MAX, clipVelocity=true, canMoverPush=true.
#include <float.h>
#include <assert.h>
typedef struct SampleMover {
    b3WorldTransform transform;
    b3CollisionPlane planes[8];
    b3Pos points[8];
    b3ShapeId shapes[8];
    int count;
} SampleMover;
static bool sample_filter(b3ShapeId shape, void* context) { (void)shape; (void)context; return true; }
static bool sample_planes(b3ShapeId shape, const b3PlaneResult* results, int count, void* context)
{
    SampleMover* self = context;
    if (!sample_filter(shape, context)) return true;
    for (int i = 0; i < count && self->count < 8; ++i) {
        assert(b3IsValidPlane(results[i].plane));
        self->planes[self->count] = (b3CollisionPlane){ .plane = results[i].plane, .pushLimit = FLT_MAX, .push = 0.0f, .clipVelocity = true };
        self->points[self->count] = b3OffsetPos(self->transform.p, results[i].point);
        self->shapes[self->count++] = shape;
    }
    return true;
}
static void sample_move(FILE* out, b3WorldId world, const ScenarioCommand* c, b3BodyId* bodies)
{
    SampleMover self = { .transform = { .p = { f32(c->values[0]), f32(c->values[1]), f32(c->values[2]) }, .q = q4(c, 3) } };
    b3Capsule capsule = { v3(c, 7), v3(c, 10), f32(c->values[13]) };
    b3Vec3 velocity = v3(c, 14);
    float pogoVelocity = f32(c->values[17]), timeStep = f32(c->values[18]);
    float pogoRestLength = 3.0f * capsule.radius;
    float rayLength = pogoRestLength + capsule.radius;
    b3Pos rayOrigin = b3TransformWorldPoint(self.transform, capsule.center1);
    b3Vec3 rayTranslation = b3MulSV(-rayLength, b3Vec3_axisY);
    b3QueryFilter skipTeamFilter = { .categoryBits = 1, .maskBits = ~2u }; skipTeamFilter.name = "pogo";
    b3RayResult rayResult = b3World_CastRayClosest(world, rayOrigin, rayTranslation, skipTeamFilter);
    bool suppressPogo = velocity.y > 0.0f;
    bool onGround;
    if (!rayResult.hit || suppressPogo) { onGround = false; pogoVelocity = 0.0f; }
    else {
        onGround = true;
        float pogoCurrentLength = rayResult.fraction * rayLength;
        float zeta = 0.7f, hertz = 4.0f;
        float omega = 2.0f * B3_PI * hertz;
        float omegaH = omega * timeStep;
        pogoVelocity = (pogoVelocity - omega * omegaH * (pogoCurrentLength - pogoRestLength)) / (1.0f + 2.0f * zeta * omegaH + omegaH * omegaH);
    }
    b3Pos startPosition = self.transform.p;
    b3Pos target = b3OffsetPos(b3OffsetPos(self.transform.p, b3MulSV(timeStep, velocity)), b3MulSV(timeStep * pogoVelocity, b3Vec3_axisY));
    b3QueryFilter moverFilter = { .categoryBits = 1, .maskBits = ~0u, .id = 1, .name = "mover_collide" };
    b3QueryFilter castFilter = { .categoryBits = 1, .maskBits = ~2u, .id = 1, .name = "mover_cast" };
    int totalIterations = 0, passes = 0;
    float tolerance = 0.01f;
    #ifdef B3_MOVER_SENTINEL
    const int passLimit = 1;
#else
    const int passLimit = 5;
#endif
    for (int iteration = 0; iteration < passLimit; ++iteration) {
        ++passes; self.count = 0;
        b3Capsule mover = { capsule.center1, capsule.center2, capsule.radius };
        b3World_CollideMover(world, self.transform.p, &mover, moverFilter, sample_planes, &self);
        b3Vec3 targetDelta = b3SubPos(target, self.transform.p);
        b3PlaneSolverResult result = b3SolvePlanes(targetDelta, self.planes, self.count);
        totalIterations += result.iterationCount;
        b3Vec3 delta = result.delta;
        float fraction = b3World_CastMover(world, self.transform.p, &mover, delta, castFilter, sample_filter, &self);
        delta = b3MulSV(fraction, delta);
        self.transform.p = b3OffsetPos(self.transform.p, delta);
        if (b3LengthSquared(delta) < tolerance * tolerance) break;
    }
    b3Vec3 impulses[SCENARIO_BODY_CAPACITY] = { 0 };
    for (int i = 0; i < self.count; ++i) {
        b3BodyId bodyId = b3Shape_GetBody(self.shapes[i]);
        if (b3Body_GetType(bodyId) != b3_dynamicBody) continue;
        b3Pos point = self.points[i]; b3Vec3 normal = b3Neg(self.planes[i].plane.normal);
        float invMassA = 0.0f, invMassB = b3Body_GetInverseMass(bodyId);
        b3Matrix3 invIB = b3Body_GetWorldInverseRotationalInertia(bodyId);
        b3Pos pB = b3Body_GetWorldCenter(bodyId);
        b3Vec3 rB = b3SubPos(point, pB), rnB = b3Cross(rB, normal);
        float kNormal = invMassA + invMassB + b3Dot(rnB, b3MulMV(invIB, rnB));
        float normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
        b3Vec3 vB = b3Body_GetLinearVelocity(bodyId), omegaB = b3Body_GetAngularVelocity(bodyId);
        b3Vec3 vrB = b3Add(vB, b3Cross(omegaB, rB));
        float vn = b3Dot(b3Sub(vrB, velocity), normal);
        float impulse = b3MaxFloat(-normalMass * vn, 0.0f);
        b3Vec3 P = b3MulSV(impulse, normal);
        velocity = b3MulSub(velocity, invMassA, P);
        b3Body_ApplyLinearImpulse(bodyId, P, point, true);
        for (int j = 0; j < SCENARIO_BODY_CAPACITY; ++j) if (B3_ID_EQUALS(bodyId, bodies[j])) impulses[j] = b3Add(impulses[j], P);
    }
    if (HAS(c, 0)) velocity = b3ClipVector(velocity, self.planes, self.count);
    else if (timeStep > 0.0f) velocity = b3MulSV(1.0f / timeStep, b3SubPos(self.transform.p, startPosition));
    fprintf(out, "{\"receiptId\":\"%s\",\"position\":", c->id); pos(out, self.transform.p);
    fputs(",\"velocity\":", out); vec3(out, velocity);
    fputs(",\"pogoVelocity\":", out); hex32(out, bits(pogoVelocity));
    fputs(",\"onGround\":", out); hex32(out, bits(onGround ? 1.0f : 0.0f));
    fputs(",\"planeCount\":", out); hex32(out, bits((float)self.count));
    fprintf(out, ",\"passes\":%d,\"solverIterations\":%d,\"impulses\":[", passes, totalIterations);
    int first = 1;
    for (int j = 0; j < SCENARIO_BODY_CAPACITY; ++j) if (b3Body_IsValid(bodies[j]) && b3Body_GetType(bodies[j]) == b3_dynamicBody) {
        if (!first) fputc(',', out); first = 0;
        fprintf(out, "{\"body\":\"b%d\",\"impulse\":", j); vec3(out, impulses[j]); fputc('}', out);
    }
    fputs("]}", out);
}
