#include "box3d_oracle_adapter.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "oracle_hooks.h"
#include <string.h>

static void writeLocalManifold(FILE* out, const b3LocalManifold* manifold)
{
    oracle_key(out, "normal"); oracle_vec3(out, manifold->normal); fputc(',', out);
    oracle_key(out, "pointCount"); oracle_i32(out, manifold->pointCount); fputc(',', out);
    oracle_key(out, "points"); fputc('[', out);
    for (int i = 0; i < manifold->pointCount; ++i) {
        if (i) fputc(',', out);
        const b3LocalManifoldPoint* point = manifold->points + i;
        fputs("{\"point\":", out); oracle_vec3(out, point->point); fputs(",\"separation\":", out); oracle_f32(out, point->separation);
        fputs(",\"triangleIndex\":", out); oracle_i32(out, point->triangleIndex); fputs(",\"feature\":[", out);
        oracle_u32(out, ((uint32_t)point->pair.owner1 << 24) | ((uint32_t)point->pair.index1 << 16) |
            ((uint32_t)point->pair.owner2 << 8) | point->pair.index2);
        fputs("]}", out);
    }
    fputc(']', out);
}

static void writeConvexManifold(FILE* out, bool* first)
{
    b3BoxHull hullA = b3MakeBoxHull(1.0f, 1.0f, 1.0f);
    b3BoxHull hullB = b3MakeBoxHull(1.0f, 1.0f, 1.0f);
    b3Transform transform = { (b3Vec3){1.25f, 0.1f, 0.0f}, (b3Quat){ {0.0f, 0.0f, 0.0f}, 1.0f } };
    b3LocalManifoldPoint points[32] = { 0 };
    b3LocalManifold manifold = { 0 };
    manifold.points = points;
    b3SATCache cache = { 0 };
    b3CollideHulls(&manifold, 32, &hullA.base, &hullB.base, transform, &cache);
    oracle_case_begin(out, first, "o4.convex-manifold.scalar-or-simd", "convex-manifold", "b3CollideHulls", "{\"shapes\":\"box-box\",\"capacity\":\"0x00000020\"}");
    writeLocalManifold(out, &manifold); fputc(',', out);
    oracle_key(out, "cacheHit"); oracle_u32(out, cache.hit); fputc(',', out);
    oracle_key(out, "hookVisits"); oracle_u32(out, b3OracleO4ConvexManifoldVisits);
    oracle_case_end(out);
}

static b3WorldId makeContactWorld(void)
{
    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = (b3Vec3){0.0f, -9.8f, 0.0f};
    def.enableSleep = false;
    def.workerCount = 1;
    return b3CreateWorld(&def);
}

static b3BodyId makeContactBody(b3WorldId world, b3BodyType type, b3Pos position)
{
    b3BodyDef def = b3DefaultBodyDef();
    def.type = type;
    def.position = position;
    def.enableSleep = false;
    return b3CreateBody(world, &def);
}

static void writeContactData(FILE* out, const b3ContactData* data, int count)
{
    oracle_key(out, "contactCount"); oracle_i32(out, count); fputc(',', out);
    oracle_key(out, "manifolds"); fputc('[', out);
    for (int i = 0; i < count; ++i) {
        if (i) fputc(',', out);
        const b3ContactData* item = data + i;
        fputs("{\"manifoldCount\":", out); oracle_i32(out, item->manifoldCount); fputs(",\"items\":[", out);
        for (int m = 0; m < item->manifoldCount; ++m) {
            if (m) fputc(',', out);
            const b3Manifold* manifold = item->manifolds + m;
            fputs("{\"normal\":", out); oracle_vec3(out, manifold->normal); fputs(",\"pointCount\":", out); oracle_i32(out, manifold->pointCount);
            fputs(",\"points\":[", out);
            for (int p = 0; p < manifold->pointCount; ++p) {
                if (p) fputc(',', out);
                fputs("{\"anchorA\":", out); oracle_vec3(out, manifold->points[p].anchorA); fputs(",\"anchorB\":", out); oracle_vec3(out, manifold->points[p].anchorB);
                fputs(",\"separation\":", out); oracle_f32(out, manifold->points[p].separation); fputs(",\"featureId\":", out); oracle_u32(out, manifold->points[p].featureId);
                fputs(",\"triangleIndex\":", out); oracle_i32(out, manifold->points[p].triangleIndex); fputs("}", out);
            }
            fputs("]}", out);
        }
        fputs("]}", out);
    }
    fputc(']', out);
}

static void writeMeshContact(FILE* out, bool* first)
{
    b3WorldId world = makeContactWorld();
    b3BodyId ground = makeContactBody(world, b3_staticBody, (b3Pos){0.0, 0.0, 0.0});
    b3BodyId ball = makeContactBody(world, b3_dynamicBody, (b3Pos){0.0, 0.55, 0.0});
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3MeshData* mesh = b3CreateBoxMesh((b3Vec3){0.0f, 0.0f, 0.0f}, (b3Vec3){2.0f, 0.2f, 2.0f}, true);
    b3CreateMeshShape(ground, &shapeDef, mesh, (b3Vec3){1.0f, 1.0f, 1.0f});
    b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    b3CreateSphereShape(ball, &shapeDef, &sphere);
    b3World_Step(world, 1.0f / 60.0f, 1);
    int capacity = b3Body_GetContactCapacity(ball);
    b3ContactData data[4] = {0};
    int count = b3Body_GetContactData(ball, data, capacity < 4 ? capacity : 4);
    oracle_case_begin(out, first, "o4.mesh-contact.scalar-or-simd", "mesh-contact", "b3ComputeMeshManifolds", "{\"mesh\":\"box\",\"visitor\":\"sphere\"}");
    writeContactData(out, data, count); fputc(',', out);
    oracle_key(out, "hookVisits"); oracle_u32(out, b3OracleO4MeshContactVisits);
    oracle_case_end(out);
    b3DestroyWorld(world);
    b3DestroyMesh(mesh);
}

static void writeConvexContact(FILE* out, bool* first)
{
    b3WorldId world = makeContactWorld();
    b3BodyId a = makeContactBody(world, b3_dynamicBody, (b3Pos){0.0, 0.0, 0.0});
    b3BodyId b = makeContactBody(world, b3_staticBody, (b3Pos){1.5, 0.0, 0.0});
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    b3ShapeId shapeA = b3CreateSphereShape(a, &shapeDef, &sphere);
    b3CreateSphereShape(b, &shapeDef, &sphere);
    b3World_Step(world, 1.0f / 60.0f, 1);
    b3ContactData data[4] = {0};
    int count = b3Shape_GetContactData(shapeA, data, 4);
    oracle_case_begin(out, first, "o4.convex-contact.scalar-or-simd", "convex-contact", "b3UpdateConvexContact", "{\"shapes\":\"sphere-sphere\",\"steps\":\"0x00000001\"}");
    writeContactData(out, data, count); fputc(',', out);
    oracle_key(out, "hookVisits"); oracle_u32(out, b3OracleO4ConvexContactVisits);
    oracle_case_end(out);
    b3DestroyWorld(world);
}

static void createBodiesAndBase(b3WorldId* worldOut, b3BodyId* aOut, b3BodyId* bOut)
{
    *worldOut = makeContactWorld();
    *aOut = makeContactBody(*worldOut, b3_dynamicBody, (b3Pos){0.0, 0.0, 0.0});
    *bOut = makeContactBody(*worldOut, b3_dynamicBody, (b3Pos){1.0, 0.0, 0.0});
    (void)bOut;
}

static void writeJointRecord(FILE* out, bool* first, const char* id, const char* symbol, b3JointId joint, b3WorldId world, float specific, b3Vec3 vector, bool hasVector)
{
    b3World_Step(world, 1.0f / 60.0f, 1);
    b3Vec3 force = b3Joint_GetConstraintForce(joint);
    b3Vec3 torque = b3Joint_GetConstraintTorque(joint);
    oracle_case_begin(out, first, id, "joint", symbol, "{\"bodies\":\"dynamic-dynamic\",\"steps\":\"0x00000001\"}");
    oracle_key(out, "valid"); oracle_u32(out, b3Joint_IsValid(joint)); fputc(',', out);
    oracle_key(out, "type"); oracle_u32(out, (uint32_t)b3Joint_GetType(joint)); fputc(',', out);
    oracle_key(out, "force"); oracle_vec3(out, force); fputc(',', out);
    oracle_key(out, "torque"); oracle_vec3(out, torque); fputc(',', out);
    oracle_key(out, "linearSeparation"); oracle_f32(out, b3Joint_GetLinearSeparation(joint)); fputc(',', out);
    oracle_key(out, "angularSeparation"); oracle_f32(out, b3Joint_GetAngularSeparation(joint)); fputc(',', out);
    oracle_key(out, "specific"); oracle_f32(out, specific);
    if (hasVector) { fputc(',', out); oracle_key(out, "specificVector"); oracle_vec3(out, vector); }
    fputc(',', out); oracle_key(out, "hookVisits"); oracle_u32(out, b3OracleO4JointVisits);
    oracle_case_end(out);
}

static void writeJoints(FILE* out, bool* first)
{
    b3WorldId world; b3BodyId a, b;
    createBodiesAndBase(&world, &a, &b);
    b3ParallelJointDef parallel = b3DefaultParallelJointDef(); parallel.base.bodyIdA = a; parallel.base.bodyIdB = b;
    b3JointId parallelId = b3CreateParallelJoint(world, &parallel);
    writeJointRecord(out, first, "o4.joint.parallel.scalar-or-simd", "b3CreateParallelJoint", parallelId, world, b3ParallelJoint_GetSpringHertz(parallelId), (b3Vec3){0}, false); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3DistanceJointDef distance = b3DefaultDistanceJointDef(); distance.base.bodyIdA = a; distance.base.bodyIdB = b; distance.length = 1.0f;
    b3JointId distanceId = b3CreateDistanceJoint(world, &distance);
    writeJointRecord(out, first, "o4.joint.distance.scalar-or-simd", "b3CreateDistanceJoint", distanceId, world, b3DistanceJoint_GetLength(distanceId), (b3Vec3){0}, false); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3MotorJointDef motor = b3DefaultMotorJointDef(); motor.base.bodyIdA = a; motor.base.bodyIdB = b;
    b3JointId motorId = b3CreateMotorJoint(world, &motor);
    writeJointRecord(out, first, "o4.joint.motor.scalar-or-simd", "b3CreateMotorJoint", motorId, world, b3MotorJoint_GetMaxVelocityForce(motorId), b3MotorJoint_GetLinearVelocity(motorId), true); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3FilterJointDef filter = b3DefaultFilterJointDef(); filter.base.bodyIdA = a; filter.base.bodyIdB = b;
    b3JointId filterId = b3CreateFilterJoint(world, &filter);
    writeJointRecord(out, first, "o4.joint.filter.scalar-or-simd", "b3CreateFilterJoint", filterId, world, 0.0f, (b3Vec3){0}, false); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3PrismaticJointDef prismatic = b3DefaultPrismaticJointDef(); prismatic.base.bodyIdA = a; prismatic.base.bodyIdB = b;
    b3JointId prismaticId = b3CreatePrismaticJoint(world, &prismatic);
    writeJointRecord(out, first, "o4.joint.prismatic.scalar-or-simd", "b3CreatePrismaticJoint", prismaticId, world, b3PrismaticJoint_GetTranslation(prismaticId), (b3Vec3){0}, false); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3RevoluteJointDef revolute = b3DefaultRevoluteJointDef(); revolute.base.bodyIdA = a; revolute.base.bodyIdB = b;
    b3JointId revoluteId = b3CreateRevoluteJoint(world, &revolute);
    writeJointRecord(out, first, "o4.joint.revolute.scalar-or-simd", "b3CreateRevoluteJoint", revoluteId, world, b3RevoluteJoint_GetAngle(revoluteId), (b3Vec3){0}, false); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3SphericalJointDef spherical = b3DefaultSphericalJointDef(); spherical.base.bodyIdA = a; spherical.base.bodyIdB = b;
    b3JointId sphericalId = b3CreateSphericalJoint(world, &spherical);
    writeJointRecord(out, first, "o4.joint.spherical.scalar-or-simd", "b3CreateSphericalJoint", sphericalId, world, b3SphericalJoint_GetConeAngle(sphericalId), b3SphericalJoint_GetMotorVelocity(sphericalId), true); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3WeldJointDef weld = b3DefaultWeldJointDef(); weld.base.bodyIdA = a; weld.base.bodyIdB = b;
    b3JointId weldId = b3CreateWeldJoint(world, &weld);
    writeJointRecord(out, first, "o4.joint.weld.scalar-or-simd", "b3CreateWeldJoint", weldId, world, b3WeldJoint_GetLinearHertz(weldId), (b3Vec3){0}, false); b3DestroyWorld(world);

    createBodiesAndBase(&world, &a, &b);
    b3WheelJointDef wheel = b3DefaultWheelJointDef(); wheel.base.bodyIdA = a; wheel.base.bodyIdB = b;
    b3JointId wheelId = b3CreateWheelJoint(world, &wheel);
    writeJointRecord(out, first, "o4.joint.wheel.scalar-or-simd", "b3CreateWheelJoint", wheelId, world, b3WheelJoint_GetSteeringAngle(wheelId), (b3Vec3){0}, false); b3DestroyWorld(world);
}

void oracle_write_o4(FILE* out, bool* first)
{
    writeConvexManifold(out, first);
    writeMeshContact(out, first);
    writeConvexContact(out, first);
    writeJoints(out, first);
}
