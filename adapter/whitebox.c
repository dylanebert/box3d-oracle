#include "box3d_oracle_adapter.h"
#include "oracle_hooks.h"
#include "physics_world.h"
#include <box3d/box3d.h>
#include <string.h>

static b3WorldId makeWorld(void)
{
    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = (b3Vec3){ 0.0f, -9.8f, 0.0f };
    def.enableSleep = false;
    def.workerCount = 1;
    return b3CreateWorld(&def);
}

static b3BodyId makeBody(b3WorldId world, b3Pos position, b3Vec3 velocity)
{
    b3BodyDef def = b3DefaultBodyDef();
    def.type = b3_dynamicBody;
    def.position = position;
    def.linearVelocity = velocity;
    def.enableSleep = false;
    return b3CreateBody(world, &def);
}

static void writeHash(FILE* out, bool* first)
{
    b3WorldId worldId = makeWorld();
    makeBody(worldId, (b3Pos){ 3.0, -2.0, 5.0 }, (b3Vec3){ 1.0f, -2.0f, 0.5f });
    b3World* world = b3GetWorldFromId(worldId);
    uint64_t hash = b3OracleCallHashWorldState(world);
    oracle_case_begin(out, first, "whitebox.world-hash.v2", "whitebox", "b3HashWorldState", "{\"bodyCount\":\"0x00000001\"}");
    oracle_key(out, "hash"); oracle_u64(out, hash);
    oracle_case_end(out);
    b3DestroyWorld(worldId);
}

static void writeIntegrateVelocities(FILE* out, bool* first)
{
    b3WorldId worldId = makeWorld();
    b3BodyId body = makeBody(worldId, (b3Pos){ 0.0, 0.0, 0.0 }, (b3Vec3){ 2.0f, 3.0f, -1.0f });
    b3World* world = b3GetWorldFromId(worldId);
    b3OracleCallIntegrateVelocities(world, 0.016f);
    b3Vec3 linear = b3Body_GetLinearVelocity(body);
    b3Vec3 angular = b3Body_GetAngularVelocity(body);
    oracle_case_begin(out, first, "whitebox.integrate-velocities.v2", "whitebox", "b3IntegrateVelocitiesTask", "{\"h\":\"0x3c83126f\",\"body\":\"dynamic\"}");
    oracle_key(out, "linearVelocity"); oracle_vec3(out, linear); fputc(',', out);
    oracle_key(out, "angularVelocity"); oracle_vec3(out, angular);
    oracle_case_end(out);
    b3DestroyWorld(worldId);
}

static void writeIntegratePositions(FILE* out, bool* first)
{
    b3WorldId worldId = makeWorld();
    makeBody(worldId, (b3Pos){ 0.0, 0.0, 0.0 }, (b3Vec3){ 2.0f, 3.0f, -1.0f });
    b3World* world = b3GetWorldFromId(worldId);
    b3Vec3 deltaPosition;
    b3Quat deltaRotation;
    b3OracleCallIntegratePositions(world, 0.016f, &deltaPosition, &deltaRotation);
    oracle_case_begin(out, first, "whitebox.integrate-positions.v2", "whitebox", "b3IntegratePositionsTask", "{\"h\":\"0x3c83126f\",\"body\":\"dynamic\"}");
    oracle_key(out, "deltaPosition"); oracle_vec3(out, deltaPosition); fputc(',', out);
    oracle_key(out, "deltaRotation");
    fputc('[', out); oracle_f32(out, deltaRotation.v.x); fputc(',', out); oracle_f32(out, deltaRotation.v.y); fputc(',', out); oracle_f32(out, deltaRotation.v.z); fputc(',', out); oracle_f32(out, deltaRotation.s); fputc(']', out);
    oracle_case_end(out);
    b3DestroyWorld(worldId);
}

static void writeFinalize(FILE* out, bool* first)
{
    b3WorldId worldId = makeWorld();
    b3BodyId body = makeBody(worldId, (b3Pos){ 1.0, 2.0, 3.0 }, (b3Vec3){ 0.0f, 0.0f, 0.0f });
    b3World* world = b3GetWorldFromId(worldId);
    b3OracleCallFinalize(world, 0.016f);
    b3WorldTransform transform = b3Body_GetTransform(body);
    oracle_case_begin(out, first, "whitebox.finalize.v2", "whitebox", "b3FinalizeBodiesTask", "{\"dt\":\"0x3c83126f\",\"body\":\"dynamic\"}");
    oracle_key(out, "position"); oracle_vec3(out, transform.p); fputc(',', out);
    oracle_key(out, "rotation");
    fputc('[', out); oracle_f32(out, transform.q.v.x); fputc(',', out); oracle_f32(out, transform.q.v.y); fputc(',', out); oracle_f32(out, transform.q.v.z); fputc(',', out); oracle_f32(out, transform.q.s); fputc(']', out);
    oracle_case_end(out);
    b3DestroyWorld(worldId);
}

static void writeRecycle(FILE* out, bool* first)
{
    b3WorldId worldId = makeWorld();
    b3BodyId a = makeBody(worldId, (b3Pos){ 0.0, 0.0, 0.0 }, (b3Vec3){ 0.0f, 0.0f, 0.0f });
    b3BodyId b = makeBody(worldId, (b3Pos){ 1.5, 0.0, 0.0 }, (b3Vec3){ 0.0f, 0.0f, 0.0f });
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
    b3CreateSphereShape(a, &shapeDef, &sphere);
    b3CreateSphereShape(b, &shapeDef, &sphere);
    b3OracleResetRecycleVisits();
    b3World_Step(worldId, 1.0f / 60.0f, 1);
    b3World_Step(worldId, 1.0f / 60.0f, 1);
    b3Counters counters = b3World_GetCounters(worldId);
    oracle_case_begin(out, first, "whitebox.recycle.v2", "whitebox", "b3CollideTask", "{\"shape\":\"overlapping-spheres\",\"steps\":\"0x00000002\"}");
    oracle_key(out, "recycledContactCount"); oracle_i32(out, counters.recycledContactCount); fputc(',', out);
    oracle_key(out, "collideTaskVisits"); oracle_u32(out, b3OracleRecycleVisits);
    oracle_case_end(out);
    b3DestroyWorld(worldId);
}

void oracle_write_whitebox(FILE* out, bool* first)
{
    writeHash(out, first);
    writeIntegrateVelocities(out, first);
    writeIntegratePositions(out, first);
    writeFinalize(out, first);
    writeRecycle(out, first);
}
