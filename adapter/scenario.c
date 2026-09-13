#include "box3d_oracle_adapter.h"
#include "scenario_commands.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "oracle_hooks.h"
#include "recording.h"
#include "physics_world.h"
#include <string.h>

#define SCENARIO_MAX_STEPS 2001
typedef struct { b3Pos p; b3Quat q; b3Vec3 v; b3Vec3 w; uint64_t hash; } ScenarioObservation;

static void writeObservation(FILE* out, const ScenarioObservation* item, int step)
{
    fputs("{\"step\":", out); oracle_i32(out, step); fputs(",\"position\":", out); oracle_vec3(out, (b3Vec3){(float)item->p.x, (float)item->p.y, (float)item->p.z});
    fputs(",\"rotation\":[", out); oracle_f32(out, item->q.v.x); fputc(',', out); oracle_f32(out, item->q.v.y); fputc(',', out); oracle_f32(out, item->q.v.z); fputc(',', out); oracle_f32(out, item->q.s); fputc(']', out);
    fputs(",\"linearVelocity\":", out); oracle_vec3(out, item->v); fputs(",\"angularVelocity\":", out); oracle_vec3(out, item->w); fputc('}', out);
}

static void writeScenario(FILE* out, bool* first, const OracleScenarioCommand* command)
{
    float gravityY = command->gravityY;
#ifdef BOX3D_SCENARIO_MUTATED
    gravityY = -9.0f;
#endif
    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.gravity = (b3Vec3){0.0f, gravityY, 0.0f};
    worldDef.enableSleep = command->sleep != 0;
    worldDef.enableContinuous = command->continuous != 0;
    worldDef.workerCount = 1;
    b3WorldId worldId = b3CreateWorld(&worldDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull hull = b3MakeBoxHull(0.25f, 0.25f, 0.25f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = (b3Pos){0.0, 5.0, 0.0};
    b3BodyId body;
    if (strcmp(command->setupKind, "free") == 0) {
        body = b3CreateBody(worldId, &bodyDef);
        b3CreateHullShape(body, &shapeDef, &hull.base);
    } else if (strcmp(command->setupKind, "sphere-drop") == 0) {
        b3BodyDef groundDef = b3DefaultBodyDef();
        groundDef.position = (b3Pos){0.0, -1.0, 0.0};
        b3BodyId ground = b3CreateBody(worldId, &groundDef);
        b3BoxHull groundHull = b3MakeBoxHull(20.0f, 1.0f, 20.0f);
        b3CreateHullShape(ground, &shapeDef, &groundHull.base);
        b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
        bodyDef.position = (b3Pos){0.0, 1.0, 0.0};
        body = b3CreateBody(worldId, &bodyDef);
        b3CreateSphereShape(body, &shapeDef, &sphere);
        for (int i = 1; i < 5; ++i) { bodyDef.position.y = 1.0 + 1.5 * i; b3BodyId extra = b3CreateBody(worldId, &bodyDef); b3CreateSphereShape(extra, &shapeDef, &sphere); }
    } else if (strcmp(command->setupKind, "stack") == 0 || strcmp(command->setupKind, "bench") == 0) {
        b3BodyDef groundDef = b3DefaultBodyDef(); groundDef.position = (b3Pos){0.0, -1.0, 0.0};
        b3BodyId ground = b3CreateBody(worldId, &groundDef); b3BoxHull groundHull = b3MakeBoxHull(20.0f, 1.0f, 20.0f); b3CreateHullShape(ground, &shapeDef, &groundHull.base);
        bodyDef.position = (b3Pos){0.0, 0.5, 0.0}; body = b3CreateBody(worldId, &bodyDef); b3CreateHullShape(body, &shapeDef, &hull.base);
        int count = strcmp(command->setupKind, "bench") == 0 ? 8 : 5;
        for (int i = 1; i < count; ++i) { bodyDef.position.y = 0.5 + i; b3BodyId extra = b3CreateBody(worldId, &bodyDef); b3CreateHullShape(extra, &shapeDef, &hull.base); }
    } else {
        b3BodyDef anchorDef = b3DefaultBodyDef(); anchorDef.position = (b3Pos){0.0, 5.0, 0.0};
        b3BodyId anchor = b3CreateBody(worldId, &anchorDef); b3BoxHull anchorHull = b3MakeBoxHull(0.25f, 0.25f, 0.25f); b3CreateHullShape(anchor, &shapeDef, &anchorHull.base);
        body = b3CreateBody(worldId, &bodyDef); b3CreateHullShape(body, &shapeDef, &hull.base);
    }

    ScenarioObservation observations[SCENARIO_MAX_STEPS];
    if (command->steps >= SCENARIO_MAX_STEPS) { b3DestroyWorld(worldId); return; }
    b3World* world = b3GetWorldFromId(worldId);
    for (int step = 0; step < command->steps; ++step) {
        b3World_Step(worldId, command->dt, command->subSteps);
        observations[step].p = b3Body_GetPosition(body);
        observations[step].q = b3Body_GetRotation(body);
        observations[step].v = b3Body_GetLinearVelocity(body);
        observations[step].w = b3Body_GetAngularVelocity(body);
        observations[step].hash = b3OracleCallHashWorldState(world);
    }

    char input[512];
    uint32_t gravityBits, dtBits;
    memcpy(&gravityBits, &gravityY, sizeof gravityBits);
    memcpy(&dtBits, &command->dt, sizeof dtBits);
    snprintf(input, sizeof(input), "{\"name\":\"%s\",\"legacyBuilder\":\"%s\",\"setupKind\":\"%s\",\"gravityY\":\"0x%08x\",\"timeStep\":\"0x%08x\",\"subStepCount\":%d,\"repeat\":%d}", command->name, command->legacyBuilder, command->setupKind, gravityBits, dtBits, command->subSteps, command->steps);
    oracle_case_begin(out, first, command->id, "scenario", "b3World_Step", input);
    oracle_key(out, "publicObservations"); fputc('[', out);
    for (int step = 0; step < command->steps; ++step) { if (step) fputc(',', out); writeObservation(out, observations + step, step); }
    fputc(']', out); fputc(',', out);
    oracle_key(out, "whiteBoxObservations"); fputc('[', out);
    for (int step = 0; step < command->steps; ++step) { if (step) fputc(',', out); fputs("{\"step\":", out); oracle_i32(out, step); fputs(",\"hash\":", out); oracle_u64(out, observations[step].hash); fputc('}', out); }
    fputc(']', out); fputc(',', out); oracle_key(out, "whiteBoxSource"); fputs("\"b3HashWorldState\"", out);
    oracle_case_end(out);
    b3DestroyWorld(worldId);
}

void oracle_write_scenarios(FILE* out, bool* first)
{
    for (int i = 0; i < ORACLE_SCENARIO_COUNT; ++i) writeScenario(out, first, ORACLE_SCENARIOS + i);
}
