#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "oracle_hooks.h"
#include "scenario_table.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float f32(uint32_t bits) { float value; memcpy(&value, &bits, sizeof(value)); return value; }
static void hex32(FILE* out, uint32_t bits) { fprintf(out, "\"0x%08x\"", bits); }
static void hex64(FILE* out, uint64_t bits) { fprintf(out, "\"0x%016llx\"", (unsigned long long)bits); }
static uint32_t bits(float value) { uint32_t result; memcpy(&result, &value, sizeof(result)); return result; }
static void vec3(FILE* out, b3Vec3 value) { fputc('[', out); hex32(out, bits(value.x)); fputc(',', out); hex32(out, bits(value.y)); fputc(',', out); hex32(out, bits(value.z)); fputc(']', out); }
static void pos(FILE* out, b3Pos value) { fputc('[', out); hex32(out, bits((float)value.x)); fputc(',', out); hex32(out, bits((float)value.y)); fputc(',', out); hex32(out, bits((float)value.z)); fputc(']', out); }
static void command_id(FILE* out, const char* id) { fprintf(out, "\"%s\"", id); }
static void copy_file(FILE* out, FILE* input) { int c; rewind(input); while ((c = fgetc(input)) != EOF) fputc(c, out); }

static int run_scenario(int index, FILE* out)
{
    if (index < 0 || index >= scenario_record_count) { fprintf(stderr, "scenario index out of range: %d\n", index); return 2; }
    const ScenarioRecord* record = scenario_records + index;
    b3WorldId world = b3_nullWorldId;
    b3BodyId bodies[64] = { 0 };
    b3BoxHull boxes[64];
    b3Sphere spheres[64];
    int boxCount = 0, sphereCount = 0;
    int consumed = 0;
    int observed = 0;
    int hashes = 0;
    FILE* observationOutput = tmpfile();
    FILE* hashOutput = tmpfile();
    if (observationOutput == NULL || hashOutput == NULL) { fprintf(stderr, "temporary scenario output failed\n"); return 2; }

    for (int i = 0; i < record->commandCount; ++i) {
        const ScenarioCommand* command = record->commands + i;
        consumed += 1;
        switch (command->op) {
        case SCENARIO_WORLD: {
            b3WorldDef def = b3DefaultWorldDef();
            def.gravity = (b3Vec3){ f32(command->values[0]), f32(command->values[1]), f32(command->values[2]) };
            def.enableSleep = command->flag0 != 0;
            def.enableContinuous = command->flag1 != 0;
            def.workerCount = 1;
            world = b3CreateWorld(&def);
            break;
        }
        case SCENARIO_BODY: {
            if (B3_IS_NULL(world)) { fprintf(stderr, "body before world in %s\n", record->id); return 2; }
            b3BodyDef def = b3DefaultBodyDef();
            def.type = (b3BodyType)command->kind;
            def.position = (b3Pos){ f32(command->values[0]), f32(command->values[1]), f32(command->values[2]) };
            def.linearVelocity = (b3Vec3){ f32(command->values[3]), f32(command->values[4]), f32(command->values[5]) };
            def.angularVelocity = (b3Vec3){ f32(command->values[6]), f32(command->values[7]), f32(command->values[8]) };
            if (command->values[9] != 0) def.angularDamping = f32(command->values[9]);
            if (command->a < 0 || command->a >= 64) { fprintf(stderr, "invalid body id in %s\n", record->id); return 2; }
            bodies[command->a] = b3CreateBody(world, &def);
            break;
        }
        case SCENARIO_BOX_RESOURCE:
            if (command->a < 0 || command->a >= 64) return 2;
            boxes[command->a] = b3MakeBoxHull(f32(command->values[0]), f32(command->values[1]), f32(command->values[2]));
            if (command->a >= boxCount) boxCount = command->a + 1;
            break;
        case SCENARIO_SPHERE_RESOURCE:
            if (command->a < 0 || command->a >= 64) return 2;
            spheres[command->a] = (b3Sphere){ { 0.0f, 0.0f, 0.0f }, f32(command->values[0]) };
            if (command->a >= sphereCount) sphereCount = command->a + 1;
            break;
        case SCENARIO_SHAPE: {
            if (command->a < 0 || command->a >= 64 || command->b < 0 || command->b >= 64 || !b3Body_IsValid(bodies[command->b])) { fprintf(stderr, "invalid shape reference in %s\n", record->id); return 2; }
            b3ShapeDef def = b3DefaultShapeDef();
            int resource = (int)command->values[0];
            if (command->kind == 1) b3CreateHullShape(bodies[command->b], &def, &boxes[resource].base);
            else if (command->kind == 2) b3CreateSphereShape(bodies[command->b], &def, &spheres[resource]);
            else { fprintf(stderr, "unknown shape kind in %s\n", record->id); return 2; }
            break;
        }
        case SCENARIO_STEP:
            if (B3_IS_NULL(world)) return 2;
            b3World_Step(world, f32(command->values[0]), command->a);
            break;
        case SCENARIO_OBSERVE: {
            if (observed > 0) fputc(',', observationOutput);
            fprintf(observationOutput, "{\"step\":%d,\"bodies\":[", command->step);
            for (int j = 0; j < command->bodyCount; ++j) {
                if (j) fputc(',', observationOutput);
                int body = command->bodies[j];
                if (body < 0 || body >= 64 || !b3Body_IsValid(bodies[body])) { fprintf(stderr, "unknown observed body in %s\n", record->id); return 2; }
                b3WorldTransform transform = b3Body_GetTransform(bodies[body]);
                b3Vec3 linear = b3Body_GetLinearVelocity(bodies[body]);
                b3Vec3 angular = b3Body_GetAngularVelocity(bodies[body]);
                fprintf(observationOutput, "{\"id\":\"b%d\",\"p\":", body); pos(observationOutput, transform.p); fputs(",\"q\":[", observationOutput); hex32(observationOutput, bits(transform.q.v.x)); fputc(',', observationOutput); hex32(observationOutput, bits(transform.q.v.y)); fputc(',', observationOutput); hex32(observationOutput, bits(transform.q.v.z)); fputc(',', observationOutput); hex32(observationOutput, bits(transform.q.s)); fputs("],\"v\":", observationOutput); vec3(observationOutput, linear); fputs(",\"w\":", observationOutput); vec3(observationOutput, angular); fputc('}', observationOutput);
            }
            fputs("],\"receiptId\":", observationOutput); command_id(observationOutput, command->id); fputc('}', observationOutput); observed += 1;
            break;
        }
        case SCENARIO_HASH:
            if (B3_IS_NULL(world)) return 2;
            if (observed == 0) { fprintf(stderr, "hash before observation in %s\n", record->id); return 2; }
            if (hashes > 0) fputc(',', hashOutput);
            fprintf(hashOutput, "{\"step\":%d,\"value\":", command->step); hex64(hashOutput, b3OracleCallHashWorldStateId(world));
            fputs(",\"receiptId\":", hashOutput); command_id(hashOutput, command->id); fputc('}', hashOutput); hashes += 1;
            break;
        default:
            fprintf(stderr, "unknown command opcode %d in %s\n", command->op, record->id); return 2;
        }
    }
    if (B3_IS_NON_NULL(world)) b3DestroyWorld(world);
    fputs("{\"schema\":\"box3d-oracle/scenario-output/v1\",\"id\":", out); command_id(out, record->id);
    fputs(",\"name\":", out); command_id(out, record->name);
    fprintf(out, ",\"corpusDigest\":\"%s\",\"observations\":[", SCENARIO_CORPUS_DIGEST); copy_file(out, observationOutput);
    fputs("],\"hashes\":[", out); copy_file(out, hashOutput);
    fputs("],\"receipt\":{\"corpusDigest\":\"" SCENARIO_CORPUS_DIGEST "\",\"consumedCommands\":[", out);
    for (int i = 0; i < record->commandCount; ++i) { if (i) fputc(',', out); command_id(out, record->commands[i].id); }
    fputs("],\"observationIds\":[", out);
    int wroteObservationId = 0;
    for (int i = 0; i < record->commandCount; ++i) if (record->commands[i].op == SCENARIO_OBSERVE) { if (wroteObservationId++) fputc(',', out); command_id(out, record->commands[i].id); }
    fputs("]}}\n", out);
    (void)consumed; (void)hashes; (void)boxCount; (void)sphereCount;
    return 0;
}

int main(int argc, char** argv)
{
    if (argc != 3 || strcmp(argv[1], "--index") != 0) { fprintf(stderr, "usage: scenario-adapter --index <integer> <output.json>\n"); return 2; }
    char* end = NULL; long index = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') { fprintf(stderr, "scenario index must be an integer\n"); return 2; }
    return run_scenario((int)index, stdout);
}
