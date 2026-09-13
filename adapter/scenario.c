#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "oracle_hooks.h"
#include "scenario_table.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float f32(uint32_t bits) { float value; memcpy(&value, &bits, sizeof(value)); return value; }
static uint32_t bits(float value) { uint32_t result; memcpy(&result, &value, sizeof(result)); return result; }
static void hex32(FILE* out, uint32_t value) { fprintf(out, "\"0x%08x\"", value); }
static void hex64(FILE* out, uint64_t value) { fprintf(out, "\"0x%016llx\"", (unsigned long long)value); }
static void vec3(FILE* out, b3Vec3 value) { fputc('[', out); hex32(out, bits(value.x)); fputc(',', out); hex32(out, bits(value.y)); fputc(',', out); hex32(out, bits(value.z)); fputc(']', out); }
static void pos(FILE* out, b3Pos value) { fputc('[', out); hex32(out, bits((float)value.x)); fputc(',', out); hex32(out, bits((float)value.y)); fputc(',', out); hex32(out, bits((float)value.z)); fputc(']', out); }
static void copy_file(FILE* out, FILE* input) { int c; rewind(input); while ((c = fgetc(input)) != EOF) fputc(c, out); }
static b3Vec3 v3(const ScenarioCommand* c, int offset) { return (b3Vec3){ f32(c->values[offset]), f32(c->values[offset + 1]), f32(c->values[offset + 2]) }; }
static b3Quat q4(const ScenarioCommand* c, int offset) { return (b3Quat){ .v = v3(c, offset), .s = f32(c->values[offset + 3]) }; }
static b3Transform xf(const ScenarioCommand* c, int offset) { return (b3Transform){ .p = v3(c, offset), .q = q4(c, offset + 3) }; }
#define HAS(c, n) (((c)->flags & (UINT64_C(1) << (n))) != 0)
#define JV(c, n) f32((c)->values[14 + (n)])
#define JBOOL(c, n) HAS((c), (n))
#define ENABLE(c, n) HAS((c), (48 + (n)))
#define E_SPRING 0
#define E_LIMIT 1
#define E_MOTOR 2
#define E_CONE 3
#define E_TWIST 4
#define E_SUSPENSION_SPRING 5
#define E_SUSPENSION_LIMIT 6
#define E_SPIN 7
#define E_STEERING 8
#define E_STEERING_LIMIT 9
#define SHAPE_SENSOR (UINT64_C(1) << 60)
#define SHAPE_SENSOR_EVENTS (UINT64_C(1) << 59)

static b3SurfaceMaterial make_material(const ScenarioMaterialData* source)
{
    b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
    material.friction = f32(source->friction); material.restitution = f32(source->restitution); material.rollingResistance = f32(source->rollingResistance);
    material.tangentVelocity = (b3Vec3){ f32(source->tangentVelocity[0]), f32(source->tangentVelocity[1]), f32(source->tangentVelocity[2]) };
    material.userMaterialId = source->userMaterialId; material.customColor = source->customColor;
    return material;
}

static b3Transform child_transform(const ScenarioCompoundChild* child)
{
    return (b3Transform){ .p = (b3Vec3){ f32(child->transform[0]), f32(child->transform[1]), f32(child->transform[2]) }, .q = (b3Quat){ .v = (b3Vec3){ f32(child->transform[3]), f32(child->transform[4]), f32(child->transform[5]) }, .s = f32(child->transform[6]) } };
}

static b3CompoundData* make_compound(const ScenarioCompoundData* source, b3BoxHull* boxes, b3Sphere* spheres, b3Capsule* capsules, b3MeshData** meshes)
{
    b3CompoundDef def = { 0 }; b3CompoundCapsuleDef* capsuleDefs = NULL; b3CompoundHullDef* hullDefs = NULL; b3CompoundMeshDef* meshDefs = NULL; b3CompoundSphereDef* sphereDefs = NULL;
    if (source->capsuleCount) capsuleDefs = calloc((size_t)source->capsuleCount, sizeof(*capsuleDefs));
    if (source->hullCount) hullDefs = calloc((size_t)source->hullCount, sizeof(*hullDefs));
    if (source->meshCount) meshDefs = calloc((size_t)source->meshCount, sizeof(*meshDefs));
    if (source->sphereCount) sphereDefs = calloc((size_t)source->sphereCount, sizeof(*sphereDefs));
    if ((source->capsuleCount && !capsuleDefs) || (source->hullCount && !hullDefs) || (source->meshCount && !meshDefs) || (source->sphereCount && !sphereDefs)) { free(capsuleDefs); free(hullDefs); free(meshDefs); free(sphereDefs); return NULL; }
    for (int i = 0; i < source->capsuleCount; ++i) { const ScenarioCompoundChild* c = source->capsules + i; capsuleDefs[i].capsule = capsules[c->resource]; capsuleDefs[i].material = make_material(c->materials); }
    for (int i = 0; i < source->hullCount; ++i) { const ScenarioCompoundChild* c = source->hulls + i; hullDefs[i].hull = &boxes[c->resource].base; hullDefs[i].transform = child_transform(c); hullDefs[i].material = make_material(c->materials); }
    for (int i = 0; i < source->meshCount; ++i) { const ScenarioCompoundChild* c = source->meshes + i; meshDefs[i].meshData = meshes[c->resource]; meshDefs[i].transform = child_transform(c); meshDefs[i].scale = (b3Vec3){ f32(c->scale[0]), f32(c->scale[1]), f32(c->scale[2]) }; b3SurfaceMaterial* materials = calloc((size_t)c->materialCount, sizeof(*materials)); if (!materials) { free(capsuleDefs); free(hullDefs); free(meshDefs); free(sphereDefs); return NULL; } for (int j = 0; j < c->materialCount; ++j) materials[j] = make_material(c->materials + j); meshDefs[i].materials = materials; meshDefs[i].materialCount = c->materialCount; }
    for (int i = 0; i < source->sphereCount; ++i) { const ScenarioCompoundChild* c = source->spheres + i; sphereDefs[i].sphere = spheres[c->resource]; sphereDefs[i].material = make_material(c->materials); }
    def.capsules = capsuleDefs; def.capsuleCount = source->capsuleCount; def.hulls = hullDefs; def.hullCount = source->hullCount; def.meshes = meshDefs; def.meshCount = source->meshCount; def.spheres = sphereDefs; def.sphereCount = source->sphereCount;
    b3CompoundData* result = b3CreateCompound(&def);
    for (int i = 0; i < source->meshCount; ++i) free((void*)meshDefs[i].materials);
    free(capsuleDefs); free(hullDefs); free(meshDefs); free(sphereDefs); return result;
}

static b3MeshData* make_mesh(const ScenarioMeshData* source)
{
    b3Vec3* vertices = malloc((size_t)source->vertexCount * sizeof(*vertices));
    int32_t* indices = malloc((size_t)source->triangleCount * 3u * sizeof(*indices));
    if (!vertices || !indices) { free(vertices); free(indices); return NULL; }
    for (int i = 0; i < source->vertexCount; ++i) vertices[i] = (b3Vec3){ f32(source->vertices[3 * i]), f32(source->vertices[3 * i + 1]), f32(source->vertices[3 * i + 2]) };
    for (int i = 0; i < source->triangleCount * 3; ++i) indices[i] = source->indices[i];
    b3MeshDef def = { 0 }; def.vertices = vertices; def.indices = indices; def.vertexCount = source->vertexCount; def.triangleCount = source->triangleCount; def.useMedianSplit = source->useMedianSplit != 0; def.identifyEdges = source->identifyEdges != 0;
    b3MeshData* mesh = b3CreateMesh(&def, NULL, 0); free(vertices); free(indices); return mesh;
}

static b3HullData* make_hull(const ScenarioHullData* source)
{
    b3Vec3* points = malloc((size_t)source->pointCount * sizeof(*points));
    if (!points) return NULL;
    for (int i = 0; i < source->pointCount; ++i) points[i] = (b3Vec3){ f32(source->points[3 * i]), f32(source->points[3 * i + 1]), f32(source->points[3 * i + 2]) };
    b3HullData* hull = b3CreateHull(points, source->pointCount, source->pointCount); free(points); return hull;
}

static b3HeightFieldData* make_height_field(const ScenarioHeightFieldData* source)
{
    int heightCount = source->countX * source->countZ; int cellCount = (source->countX - 1) * (source->countZ - 1);
    float* heights = malloc((size_t)heightCount * sizeof(*heights)); uint8_t* materials = malloc((size_t)cellCount * sizeof(*materials));
    if (!heights || !materials) { free(heights); free(materials); return NULL; }
    for (int i = 0; i < heightCount; ++i) heights[i] = f32(source->samples[i]); for (int i = 0; i < cellCount; ++i) materials[i] = source->materials[i];
    b3HeightFieldDef def = { 0 }; def.heights = heights; def.materialIndices = materials; def.scale = (b3Vec3){ f32(source->scaleX), f32(source->scaleY), f32(source->scaleZ) }; def.countX = source->countX; def.countZ = source->countZ; def.globalMinimumHeight = f32(source->minHeight); def.globalMaximumHeight = f32(source->maxHeight); def.clockwiseWinding = source->clockwiseWinding != 0;
    b3HeightFieldData* field = b3CreateHeightField(&def); free(heights); free(materials); return field;
}

static b3Transform normalized_frame(const ScenarioCommand* c, int offset) { b3Transform result = xf(c, offset); result.q = b3NormalizeQuat(result.q); return result; }
static void set_base(b3JointDef* base, const ScenarioCommand* c, b3BodyId* bodies)
{
    base->bodyIdA = bodies[c->a]; base->bodyIdB = bodies[c->b];
    base->localFrameA = HAS(c, 55) ? normalized_frame(c, 0) : xf(c, 0); base->localFrameB = HAS(c, 55) ? normalized_frame(c, 7) : xf(c, 7);
}

static int create_joint(const ScenarioCommand* c, b3WorldId world, b3BodyId* bodies)
{
    if (c->a < 0 || c->a >= 512 || c->b < 0 || c->b >= 512 || !b3Body_IsValid(bodies[c->a]) || !b3Body_IsValid(bodies[c->b])) return 0;
    switch (c->op) {
    case SCENARIO_REVOLUTE: {
        b3RevoluteJointDef d = b3DefaultRevoluteJointDef(); set_base(&d.base, c, bodies);
        if (JBOOL(c, 0)) d.targetAngle = JV(c, 0); if (JBOOL(c, 1)) d.hertz = JV(c, 1); if (JBOOL(c, 2)) d.dampingRatio = JV(c, 2);
        if (JBOOL(c, 3)) d.lowerAngle = JV(c, 3); if (JBOOL(c, 4)) d.upperAngle = JV(c, 4); if (JBOOL(c, 5)) d.maxMotorTorque = JV(c, 5); if (JBOOL(c, 6)) d.motorSpeed = JV(c, 6);
        d.enableSpring = ENABLE(c, E_SPRING); d.enableLimit = ENABLE(c, E_LIMIT); d.enableMotor = ENABLE(c, E_MOTOR);
        (void)b3CreateRevoluteJoint(world, &d); return 1;
    }
    case SCENARIO_WELD: { b3WeldJointDef d = b3DefaultWeldJointDef(); set_base(&d.base, c, bodies); if (JBOOL(c, 10)) d.linearHertz = JV(c, 10); if (JBOOL(c, 11)) d.linearDampingRatio = JV(c, 11); if (JBOOL(c, 13)) d.angularHertz = JV(c, 13); if (JBOOL(c, 14)) d.angularDampingRatio = JV(c, 14); (void)b3CreateWeldJoint(world, &d); return 1; }
    case SCENARIO_PARALLEL: { b3ParallelJointDef d = b3DefaultParallelJointDef(); set_base(&d.base, c, bodies); if (JBOOL(c, 1)) d.hertz = JV(c, 1); if (JBOOL(c, 2)) d.dampingRatio = JV(c, 2); if (JBOOL(c, 7)) d.maxTorque = JV(c, 7); (void)b3CreateParallelJoint(world, &d); return 1; }
    case SCENARIO_MOTOR: { b3MotorJointDef d = b3DefaultMotorJointDef(); set_base(&d.base, c, bodies); if (JBOOL(c, 38)) d.linearVelocity = v3(c, 56); if (JBOOL(c, 8)) d.maxVelocityForce = JV(c, 8); if (JBOOL(c, 39)) d.angularVelocity = v3(c, 59); if (JBOOL(c, 9)) d.maxVelocityTorque = JV(c, 9); if (JBOOL(c, 10)) d.linearHertz = JV(c, 10); if (JBOOL(c, 11)) d.linearDampingRatio = JV(c, 11); if (JBOOL(c, 12)) d.maxSpringForce = JV(c, 12); if (JBOOL(c, 13)) d.angularHertz = JV(c, 13); if (JBOOL(c, 14)) d.angularDampingRatio = JV(c, 14); if (JBOOL(c, 15)) d.maxSpringTorque = JV(c, 15); (void)b3CreateMotorJoint(world, &d); return 1; }
    case SCENARIO_DISTANCE: { b3DistanceJointDef d = b3DefaultDistanceJointDef(); set_base(&d.base, c, bodies); if (JBOOL(c, 16)) d.length = JV(c, 16); if (JBOOL(c, 1)) d.hertz = JV(c, 1); if (JBOOL(c, 2)) d.dampingRatio = JV(c, 2); if (JBOOL(c, 17)) d.minLength = JV(c, 17); if (JBOOL(c, 18)) d.maxLength = JV(c, 18); if (JBOOL(c, 19)) d.maxMotorForce = JV(c, 19); if (JBOOL(c, 6)) d.motorSpeed = JV(c, 6); d.enableSpring = ENABLE(c, E_SPRING); d.enableLimit = ENABLE(c, E_LIMIT); d.enableMotor = ENABLE(c, E_MOTOR); (void)b3CreateDistanceJoint(world, &d); return 1; }
    case SCENARIO_PRISMATIC: { b3PrismaticJointDef d = b3DefaultPrismaticJointDef(); set_base(&d.base, c, bodies); if (JBOOL(c, 1)) d.hertz = JV(c, 1); if (JBOOL(c, 2)) d.dampingRatio = JV(c, 2); if (JBOOL(c, 20)) d.targetTranslation = JV(c, 20); if (JBOOL(c, 21)) d.lowerTranslation = JV(c, 21); if (JBOOL(c, 22)) d.upperTranslation = JV(c, 22); if (JBOOL(c, 19)) d.maxMotorForce = JV(c, 19); if (JBOOL(c, 6)) d.motorSpeed = JV(c, 6); d.enableSpring = ENABLE(c, E_SPRING); d.enableLimit = ENABLE(c, E_LIMIT); d.enableMotor = ENABLE(c, E_MOTOR); (void)b3CreatePrismaticJoint(world, &d); return 1; }
    case SCENARIO_SPHERICAL: { b3SphericalJointDef d = b3DefaultSphericalJointDef(); set_base(&d.base, c, bodies); if (JBOOL(c, 1)) d.hertz = JV(c, 1); if (JBOOL(c, 2)) d.dampingRatio = JV(c, 2); if (JBOOL(c, 23)) d.coneAngle = JV(c, 23); if (JBOOL(c, 24)) d.lowerTwistAngle = JV(c, 24); if (JBOOL(c, 25)) d.upperTwistAngle = JV(c, 25); if (JBOOL(c, 5)) d.maxMotorTorque = JV(c, 5); if (JBOOL(c, 40)) d.motorVelocity = v3(c, 62); d.enableSpring = ENABLE(c, E_SPRING); d.enableConeLimit = ENABLE(c, E_CONE); d.enableTwistLimit = ENABLE(c, E_TWIST); d.enableMotor = ENABLE(c, E_MOTOR); (void)b3CreateSphericalJoint(world, &d); return 1; }
    case SCENARIO_WHEEL: { b3WheelJointDef d = b3DefaultWheelJointDef(); set_base(&d.base, c, bodies); if (JBOOL(c, 32)) d.suspensionHertz = JV(c, 32); if (JBOOL(c, 33)) d.suspensionDampingRatio = JV(c, 33); if (JBOOL(c, 34)) d.lowerSuspensionLimit = JV(c, 34); if (JBOOL(c, 35)) d.upperSuspensionLimit = JV(c, 35); if (JBOOL(c, 30)) d.maxSpinTorque = JV(c, 30); if (JBOOL(c, 31)) d.spinSpeed = JV(c, 31); if (JBOOL(c, 36)) d.steeringHertz = JV(c, 36); if (JBOOL(c, 37)) d.steeringDampingRatio = JV(c, 37); if (JBOOL(c, 27)) d.targetSteeringAngle = JV(c, 27); if (JBOOL(c, 26)) d.maxSteeringTorque = JV(c, 26); if (JBOOL(c, 28)) d.lowerSteeringLimit = JV(c, 28); if (JBOOL(c, 29)) d.upperSteeringLimit = JV(c, 29); d.enableSuspensionSpring = ENABLE(c, E_SUSPENSION_SPRING) ? true : d.enableSuspensionSpring; d.enableSuspensionLimit = ENABLE(c, E_SUSPENSION_LIMIT); d.enableSpinMotor = ENABLE(c, E_SPIN); d.enableSteering = ENABLE(c, E_STEERING); d.enableSteeringLimit = ENABLE(c, E_STEERING_LIMIT); (void)b3CreateWheelJoint(world, &d); return 1; }
    default: return 0;
    }
}

static int same_shape(b3ShapeId a, b3ShapeId b) { return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation; }
static void shape_label(FILE* out, b3ShapeId id, b3ShapeId* shapes) { for (int i = 0; i < 512; ++i) if (same_shape(id, shapes[i])) { fprintf(out, "s%d", i); return; } fputs("unknown", out); }

static int run_scenario(int index, FILE* out)
{
    if (index < 0 || index >= scenario_record_count) return 2;
    const ScenarioRecord* record = scenario_records + index; b3WorldId world = b3_nullWorldId; b3BodyId bodies[512] = { 0 }; b3ShapeId shapes[512] = { 0 }; b3BoxHull boxes[512]; b3Sphere spheres[512]; b3Capsule capsules[512]; b3HullData* hulls[512] = { 0 }; b3MeshData* meshes[512] = { 0 }; b3HeightFieldData* heightFields[512] = { 0 }; int jointCount = 0; int observations = 0; int eventObservations = 0; FILE* observationOutput = tmpfile(); FILE* hashOutput = tmpfile(); FILE* eventOutput = tmpfile();
    if (!observationOutput || !hashOutput || !eventOutput) return 2;
    for (int i = 0; i < record->commandCount; ++i) {
        const ScenarioCommand* c = record->commands + i;
        switch (c->op) {
        case SCENARIO_WORLD: { b3WorldDef d = b3DefaultWorldDef(); d.gravity = v3(c, 0); d.enableSleep = (c->flags & (UINT64_C(1) << 62)) != 0; d.enableContinuous = (c->flags & (UINT64_C(1) << 63)) != 0; d.workerCount = 1; if (c->capacity[0] > 0) { d.capacity.staticShapeCount = c->capacity[0]; d.capacity.dynamicShapeCount = c->capacity[1]; d.capacity.staticBodyCount = c->capacity[2]; d.capacity.dynamicBodyCount = c->capacity[3]; d.capacity.contactCount = c->capacity[4]; } world = b3CreateWorld(&d); break; }
        case SCENARIO_BODY: case SCENARIO_SPAWN: { if (!B3_IS_NON_NULL(world) || c->a < 0 || c->a >= 512) return 2; b3BodyDef d = b3DefaultBodyDef(); d.type = (b3BodyType)c->kind; d.position = (b3Pos){ f32(c->values[0]), f32(c->values[1]), f32(c->values[2]) }; d.rotation = q4(c, 3); d.linearVelocity = v3(c, 7); d.angularVelocity = v3(c, 10); d.linearDamping = f32(c->values[13]); d.angularDamping = f32(c->values[14]); d.sleepThreshold = f32(c->values[15]); d.isBullet = HAS(c, 61); bodies[c->a] = b3CreateBody(world, &d); break; }
        case SCENARIO_BOX_RESOURCE: { b3Vec3 center = v3(c, 3); boxes[c->a] = (center.x == 0.0f && center.y == 0.0f && center.z == 0.0f) ? b3MakeBoxHull(f32(c->values[0]), f32(c->values[1]), f32(c->values[2])) : b3MakeOffsetBoxHull(f32(c->values[0]), f32(c->values[1]), f32(c->values[2]), center); break; }
        case SCENARIO_SPHERE_RESOURCE: spheres[c->a] = (b3Sphere){ v3(c, 0), f32(c->values[3]) }; break;
        case SCENARIO_CAPSULE_RESOURCE: capsules[c->a] = (b3Capsule){ v3(c, 0), v3(c, 3), f32(c->values[6]) }; break;
        case SCENARIO_HULL_RESOURCE: if (c->hull == NULL || c->a < 0 || c->a >= 512 || (hulls[c->a] = make_hull(c->hull)) == NULL) return 2; break;
        case SCENARIO_MESH_RESOURCE: if (c->mesh == NULL || c->a < 0 || c->a >= 512 || (meshes[c->a] = make_mesh(c->mesh)) == NULL) return 2; break;
        case SCENARIO_HEIGHT_RESOURCE: if (c->heightField == NULL || c->a < 0 || c->a >= 512 || (heightFields[c->a] = make_height_field(c->heightField)) == NULL) return 2; break;
        case SCENARIO_COMPOUND_RESOURCE: break;
        case SCENARIO_SHAPE: { if (!b3Body_IsValid(bodies[c->b])) return 2; b3ShapeDef d = b3DefaultShapeDef(); d.baseMaterial.friction = f32(c->values[16]); d.baseMaterial.restitution = f32(c->values[17]); d.baseMaterial.rollingResistance = f32(c->values[1]); d.filter.groupIndex = (int)c->values[2]; d.density = f32(c->values[15]); d.updateBodyMass = !HAS(c, 57); d.invokeContactCreation = !HAS(c, 58); d.isSensor = (c->flags & SHAPE_SENSOR) != 0; d.enableSensorEvents = (c->flags & SHAPE_SENSOR_EVENTS) != 0; int resource = (int)c->values[0]; if (c->kind == 1) shapes[c->a] = b3CreateHullShape(bodies[c->b], &d, &boxes[resource].base); else if (c->kind == 2) shapes[c->a] = b3CreateSphereShape(bodies[c->b], &d, &spheres[resource]); else if (c->kind == 3) shapes[c->a] = b3CreateCapsuleShape(bodies[c->b], &d, &capsules[resource]); else if (c->kind == 4) shapes[c->a] = b3CreateMeshShape(bodies[c->b], &d, meshes[resource], v3(c, 3)); else if (c->kind == 5) shapes[c->a] = b3CreateHeightFieldShape(bodies[c->b], &d, heightFields[resource]); else if (c->kind == 7) shapes[c->a] = b3CreateHullShape(bodies[c->b], &d, hulls[resource]); else if (c->kind == 6 && c->compound != NULL) { b3CompoundData* compound = make_compound(c->compound, boxes, spheres, capsules, meshes); if (!compound) return 2; shapes[c->a] = b3CreateBakedCompoundShape(bodies[c->b], &d, compound); b3DestroyCompound(compound); } else return 2; if (!b3Shape_IsValid(shapes[c->a])) return 2; break; }
        case SCENARIO_FILTER: { if (!B3_IS_NON_NULL(world) || c->a < 0 || c->a >= 512 || c->b < 0 || c->b >= 512 || !b3Body_IsValid(bodies[c->a]) || !b3Body_IsValid(bodies[c->b])) return 2; b3FilterJointDef d = b3DefaultFilterJointDef(); d.base.bodyIdA = bodies[c->a]; d.base.bodyIdB = bodies[c->b]; if (!b3Joint_IsValid(b3CreateFilterJoint(world, &d))) return 2; jointCount++; break; }
        case SCENARIO_REVOLUTE: case SCENARIO_WELD: case SCENARIO_PARALLEL: case SCENARIO_MOTOR: case SCENARIO_DISTANCE: case SCENARIO_PRISMATIC: case SCENARIO_SPHERICAL: case SCENARIO_WHEEL: if (!B3_IS_NON_NULL(world) || !create_joint(c, world, bodies)) return 2; jointCount++; break;
        case SCENARIO_APPLY_MASS: if (!b3Body_IsValid(bodies[c->a])) return 2; b3Body_ApplyMassFromShapes(bodies[c->a]); break;
        case SCENARIO_SET_VELOCITY: if (!b3Body_IsValid(bodies[c->a])) return 2; b3Body_SetLinearVelocity(bodies[c->a], v3(c, 0)); b3Body_SetAngularVelocity(bodies[c->a], v3(c, 3)); break;
        case SCENARIO_TARGET_TRANSFORM: if (!b3Body_IsValid(bodies[c->a])) return 2; b3Body_SetTargetTransform(bodies[c->a], xf(c, 0), f32(c->values[7]), HAS(c, 56)); break;
        case SCENARIO_STEP: if (!B3_IS_NON_NULL(world)) return 2; b3World_Step(world, f32(c->values[0]), c->substeps); break;
        case SCENARIO_OBSERVE: { if (!B3_IS_NON_NULL(world)) return 2; if (observations) fputc(',', observationOutput); fprintf(observationOutput, "{\"step\":%d,\"bodies\":[", c->step); for (int j = 0; j < c->bodyCount; ++j) { if (j) fputc(',', observationOutput); if (c->bodies[j] < 0 || c->bodies[j] >= 512 || !b3Body_IsValid(bodies[c->bodies[j]])) return 2; b3WorldTransform t = b3Body_GetTransform(bodies[c->bodies[j]]); fprintf(observationOutput, "{\"id\":\"b%d\",\"p\":", c->bodies[j]); pos(observationOutput, t.p); fputs(",\"q\":[", observationOutput); hex32(observationOutput, bits(t.q.v.x)); fputc(',', observationOutput); hex32(observationOutput, bits(t.q.v.y)); fputc(',', observationOutput); hex32(observationOutput, bits(t.q.v.z)); fputc(',', observationOutput); hex32(observationOutput, bits(t.q.s)); fputs("],\"v\":", observationOutput); vec3(observationOutput, b3Body_GetLinearVelocity(bodies[c->bodies[j]])); fputs(",\"w\":", observationOutput); vec3(observationOutput, b3Body_GetAngularVelocity(bodies[c->bodies[j]])); fputc('}', observationOutput); } fputs("],\"receiptId\":\"", observationOutput); fputs(c->id, observationOutput); fputs("\"}", observationOutput); observations++; break; }
        case SCENARIO_HASH: { if (!B3_IS_NON_NULL(world) || observations == 0) return 2; if (observations > 1) fputc(',', hashOutput); fprintf(hashOutput, "{\"step\":%d,\"value\":", c->step); hex64(hashOutput, b3OracleCallHashWorldStateId(world)); fprintf(hashOutput, ",\"receiptId\":\"%s\"}", c->id); break; }
        case SCENARIO_SENSOR_EVENTS: { if (!B3_IS_NON_NULL(world) || c->a < 0 || c->a >= 512 || !b3Shape_IsValid(shapes[c->a]) || !b3Shape_IsSensor(shapes[c->a])) return 2; b3SensorEvents events = b3World_GetSensorEvents(world); if (eventObservations++) fputc(',', eventOutput); fprintf(eventOutput, "{\"step\":%d,\"begin\":[", c->step); int first = 1; for (int j = 0; j < events.beginCount; ++j) if (same_shape(events.beginEvents[j].sensorShapeId, shapes[c->a])) { if (!first) fputc(',', eventOutput); first = 0; fputs("{\"sensor\":\"", eventOutput); shape_label(eventOutput, events.beginEvents[j].sensorShapeId, shapes); fputs("\",\"visitor\":\"", eventOutput); shape_label(eventOutput, events.beginEvents[j].visitorShapeId, shapes); fputs("\"}", eventOutput); } fputs("],\"end\":[", eventOutput); first = 1; for (int j = 0; j < events.endCount; ++j) if (same_shape(events.endEvents[j].sensorShapeId, shapes[c->a])) { if (!first) fputc(',', eventOutput); first = 0; fputs("{\"sensor\":\"", eventOutput); shape_label(eventOutput, events.endEvents[j].sensorShapeId, shapes); fputs("\",\"visitor\":\"", eventOutput); shape_label(eventOutput, events.endEvents[j].visitorShapeId, shapes); fputs("\"}", eventOutput); } fprintf(eventOutput, "],\"receiptId\":\"%s\"}", c->id); break; }
        default: return 2;
        }
    }
    if (jointCount != record->requiredJointCount) return 2;
    fputs("{\"schema\":\"box3d-oracle/scenario-output/v1\",\"id\":\"", out); fputs(record->id, out); fputs("\",\"name\":\"", out); fputs(record->name, out); fprintf(out, "\",\"corpusDigest\":\"%s\",\"observations\":[", SCENARIO_CORPUS_DIGEST); copy_file(out, observationOutput); fprintf(out, "],\"hashes\":["); copy_file(out, hashOutput); fprintf(out, "],\"sensorEvents\":["); copy_file(out, eventOutput); fprintf(out, "],\"receipt\":{\"corpusDigest\":\"%s\",\"consumedCommands\":[", SCENARIO_CORPUS_DIGEST); for (int i = 0; i < record->commandCount; ++i) { if (i) fputc(',', out); fprintf(out, "\"%s\"", record->commands[i].id); } fputs("],\"observationIds\":[", out); int first = 1; for (int i = 0; i < record->commandCount; ++i) if (record->commands[i].op == SCENARIO_OBSERVE) { if (!first) fputc(',', out); first = 0; fprintf(out, "\"%s\"", record->commands[i].id); } fputs("]}}\n", out); if (B3_IS_NON_NULL(world)) b3DestroyWorld(world); for (int i = 0; i < 512; ++i) { if (meshes[i]) b3DestroyMesh(meshes[i]); if (heightFields[i]) b3DestroyHeightField(heightFields[i]); if (hulls[i]) b3DestroyHull(hulls[i]); } fclose(observationOutput); fclose(hashOutput); fclose(eventOutput); return 0;
}
int main(int argc, char** argv) { if (argc != 3 || strcmp(argv[1], "--index") != 0) return 2; char* end = NULL; long index = strtol(argv[2], &end, 10); if (end == argv[2] || *end) return 2; return run_scenario((int)index, stdout); }
