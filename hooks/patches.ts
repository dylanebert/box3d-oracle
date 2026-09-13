export type OraclePatch = {
  file: string;
  marker: string;
  anchor: string;
  addition: string;
};

export const PATCHES: OraclePatch[] = [
  {
    file: "src/recording.c",
    marker: "recording.hash-world-state",
    anchor: '#include "recording.h"\n',
    addition: '#include "recording.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\nuint32_t b3OracleO4ConvexManifoldVisits = 0;\nuint32_t b3OracleO4MeshContactVisits = 0;\nuint32_t b3OracleO4ConvexContactVisits = 0;\nuint32_t b3OracleO4JointVisits = 0;\n#endif\n',
  },
  {
    file: "src/recording.c",
    marker: "recording.hash-world-state.sentinel",
    anchor: "\tuint64_t hash = B3_SNAP_FNV_INIT;\n",
    addition: "\tuint64_t hash = B3_SNAP_FNV_INIT;\n#ifdef B3_ORACLE_SENTINELS\n\thash ^= 0x9e3779b97f4a7c15ull;\n#endif\n",
  },
  {
    file: "src/recording.c",
    marker: "recording.hash-world-state.bridge",
    anchor: "\treturn hash;\n}\n",
    addition: "\treturn hash;\n}\n#ifdef B3_ORACLE_HOOKS\nuint64_t b3OracleCallHashWorldState( b3World* world )\n{\n\treturn b3HashWorldState( world );\n}\n#endif\n",
  },
  {
    file: "src/solver.c",
    marker: "solver.scalar-phases",
    anchor: '#include "solver.h"\n',
    addition: '#include "solver.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
  },
  {
    file: "src/solver.c",
    marker: "solver.integrate-velocities.sentinel",
    anchor: "\tstate->angularVelocity = w;\n\t}\n\n\tb3TracyCZoneEnd( integrate_velocity );\n",
    addition: "\tstate->angularVelocity = w;\n#ifdef B3_ORACLE_SENTINELS\n\t\tstate->linearVelocity.x += 0x1p-20f;\n#endif\n\t}\n\n\tb3TracyCZoneEnd( integrate_velocity );\n",
  },
  {
    file: "src/solver.c",
    marker: "solver.integrate-positions.sentinel",
    anchor: "\t\tstate->deltaRotation = b3IntegrateRotation( state->deltaRotation, b3MulSV( h, w ) );\n\t}\n\n\tb3TracyCZoneEnd( integrate_positions );\n",
    addition: "\t\tstate->deltaRotation = b3IntegrateRotation( state->deltaRotation, b3MulSV( h, w ) );\n#ifdef B3_ORACLE_SENTINELS\n\t\tstate->deltaPosition.y += 0x1p-20f;\n#endif\n\t}\n\n\tb3TracyCZoneEnd( integrate_positions );\n",
  },
  {
    file: "src/solver.c",
    marker: "solver.finalize.sentinel",
    anchor: "\t\tsim->transform.p = b3OffsetPos( sim->center, b3Neg( b3RotateVector( sim->transform.q, sim->localCenter ) ) );\n",
    addition: "\t\tsim->transform.p = b3OffsetPos( sim->center, b3Neg( b3RotateVector( sim->transform.q, sim->localCenter ) ) );\n#ifdef B3_ORACLE_SENTINELS\n\t\tsim->transform.p.x += 0x1p-20f;\n#endif\n",
  },
  {
    file: "src/solver.c",
    marker: "solver.scalar-phases.bridges",
    anchor: "typedef struct b3BlockDim\n",
    addition: "#ifdef B3_ORACLE_HOOKS\nvoid b3OracleCallIntegrateVelocities( b3World* world, float h )\n{\n\tb3SolverSet* set = world->solverSets.data + b3_awakeSet;\n\tb3StepContext context = { 0 };\n\tcontext.world = world;\n\tcontext.states = set->bodyStates.data;\n\tcontext.sims = set->bodySims.data;\n\tcontext.h = h;\n\tcontext.inv_h = h != 0.0f ? 1.0f / h : 0.0f;\n\tcontext.inv_dt = context.inv_h;\n\tcontext.maxLinearVelocity = world->maxLinearSpeed;\n\tb3SolverBlock block = { 0, (uint16_t)set->bodyStates.count, b3_bodyBlock, 0 };\n\tb3IntegrateVelocitiesTask( block, &context );\n}\n\nvoid b3OracleCallIntegratePositions( b3World* world, float h, b3Vec3* deltaPosition, b3Quat* deltaRotation )\n{\n\tb3SolverSet* set = world->solverSets.data + b3_awakeSet;\n\tb3StepContext context = { 0 };\n\tcontext.world = world;\n\tcontext.states = set->bodyStates.data;\n\tcontext.sims = set->bodySims.data;\n\tcontext.h = h;\n\tcontext.inv_h = h != 0.0f ? 1.0f / h : 0.0f;\n\tcontext.inv_dt = context.inv_h;\n\tcontext.maxLinearVelocity = world->maxLinearSpeed;\n\tb3SolverBlock block = { 0, (uint16_t)set->bodyStates.count, b3_bodyBlock, 0 };\n\tb3IntegratePositionsTask( block, &context );\n\t*deltaPosition = set->bodyStates.data[0].deltaPosition;\n\t*deltaRotation = set->bodyStates.data[0].deltaRotation;\n}\n\nvoid b3OracleCallFinalize( b3World* world, float dt )\n{\n\tb3SolverSet* set = world->solverSets.data + b3_awakeSet;\n\tb3Array_Resize( world->bodyMoveEvents, set->bodySims.count );\n\tb3StepContext context = { 0 };\n\tcontext.world = world;\n\tcontext.states = set->bodyStates.data;\n\tcontext.sims = set->bodySims.data;\n\tcontext.dt = dt;\n\tcontext.inv_dt = dt != 0.0f ? 1.0f / dt : 0.0f;\n\tcontext.maxLinearVelocity = world->maxLinearSpeed;\n\tb3FinalizeBodiesTask( 0, set->bodySims.count, 0, &context );\n}\n#endif\n\ntypedef struct b3BlockDim\n",
  },
  {
    file: "src/physics_world.c",
    marker: "physics.recycle",
    anchor: '#include "physics_world.h"\n',
    addition: '#include "physics_world.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\nuint32_t b3OracleRecycleVisits = 0;\n#endif\n',
  },
  {
    file: "src/physics_world.c",
    marker: "physics.recycle.sentinel",
    anchor: "\t\t\t\t\ttaskContext->recycledContactCount += 1;\n",
    addition: "\t\t\t\t\ttaskContext->recycledContactCount += 1;\n#ifdef B3_ORACLE_SENTINELS\n\t\t\t\t\t++b3OracleRecycleVisits;\n#endif\n",
  },
  {
    file: "src/physics_world.c",
    marker: "physics.recycle.bridge",
    anchor: "static void b3AddNonTouchingContact( b3World* world, b3Contact* contact )\n",
    addition: "#ifdef B3_ORACLE_HOOKS\nvoid b3OracleResetRecycleVisits( void )\n{\n\tb3OracleRecycleVisits = 0;\n}\n#endif\n\nstatic void b3AddNonTouchingContact( b3World* world, b3Contact* contact )\n",
  },
  {
    file: "src/convex_manifold.c",
    marker: "o4.convex-manifold.hooks",
    anchor: '#include "algorithm.h"\n',
    addition: '#include "algorithm.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
  },
  {
    file: "src/convex_manifold.c",
    marker: "o4.convex-manifold.visit.simd",
    anchor: "#if B3_SIMD_COLLIDE_HULLS == 1\n\nvoid b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,\n\t\t\t\t\t b3Transform transformBtoA, b3SATCache* cache )\n{\n\tmanifold->pointCount = 0;\n",
    addition: "#if B3_SIMD_COLLIDE_HULLS == 1\n\nvoid b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,\n\t\t\t\t\t b3Transform transformBtoA, b3SATCache* cache )\n{\n#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4ConvexManifoldVisits;\n#endif\n\tmanifold->pointCount = 0;\n",
  },
  {
    file: "src/convex_manifold.c",
    marker: "o4.convex-manifold.visit.scalar",
    anchor: "\treturn (b3EdgeQuery){\n\t\t.normal = maxNormal,\n\t\t.separation = maxSeparation,\n\t\t.indexA = maxIndexA,\n\t\t.indexB = maxIndexB,\n\t};\n}\n\nvoid b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,\n\t\t\t\t\t b3Transform transformBtoA, b3SATCache* cache )\n{\n\tmanifold->pointCount = 0;\n",
    addition: "\treturn (b3EdgeQuery){\n\t\t.normal = maxNormal,\n\t\t.separation = maxSeparation,\n\t\t.indexA = maxIndexA,\n\t\t.indexB = maxIndexB,\n\t};\n}\n\nvoid b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,\n\t\t\t\t\t b3Transform transformBtoA, b3SATCache* cache )\n{\n#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4ConvexManifoldVisits;\n#endif\n\tmanifold->pointCount = 0;\n",
  },
  {
    file: "src/mesh_contact.c",
    marker: "o4.mesh-contact.hooks",
    anchor: '#include "contact.h"\n',
    addition: '#include "contact.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
  },
  {
    file: "src/mesh_contact.c",
    marker: "o4.mesh-contact.visit",
    anchor: "b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB, bool isFast, b3Arena arena )\n{\n\tB3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );\n",
    addition: "b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB, bool isFast, b3Arena arena )\n{\n#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4MeshContactVisits;\n#endif\n\tB3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );\n",
  },
  {
    file: "src/contact.c",
    marker: "o4.convex-contact.hooks",
    anchor: '#include "contact.h"\n',
    addition: '#include "contact.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
  },
  {
    file: "src/contact.c",
    marker: "o4.convex-contact.visit",
    anchor: "{\n\t// Compute new manifold\n\tbool touching = b3ComputeConvexManifold",
    addition: "{\n#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4ConvexContactVisits;\n#endif\n\t// Compute new manifold\n\tbool touching = b3ComputeConvexManifold",
  },
  {
    file: "src/joint.c",
    marker: "o4.joint.hooks",
    anchor: '#include "joint.h"\n',
    addition: '#include "joint.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
  },
  ...["Parallel", "Distance", "Motor", "Filter", "Prismatic", "Revolute", "Spherical", "Weld", "Wheel"].map((name) => ({
    file: "src/joint.c",
    marker: `o4.joint.${name.toLowerCase()}.visit`,
    anchor: `b3JointId b3Create${name}Joint( b3WorldId worldId, const b3${name}JointDef* def )\n{\n`,
    addition: `b3JointId b3Create${name}Joint( b3WorldId worldId, const b3${name}JointDef* def )\n{\n#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4JointVisits;\n#endif\n`,
  })) as OraclePatch[],
];

export const O4_DECLARED_SYMBOLS = [
  { symbol: "b3CollideHulls", file: "src/convex_manifold.c", family: "convex-manifold", vector: "o4.convex-manifold" },
  { symbol: "b3ComputeMeshManifolds", file: "src/mesh_contact.c", family: "mesh-contact", vector: "o4.mesh-contact" },
  { symbol: "b3UpdateConvexContact", file: "src/contact.c", family: "convex-contact", vector: "o4.convex-contact" },
  { symbol: "b3CreateParallelJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.parallel" },
  { symbol: "b3CreateDistanceJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.distance" },
  { symbol: "b3CreateMotorJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.motor" },
  { symbol: "b3CreateFilterJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.filter" },
  { symbol: "b3CreatePrismaticJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.prismatic" },
  { symbol: "b3CreateRevoluteJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.revolute" },
  { symbol: "b3CreateSphericalJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.spherical" },
  { symbol: "b3CreateWeldJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.weld" },
  { symbol: "b3CreateWheelJoint", file: "src/joint.c", family: "joint", vector: "o4.joint.wheel" },
] as const;

export const DECLARED_SYMBOLS = [
  { symbol: "b3HashWorldState", file: "src/recording.c", vector: "whitebox.world-hash.v2" },
  { symbol: "b3IntegrateVelocitiesTask", file: "src/solver.c", vector: "whitebox.integrate-velocities.v2" },
  { symbol: "b3IntegratePositionsTask", file: "src/solver.c", vector: "whitebox.integrate-positions.v2" },
  { symbol: "b3FinalizeBodiesTask", file: "src/solver.c", vector: "whitebox.finalize.v2" },
  { symbol: "b3CollideTask", file: "src/physics_world.c", vector: "whitebox.recycle.v2" },
] as const;
