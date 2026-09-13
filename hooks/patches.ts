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
    addition: '#include "recording.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
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
];

export const DECLARED_SYMBOLS = [
  { symbol: "b3HashWorldState", file: "src/recording.c", vector: "whitebox.world-hash.v2" },
  { symbol: "b3IntegrateVelocitiesTask", file: "src/solver.c", vector: "whitebox.integrate-velocities.v2" },
  { symbol: "b3IntegratePositionsTask", file: "src/solver.c", vector: "whitebox.integrate-positions.v2" },
  { symbol: "b3FinalizeBodiesTask", file: "src/solver.c", vector: "whitebox.finalize.v2" },
  { symbol: "b3CollideTask", file: "src/physics_world.c", vector: "whitebox.recycle.v2" },
] as const;
