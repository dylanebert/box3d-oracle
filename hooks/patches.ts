export type OracleSite = {
  anchor: string;
  addition: string;
};

/** One marker at one site. `alternatives` spell the same site as other upstream revisions write it; exactly one of the anchors may occur, once. */
export type OraclePatch = OracleSite & {
  file: string;
  marker: string;
  alternatives?: OracleSite[];
  /** A site only the step lane's revision has; other revisions skip it, and the step lane requires it. */
  optional?: boolean;
};

const STEP = "#if defined( B3_ORACLE_HOOKS ) && defined( B3_ORACLE_STEP_HOOKS )\n";

/** A joint limit's `c > 0` test: the hook may move c to the threshold's other side. */
function limitSite(file: string, marker: string, line: string, indent: string, call: string): OraclePatch {
  const anchor = `${indent}${line}\n${indent}float bias = 0.0f;\n${indent}float massScale = 1.0f;\n${indent}float impulseScale = 0.0f;\n`;
  return { file, marker, optional: true, anchor, addition: `${anchor}${STEP}${indent}if ( b3OracleLimitDecision != NULL )\n${indent}{\n${indent}\tc = b3OracleLimitDecision( ${call} );\n${indent}}\n#endif\n` };
}

const visitConvexManifold = "#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4ConvexManifoldVisits;\n#endif\n";

/** The hull manifold body: the live SIMD definition through 79fc559, the single definition at 51f056e, and b3CollideHullsAtWidth from e77352c, which b3CollideHulls and world contacts call. */
function convexManifoldVisit(): OraclePatch {
  const opening = "void b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,\n\t\t\t\t\t b3Transform transformBtoA, b3SATCache* cache )\n{\n";
  const atWidth = "void b3CollideHullsAtWidth( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,\n\t\t\t\t\t\t\tb3Transform transformBtoA, b3SATCache* cache, int simdWidth )\n{\n";
  const site = (prefix: string): OracleSite => ({ anchor: `${prefix}\tmanifold->pointCount = 0;\n`, addition: `${prefix}${visitConvexManifold}\tmanifold->pointCount = 0;\n` });
  return { file: "src/convex_manifold.c", marker: "o4.convex-manifold.visit", ...site(atWidth), alternatives: [site(`#if B3_SIMD_COLLIDE_HULLS == 1\n\n${opening}`), site(`#undef NV\n\n${opening}`)] };
}

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
    addition: "\treturn hash;\n}\n#ifdef B3_ORACLE_HOOKS\nuint64_t b3OracleCallHashWorldState( b3World* world )\n{\n\treturn b3HashWorldState( world );\n}\nuint64_t b3OracleCallHashWorldStateId( b3WorldId worldId )\n{\n\treturn b3HashWorldState( b3GetWorld( worldId.index1 - 1 ) );\n}\n#endif\n",
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
    addition: '#include "physics_world.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\nuint32_t b3OracleRecycleVisits = 0;\nint b3OracleContactId = -1;\n#endif\n',
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
    file: "src/physics_world.c",
    marker: "physics.remove-non-touching.bridge",
    anchor: "// Narrow-phase collision\nstatic void b3Collide( b3StepContext* context )\n",
    addition: "#ifdef B3_ORACLE_HOOKS\nvoid b3OracleRemoveNonTouchingContact( b3World* world, int setIndex, int localIndex )\n{\n\tb3RemoveNonTouchingContact( world, setIndex, localIndex );\n}\n#endif\n\n// Narrow-phase collision\nstatic void b3Collide( b3StepContext* context )\n",
  },
  {
    file: "src/physics_world.c",
    marker: "physics.collide.contact-id",
    anchor: "\t\t// This updates solid contacts\n",
    addition: "\t\t// This updates solid contacts\n#ifdef B3_ORACLE_HOOKS\n\t\tb3OracleContactId = contactIndex;\n#endif\n",
  },
  {
    file: "src/math_functions.c",
    marker: "math.segment-distance.hooks",
    anchor: '#include "math_internal.h"\n',
    addition: '#include "math_internal.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\nb3OracleSegmentHook* b3OracleSegmentDecision = NULL;\nb3OracleSeparationHook* b3OracleSeparationDecision = NULL;\nb3OracleLimitHook* b3OracleLimitDecision = NULL;\nb3OracleTranslationHook* b3OracleTranslationDecision = NULL;\nb3OracleStage* b3OracleStageHook = NULL;\nb3OracleStage* b3OracleStageBeginHook = NULL;\nb3OracleRollingHook* b3OracleRollingDecision = NULL;\n#endif\n',
  },
  {
    file: "src/math_functions.c",
    marker: "math.segment-distance.denom",
    anchor: "\tfloat s1 = denom > 1000.0f * FLT_MIN ? b3ClampFloat( ( b * f - c * e ) / denom, 0.0f, 1.0f ) : 0.0f;\n",
    addition: "\tfloat s1 = denom > 1000.0f * FLT_MIN ? b3ClampFloat( ( b * f - c * e ) / denom, 0.0f, 1.0f ) : 0.0f;\n#ifdef B3_ORACLE_HOOKS\n\tif ( b3OracleSegmentDecision != NULL && b3OracleSegmentDecision( a * e, b * b, denom, 1000.0f * FLT_MIN ) )\n\t{\n\t\tfloat ratio = ( b * f - c * e ) / denom;\n\t\ts1 = denom > 1000.0f * FLT_MIN ? 0.0f : ( ratio > 0.0f ? ( ratio < 1.0f ? ratio : 1.0f ) : 0.0f );\n\t}\n#endif\n",
  },
  {
    file: "src/contact_solver_wide.inl",
    marker: "step.contact-push.hooks",
    optional: true,
    anchor: '#include "simd_wide.h"\n',
    addition: `#include "simd_wide.h"\n${STEP}#include "oracle_hooks.h"\n#endif\n`,
  },
  {
    file: "src/contact_solver_wide.inl",
    marker: "step.contact-push.separation",
    optional: true,
    anchor: "\t\t\t\tb3FloatW separated = b3GreaterThanW( s, zeroW );\n",
    addition: `\t\t\t\tb3FloatW separated = b3GreaterThanW( s, zeroW );\n${STEP}\t\t\t\tif ( b3OracleSeparationDecision != NULL )\n\t\t\t\t{\n\t\t\t\t\tb3OracleSeparationDecision( c->contacts, B3_SIMD_WIDTH, &s, &separated, &cm->normal, &bA.dp, &bB.dp, &bA.dq, &bB.dq, &rA, &rB, &normalA, &normalB, &normalSeparation, &cp->baseSeparations, &cp->normalMasses, pointIndex );\n\t\t\t\t}\n#endif\n`,
  },
  {
    file: "src/contact_solver_wide.inl",
    marker: "step.contact-relax.rolling",
    optional: true,
    anchor: "\t\t\t\tb3FloatW mask = b3GreaterThanW( lengthSquared, b3MulAddW( epsilonW, maxImpulse, maxImpulse ) );\n",
    addition: `\t\t\t\tb3FloatW mask = b3GreaterThanW( lengthSquared, b3MulAddW( epsilonW, maxImpulse, maxImpulse ) );\n${STEP}\t\t\t\tif ( b3OracleRollingDecision != NULL )\n\t\t\t\t{\n\t\t\t\t\tb3OracleRollingDecision( c->contacts, B3_SIMD_WIDTH, &lengthSquared, &mask, &maxImpulse, &oldImpulse, &cm->rollingImpulse, &c->rollingMass, &bA.w, &bB.w, &cm->rollingMask );\n\t\t\t\t}\n#endif\n`,
  },
  ...["revolute_joint.c", "spherical_joint.c", "prismatic_joint.c"].map((file) => ({
    file: `src/${file}`,
    marker: `step.${file.split("_")[0]}.hooks`,
    optional: true,
    anchor: '#include "joint.h"\n',
    addition: `#include "joint.h"\n${STEP}#include "oracle_hooks.h"\n#endif\n`,
  })) as OraclePatch[],
  limitSite("src/revolute_joint.c", "step.revolute.lower", "float c = angle - joint->lowerAngle;", "\t\t\t\t", "base->jointId, 0, useBias, c, joint->lowerAngle, stateA->deltaRotation, stateB->deltaRotation, joint->frameA.q, joint->frameB.q, relQ"),
  limitSite("src/revolute_joint.c", "step.revolute.upper", "float c = joint->upperAngle - angle;", "\t\t\t\t", "base->jointId, 1, useBias, c, joint->upperAngle, stateA->deltaRotation, stateB->deltaRotation, joint->frameA.q, joint->frameB.q, relQ"),
  limitSite("src/spherical_joint.c", "step.spherical.twist-lower", "float c = twistAngle - joint->lowerTwistAngle;", "\t\t\t\t", "base->jointId, 2, useBias, c, joint->lowerTwistAngle, stateA->deltaRotation, stateB->deltaRotation, joint->frameA.q, joint->frameB.q, relQ"),
  limitSite("src/spherical_joint.c", "step.spherical.twist-upper", "float c = joint->upperTwistAngle - twistAngle;", "\t\t\t\t", "base->jointId, 3, useBias, c, joint->upperTwistAngle, stateA->deltaRotation, stateB->deltaRotation, joint->frameA.q, joint->frameB.q, relQ"),
  limitSite("src/spherical_joint.c", "step.spherical.cone", "float c = joint->coneAngle - swingAngle;", "\t\t\t", "base->jointId, 4, useBias, c, joint->coneAngle, stateA->deltaRotation, stateB->deltaRotation, joint->frameA.q, joint->frameB.q, relQ"),
  ...([["lower", "float C = jointTranslation - joint->lowerTranslation;", 5, "joint->lowerTranslation"], ["upper", "float C = joint->upperTranslation - jointTranslation;", 6, "joint->upperTranslation"]] as const).map(([name, line, code, limit]) => ({
    file: "src/prismatic_joint.c",
    marker: `step.prismatic.${name}`,
    optional: true,
    anchor: `\t\t\t${line}\n`,
    addition: `\t\t\t${line}\n${STEP}\t\t\tif ( b3OracleTranslationDecision != NULL )\n\t\t\t{\n\t\t\t\tC = b3OracleTranslationDecision( base->jointId, ${code}, useBias, C, ${limit}, speculativeDistance, stateA->deltaPosition, stateB->deltaPosition, joint->deltaCenter, rA, rB, joint->frameA.p, joint->frameB.p, stateA->deltaRotation, stateB->deltaRotation, joint->jointAxis, jointAxis, d );\n\t\t\t}\n#endif\n`,
  })) as OraclePatch[],
  {
    file: "src/solver.c",
    marker: "step.solver.stage-begin",
    optional: true,
    anchor: "\tint blockCount = stage->blockCount;\n\tif ( blockCount == 0 )\n\t{\n\t\treturn;\n\t}\n\n\tconst int workerIndex = 0;\n",
    addition: `\tint blockCount = stage->blockCount;\n\tif ( blockCount == 0 )\n\t{\n\t\treturn;\n\t}\n\n\tconst int workerIndex = 0;\n${STEP}\tif ( b3OracleStageBeginHook != NULL )\n\t{\n\t\tb3OracleStageBeginHook( (int)stage->type, stage->colorIndex, context );\n\t}\n#endif\n`,
  },
  {
    file: "src/contact_solver_wide.inl",
    marker: "step.contact-lanes",
    optional: true,
    anchor: "void B3_WIDE( b3PushContacts_Convex )( b3SolverBlock block, b3StepContext* context )\n{\n",
    addition: `${STEP}int B3_WIDE( b3OracleContactLanes )( void* wideConstraints, int count, int contactId, float** fields )\n{\n\tb3ConvexConstraintWide* constraints = wideConstraints;\n\tfor ( int i = 0; i < count; ++i )\n\t{\n\t\tb3ConvexConstraintWide* c = constraints + i;\n\t\tfor ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )\n\t\t{\n\t\t\tif ( c->base.contacts[lane] == NULL || c->base.contacts[lane]->contactId != contactId ) continue;\n\t\t\tb3ContactManifoldWide* m = &c->manifold;\n\t\t\tint n = 0;\n\t\t\tfor ( int k = 0; k < 2; ++k ) fields[n++] = (float*)&m->points[k].normalImpulses + lane;\n\t\t\tfor ( int k = 0; k < 2; ++k ) fields[n++] = (float*)&m->points[k].totalNormalImpulses + lane;\n\t\t\tfields[n++] = (float*)&m->frictionImpulse.x + lane;\n\t\t\tfields[n++] = (float*)&m->frictionImpulse.y + lane;\n\t\t\tfields[n++] = (float*)&m->twistImpulse + lane;\n\t\t\tfields[n++] = (float*)&m->rollingImpulse.X + lane;\n\t\t\tfields[n++] = (float*)&m->rollingImpulse.Y + lane;\n\t\t\tfields[n++] = (float*)&m->rollingImpulse.Z + lane;\n\t\t\tfor ( int k = 0; k < 2; ++k ) fields[n++] = (float*)&m->points[k].baseSeparations + lane;\n\t\t\tb3FloatW* prepared[] = { &m->normal.X, &m->normal.Y, &m->normal.Z, &m->tangent1.X, &m->tangent1.Y, &m->tangent1.Z, &m->tangent2.X, &m->tangent2.Y, &m->tangent2.Z, &m->centerA.X, &m->centerA.Y, &m->centerA.Z, &m->centerB.X, &m->centerB.Y, &m->centerB.Z, &m->twistMass, &m->tangentMass.cxx, &m->tangentMass.cxy, &m->tangentMass.cyy, &m->tangentVelocity1, &m->tangentVelocity2 };\n\t\t\tfor ( int k = 0; k < (int)( sizeof( prepared ) / sizeof( prepared[0] ) ); ++k ) fields[n++] = (float*)prepared[k] + lane;\n\t\t\tfor ( int k = 0; k < 2; ++k )\n\t\t\t{\n\t\t\t\tb3ContactConstraintPointWide* p = m->points + k;\n\t\t\t\tb3FloatW* point[] = { &p->anchorAs.X, &p->anchorAs.Y, &p->anchorAs.Z, &p->anchorBs.X, &p->anchorBs.Y, &p->anchorBs.Z, &p->normalMasses, &p->leverArms };\n\t\t\t\tfor ( int j = 0; j < 8; ++j ) fields[n++] = (float*)point[j] + lane;\n\t\t\t}\n\t\t\tb3FloatW* base[] = { &c->base.invMassA, &c->base.invMassB, &c->base.invIA.cxx, &c->base.invIA.cxy, &c->base.invIA.cxz, &c->base.invIA.cyy, &c->base.invIA.cyz, &c->base.invIA.czz, &c->base.invIB.cxx, &c->base.invIB.cxy, &c->base.invIB.cxz, &c->base.invIB.cyy, &c->base.invIB.cyz, &c->base.invIB.czz, &c->base.friction };\n\t\t\tfor ( int k = 0; k < (int)( sizeof( base ) / sizeof( base[0] ) ); ++k ) fields[n++] = (float*)base[k] + lane;\n\t\t\treturn n;\n\t\t}\n\t}\n\treturn 0;\n}\n#endif\n\nvoid B3_WIDE( b3PushContacts_Convex )( b3SolverBlock block, b3StepContext* context )\n{\n`,
  },
  {
    file: "src/solver.c",
    marker: "step.solver.stage",
    optional: true,
    anchor: "\t\tb3AtomicStoreInt( &stage->completionCount, 0 );\n\t}\n",
    addition: `\t\tb3AtomicStoreInt( &stage->completionCount, 0 );\n\t}\n${STEP}\tif ( b3OracleStageHook != NULL )\n\t{\n\t\tb3OracleStageHook( (int)stage->type, stage->colorIndex, context );\n\t}\n#endif\n`,
  },
  {
    file: "src/convex_manifold.c",
    marker: "o4.convex-manifold.hooks",
    anchor: '#include "algorithm.h"\n',
    addition: '#include "algorithm.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
  },
  convexManifoldVisit(),
  {
    file: "src/mesh_contact.c",
    marker: "o4.mesh-contact.hooks",
    anchor: '#include "contact.h"\n',
    addition: '#include "contact.h"\n#ifdef B3_ORACLE_HOOKS\n#include "oracle_hooks.h"\n#endif\n',
  },
  {
    file: "src/mesh_contact.c",
    marker: "o4.mesh-contact.visit",
    anchor: "bool isFast, b3Arena arena )\n{\n\tB3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );\n",
    addition: "bool isFast, b3Arena arena )\n{\n#ifdef B3_ORACLE_SENTINELS\n\t++b3OracleO4MeshContactVisits;\n#endif\n\tB3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );\n",
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
