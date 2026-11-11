#ifndef B3_ORACLE_HOOKS_H
#define B3_ORACLE_HOOKS_H

#include <stdbool.h>
#include <stdint.h>
#include "box3d/math_functions.h"
#include "box3d/id.h"

typedef struct b3World b3World;

uint64_t b3OracleCallHashWorldState( b3World* world );
uint64_t b3OracleCallHashWorldStateId( b3WorldId worldId );
void b3OracleCallIntegrateVelocities( b3World* world, float h );
void b3OracleCallIntegratePositions( b3World* world, float h, b3Vec3* deltaPosition, b3Quat* deltaRotation );
void b3OracleCallFinalize( b3World* world, float dt );
void b3OracleResetRecycleVisits( void );
void b3OracleRemoveNonTouchingContact( b3World* world, int setIndex, int localIndex );
// The contact b3CollideTask is updating, for hooks inside its narrow phase.
extern int b3OracleContactId;
// b3SegmentDistance's test denom > threshold, with denom = a*e - b*b. Returning true takes the other
// side of the test, as a contraction of a*e - b*b within rounding of the threshold may.
typedef bool b3OracleSegmentHook( float ae, float bb, float denom, float threshold );
extern b3OracleSegmentHook* b3OracleSegmentDecision;

// The step lane's decision hooks (B3_ORACLE_STEP_HOOKS), each at a discrete test of the solver whose
// deciding quantity the adapter bounds from these operands.
typedef struct b3Contact b3Contact;
// Contact push's `s > 0` per lane: wide values as B3_SIMD_WIDTH floats; the hook may rewrite
// `separated`'s lanes. The step lane's scalar build holds a lane's mask as 1.0f or 0.0f. A lane
// without this point has zero normal mass.
typedef void b3OracleSeparationHook( b3Contact* const* contacts, int width, const void* s, void* separated, const void* normal,
									 const void* dpA, const void* dpB, const void* dqA, const void* dqB, const void* rA,
									 const void* rB, const void* normalA, const void* normalB, const void* normalSeparation,
									 const void* baseSeparation, const void* normalMass );
extern b3OracleSeparationHook* b3OracleSeparationDecision;
// A revolute or spherical limit's `c > 0`, from its relative rotation; returns the c to test.
typedef float b3OracleLimitHook( int jointId, int limit, bool useBias, float c, float limitValue, b3Quat dqA, b3Quat dqB,
								 b3Quat frameA, b3Quat frameB, b3Quat relQ );
extern b3OracleLimitHook* b3OracleLimitDecision;
// A prismatic limit's `C < speculativeDistance` and `C > 0`, from its translation's operands; returns the C to test.
typedef float b3OracleTranslationHook( int jointId, int limit, bool useBias, float C, float limitValue, float speculativeDistance, b3Vec3 dcA, b3Vec3 dcB,
									   b3Vec3 deltaCenter, b3Vec3 rA, b3Vec3 rB, b3Vec3 frameA, b3Vec3 frameB, b3Quat dqA,
									   b3Quat dqB, b3Vec3 localAxis, b3Vec3 axis, b3Vec3 d );
extern b3OracleTranslationHook* b3OracleTranslationDecision;
// After each solver stage the main thread executes, with its b3SolverStageType and graph color.
typedef struct b3StepContext b3StepContext;
typedef void b3OracleStage( int type, int color, b3StepContext* context );
extern b3OracleStage* b3OracleStageHook;
extern uint32_t b3OracleRecycleVisits;
extern uint32_t b3OracleO4ConvexManifoldVisits;
extern uint32_t b3OracleO4MeshContactVisits;
extern uint32_t b3OracleO4ConvexContactVisits;
extern uint32_t b3OracleO4JointVisits;

#endif
