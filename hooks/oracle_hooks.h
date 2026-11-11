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
extern uint32_t b3OracleRecycleVisits;
extern uint32_t b3OracleO4ConvexManifoldVisits;
extern uint32_t b3OracleO4MeshContactVisits;
extern uint32_t b3OracleO4ConvexContactVisits;
extern uint32_t b3OracleO4JointVisits;

#endif
