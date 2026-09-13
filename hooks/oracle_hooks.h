#ifndef B3_ORACLE_HOOKS_H
#define B3_ORACLE_HOOKS_H

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
extern uint32_t b3OracleRecycleVisits;
extern uint32_t b3OracleO4ConvexManifoldVisits;
extern uint32_t b3OracleO4MeshContactVisits;
extern uint32_t b3OracleO4ConvexContactVisits;
extern uint32_t b3OracleO4JointVisits;

#endif
