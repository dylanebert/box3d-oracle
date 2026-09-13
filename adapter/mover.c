#include "box3d_oracle_adapter.h"
#include <box3d/collision.h>
#include <float.h>

void oracle_write_mover(FILE* out, bool* first)
{
    b3CollisionPlane planes[2] = {
        { { { 1.0f, 0.0f, 0.0f }, 0.0f }, FLT_MAX, 0.0f, true },
        { { { 0.0f, 1.0f, 0.0f }, 0.0f }, FLT_MAX, 0.0f, true },
    };
    b3PlaneSolverResult result = b3SolvePlanes((b3Vec3){ -2.0f, -3.0f, 4.0f }, planes, 2);
    oracle_case_begin(out, first, "mover.solve-planes.v1", "mover", "b3SolvePlanes", "{\"targetDelta\":[\"0xc0000000\",\"0xc0400000\",\"0x40800000\"],\"planes\":[{\"normal\":[\"0x3f800000\",\"0x00000000\",\"0x00000000\"],\"offset\":\"0x00000000\",\"pushLimit\":\"0x7f800000\",\"clipVelocity\":true},{\"normal\":[\"0x00000000\",\"0x3f800000\",\"0x00000000\"],\"offset\":\"0x00000000\",\"pushLimit\":\"0x7f800000\",\"clipVelocity\":true}]}");
    oracle_key(out, "delta"); oracle_vec3(out, result.delta); fputc(',', out); oracle_key(out, "iterationCount"); oracle_i32(out, result.iterationCount); oracle_case_end(out);

    b3Vec3 clipped = b3ClipVector((b3Vec3){ -2.0f, -3.0f, 4.0f }, planes, 2);
    oracle_case_begin(out, first, "mover.clip-vector.v1", "mover", "b3ClipVector", "{\"vector\":[\"0xc0000000\",\"0xc0400000\",\"0x40800000\"],\"planeCount\":\"0x00000002\"}");
    oracle_key(out, "value"); oracle_vec3(out, clipped); oracle_case_end(out);
}
