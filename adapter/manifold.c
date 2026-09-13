#include "box3d_oracle_adapter.h"
#include <box3d/collision.h>

void oracle_write_manifold(FILE* out, bool* first)
{
    b3LocalManifoldPoint points[4] = { 0 };
    b3LocalManifold manifold = { 0 };
    manifold.points = points;
    b3Sphere a = { { 0.0f, 0.0f, 0.0f }, 1.0f };
    b3Sphere b = { { 0.0f, 0.0f, 0.0f }, 1.0f };
    b3CollideSpheres(&manifold, 4, &a, &b, (b3Transform){ { 1.5f, 0.0f, 0.0f }, b3Quat_identity });
    oracle_case_begin(out, first, "manifold.spheres.v1", "manifold", "b3CollideSpheres", "{\"sphereA\":{\"center\":[\"0x00000000\",\"0x00000000\",\"0x00000000\"],\"radius\":\"0x3f800000\"},\"sphereB\":{\"center\":[\"0x00000000\",\"0x00000000\",\"0x00000000\"],\"radius\":\"0x3f800000\"},\"transformBtoA\":{\"translation\":[\"0x3fc00000\",\"0x00000000\",\"0x00000000\"]}}");
    oracle_key(out, "normal"); oracle_vec3(out, manifold.normal); fputc(',', out); oracle_key(out, "pointCount"); oracle_i32(out, manifold.pointCount); fputc(',', out); oracle_key(out, "points"); fputc('[', out);
    for (int i = 0; i < manifold.pointCount; ++i) { if (i) fputc(',', out); fputs("{\"point\":", out); oracle_vec3(out, points[i].point); fputs(",\"separation\":", out); oracle_f32(out, points[i].separation); fputc('}', out); }
    fputc(']', out); oracle_case_end(out);
}
