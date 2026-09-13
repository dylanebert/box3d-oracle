#include "box3d_oracle_adapter.h"
#include <box3d/collision.h>

void oracle_write_geometry(FILE* out, bool* first)
{
    b3Sphere sphere = { { 0.25f, -0.5f, 1.0f }, 0.75f };
    b3Transform transform = { { 2.0f, 3.0f, -1.0f }, b3Quat_identity };
    b3AABB bounds = b3ComputeSphereAABB(&sphere, transform);
    oracle_case_begin(out, first, "geometry.sphere-aabb.v1", "geometry", "b3ComputeSphereAABB", "{\"center\":[\"0x3e800000\",\"0xbf000000\",\"0x3f800000\"],\"radius\":\"0x3f400000\",\"translation\":[\"0x40000000\",\"0x40400000\",\"0xbf800000\"]}");
    oracle_key(out, "aabb"); oracle_aabb(out, bounds); oracle_case_end(out);

    b3Capsule capsule = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.5f };
    b3MassData mass = b3ComputeCapsuleMass(&capsule, 2.0f);
    oracle_case_begin(out, first, "geometry.capsule-mass.v1", "geometry", "b3ComputeCapsuleMass", "{\"center1\":[\"0xbf800000\",\"0x00000000\",\"0x00000000\"],\"center2\":[\"0x3f800000\",\"0x00000000\",\"0x00000000\"],\"radius\":\"0x3f000000\",\"density\":\"0x40000000\"}");
    oracle_key(out, "mass"); oracle_f32(out, mass.mass); fputc(',', out); oracle_key(out, "center"); oracle_vec3(out, mass.center); fputc(',', out); oracle_key(out, "inertia"); oracle_vec3(out, mass.inertia.cx); oracle_case_end(out);
}
