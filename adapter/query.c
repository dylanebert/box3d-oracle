#include "box3d_oracle_adapter.h"
#include <box3d/collision.h>

void oracle_write_query(FILE* out, bool* first)
{
    b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
    b3RayCastInput input = { { -3.0f, 0.0f, 0.0f }, { 6.0f, 0.0f, 0.0f }, 1.0f };
    b3CastOutput result = b3RayCastSphere(&sphere, &input);
    oracle_case_begin(out, first, "query.ray-sphere.v1", "query", "b3RayCastSphere", "{\"sphere\":{\"center\":[\"0x00000000\",\"0x00000000\",\"0x00000000\"],\"radius\":\"0x3f800000\"},\"origin\":[\"0xc0400000\",\"0x00000000\",\"0x00000000\"],\"translation\":[\"0x40c00000\",\"0x00000000\",\"0x00000000\"],\"maxFraction\":\"0x3f800000\"}");
    oracle_key(out, "hit"); fprintf(out, "%s", result.hit ? "true" : "false"); fputc(',', out); oracle_key(out, "normal"); oracle_vec3(out, result.normal); fputc(',', out); oracle_key(out, "point"); oracle_vec3(out, result.point); fputc(',', out); oracle_key(out, "fraction"); oracle_f32(out, result.fraction); fputc(',', out); oracle_key(out, "iterations"); oracle_i32(out, result.iterations); oracle_case_end(out);
}
