#include "box3d_oracle_adapter.h"
#include <box3d/collision.h>

void oracle_write_distance(FILE* out, bool* first)
{
    b3Vec3 result = b3PointToSegmentDistance((b3Vec3){ -1.0f, 0.0f, 0.0f }, (b3Vec3){ 1.0f, 0.0f, 0.0f }, (b3Vec3){ 0.25f, 2.0f, 0.0f });
    oracle_case_begin(out, first, "distance.point-segment.v1", "distance", "b3PointToSegmentDistance", "{\"a\":[\"0xbf800000\",\"0x00000000\",\"0x00000000\"],\"b\":[\"0x3f800000\",\"0x00000000\",\"0x00000000\"],\"q\":[\"0x3e800000\",\"0x40000000\",\"0x00000000\"]}");
    oracle_key(out, "closest"); oracle_vec3(out, result); oracle_case_end(out);

    b3Vec3 pointsA[1] = { { 0.0f, 0.0f, 0.0f } };
    b3Vec3 pointsB[1] = { { 0.0f, 0.0f, 0.0f } };
    b3DistanceInput input = { { pointsA, 1, 0.0f }, { pointsB, 1, 0.0f }, { { 2.0f, 0.0f, 0.0f }, b3Quat_identity }, true };
    b3SimplexCache cache = { 0 };
    b3Simplex simplexes[4] = { 0 };
    b3DistanceOutput distance = b3ShapeDistance(&input, &cache, simplexes, 4);
    oracle_case_begin(out, first, "distance.shape.v1", "distance", "b3ShapeDistance", "{\"proxyA\":[[\"0x00000000\",\"0x00000000\",\"0x00000000\"]],\"proxyB\":[[\"0x00000000\",\"0x00000000\",\"0x00000000\"]],\"translationB\":[\"0x40000000\",\"0x00000000\",\"0x00000000\"],\"useRadii\":true}");
    oracle_key(out, "pointA"); oracle_vec3(out, distance.pointA); fputc(',', out); oracle_key(out, "pointB"); oracle_vec3(out, distance.pointB); fputc(',', out); oracle_key(out, "normal"); oracle_vec3(out, distance.normal); fputc(',', out); oracle_key(out, "distance"); oracle_f32(out, distance.distance); fputc(',', out); oracle_key(out, "iterations"); oracle_i32(out, distance.iterations); fputc(',', out); oracle_key(out, "simplexCount"); oracle_i32(out, distance.simplexCount); oracle_case_end(out);
}
