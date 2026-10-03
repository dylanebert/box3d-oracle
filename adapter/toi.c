#include "box3d_oracle_adapter.h"
#include <box3d/collision.h>
#include <string.h>

static void quat(FILE* out, b3Quat q)
{
    fputc('[', out); oracle_f32(out, q.v.x); fputc(',', out); oracle_f32(out, q.v.y); fputc(',', out); oracle_f32(out, q.v.z); fputc(',', out); oracle_f32(out, q.s); fputc(']', out);
}
static void sweep(FILE* out, b3Sweep s)
{
    fputs("{\"localCenter\":", out); oracle_vec3(out, s.localCenter);
    fputs(",\"c1\":", out); oracle_vec3(out, s.c1);
    fputs(",\"c2\":", out); oracle_vec3(out, s.c2);
    fputs(",\"q1\":", out); quat(out, s.q1);
    fputs(",\"q2\":", out); quat(out, s.q2); fputc('}', out);
}
static void proxy(FILE* out, b3ShapeProxy p)
{
    fputs("{\"points\":[", out);
    for (int i = 0; i < p.count; ++i) { if (i) fputc(',', out); oracle_vec3(out, p.points[i]); }
    fputs("],\"radius\":", out); oracle_f32(out, p.radius); fputc('}', out);
}
static float from_bits(uint32_t bits) { float value; memcpy(&value, &bits, sizeof(value)); return value; }
int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    FILE* out = fopen(argv[1], "w"); if (!out) return 3;
    const b3Vec3 box[] = {{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
    const b3Vec3 quad[] = {{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}};
    const b3Vec3 seg[] = {{2,-1,0},{2,1,0}};
    const char* names[] = {"quad_seg_translate", "box_box_rotate", "box_box_separated", "box_box_overlap"};
    fputs("{\"schema\":\"box3d-oracle/v8\",\"cases\":[", out);
    for (int i = 0; i < 4; ++i) {
        b3TOIInput input = {0};
        input.proxyA = (b3ShapeProxy){i == 0 ? quad : box, i == 0 ? 4 : 8, 0};
        input.proxyB = (b3ShapeProxy){i == 0 ? seg : box, i == 0 ? 2 : 8, 0};
        input.sweepA.q1 = input.sweepA.q2 = input.sweepB.q1 = input.sweepB.q2 = b3Quat_identity;
        input.maxFraction = 1;
        if (i == 0) input.sweepB.c2.x = -2;
        if (i == 1) { input.sweepB.c1.x = 3; input.sweepB.c2.x = 1.5f; input.sweepB.q2.v.z = from_bits(0x3e7ef101); input.sweepB.q2.s = from_bits(0x3f77f06b); }
        if (i == 2) { input.sweepB.c1.x = 5; input.sweepB.c2.x = 4; }
        if (i == 3) input.sweepB.c1.x = 0.5f;
        b3TOIOutput result = b3TimeOfImpact(&input);
        fprintf(out, "%s{\"id\":\"toi.%s.v1\",\"family\":\"toi\",\"symbol\":\"b3TimeOfImpact\",\"input\":{\"proxyA\":", i ? "," : "", names[i]);
        proxy(out, input.proxyA); fputs(",\"proxyB\":", out); proxy(out, input.proxyB);
        fputs(",\"sweepA\":", out); sweep(out, input.sweepA); fputs(",\"sweepB\":", out); sweep(out, input.sweepB);
        fputs(",\"maxFraction\":", out); oracle_f32(out, input.maxFraction); fputs("},\"output\":{", out);
        oracle_key(out, "state"); oracle_i32(out, result.state);
        fputs(",\"fraction\":", out); oracle_f32(out, result.fraction);
        fputs(",\"distance\":", out); oracle_f32(out, result.distance);
        fputs(",\"point\":", out); oracle_vec3(out, result.point);
        fputs(",\"normal\":", out); oracle_vec3(out, result.normal);
        fputs(",\"distanceIterations\":", out); oracle_i32(out, result.distanceIterations);
        fputs(",\"pushBackIterations\":", out); oracle_i32(out, result.pushBackIterations);
        fputs(",\"rootIterations\":", out); oracle_i32(out, result.rootIterations); oracle_case_end(out);
    }
    fputs("]}\n", out); return fclose(out) == 0 ? 0 : 4;
}
