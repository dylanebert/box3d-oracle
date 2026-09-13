#include "box3d_oracle_adapter.h"
#include <box3d/math_functions.h>

void oracle_write_math(FILE* out, bool* first)
{
    b3Vec3 a = { 0.25f, -2.0f, 3.5f };
    b3Vec3 b = { -1.0f, 4.0f, 0.5f };
    b3CosSin cs = b3ComputeCosSin(0.75f);
    b3Vec3 sum = b3Add(a, b);
    oracle_case_begin(out, first, "math.add.v1", "math", "b3Add", "{\"a\":[\"0x3e800000\",\"0xc0000000\",\"0x40600000\"],\"b\":[\"0xbf800000\",\"0x40800000\",\"0x3f000000\"]}");
    oracle_key(out, "value"); oracle_vec3(out, sum); oracle_case_end(out);

    oracle_case_begin(out, first, "math.dot.v1", "math", "b3Dot", "{\"a\":[\"0x3e800000\",\"0xc0000000\",\"0x40600000\"],\"b\":[\"0xbf800000\",\"0x40800000\",\"0x3f000000\"]}");
    oracle_key(out, "value"); oracle_f32(out, b3Dot(a, b)); oracle_case_end(out);

    oracle_case_begin(out, first, "math.cos-sin.v1", "math", "b3ComputeCosSin", "{\"radians\":\"0x3f400000\"}");
    oracle_key(out, "cosine"); oracle_f32(out, cs.cosine); fputc(',', out); oracle_key(out, "sine"); oracle_f32(out, cs.sine); oracle_case_end(out);
}
