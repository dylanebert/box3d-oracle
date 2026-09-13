#include "box3d_oracle_adapter.h"
#include <string.h>

static uint32_t bits(float value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

void oracle_case_begin(FILE* out, bool* first, const char* id, const char* family, const char* symbol, const char* input)
{
    fprintf(out, "%s{\"id\":\"%s\",\"family\":\"%s\",\"symbol\":\"%s\",\"input\":%s,\"output\":{", *first ? "" : ",", id, family, symbol, input);
    *first = false;
}

void oracle_case_end(FILE* out) { fputs("}}", out); }
void oracle_key(FILE* out, const char* key) { fprintf(out, "\"%s\":", key); }
void oracle_u32(FILE* out, uint32_t value) { fprintf(out, "\"0x%08x\"", value); }
void oracle_u64(FILE* out, uint64_t value) { fprintf(out, "\"0x%016llx\"", (unsigned long long)value); }
void oracle_i32(FILE* out, int32_t value) { fprintf(out, "\"0x%08x\"", (uint32_t)value); }
void oracle_f32(FILE* out, float value) { oracle_u32(out, bits(value)); }
void oracle_vec3(FILE* out, b3Vec3 value)
{
    fputc('[', out); oracle_f32(out, value.x); fputc(',', out); oracle_f32(out, value.y); fputc(',', out); oracle_f32(out, value.z); fputc(']', out);
}
void oracle_aabb(FILE* out, b3AABB value)
{
    fputs("{\"lower\":", out); oracle_vec3(out, value.lowerBound); fputs(",\"upper\":", out); oracle_vec3(out, value.upperBound); fputc('}', out);
}
