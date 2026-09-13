#ifndef BOX3D_ORACLE_ADAPTER_H
#define BOX3D_ORACLE_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <box3d/math_functions.h>

void oracle_write_math(FILE* out, bool* first);
void oracle_write_geometry(FILE* out, bool* first);
void oracle_write_distance(FILE* out, bool* first);
void oracle_write_tree(FILE* out, bool* first);
void oracle_write_manifold(FILE* out, bool* first);
void oracle_write_query(FILE* out, bool* first);
void oracle_write_mover(FILE* out, bool* first);

void oracle_case_begin(FILE* out, bool* first, const char* id, const char* family, const char* symbol, const char* input);
void oracle_case_end(FILE* out);
void oracle_key(FILE* out, const char* key);
void oracle_u32(FILE* out, uint32_t value);
void oracle_u64(FILE* out, uint64_t value);
void oracle_i32(FILE* out, int32_t value);
void oracle_f32(FILE* out, float value);
void oracle_vec3(FILE* out, struct b3Vec3 value);
void oracle_aabb(FILE* out, struct b3AABB value);

#endif
