#include "box3d_oracle_adapter.h"
#include <stdio.h>

void oracle_write_whitebox(FILE* out, bool* first);
void oracle_write_o4(FILE* out, bool* first);

int main(int argc, char** argv)
{
    if (argc != 2) { fprintf(stderr, "usage: box3d-o4-adapter <output>\n"); return 2; }
    FILE* out = fopen(argv[1], "wb");
    if (!out) { perror(argv[1]); return 3; }
    bool first = true;
    fputs("{\"schema\":\"box3d-oracle/v3\",\"cases\":[", out);
    oracle_write_math(out, &first);
    oracle_write_geometry(out, &first);
    oracle_write_distance(out, &first);
    oracle_write_tree(out, &first);
    oracle_write_manifold(out, &first);
    oracle_write_query(out, &first);
    oracle_write_mover(out, &first);
    oracle_write_whitebox(out, &first);
    oracle_write_o4(out, &first);
    fputs("]}\n", out);
    fclose(out);
    return 0;
}
