#include "box3d_oracle_adapter.h"
#include <stdio.h>

int main(int argc, char** argv)
{
    if (argc != 2) { fprintf(stderr, "usage: box3d-public-adapter <output>\n"); return 2; }
    FILE* out = fopen(argv[1], "wb");
    if (!out) { perror(argv[1]); return 3; }
    bool first = true;
    fputs("{\"schema\":\"box3d-oracle/v1\",\"cases\":[", out);
    oracle_write_math(out, &first);
    oracle_write_geometry(out, &first);
    oracle_write_distance(out, &first);
    oracle_write_tree(out, &first);
    oracle_write_manifold(out, &first);
    oracle_write_query(out, &first);
    oracle_write_mover(out, &first);
    fputs("]}\n", out);
    fclose(out);
    return 0;
}
