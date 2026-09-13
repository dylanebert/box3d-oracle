#include "box3d_oracle_adapter.h"
#include <stdio.h>

void oracle_write_scenarios(FILE* out, bool* first);

int main(int argc, char** argv)
{
    if (argc != 2) { fprintf(stderr, "usage: box3d-scenario-adapter <output>\n"); return 2; }
    FILE* out = fopen(argv[1], "wb");
    if (!out) { perror(argv[1]); return 2; }
    bool first = true;
#ifdef BOX3D_SCENARIO_V2
    fputs("{\"schema\":\"shallot-physics-scenario/v2\",\"cases\":[", out);
#else
    fputs("{\"schema\":\"shallot-physics-scenario/v1\",\"cases\":[", out);
#endif
    oracle_write_scenarios(out, &first);
    fputs("]}\n", out);
    fclose(out);
    return 0;
}
