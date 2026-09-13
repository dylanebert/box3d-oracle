#include "box3d_oracle_adapter.h"
#include <box3d/collision.h>

struct tree_capture { int count; int ids[8]; uint64_t data[8]; };
static bool tree_callback(int proxyId, uint64_t userData, void* context)
{
    struct tree_capture* capture = context;
    if (capture->count < 8) { capture->ids[capture->count] = proxyId; capture->data[capture->count] = userData; capture->count += 1; }
    return true;
}

void oracle_write_tree(FILE* out, bool* first)
{
    b3DynamicTree tree = b3DynamicTree_Create(8);
    int firstProxy = b3DynamicTree_CreateProxy(&tree, (b3AABB){ { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } }, 1, 0x1122334455667788ULL);
    int secondProxy = b3DynamicTree_CreateProxy(&tree, (b3AABB){ { 2.0f, -1.0f, -1.0f }, { 4.0f, 1.0f, 1.0f } }, 1, 0x8877665544332211ULL);
    struct tree_capture capture = { 0 };
    b3TreeStats stats = b3DynamicTree_Query(&tree, (b3AABB){ { -2.0f, -2.0f, -2.0f }, { 2.5f, 2.0f, 2.0f } }, 1, false, tree_callback, &capture);
    oracle_case_begin(out, first, "tree.query.v1", "tree", "b3DynamicTree_Query", "{\"proxies\":[{\"aabb\":[\"0xbf800000\",\"0xbf800000\",\"0xbf800000\",\"0x3f800000\",\"0x3f800000\",\"0x3f800000\"],\"category\":\"0x0000000000000001\",\"userData\":\"0x1122334455667788\"},{\"aabb\":[\"0x40000000\",\"0xbf800000\",\"0xbf800000\",\"0x40800000\",\"0x3f800000\",\"0x3f800000\"],\"category\":\"0x0000000000000001\",\"userData\":\"0x8877665544332211\"}],\"queryAabb\":[\"0xc0000000\",\"0xc0000000\",\"0xc0000000\",\"0x40200000\",\"0x40000000\",\"0x40000000\"],\"mask\":\"0x0000000000000001\"}");
    oracle_key(out, "stats"); fprintf(out, "{\"nodeVisits\":"); oracle_i32(out, stats.nodeVisits); fputc(',', out); oracle_key(out, "leafVisits"); oracle_i32(out, stats.leafVisits); fputc('}', out); fputc(',', out);
    oracle_key(out, "hits"); fputc('[', out); for (int i = 0; i < capture.count; ++i) { if (i) fputc(',', out); fputs("{\"proxyId\":", out); oracle_i32(out, capture.ids[i]); fputs(",\"userData\":", out); oracle_u64(out, capture.data[i]); fputc('}', out); } fputc(']', out); oracle_case_end(out);
    (void)firstProxy; (void)secondProxy;
    b3DynamicTree_Destroy(&tree);
}
