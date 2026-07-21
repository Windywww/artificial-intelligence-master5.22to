#include "sokoban_engine.h"

#include <assert.h>
#include <string.h>

int main(void)
{
    uint8_t map[MAP_SIZE] = {0};
    SokobanContext ctx;
    memset(&ctx, 0, sizeof(ctx));

    // 此地图至少需要进入一个子节点；测试构建把累计预算设为 1。
    map[0] = 5;
    map[1] = 2;
    map[2] = 3;

    build_map_info(&ctx, map, 0);
    assert(!solve(&ctx));
    assert(ctx.total_explored_nodes == 1);
    return 0;
}
