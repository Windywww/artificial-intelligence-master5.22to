#include "sokoban_engine.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#ifdef SOKOBAN_ENGINE_TEST
uint16_t sokoban_test_wall_action_penalty(uint8_t wall_type);
uint16_t sokoban_test_wall_heuristic_penalty(uint8_t wall_type);
void sokoban_test_refresh_distances(SokobanContext *ctx, const uint8_t *walls, uint8_t bomb_count);
#endif

// 六箱比赛回归地图，独立于 algorithm_main.c 当前选择的地图。
static const uint8_t b678[MAP_SIZE] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 3, 2, 0, 0, 1, 0, 3, 3, 3, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 2, 0, 0, 1,
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1,
    1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1,
    1, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 1, 0, 0, 1,
    1, 5, 4, 0, 0, 1, 2, 0, 0, 0, 0, 1, 1, 0, 3, 1,
    1, 0, 4, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1,
    1, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 2, 0, 1, 0, 0, 4, 0, 0, 0, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

static void test_capacity_30(void) {
    uint8_t map[MAP_SIZE];
    SokobanContext ctx;
    memset(map, 0, sizeof(map));
    memset(&ctx, 0, sizeof(ctx));

    map[MAP_SIZE - 1] = 5;
    for (int i = 0; i < MAX_BOXES; i++) {
        map[i * 2] = 2;
        map[i * 2 + 1] = 3;
    }

    build_map_info(&ctx, map, 0);
    assert(ctx.initial_state.box_count == 30);
    assert(ctx.goal_count == 30);
    assert(ctx.initial_state.active_goals_mask == UINT32_C(0x3fffffff));
}

static void test_open_outer_edge(void) {
    uint8_t map[MAP_SIZE];
    SokobanContext ctx;
    memset(map, 0, sizeof(map));
    memset(&ctx, 0, sizeof(ctx));

    map[0] = 5;
    map[1] = 2;
    map[2] = 3;

    build_map_info(&ctx, map, 0);
    assert(ctx.boundary_walls[0] == 0);
    assert(solve(&ctx));
    assert(ctx.solution_actions_len == 1);
}

static void test_capacity_overflow_rejected(void) {
    uint8_t map[MAP_SIZE];
    SokobanContext ctx;
    memset(map, 0, sizeof(map));
    memset(&ctx, 0, sizeof(ctx));

    map[MAP_SIZE - 1] = 5;
    for (int i = 0; i < MAX_BOXES + 1; i++) map[i] = 2;
    for (int i = 0; i < MAX_GOALS; i++) map[40 + i] = 3;

    build_map_info(&ctx, map, 0);
    assert(!ctx.map_valid);
    assert(!solve(&ctx));
}

static void test_boundary_wall_is_not_explodable(void) {
    uint8_t map[MAP_SIZE];
    SokobanContext ctx;
    memset(map, 0, sizeof(map));
    memset(&ctx, 0, sizeof(ctx));

    map[0] = 1;
    map[MAP_SIZE - 1] = 5;

    build_map_info(&ctx, map, 0);
    assert(ctx.boundary_walls[0] == 1);
    assert(ctx.boundary_walls[1] == 0);

    for (uint8_t i = 0; i < ctx.explosion_area_count[1]; i++) {
        assert(ctx.explosion_areas[1][i] != 0);
    }
}

static void test_wall_priority_and_default_weight(void) {
    SokobanContext ctx;
    memset(&ctx, 0, sizeof(ctx));

    build_map_info(&ctx, b678, 0);
    assert(ctx.initial_walls[0] == WALL_NORMAL);
    assert(ctx.initial_walls[1 * WIDTH + 5] == WALL_SEPARATOR);
    // (5,4) 先被识别为隔离墙，随后因邻近死锁箱子而升级为最高优先级。
    assert(ctx.initial_walls[4 * WIDTH + 5] == WALL_DEADLOCK);
    assert(ctx.current_weight == 3.0f);
    assert(ctx.min_weight == 3.0f);
}

#ifdef SOKOBAN_ENGINE_TEST
static void test_wall_costs_and_bomb_sensitive_cache(void) {
    assert(sokoban_test_wall_action_penalty(WALL_NORMAL) == 2);
    assert(sokoban_test_wall_action_penalty(WALL_SEPARATOR) == 1);
    assert(sokoban_test_wall_action_penalty(WALL_DEADLOCK) == 0);
    assert(sokoban_test_wall_heuristic_penalty(WALL_NORMAL) == 20);
    assert(sokoban_test_wall_heuristic_penalty(WALL_SEPARATOR) == 10);
    assert(sokoban_test_wall_heuristic_penalty(WALL_DEADLOCK) == 0);

    uint8_t map[MAP_SIZE] = {0};
    SokobanContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    map[0] = 5;
    map[1] = 4;
    map[1 * WIDTH + 4] = 3;
    // 完整竖墙把测试点与目标分隔开，只有剩余炸弹时才允许启发式穿墙。
    for (int y = 0; y < HEIGHT; y++)
        map[y * WIDTH + 2] = 1;

    build_map_info(&ctx, map, 0);
    sokoban_test_refresh_distances(&ctx, ctx.initial_walls, 1);
    assert(ctx.cached_dist_table[0][1 * WIDTH + 1] < INF_DIST);
    sokoban_test_refresh_distances(&ctx, ctx.initial_walls, 0);
    assert(ctx.cached_bomb_count == 0);
    assert(ctx.cached_dist_table[0][1 * WIDTH + 1] == INF_DIST);
}
#endif

static void test_b678_regression(void) {
    SokobanContext ctx;
    memset(&ctx, 0, sizeof(ctx));

    build_map_info(&ctx, b678, 0);
    assert(ctx.initial_state.box_count == 6);
    assert(ctx.goal_count == 6);
    assert(solve(&ctx));
    assert(ctx.solution_actions_len <= MAX_STEPS);
    assert(ctx.total_explored_nodes <= 700000);
}

int main(void) {
    test_capacity_30();
    test_open_outer_edge();
    test_capacity_overflow_rejected();
    test_boundary_wall_is_not_explodable();
    test_wall_priority_and_default_weight();
#ifdef SOKOBAN_ENGINE_TEST
    test_wall_costs_and_bomb_sensitive_cache();
#endif
    test_b678_regression();
    puts("sokoban_engine_test: OK");
    return 0;
}
