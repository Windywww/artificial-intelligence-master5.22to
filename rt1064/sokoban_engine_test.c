#include "sokoban_engine.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

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

int main(void) {
    test_capacity_30();
    test_open_outer_edge();
    test_capacity_overflow_rejected();
    test_boundary_wall_is_not_explodable();
    puts("sokoban_engine_test: OK");
    return 0;
}
