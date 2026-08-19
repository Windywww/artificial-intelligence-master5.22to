#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "sokoban_engine.h"

typedef struct
{
    uint8_t pos;
    uint8_t target_info;
    uint8_t direction;
} ReconCandidate;

extern EntityData mapin_goals[MAX_GOALS];
extern EntityData mapin_boxes[MAX_BOXES];
extern uint8_t length_mapin_goals;
extern uint8_t length_mapin_boxes;

volatile float time_line = 0.0f;
volatile uint8_t image_rx_state = 0U;
volatile uint8_t final_image_index = 0U;
volatile uint8_t yaw_arrived_flag = 0U;
volatile uint8_t navigate_flag = 0U;
uint8_t vision_run_correct_switch = 0U;
uint8_t ban_last_vision_correct = 0U;

void system_delay_ms(uint32_t ms)
{
    (void)ms;
}

void sokoban_solver_prepare_map(SokobanContext *ctx, const uint8_t *raw_map)
{
    (void)ctx;
    (void)raw_map;
}

bool sokoban_solver_try_infer_identities(SokobanContext *ctx, State *current_state)
{
    (void)ctx;
    (void)current_state;
    return false;
}

bool sokoban_solver_select_recon_candidate(uint8_t start_pos, uint8_t current_direction,
                                            const ReconCandidate *candidates, uint8_t candidate_count,
                                            const uint8_t *obstacles, ReconCandidate *selected)
{
    (void)start_pos;
    (void)current_direction;
    (void)candidates;
    (void)candidate_count;
    (void)obstacles;
    (void)selected;
    return false;
}

bool sokoban_solver_solve_recon(SokobanContext *ctx, State *start_state,
                                const bool *obs_points, const bool *virtual_obs_points)
{
    (void)ctx;
    (void)start_state;
    (void)obs_points;
    (void)virtual_obs_points;
    return false;
}

int sokoban_recon_direction_to_angle(uint8_t direction)
{
    (void)direction;
    return 0;
}

uint8_t sokoban_recon_angle_to_direction(int current_angle)
{
    (void)current_angle;
    return 0U;
}

bool car_move(const WaypointPath *path, float yaw, uint8_t mode)
{
    (void)path;
    (void)yaw;
    (void)mode;
    return false;
}

void car_move_point(float x, float y, float yaw, uint8_t mode)
{
    (void)x;
    (void)y;
    (void)yaw;
    (void)mode;
}

void car_turn(float yaw)
{
    (void)yaw;
}

void check_image(char obj, char is_firsttime)
{
    (void)obj;
    (void)is_firsttime;
}

static void reset_state(uint8_t *map)
{
    memset(map, 0, MAP_SIZE);
    memset(mapin_goals, 0, sizeof(EntityData) * MAX_GOALS);
    memset(mapin_boxes, 0, sizeof(EntityData) * MAX_BOXES);
    length_mapin_goals = 0;
    length_mapin_boxes = 0;
    run_type_state = 0;
}

static void assert_map_unchanged(const uint8_t *before, const uint8_t *after)
{
    assert(memcmp(before, after, MAP_SIZE) == 0);
}

static void test_edge_paths(void)
{
    uint8_t map[MAP_SIZE];

    reset_state(map);
    map[1] = 5;
    assert(map_check_ifgetVisionLoc(map, 0, 0) == 0U);
    assert(map[0] == 5U && map[1] == 0U);

    reset_state(map);
    map[16] = 5;
    assert(map_check_ifgetVisionLoc(map, 0, 0) == 0U);
    assert(map[0] == 5U && map[16] == 0U);

    reset_state(map);
    map[190] = 5;
    assert(map_check_ifgetVisionLoc(map, 191, 191) == 0U);
    assert(map[191] == 5U && map[190] == 0U);

    reset_state(map);
    map[159] = 5;
    assert(map_check_ifgetVisionLoc(map, 191, 191) == 0U);
    assert(map[191] == 5U && map[159] == 0U);
}

static void test_normal_map_updates(void)
{
    uint8_t map[MAP_SIZE];

    reset_state(map);
    map[1] = 5;
    map[2] = 2;
    assert(map_check_ifgetVisionLoc(map, 2, 2) == 1U);
    assert(map[1] == 0U && map[2] == 5U && map[3] == 2U);

    reset_state(map);
    map[1] = 5;
    map[2] = 4;
    map[3] = 1;
    assert(map_check_ifgetVisionLoc(map, 2, 2) == 0U);
    assert(map[1] == 0U && map[2] == 5U && map[3] == 1U);

    reset_state(map);
    map[0] = 5;
    assert(map_check_ifgetVisionLoc(map, 17, 17) == 0U);
    assert(map[0] == 0U && map[17] == 5U);

    reset_state(map);
    map[1] = 5;
    map[5] = 4;
    assert(map_check_ifgetVisionLoc(map, 2, 2) == 1U);
    assert(map[2] == 5U);
}

static void test_classified_box_updates(void)
{
    uint8_t map[MAP_SIZE];

    reset_state(map);
    run_type_state = 1U;
    map[1] = 5;
    map[2] = 2;
    mapin_boxes[0].pos = 2U;
    mapin_boxes[0].id = 1U;
    length_mapin_boxes = 1U;
    assert(map_check_ifgetVisionLoc(map, 2, 2) == 1U);
    assert(map[2] == 5U && map[3] == 2U && mapin_boxes[0].pos == 3U);

    reset_state(map);
    run_type_state = 1U;
    map[1] = 5;
    map[2] = 2;
    map[3] = 3;
    mapin_boxes[0].pos = 2U;
    mapin_boxes[0].id = 1U;
    mapin_goals[0].pos = 3U;
    mapin_goals[0].id = 1U;
    length_mapin_boxes = 1U;
    length_mapin_goals = 1U;
    assert(map_check_ifgetVisionLoc(map, 2, 2) == 0U);
    assert(map[2] == 5U && map[3] == 0U);
    assert(length_mapin_boxes == 0U && length_mapin_goals == 0U);
}

static void test_invalid_inputs_leave_map_intact(void)
{
    uint8_t map[MAP_SIZE];
    uint8_t before[MAP_SIZE];

    reset_state(map);
    map[1] = 5;
    memcpy(before, map, MAP_SIZE);
    assert(map_check_ifgetVisionLoc(map, 192U, 192U) == 0U);
    assert_map_unchanged(before, map);

    reset_state(map);
    memcpy(before, map, MAP_SIZE);
    assert(map_check_ifgetVisionLoc(map, 1U, 1U) == 0U);
    assert_map_unchanged(before, map);

    reset_state(map);
    map[1] = 5;
    map[0] = 2;
    memcpy(before, map, MAP_SIZE);
    assert(map_check_ifgetVisionLoc(map, 0U, 0U) == 0U);
    assert_map_unchanged(before, map);

    reset_state(map);
    map[1] = 5;
    map[2] = 2;
    map[3] = 3;
    run_type_state = 1U;
    memcpy(before, map, MAP_SIZE);
    assert(map_check_ifgetVisionLoc(map, 2U, 2U) == 0U);
    assert_map_unchanged(before, map);

    assert(map_check_ifgetVisionLoc(NULL, 0U, 0U) == 0U);
}

static void test_selective_visual_splitting(void)
{
    State state;
    memset(&state, 0, sizeof(state));

    assert(!sokoban_test_straight_segment_needs_visual_split(0U, 5U, &state));
    assert(!sokoban_test_straight_segment_needs_visual_split(0U, 6U, &state));

    state.box_count = 1U;
    state.boxes[0].pos = 19U;
    assert(sokoban_test_straight_segment_needs_visual_split(0U, 6U, &state));

    memset(&state, 0, sizeof(state));
    state.bomb_count = 1U;
    state.bombs[0] = 7U;
    assert(sokoban_test_straight_segment_needs_visual_split(0U, 6U, &state));

    state.bombs[0] = 28U;
    assert(sokoban_test_straight_segment_needs_visual_split(15U, 9U, &state));

    state.bombs[0] = 49U;
    assert(sokoban_test_straight_segment_needs_visual_split(0U, 96U, &state));

    state.bombs[0] = 129U;
    assert(sokoban_test_straight_segment_needs_visual_split(176U, 80U, &state));

    memset(&state, 0, sizeof(state));
    assert(!sokoban_test_straight_segment_needs_visual_split(9U, 15U, &state));
    state.box_count = 1U;
    state.boxes[0].pos = 28U;
    assert(sokoban_test_straight_segment_needs_visual_split(9U, 15U, &state));
    state.box_count = 0U;
    assert(!sokoban_test_straight_segment_needs_visual_split(9U, 15U, &state));
}

static void test_split_point_distribution(void)
{
    State state;
    WaypointPath smooth_path;
    WaypointPath output;

    memset(&state, 0, sizeof(state));
    memset(&smooth_path, 0, sizeof(smooth_path));
    memset(&output, 0, sizeof(output));
    smooth_path.length = 2U;
    smooth_path.points[0] = 0U;
    smooth_path.points[1] = 11U;
    assert(sokoban_test_append_smooth_path(&output, &smooth_path, &state));
    assert(output.length == 2U && output.points[0] == 0U && output.points[1] == 11U);

    memset(&output, 0, sizeof(output));
    state.box_count = 1U;
    state.boxes[0].pos = 21U;
    assert(sokoban_test_append_smooth_path(&output, &smooth_path, &state));
    assert(output.length == 4U);
    assert(output.points[0] == 0U && output.points[1] == 4U);
    assert(output.points[2] == 7U && output.points[3] == 11U);
}

static void test_generate_path_preserves_explosion_marker(void)
{
    SokobanContext ctx;
    WaypointPath path;
    memset(&ctx, 0, sizeof(ctx));
    memset(&path, 0, sizeof(path));

    ctx.initial_state.car_pos = 0U;
    ctx.initial_state.bomb_count = 1U;
    ctx.initial_state.bombs[0] = 2U;
    ctx.solution_actions_len = 1U;
    ctx.solution_actions[0] = (MacroAction){1U, 2U, true, false};

    assert(generate_path(&ctx, &path));
    assert(path.length == 3U);
    assert(path.points[0] == 0U && path.points[1] == 2U);
    assert(path.points[2] == 255U);
    assert(ctx.initial_state.car_pos == 2U && ctx.initial_state.bomb_count == 0U);
}

static void test_generate_path_merges_collinear_pushes(void)
{
    SokobanContext ctx;
    WaypointPath path;
    memset(&ctx, 0, sizeof(ctx));
    memset(&path, 0, sizeof(path));

    ctx.initial_state.car_pos = 0U;
    ctx.initial_state.box_count = 1U;
    ctx.initial_state.boxes[0].pos = 2U;
    ctx.solution_actions_len = 2U;
    ctx.solution_actions[0] = (MacroAction){1U, 2U, false, false};
    ctx.solution_actions[1] = (MacroAction){2U, 3U, false, false};

    assert(generate_path(&ctx, &path));
    assert(path.length == 2U);
    assert(path.points[0] == 0U && path.points[1] == 3U);
}

int main(void)
{
    test_edge_paths();
    test_normal_map_updates();
    test_classified_box_updates();
    test_invalid_inputs_leave_map_intact();
    test_selective_visual_splitting();
    test_split_point_distribution();
    test_generate_path_preserves_explosion_marker();
    test_generate_path_merges_collinear_pushes();
    return 0;
}
