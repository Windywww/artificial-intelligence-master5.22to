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

void system_delay_ms(uint32_t ms) { (void)ms; }
void sokoban_solver_prepare_map(SokobanContext *ctx, const uint8_t *raw_map)
{
    (void)ctx;
    (void)raw_map;
}
bool sokoban_solver_try_infer_identities(SokobanContext *ctx, State *state)
{
    (void)ctx;
    (void)state;
    return false;
}
bool sokoban_solver_select_recon_candidate(uint8_t start_pos, uint8_t current_direction,
                                            const ReconCandidate *candidates, uint8_t candidate_count,
                                            const uint8_t *obstacles, ReconCandidate *selected)
{
    (void)start_pos; (void)current_direction; (void)candidates;
    (void)candidate_count; (void)obstacles; (void)selected;
    return false;
}
bool sokoban_solver_solve_recon(SokobanContext *ctx, State *state,
                                const bool *obs_points, const bool *virtual_obs_points)
{
    (void)ctx; (void)state; (void)obs_points; (void)virtual_obs_points;
    return false;
}
int sokoban_recon_direction_to_angle(uint8_t direction) { (void)direction; return 0; }
uint8_t sokoban_recon_angle_to_direction(int current_angle) { (void)current_angle; return 0U; }
bool car_move(const WaypointPath *path, float yaw, uint8_t mode)
{
    (void)path; (void)yaw; (void)mode; return false;
}
void car_move_point(float x, float y, float yaw, uint8_t mode)
{
    (void)x; (void)y; (void)yaw; (void)mode;
}
void car_turn(float yaw) { (void)yaw; }
void check_image(char obj, char is_firsttime) { (void)obj; (void)is_firsttime; }

static void reset_map(uint8_t *map)
{
    memset(map, 0, MAP_SIZE);
    memset(mapin_goals, 0, sizeof(mapin_goals));
    memset(mapin_boxes, 0, sizeof(mapin_boxes));
    length_mapin_goals = 0U;
    length_mapin_boxes = 0U;
    run_type_state = 0U;
}

static void test_map_boundaries_and_transactionality(void)
{
    uint8_t map[MAP_SIZE], before[MAP_SIZE];
    reset_map(map); map[1] = 5U;
    assert(map_check_ifgetVisionLoc(map, 0U, 0U) == 0U);
    assert(map[0] == 5U && map[1] == 0U);

    reset_map(map); map[16] = 5U;
    assert(map_check_ifgetVisionLoc(map, 0U, 0U) == 0U);
    assert(map[0] == 5U && map[16] == 0U);

    reset_map(map); map[190] = 5U;
    assert(map_check_ifgetVisionLoc(map, 191U, 191U) == 0U);
    assert(map[191] == 5U && map[190] == 0U);

    reset_map(map); map[159] = 5U;
    assert(map_check_ifgetVisionLoc(map, 191U, 191U) == 0U);
    assert(map[191] == 5U && map[159] == 0U);

    reset_map(map); map[14] = 5U;
    assert(map_check_ifgetVisionLoc(map, 14U, 15U) == 0U);
    reset_map(map); map[175] = 5U;
    assert(map_check_ifgetVisionLoc(map, 175U, 191U) == 0U);

    reset_map(map); map[1] = 5U; memcpy(before, map, MAP_SIZE);
    assert(map_check_ifgetVisionLoc(map, 192U, 192U) == 0U);
    assert(memcmp(before, map, MAP_SIZE) == 0);
    reset_map(map); memcpy(before, map, MAP_SIZE);
    assert(map_check_ifgetVisionLoc(map, 1U, 1U) == 0U);
    assert(memcmp(before, map, MAP_SIZE) == 0);
    assert(map_check_ifgetVisionLoc(NULL, 0U, 0U) == 0U);
}

static void test_map_pushes(void)
{
    uint8_t map[MAP_SIZE], before[MAP_SIZE];
    reset_map(map); map[1] = 5U; map[2] = 2U;
    assert(map_check_ifgetVisionLoc(map, 2U, 2U) == 1U);
    assert(map[2] == 5U && map[3] == 2U);

    reset_map(map); map[1] = 5U; map[2] = 4U; map[3] = 1U;
    assert(map_check_ifgetVisionLoc(map, 2U, 2U) == 0U);
    assert(map[2] == 5U && map[3] == 1U);

    reset_map(map); run_type_state = 1U; map[1] = 5U; map[2] = 2U; map[3] = 3U;
    mapin_boxes[0].pos = 2U; mapin_boxes[0].id = 1U;
    mapin_goals[0].pos = 3U; mapin_goals[0].id = 1U;
    length_mapin_boxes = 1U; length_mapin_goals = 1U;
    assert(map_check_ifgetVisionLoc(map, 2U, 2U) == 0U);
    assert(map[2] == 5U && map[3] == 0U);
    assert(length_mapin_boxes == 0U && length_mapin_goals == 0U);

    reset_map(map); run_type_state = 1U; map[1] = 5U; map[2] = 2U; map[3] = 3U;
    memcpy(before, map, MAP_SIZE);
    assert(map_check_ifgetVisionLoc(map, 2U, 2U) == 0U);
    assert(memcmp(before, map, MAP_SIZE) == 0);
}

static void test_selective_path_split(void)
{
    WaypointPath path;
    State states[MAP_SIZE];
    memset(states, 0, sizeof(states));
    path.points[0] = 80U; path.points[1] = 91U; path.length = 2U;
    assert(sokoban_test_finalize_path(&path, states, 2U));
    assert(path.length == 2U);

    // For (1,1)->(2,1), the start-side cells (1,0)/(1,2) are not part of
    // this segment's correction decision.
    path.points[0] = 17U; path.points[1] = 18U; path.length = 2U;
    memset(states, 0, sizeof(states));
    states[0].box_count = 1U; states[0].boxes[0].pos = 1U;
    states[1] = states[0];
    assert(!sokoban_test_straight_segment_needs_visual_split(17U, 18U, &states[0]));

    // The destination-side cell (2,0) does trigger the same segment.
    path.points[0] = 17U; path.points[1] = 18U; path.length = 2U;
    memset(states, 0, sizeof(states));
    states[0].box_count = 1U; states[0].boxes[0].pos = 2U;
    states[1] = states[0];
    assert(sokoban_test_straight_segment_needs_visual_split(17U, 18U, &states[0]));

    path.points[0] = 15U; path.points[1] = 191U; path.length = 2U;
    memset(states, 0, sizeof(states));
    states[0].bomb_count = 1U; states[0].bombs[0] = 94U;
    states[1] = states[0];
    assert(sokoban_test_finalize_path(&path, states, 2U));
    assert(path.length == 4U);
    for (uint16_t i = 1U; i < path.length; i++)
    {
        int delta = (int)(path.points[i] / WIDTH) - (int)(path.points[i - 1U] / WIDTH);
        assert(delta <= MAX_L && delta >= -MAX_L);
    }

    path.points[0] = 80U; path.points[1] = 91U; path.length = 2U;
    states[0].box_count = 1U; states[0].boxes[0].pos = 67U;
    states[1] = states[0];
    assert(sokoban_test_finalize_path(&path, states, 2U));
    assert(path.length == 4U);
    for (uint16_t i = 1U; i < path.length; i++)
        assert((path.points[i] % WIDTH) - (path.points[i - 1U] % WIDTH) <= MAX_L);

    path.points[0] = 80U; path.points[1] = 91U; path.length = 2U;
    memset(states, 0, sizeof(states));
    states[0].box_count = 1U; states[0].boxes[0].pos = 92U;
    states[1] = states[0];
    assert(sokoban_test_finalize_path(&path, states, 2U));
    assert(path.length == 4U);

    path.points[0] = 80U; path.points[1] = 91U; path.length = 2U;
    memset(states, 0, sizeof(states)); states[0].box_count = 1U; states[0].boxes[0].pos = 85U;
    states[1] = states[0];
    assert(sokoban_test_finalize_path(&path, states, 2U));
    assert(path.length == 2U);
}

static void test_merge_then_split_and_marker(void)
{
    WaypointPath path;
    State states[MAP_SIZE];
    memset(states, 0, sizeof(states));
    path.points[0] = 80U; path.points[1] = 83U; path.points[2] = 86U; path.points[3] = 91U;
    path.length = 4U;
    states[0].bomb_count = 1U; states[0].bombs[0] = 68U;
    states[1] = states[0]; states[2] = states[0]; states[3] = states[0];
    assert(sokoban_test_finalize_path(&path, states, 4U));
    assert(path.length == 4U);
    for (uint16_t i = 1U; i < path.length; i++)
        assert(path.points[i] != path.points[i - 1U]);

    memset(states, 0, sizeof(states));
    path.points[0] = 80U; path.points[1] = 91U; path.points[2] = 255U; path.points[3] = 100U;
    path.length = 4U;
    assert(sokoban_test_finalize_path(&path, states, 4U));
    assert(path.length == 4U && path.points[2] == 255U);

    path.points[0] = 80U; path.points[1] = 255U; path.points[2] = 255U; path.points[3] = 100U;
    assert(sokoban_test_finalize_path(&path, states, 4U));
    assert(path.length == 4U && path.points[1] == 255U && path.points[2] == 255U);
}

static void test_dynamic_state_is_scoped_to_raw_range(void)
{
    WaypointPath path;
    State states[MAP_SIZE];
    const uint8_t raw_points[] = {0U, 1U, 2U, 18U, 34U, 50U, 66U, 82U, 98U};
    memset(states, 0, sizeof(states));
    memcpy(path.points, raw_points, sizeof(raw_points));
    path.length = (uint16_t)sizeof(raw_points);

    states[0].box_count = 1U;
    states[0].boxes[0].pos = 49U;
    states[1] = states[0];
    assert(sokoban_test_finalize_path(&path, states, path.length));
    assert(path.length == 3U);
    assert(path.points[0] == 0U && path.points[1] == 2U && path.points[2] == 98U);
}

static void test_generate_path_replay_and_capacity(void)
{
    SokobanContext ctx;
    WaypointPath path;
    memset(&ctx, 0, sizeof(ctx));
    ctx.initial_state.car_pos = 0U;
    ctx.initial_state.box_count = 1U;
    ctx.initial_state.boxes[0].pos = 1U;
    ctx.solution_actions_len = 6U;
    for (uint8_t i = 0U; i < ctx.solution_actions_len; i++)
    {
        ctx.solution_actions[i].move_to = i;
        ctx.solution_actions[i].push_to = (uint8_t)(i + 1U);
    }
    assert(generate_path(&ctx, &path));
    assert(path.length == 2U && path.points[0] == 0U && path.points[1] == 6U);

    memset(&ctx, 0, sizeof(ctx));
    ctx.initial_state.car_pos = 0U;
    ctx.initial_state.box_count = 1U;
    ctx.initial_state.boxes[0].pos = 1U;
    ctx.initial_state.bomb_count = 1U;
    ctx.initial_state.bombs[0] = 16U;
    ctx.solution_actions_len = 6U;
    for (uint8_t i = 0U; i < ctx.solution_actions_len; i++)
    {
        ctx.solution_actions[i].move_to = i;
        ctx.solution_actions[i].push_to = (uint8_t)(i + 1U);
    }
    assert(generate_path(&ctx, &path));
    assert(path.length > 0U);

    memset(&ctx, 0, sizeof(ctx));
    ctx.initial_state.car_pos = 0U;
    ctx.initial_state.bomb_count = 1U;
    ctx.initial_state.bombs[0] = 1U;
    ctx.solution_actions_len = 1U;
    ctx.solution_actions[0].move_to = 0U;
    ctx.solution_actions[0].push_to = 1U;
    ctx.solution_actions[0].is_explode = true;
    assert(generate_path(&ctx, &path));
    assert(path.length == 3U && path.points[2] == 255U);
}

int main(void)
{
    test_map_boundaries_and_transactionality();
    test_map_pushes();
    test_selective_path_split();
    test_merge_then_split_and_marker();
    test_dynamic_state_is_scoped_to_raw_range();
    test_generate_path_replay_and_capacity();
    return 0;
}
