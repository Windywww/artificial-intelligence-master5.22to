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

int main(void)
{
    test_edge_paths();
    test_normal_map_updates();
    test_classified_box_updates();
    test_invalid_inputs_leave_map_intact();
    return 0;
}
