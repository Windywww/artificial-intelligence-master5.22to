#include "sokoban_engine.h"
#include "sokoban_solver_internal.h"
#include <string.h>
#include "myUart.h"
#include "move_control.h"
#include "WIFI2SPI.h"

#ifndef ENTER_GOAL
#define ENTER_GOAL 1 // 识别时能否进入目标点 1=能
#endif
#define MAX_ID 12 // id 可能的取值个数
#define UNKNOWN 11
#define ERROR 99.0f
#define MAX_RECON_CANDIDATES ((MAX_BOXES + MAX_GOALS) * 4)

int angle = 0;

static bool get_micro_path(uint8_t start_pos, uint8_t target_pos,
                           const uint8_t *obstacles, WaypointPath *out_path);
static void get_smooth_path(const WaypointPath *grid_path, const uint8_t *obstacles,
                            WaypointPath *out_smooth_path);
void goal_box_giveRelation(SokobanContext *ctx);

static inline int neighbor_index(int idx, int direction)
{
    int x = idx % WIDTH;
    int y = idx / WIDTH;
    if (direction == 0)
        y--;
    else if (direction == 1)
        y++;
    else if (direction == 2)
        x--;
    else
        x++;
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
        return -1;
    return y * WIDTH + x;
}

static inline bool valid_map_xy(int x, int y)
{
    return x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT;
}

static inline bool is_dynamic_map_object(uint8_t type)
{
    return type == 2U || type == 4U || type == 6U || type == 7U;
}

static inline bool can_consume_goal(uint8_t box_type, uint8_t goal_type)
{
    return box_type != UNKNOWN && goal_type != UNKNOWN && box_type == goal_type;
}

bool build_map_info(SokobanContext *ctx, const uint8_t *raw_map, uint8_t cls)
{
    run_type_state = cls;
    if (ctx == NULL || raw_map == NULL)
        return false;

    sokoban_solver_prepare_map(ctx, raw_map);
    if (!ctx->map_valid)
        return false;
    State *current_state = &ctx->initial_state;
    if (cls == 0)
    {
        for (uint8_t j = 0; j < current_state->box_count; j++)
        {
            current_state->boxes[j].id = NO_CLS;
        }
        for (uint8_t j = 0; j < ctx->goal_count; j++)
        {
            ctx->goals[j].id = NO_CLS;
            ctx->goal_type_map[ctx->goals[j].pos] = NO_CLS;
        }
        return true;
    }
    // 只有一对儿的情况
    if (ctx->goal_count == 1 && ctx->initial_state.box_count == 1)
    {
        ctx->goals[0].id = 1;
        current_state->boxes[0].id = 1;
        return true;
    }

    uint8_t unid_boxes = current_state->box_count;
    uint8_t unid_goals = ctx->goal_count;
    // Each bit records a failed target direction from this viewpoint.
    uint8_t failed[MAP_SIZE] = {0};

    bool is_first = (cls == 2) ? true : false;
    while (unid_boxes > 0 || unid_goals > 0)
    {
        uint8_t obstacles[MAP_SIZE];
        uint8_t the_goals[MAP_SIZE];
        memcpy(obstacles, ctx->cached_walls, MAP_SIZE);
        memset(the_goals, 0, sizeof(the_goals));
        for (int i = 0; i < current_state->box_count; i++)
            obstacles[current_state->boxes[i].pos] = 1;
        for (int i = 0; i < current_state->bomb_count; i++)
            obstacles[current_state->bombs[i]] = 1;
        for (int i = 0; i < ctx->goal_count; i++)
        {
            if (current_state->active_goals_mask & (1U << i))
            {
                the_goals[ctx->goals[i].pos] = 1;
            }
        }

        bool observation_points[MAP_SIZE] = {false};
        bool virtual_obs_points[MAP_SIZE] = {false}; // 虚拟视点, 给IDA*用的
        // 候选列表保留“视点 + 目标 + 朝向”，避免多个目标共用同一视点时互相覆盖。
        ReconCandidate candidates[MAX_RECON_CANDIDATES];
        uint8_t candidate_count = 0;
        WaypointPath path;
        WaypointPath smooth_path;

        // bool skip_goals = (unid_goals == 1 && unid_boxes > 1);
        bool skip_goals = false;
        if (!skip_goals)
        {
            for (int i = 0; i < ctx->goal_count; i++)
            {
                if (ctx->goals[i].id == UNKNOWN)
                {
                    uint8_t g_pos = ctx->goals[i].pos;
                    for (int d = 0; d < 4; d++)
                    {
                        int n = neighbor_index(g_pos, d);
                        if (n >= 0 && !(failed[n] & (1U << (d ^ 1))))
                        {
                            virtual_obs_points[n] = true; // used for IDA*
                            if (the_goals[n] && ENTER_GOAL == 0)
                            {
                                virtual_obs_points[n] = false;
                            }
                            if (!obstacles[n] && (ENTER_GOAL || !the_goals[n]))
                            {
                                observation_points[n] = true;
                                // d ^ 1 是从观察点看向目标点的方向。
                                candidates[candidate_count++] = (ReconCandidate){(uint8_t)n, (uint8_t)i, (uint8_t)(d ^ 1)};
                            }
                        }
                    }
                }
            }
        }

        // bool skip_boxes = (unid_boxes == 1 && unid_goals > 1);
        bool skip_boxes = false;
        if (!skip_boxes && !is_first)
        {
            for (int i = 0; i < current_state->box_count; i++)
            {
                if (current_state->boxes[i].id == UNKNOWN)
                {
                    uint8_t b_pos = current_state->boxes[i].pos;
                    for (int d = 0; d < 4; d++)
                    {
                        int n = neighbor_index(b_pos, d);
                        if (n >= 0 && !(failed[n] & (1U << (d ^ 1))))
                        {
                            virtual_obs_points[n] = true; // used for IDA*
                            if (the_goals[n] && ENTER_GOAL == 0)
                            {
                                virtual_obs_points[n] = false;
                            }
                            if (!obstacles[n] && (ENTER_GOAL || !the_goals[n]))
                            {
                                observation_points[n] = true;
                                // 箱子候选在 target_info 最高位标记类型，低 7 位保留箱子索引。
                                candidates[candidate_count++] = (ReconCandidate){(uint8_t)n, (uint8_t)((1U << 7) | i), (uint8_t)(d ^ 1)};
                            }
                        }
                    }
                }
            }
        }
        if (candidate_count == 0)
        {
            return false;
        }
        ReconCandidate selected_candidate;
        // 在线选点负责降低识别阶段原地旋转；实际移动仍由 get_micro_path() 生成严格最短单段路径。
        if (sokoban_solver_select_recon_candidate(current_state->car_pos, sokoban_recon_angle_to_direction(angle),
                                                  candidates, candidate_count, obstacles, &selected_candidate) &&
            get_micro_path(current_state->car_pos, selected_candidate.pos, obstacles, &path))
        {
            uint8_t final_pos = selected_candidate.pos;
            uint8_t target_info = selected_candidate.target_info;
            bool is_box = (target_info >> 7) == 1;
            uint8_t index = target_info & 0x7FU;
            uint8_t entity_pos = is_box ? current_state->boxes[index].pos : ctx->goals[index].pos;
            get_smooth_path(&path, obstacles, &smooth_path);
            current_state->car_pos = final_pos;

            uint8_t target_direction = selected_candidate.direction;

            //--��Ϊ�˲��ߵ����һ���㣬���һ������������
            smooth_path.length--;
            if (smooth_path.length > 0U && !car_move(&smooth_path, angle, 0))
                return false;
            while (navigate_flag)
            {
                wifi_task();
            }
            uint8_t final_pos_X = final_pos % 16;
            uint8_t final_pos_Y = final_pos / 16;
            uint8_t entity_pos_X = entity_pos % 16;
            uint8_t entity_pos_Y = entity_pos / 16;

            int8_t dx = entity_pos_X - final_pos_X;
            int8_t dy = entity_pos_Y - final_pos_Y;

            float final_actual_x = final_pos_X * 0.2f + 0.1f;
            float final_actual_y = 2.4f - final_pos_Y * 0.2f - 0.1f;
            // 如果要用直接到视点而非逼近式的，赋值为-0.001f即可，逼近式的步长为0.005f，避免过冲
            float back_error = 0.01f;
            if (dx > 0)
                final_actual_x -= back_error;
            else if (dx < 0)
                final_actual_x += back_error;
            else if (dy > 0)
                final_actual_y += back_error;
            else if (dy < 0)
                final_actual_y -= back_error;

            ban_last_vision_correct = 1;
            car_move_point(final_actual_x, final_actual_y, angle, 0);
            while (navigate_flag)
            {
                wifi_task();
            }

            ban_last_vision_correct = 0;
            int target_angle = sokoban_recon_direction_to_angle(target_direction);
            bool direction_changed = sokoban_recon_angle_to_direction(angle) != target_direction;
            angle = target_angle;
            // 同方向连续识别不调用 car_turn()，避免产生空转和额外等待。
            if (direction_changed)
            {
                car_turn(angle);
                while (!yaw_arrived_flag)
                {
                    wifi_task();
                }
                system_delay_ms(TURN_DELAY_TIME_MS);
            }
            // UNKNOWN is a valid result; UINT8_MAX means the request is pending.
            final_image_index = UINT8_MAX;
            check_image(3 - is_box, 1);
            float thistime_soko = time_line;
            vision_run_correct_switch = 0;

            while (time_line - thistime_soko <= 1.2f)
            {
                if (image_rx_state == 0)
                {
                    check_image(3 - is_box, 1);
                }
                else
                {
                    check_image(3 - is_box, 0);
                }
                if (final_image_index != UINT8_MAX)
                {
                    break;
                }
            }
            while (final_image_index == UINT8_MAX)
            {
                if (image_rx_state == 0)
                {
                    check_image(3 - is_box, 1);
                }
                else
                {
                    check_image(3 - is_box, 0);
                }
                if (dx > 0)
                {
                    if (final_actual_x < final_pos_X * 0.2f + 0.1f)
                    {
                        final_actual_x += 0.005f;
                    }
                }
                else if (dx < 0)
                {
                    if (final_actual_x > final_pos_X * 0.2f + 0.1f)
                    {
                        final_actual_x -= 0.005f;
                    }
                }
                else if (dy > 0)
                {
                    if (final_actual_y > 2.4f - final_pos_Y * 0.2f - 0.1f)
                    {
                        final_actual_y -= 0.005f;
                    }
                }
                else if (dy < 0)
                {
                    if (final_actual_y < 2.4f - final_pos_Y * 0.2f - 0.1f)
                    {
                        final_actual_y += 0.005f;
                    }
                }

                car_move_point(final_actual_x, final_actual_y, angle, 0);
                uint8_t wait_ok = 0;
                while (navigate_flag)
                {
                    if (final_image_index != UINT8_MAX)
                    {
                        wait_ok = 1;
                        break;
                    }
                    wifi_task();
                }
                if (wait_ok)
                {
                    break;
                }

                if (time_line - thistime_soko >= 9)
                {
                    for (uint8_t j = 0; j < current_state->box_count; j++)
                    {
                        current_state->boxes[j].id = NO_CLS;
                    }
                    for (uint8_t j = 0; j < ctx->goal_count; j++)
                    {
                        ctx->goals[j].id = NO_CLS;
                        ctx->goal_type_map[ctx->goals[j].pos] = NO_CLS;
                    }
                    run_type_state = 0;
                    return true;
                }
            }
            vision_run_correct_switch = 0;
            // system_delay_ms(700);
            uint8_t recognized_id = final_image_index;

            if (recognized_id == NO_CLS)
            {
                for (uint8_t j = 0; j < current_state->box_count; j++)
                {
                    current_state->boxes[j].id = NO_CLS;
                }
                for (uint8_t j = 0; j < ctx->goal_count; j++)
                {
                    ctx->goals[j].id = NO_CLS;
                    ctx->goal_type_map[ctx->goals[j].pos] = NO_CLS;
                }
                return true;
            }

            if (recognized_id == UNKNOWN)
            {
                failed[final_pos] |= (uint8_t)(1U << target_direction);
                continue;
            }

            if (is_first)
            {
                is_first = false;
            }

            if (is_box)
            {
                current_state->boxes[index].id = recognized_id;
                unid_boxes--;
                // sort_boxes(current_state->boxes, current_state->box_count);
            }
            else
            {
                ctx->goals[index].id = recognized_id;
                ctx->goal_type_map[entity_pos] = recognized_id;
                unid_goals--;
            }

            while (sokoban_solver_try_infer_identities(ctx, current_state))
            {
                unid_boxes = 0;
                unid_goals = 0;
                for (int k = 0; k < current_state->box_count; k++)
                {
                    if (current_state->boxes[k].id == UNKNOWN)
                        unid_boxes++;
                }
                for (int k = 0; k < ctx->goal_count; k++)
                {
                    if (ctx->goals[k].id == UNKNOWN)
                        unid_goals++;
                }
            }
        }
        else
        {
            if (sokoban_solver_solve_recon(ctx, current_state, observation_points, virtual_obs_points))
            {
                goal_box_giveRelation(ctx);
                if (!generate_path(ctx, &smooth_path))
                    return false;
                current_state = &ctx->initial_state;

                if (!car_move(&smooth_path, angle, 0))
                    return false;
                while (navigate_flag)
                    continue;
            }
            else
            {
                return false;
            }
        }
    }

    // 最终识别兜底检查：
    // 统计每个“已知类别”的箱子和目标点数量，用来发现是否只存在一对单点类别错配。
    // UNKNOWN 表示仍未识别清楚，不能作为确定类别参与数量平衡判断。
    uint8_t final_box_counts[MAX_ID] = {0};
    uint8_t final_goal_counts[MAX_ID] = {0};

    // 统计箱子端各类别数量，跳过 UNKNOWN。
    for (int i = 0; i < current_state->box_count; i++)
    {
        if (current_state->boxes[i].id != UNKNOWN)
        {
            final_box_counts[current_state->boxes[i].id]++;
        }
    }
    // 统计目标点端各类别数量，跳过 UNKNOWN。
    for (int i = 0; i < ctx->goal_count; i++)
    {
        if (ctx->goals[i].id != UNKNOWN)
        {
            final_goal_counts[ctx->goals[i].id]++;
        }
    }
    uint8_t box_surplus_count = 0;
    uint8_t goal_surplus_count = 0;
    uint8_t err_box_id = 0;
    uint8_t err_goal_id = 0;

    // 查找类别数量差异：
    // - box_surplus_count：箱子比目标点多出来的已知类别数量；
    // - goal_surplus_count：目标点比箱子多出来的已知类别数量；
    // 循环从 1 开始，故意忽略 NO_CLS(0)，只检查实际分类 id。
    for (uint8_t id = 1; id < MAX_ID; id++)
    {
        if (id == UNKNOWN)
            continue;

        if (final_box_counts[id] > final_goal_counts[id])
        {
            box_surplus_count += (final_box_counts[id] - final_goal_counts[id]);
            err_box_id = id;
        }
        else if (final_goal_counts[id] > final_box_counts[id])
        {
            goal_surplus_count += (final_goal_counts[id] - final_box_counts[id]);
            err_goal_id = id;
        }
    }
    // 若恰好表现为“一个箱子类别多 1、另一个目标类别多 1”，
    // 且这两个可疑类别各自只出现一次，则很可能是一次箱子/目标分类错配。
    // 此时把这一只箱子和这一处目标点降级为 NO_CLS，让后续求解按“无类别约束”处理，
    // 避免因为单次识别误判直接判定整张图类别不匹配。
    if (box_surplus_count == 1 && goal_surplus_count == 1 &&
        final_box_counts[err_box_id] == 1 && final_goal_counts[err_goal_id] == 1)
    {

        // 将多出来的那个箱子类别降级为 NO_CLS。
        for (int i = 0; i < current_state->box_count; i++)
        {
            if (current_state->boxes[i].id == err_box_id)
            {
                current_state->boxes[i].id = NO_CLS;
                break;
            }
        }

        // 将多出来的那个目标点类别降级为 NO_CLS，并同步 goal_type_map。
        for (int i = 0; i < ctx->goal_count; i++)
        {
            if (ctx->goals[i].id == err_goal_id)
            {
                ctx->goals[i].id = NO_CLS;
                ctx->goal_type_map[ctx->goals[i].pos] = NO_CLS;
                break;
            }
        }
    }
    return true;
}
// 引擎主入口：执行加权 IDA* 搜索。
static bool get_micro_path(uint8_t start_pos, uint8_t target_pos, const uint8_t *obstacles, WaypointPath *out_path)
{

    if (start_pos == target_pos)
    {
        out_path->points[0] = start_pos;
        out_path->length = 1;
        return true;
    }

    if (obstacles[target_pos])
    {
        return false;
    }

    // distance 保存起点到各格子的最短步数，UINT8_MAX 表示尚未到达。
    // best_turns[pos][dir] 保存以 dir 方向到达 pos 时的最少转折数。
    // previous_direction 保存最优状态的上一段方向，用于最终回溯。
    uint8_t distance[MAP_SIZE];
    uint8_t best_turns[MAP_SIZE][4];
    uint8_t previous_direction[MAP_SIZE][4];
    memset(distance, UINT8_MAX, sizeof(distance));
    memset(best_turns, UINT8_MAX, sizeof(best_turns));
    memset(previous_direction, UINT8_MAX, sizeof(previous_direction));

    // 复用输出缓冲区作为 BFS 队列，搜索结束后再覆盖为最终路径。
    uint8_t *queue = out_path->points;
    int head = 0;
    int tail = 0;
    uint8_t target_distance = UINT8_MAX;
    queue[tail++] = start_pos;
    distance[start_pos] = 0;

    while (head < tail)
    {
        uint8_t curr = queue[head++];
        uint8_t curr_distance = distance[curr];
        // 找到终点后，只需处理完终点前一层，不再扩展更远的格子。
        if (target_distance != UINT8_MAX && curr_distance >= target_distance)
            break;

        // 对当前格子只保留最小和次小进入代价，使每个离开方向可 O(1) 求值。
        uint8_t minimum_turns = UINT8_MAX;
        uint8_t second_minimum_turns = UINT8_MAX;
        uint8_t minimum_direction = UINT8_MAX;
        uint8_t second_minimum_direction = UINT8_MAX;

        if (curr != start_pos)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                uint8_t incoming = i;
                uint8_t turns = best_turns[curr][incoming];
                if (turns < minimum_turns)
                {
                    second_minimum_turns = minimum_turns;
                    second_minimum_direction = minimum_direction;
                    minimum_turns = turns;
                    minimum_direction = incoming;
                }
                else if (turns < second_minimum_turns)
                {
                    second_minimum_turns = turns;
                    second_minimum_direction = incoming;
                }
            }
        }

        for (uint8_t i = 0; i < 4; i++)
        {
            uint8_t outgoing = i;
            int next = neighbor_index(curr, outgoing);
            if (next < 0 || obstacles[next])
                continue;

            uint8_t next_distance = (uint8_t)(curr_distance + 1);
            if (distance[next] == UINT8_MAX)
            {
                distance[next] = next_distance;
                queue[tail++] = (uint8_t)next;
                if (next == target_pos)
                    target_distance = next_distance;
            }
            if (distance[next] != next_distance)
                continue;

            uint8_t candidate = 0;
            uint8_t predecessor = UINT8_MAX;
            if (curr != start_pos)
            {
                // 沿相同方向前进不增加转折，改变方向则增加一个转折。
                uint8_t straight_turns = best_turns[curr][outgoing];
                uint8_t changed_turns = UINT8_MAX;
                uint8_t changed_direction = UINT8_MAX;

                if (minimum_direction != UINT8_MAX && minimum_direction != outgoing)
                {
                    changed_turns = (uint8_t)(minimum_turns + 1);
                    changed_direction = minimum_direction;
                }
                else if (second_minimum_direction != UINT8_MAX)
                {
                    changed_turns = (uint8_t)(second_minimum_turns + 1);
                    changed_direction = second_minimum_direction;
                }

                candidate = straight_turns;
                predecessor = outgoing;
                if (changed_turns < candidate ||
                    (changed_turns == candidate && changed_direction < predecessor))
                {
                    candidate = changed_turns;
                    predecessor = changed_direction;
                }
            }

            if (candidate < best_turns[next][outgoing])
            {
                best_turns[next][outgoing] = candidate;
                previous_direction[next][outgoing] = predecessor;
            }
        }
    }

    if (target_distance == UINT8_MAX)
        return false;

    // 终点可能从四个方向到达，选择转折数最少的状态。
    uint8_t final_direction = UINT8_MAX;
    uint8_t minimum_turns = UINT8_MAX;
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t direction = i;
        if (best_turns[target_pos][direction] < minimum_turns)
        {
            minimum_turns = best_turns[target_pos][direction];
            final_direction = direction;
        }
    }
    if (final_direction == UINT8_MAX)
        return false;

    // 从终点按到达方向反向寻找前驱，直接写回输出路径。
    out_path->length = (uint16_t)distance[target_pos] + 1U;
    uint8_t curr = target_pos;
    uint8_t direction = final_direction;
    for (int index = (int)out_path->length - 1; index >= 0; index--)
    {
        out_path->points[index] = curr;
        if (curr == start_pos)
            break;

        uint8_t incoming = previous_direction[curr][direction];
        curr = (uint8_t)neighbor_index(curr, direction ^ 1U);
        direction = incoming;
    }
    return true;
}

static bool pass(uint8_t startpoint, uint8_t endpoint, float error, const uint8_t *obstacles)
{
    static const float plus_xy[4][2] = {
        {0.1f, -0.1f}, {0.1f, 0.1f}, {-0.1f, 0.1f}, {-0.1f, -0.1f}};

    uint8_t start_x = startpoint % WIDTH;
    uint8_t start_y = startpoint / WIDTH;
    uint8_t end_x = endpoint % WIDTH;
    uint8_t end_y = endpoint / WIDTH;
    if (start_x == end_x)
    {
        int y_step = (start_y < end_y) ? 1 : -1;
        for (int y_run = start_y; y_run * y_step <= end_y * y_step; y_run += y_step)
        {
            if (obstacles[y_run * 16 + end_x])
            {
                return 0;
            }
        }
        return 1;
    }
    if (start_y == end_y)
    {
        int x_step = (start_x < end_x) ? 1 : -1;
        for (int x_run = start_x; x_run * x_step <= end_x * x_step; x_run += x_step)
        {
            if (obstacles[end_y * 16 + x_run])
            {
                return 0;
            }
        }
        return 1;
    }
    if (!IF_PASS)
    {
        return 0;
    }
    float start_xf = start_x * 0.2f + 0.1f;
    float start_yf = start_y * 0.2f + 0.1f;
    float end_xf = end_x * 0.2f + 0.1f;
    float end_yf = end_y * 0.2f + 0.1f;
    // 格子坐标：x*0.2+0.1, y*0.2+0.1
    float Line_k = (end_yf - start_yf) / (end_xf - start_xf);
    float Line_A_b = 0;
    float Line_B_b = 0;

    if ((start_x > end_x && start_y > end_y) || (start_x < end_x && start_y < end_y))
    {
        // 格子的左上点与右下点
        float Line_A_x = start_xf - 0.1f;
        float Line_A_y = start_yf + 0.1f;
        Line_A_b = Line_A_y - Line_k * Line_A_x;

        float Line_B_x = start_xf + 0.1f;
        float Line_B_y = start_yf - 0.1f;
        Line_B_b = Line_B_y - Line_k * Line_B_x;
    }
    else
    {
        float Line_A_x = start_xf + 0.1f;
        float Line_A_y = start_yf + 0.1f;
        Line_A_b = Line_A_y - Line_k * Line_A_x;

        float Line_B_x = start_xf - 0.1f;
        float Line_B_y = start_yf - 0.1f;
        Line_B_b = Line_B_y - Line_k * Line_B_x;
    }

    // 保证Line_A_b>Line_B_b
    if (Line_A_b < Line_B_b)
    {
        float temp = Line_A_b;
        Line_A_b = Line_B_b;
        Line_B_b = temp;
    }
    // 加入误差
    Line_A_b += error;
    Line_B_b -= error;

    int x_step = (start_x < end_x) ? 1 : -1;
    int y_step = (start_y < end_y) ? 1 : -1;
    for (int x_run = start_x; x_run * x_step <= end_x * x_step; x_run += x_step)
    {
        for (int y_run = start_y; y_run * y_step <= end_y * y_step; y_run += y_step)
        {
            bool in_the_way = false;
            float x_runf = x_run * 0.2f + 0.1f;
            float y_runf = y_run * 0.2f + 0.1f;
            for (uint8_t i = 0; i < 4; i++)
            {
                float x = x_runf + plus_xy[i][0];
                float y = y_runf + plus_xy[i][1];
                float y_line_max = Line_k * x + Line_A_b;
                float y_line_min = Line_k * x + Line_B_b;
                if (y < y_line_max && y > y_line_min)
                {
                    in_the_way = true;
                    break;
                }
            }
            if (in_the_way)
            {
                if (obstacles[y_run * 16 + x_run])
                {
                    return 0;
                }
            }
        }
    }
    int endpoint_x = endpoint % WIDTH;
    int endpoint_y = endpoint / WIDTH;
    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            int x = endpoint_x + dx;
            int y = endpoint_y + dy;
            if (valid_map_xy(x, y) && obstacles[y * WIDTH + x])
            {
                return 0;
            }
        }
    }

    return 1;
}
// 节点平滑
static void get_smooth_path(const WaypointPath *grid_path, const uint8_t *obstacles, WaypointPath *out_smooth_path)
{
    if (grid_path->length <= 2)
    {
        *out_smooth_path = *grid_path;
        return;
    }
    out_smooth_path->points[0] = grid_path->points[0];
    out_smooth_path->length = 1;
    int current_idx = 0;

    while (current_idx < grid_path->length - 1)
    {
        int furthest_visible = current_idx + 1;

        for (int next = grid_path->length - 1; next > current_idx; next--)
        {
            if (pass(grid_path->points[current_idx], grid_path->points[next], ERROR, obstacles))
            {
                furthest_visible = next;
                break;
            }
        }
        out_smooth_path->points[out_smooth_path->length++] = grid_path->points[furthest_visible];
        current_idx = furthest_visible;
    }
}

static bool get_final_path(WaypointPath *path)
{
    if (path->length == 0U)
    {
        return true;
    }

    uint8_t unique_points[MAP_SIZE];
    int unique_len = 0;
    unique_points[unique_len++] = path->points[0];
    for (int i = 1; i < path->length; i++)
    {

        if (path->points[i] != unique_points[unique_len - 1])
        {
            unique_points[unique_len++] = path->points[i];
        }
    }
    if (unique_len == 1)
    {
        path->length = 1U;
        path->points[0] = unique_points[0];
        return true;
    }

    uint8_t new_points[MAP_SIZE];
    int new_len = 0;
    new_points[new_len++] = unique_points[0];

    for (int i = 1; i < unique_len - 1; i++)
    {
        int p = unique_points[i - 1];
        int c = unique_points[i];
        int n = unique_points[i + 1];

        int px = p % WIDTH;
        int py = p / WIDTH;
        int cx = c % WIDTH;
        int cy = c / WIDTH;
        int nx = n % WIDTH;
        int ny = n / WIDTH;
        int dx1 = cx - px;
        int dy1 = cy - py;
        int dx2 = nx - cx;
        int dy2 = ny - cy;

        bool is_horizontal = (dy1 == 0 && dy2 == 0 && (dx1 * dx2 > 0));
        bool is_vertical = (dx1 == 0 && dx2 == 0 && (dy1 * dy2 > 0));
        if (!is_horizontal && !is_vertical)
        {
            new_points[new_len++] = c;
        }
    }
    new_points[new_len++] = unique_points[unique_len - 1];

    // 长直线按阈值均分；斜线和延时标记保持原样。
    int final_len = 0;
    unique_points[final_len++] = new_points[0];
    for (int i = 1; i < new_len; i++)
    {
        int start = new_points[i - 1];
        int end = new_points[i];

        if (start != 255 && end != 255)
        {
            int start_x = start % WIDTH;
            int start_y = start / WIDTH;
            int end_x = end % WIDTH;
            int end_y = end / WIDTH;
            int dx = end_x - start_x;
            int dy = end_y - start_y;
            bool is_straight = (dx == 0) != (dy == 0);

            if (is_straight)
            {
                int distance = dx != 0 ? (dx > 0 ? dx : -dx) : (dy > 0 ? dy : -dy);
                int split_count = distance / MAX_L;
                int segment_count = split_count + 1;

                for (int split = 1; split <= split_count; split++)
                {
                    if (final_len >= MAP_SIZE)
                    {
                        return false;
                    }

                    // 四舍五入到最近格点，使各段长度之差不超过一格。
                    int offset = (split * distance + segment_count / 2) / segment_count;
                    int x = start_x + (dx == 0 ? 0 : (dx > 0 ? offset : -offset));
                    int y = start_y + (dy == 0 ? 0 : (dy > 0 ? offset : -offset));
                    unique_points[final_len++] = (uint8_t)(y * WIDTH + x);
                }
            }
        }

        if (final_len >= MAP_SIZE)
        {
            return false;
        }
        unique_points[final_len++] = (uint8_t)end;
    }

    path->length = (uint16_t)final_len;
    for (int i = 0; i < final_len; i++)
    {
        path->points[i] = unique_points[i];
    }
    return true;
}

bool generate_path(SokobanContext *ctx, WaypointPath *out_full_path)
{
    if (ctx == NULL || out_full_path == NULL)
        return false;

    State sim_state = ctx->initial_state;
    uint8_t sim_walls[MAP_SIZE];
    memcpy(sim_walls, ctx->initial_walls, MAP_SIZE);
    out_full_path->length = 0;
    uint8_t obstacles[MAP_SIZE];
    WaypointPath micro_path;
    WaypointPath smooth_path;
    for (int i = 0; i < ctx->solution_actions_len; i++)
    {
        MacroAction act = ctx->solution_actions[i];

        memset(obstacles, 0, sizeof(obstacles));

        for (int k = 0; k < MAP_SIZE; k++)
        {
            if (sim_walls[k])
                obstacles[k] = 1;
        }

        for (int k = 0; k < sim_state.box_count; k++)
        {
            obstacles[sim_state.boxes[k].pos] = 1;
        }

        for (int k = 0; k < sim_state.bomb_count; k++)
        {
            obstacles[sim_state.bombs[k]] = 1;
        }

        if (!get_micro_path(sim_state.car_pos, act.move_to, obstacles, &micro_path))
        {
            out_full_path->length = 0;
            return false;
        }
        get_smooth_path(&micro_path, obstacles, &smooth_path);
        uint16_t required_points = smooth_path.length + 1U + (act.is_explode ? 1U : 0U);
        if (required_points > MAP_SIZE - out_full_path->length)
        {
            out_full_path->length = 0;
            return false;
        }
        out_full_path->length += smooth_path.length;
        for (int p = 0; p < smooth_path.length; p++)
        {
            out_full_path->points[out_full_path->length - smooth_path.length + p] = smooth_path.points[p];
        }
        out_full_path->points[out_full_path->length++] = act.push_to;
        if (act.is_explode)
        {
            out_full_path->points[out_full_path->length++] = 255; // 延时特殊标记符号
        }
        // ===========================================

        // ������һ֡��ͼ
        int push_dir = (int)act.push_to - (int)act.move_to;
        int next_pos_index = (int)act.push_to + push_dir;
        if (next_pos_index < 0 || next_pos_index >= MAP_SIZE)
        {
            out_full_path->length = 0;
            return false;
        }
        uint8_t next_pos = (uint8_t)next_pos_index;

        int entity_idx = -1;
        bool is_bomb_entity = false;
        for (int k = 0; k < sim_state.bomb_count; k++)
        {
            if (sim_state.bombs[k] == act.push_to)
            {
                is_bomb_entity = true;
                entity_idx = k;
                break;
            }
        }
        if (!is_bomb_entity)
        {
            for (int k = 0; k < sim_state.box_count; k++)
            {
                if (sim_state.boxes[k].pos == act.push_to)
                {
                    entity_idx = k;
                    break;
                }
            }
        }
        if (entity_idx < 0)
        {
            out_full_path->length = 0;
            return false;
        }
        if (act.is_explode)
        {
            // 移出炸弹
            sim_state.bombs[entity_idx] = sim_state.bombs[--sim_state.bomb_count];
            // 爆破 3x3 墙体
            int exp_count = ctx->explosion_area_count[next_pos];
            for (int e = 0; e < exp_count; e++)
            {
                sim_walls[ctx->explosion_areas[next_pos][e]] = 0;
            }
        }
        else if (act.is_consume)
        {
            // 移出箱子
            sim_state.boxes[entity_idx] = sim_state.boxes[--sim_state.box_count];
        }
        else
        {

            if (is_bomb_entity)
            {
                sim_state.bombs[entity_idx] = next_pos;
            }
            else
            {
                sim_state.boxes[entity_idx].pos = next_pos;
            }
        }
        sim_state.car_pos = act.push_to;
    }
    if (!get_final_path(out_full_path)) // 对整条路径进行最终的优化处理
    {
        out_full_path->length = 0;
        return false;
    }
    // 更新初始状态为最终状�?
    ctx->initial_state = sim_state;
    memcpy(ctx->initial_walls, sim_walls, MAP_SIZE);
    return true;
}

// 此函数根据tnt_loc坐标炸掉以坐标为中心3*3的墙壁，边界墙炸不到
static void boom_wall(uint8_t *map, uint8_t tnt_loc)
{

    uint8_t x = tnt_loc % 16;
    uint8_t y = tnt_loc / 16;
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            if (x + i > 0 && x + i < 15 && y + j > 0 && y + j < 11)
            {
                if (map[(y + j) * 16 + (x + i)] == 1)
                {
                    map[(y + j) * 16 + (x + i)] = 0;
                }
            }
        }
    }
}

// 在侦查函数结束后存储目标与箱子的位置，以及其id，length是长度
EntityData mapin_goals[MAX_GOALS];
uint8_t length_mapin_goals = 0;
EntityData mapin_boxes[MAX_BOXES];
uint8_t length_mapin_boxes = 0;
void goal_box_giveRelation(SokobanContext *ctx)
{
    uint8_t mapin_goals_count = 0;
    for (int i = 0; i < ctx->goal_count; i++)
    {
        if (ctx->initial_state.active_goals_mask & (1U << i))
        {
            mapin_goals[mapin_goals_count] = ctx->goals[i];
            mapin_goals_count++;
        }
    }
    length_mapin_goals = mapin_goals_count;
    length_mapin_boxes = ctx->initial_state.box_count;
    for (int i = 0; i < length_mapin_boxes; i++)
    {
        mapin_boxes[i] = ctx->initial_state.boxes[i];
    }
}

void clear_relation_in()
{
    for (int i = 0; i < MAX_GOALS; i++)
    {
        mapin_boxes->id = 0;
        mapin_goals->id = 0;
        mapin_boxes->pos = 0;
        mapin_goals->pos = 0;
    }
    length_mapin_boxes = 0;
    length_mapin_goals = 0;
}
// 任务状态定义：0=无分类关卡，1=有分类侦查阶段
uint8_t run_type_state = 0;
// 上下左右，0123
static void map_inmove_step(uint8_t *map, uint8_t direction, uint8_t car_loc)
{
    uint8_t next_step = 0;
    // 根据方向计算小车单步要走到的点位
    switch (direction)
    {
    case 0: // 向上
        next_step = car_loc - 16;
        break;
    case 1: // 向下
        next_step = car_loc + 16;
        break;
    case 2: // 向左
        next_step = car_loc - 1;
        break;
    case 3: // 向右
        next_step = car_loc + 1;
        break;
    default:
        return; // 非法方向直接退出，不处理
    }

    // -------------------------- 碰到箱子2/箱子+目的地6 --------------------------
    if (map[next_step] == 2 || map[next_step] == 6)
    {
        // 先处理原位置：2号清空为0，6号恢复为3号纯目的地
        if (map[next_step] == 2)
        {
            map[next_step] = 0;
        }
        else
        {
            map[next_step] = 3;
        }

        // 计算箱子被推动后的落点
        uint8_t push_target = 0;
        switch (direction)
        {
        case 0:
            push_target = next_step - 16;
            break; // 向上推箱子
        case 1:
            push_target = next_step + 16;
            break; // 向下推箱子
        case 2:
            push_target = next_step - 1;
            break; // 向左推箱子
        case 3:
            push_target = next_step + 1;
            break; // 向右推箱子
        }

        // 更新箱子落点的地图状态
        if (map[push_target] == 0)
        {
            map[push_target] = 2;
            if (run_type_state == 1)
            {
                bool found_box = false;
                for (int j = 0; j < length_mapin_boxes; j++)
                {
                    // 普通推动只按位置关联；未分类箱子也必须跟随更新位置。
                    if (mapin_boxes[j].pos == next_step)
                    {
                        mapin_boxes[j].pos = push_target;
                        found_box = true;
                        break;
                    }
                }
                if (!found_box)
                {
                    return_to_start_zone();
                    return;
                }
            }
        }
        else if (map[push_target] == 3)
        {
            if (run_type_state == 0)
            {
                map[push_target] = 0;
            }
            else if (run_type_state == 1)
            {
                // 目标落点必须同时找到对应箱子和目标点；两者均须已分类。
                int box_index = -1;
                for (int j = 0; j < length_mapin_boxes; j++)
                {
                    if (mapin_boxes[j].pos == next_step)
                    {
                        box_index = j;
                        break;
                    }
                }

                int goal_index = -1;
                for (int j = 0; j < length_mapin_goals; j++)
                {
                    if (mapin_goals[j].pos == push_target)
                    {
                        goal_index = j;
                        break;
                    }
                }

                if (box_index < 0 || goal_index < 0 ||
                    mapin_boxes[box_index].id == UNKNOWN ||
                    mapin_goals[goal_index].id == UNKNOWN)
                {
                    return_to_start_zone();
                    return;
                }

                mapin_boxes[box_index].pos = push_target;
                if (mapin_boxes[box_index].id == mapin_goals[goal_index].id)
                {
                    // 消除后物理删除两条关系，避免留下 UNKNOWN 箱子占位。
                    for (int j = box_index; j + 1 < length_mapin_boxes; j++)
                    {
                        mapin_boxes[j] = mapin_boxes[j + 1];
                    }
                    length_mapin_boxes--;
                    for (int j = goal_index; j + 1 < length_mapin_goals; j++)
                    {
                        mapin_goals[j] = mapin_goals[j + 1];
                    }
                    length_mapin_goals--;
                    map[push_target] = 0;
                }
                else
                {
                    map[push_target] = 6;
                }
            }
        }

        // 小车停在刚才碰到箱子的点位next_step
        if (map[next_step] == 0)
        {
            map[next_step] = 5;
        }
        else if (map[next_step] == 3)
        {
            map[next_step] = 8;
        }
    }
    // -------------------------- 碰到炸弹4/炸弹+目的地7 --------------------------
    else if (map[next_step] == 4 || map[next_step] == 7)
    {
        // 先处理原位置：4号清空为0，7号恢复为3号纯目的地
        if (map[next_step] == 4)
        {
            map[next_step] = 0;
        }
        else
        {
            map[next_step] = 3;
        }

        // 计算炸弹被推动后的落点
        uint8_t push_target = 0;
        switch (direction)
        {
        case 0:
            push_target = next_step - 16;
            break;
        case 1:
            push_target = next_step + 16;
            break;
        case 2:
            push_target = next_step - 1;
            break;
        case 3:
            push_target = next_step + 1;
            break;
        }

        // 更新炸弹落点的地图状态
        if (map[push_target] == 0)
        {
            map[push_target] = 4;
        }
        else if (map[push_target] == 3)
        {
            map[push_target] = 7;
        }
        else if (map[push_target] == 1)
        {
            boom_wall(map, push_target);
        }

        // 小车停在刚才碰到炸弹的点位next_step
        if (map[next_step] == 0)
        {
            map[next_step] = 5;
        }
        else if (map[next_step] == 3)
        {
            map[next_step] = 8;
        }
    }
    // -------------------------- 路径上没有实体，小车正常走一步 --------------------------
    else
    {
        if (map[next_step] == 0)
        {
            map[next_step] = 5;
        }
        else if (map[next_step] == 3)
        {
            map[next_step] = 8;
        }
    }

    // 最后统一清空小车原位置的状态
    if (map[car_loc] == 5)
    {
        map[car_loc] = 0;
    }
    else if (map[car_loc] == 8)
    {
        map[car_loc] = 3;
    }
}

// 6	箱子+目的地	箱子推入目的地，ID不匹配时生成的混合体，小车可把箱子重新推出来
// 7	炸弹+目的地	炸弹被推到目的地生成的混合体，保留炸弹爆炸属性
// 8	小车+目的地	小车停靠在目的地上生成的混合体，小车可以正常驶离该点位
// 此函数用来更新地图，并判断car_to这个点是否需要获取视觉坐标
// 每次到达某个节点时用car_to,而car_to_to表示下一个节点，用来判断car_to这个点是否需要获取视觉坐标,
// 仅当从car_to到car_to_to的路径两侧有箱子，炸弹时才需要获取视觉坐标
uint8_t map_check_ifgetVisionLoc(uint8_t *map, uint8_t car_to, uint8_t car_to_to)
{
    uint8_t car_from = 0;
    uint8_t curr_car_loc;
    // 扫描获取小车当前位置，兼容5和8两种状态
    for (uint8_t i = 0; i < MAP_SIZE; i++)
    {
        if (map[i] == 5 || map[i] == 8)
        {
            car_from = i;
            break;
        }
    }
    curr_car_loc = car_from;

    // -------------------------- 水平方向同一路径直接逐格走到底 --------------------------
    if ((car_from / 16) == (car_to / 16) && car_from != car_to)
    {
        if (car_from < car_to)
        {
            for (uint8_t step_cnt = 0; step_cnt < (car_to - car_from); step_cnt++)
            {
                map_inmove_step(map, 3, curr_car_loc); // 向右走一步
                curr_car_loc += 1;
            }
        }
        else
        {
            for (uint8_t step_cnt = 0; step_cnt < (car_from - car_to); step_cnt++)
            {
                map_inmove_step(map, 2, curr_car_loc); // 向左走一步
                curr_car_loc -= 1;
            }
        }
    }
    // -------------------------- 垂直方向同一路径直接逐格走到底 --------------------------
    else if ((car_from % 16) == (car_to % 16) && car_from != car_to)
    {
        if (car_from < car_to)
        {
            for (uint8_t step_cnt = 0; step_cnt < ((car_to - car_from) / 16); step_cnt++)
            {
                map_inmove_step(map, 1, curr_car_loc); // 向下走一步
                curr_car_loc += 16;
            }
        }
        else
        {
            for (uint8_t step_cnt = 0; step_cnt < ((car_from - car_to) / 16); step_cnt++)
            {
                map_inmove_step(map, 0, curr_car_loc); // 向上走一步
                curr_car_loc -= 16;
            }
        }
    }
    // -------------------------- 非水平/非垂直的位移场景，直接沿用原有跳转逻辑 --------------------------
    else
    {
        // 清空小车原位置
        if (map[car_from] == 5)
        {
            map[car_from] = 0;
        }
        else if (map[car_from] == 8)
        {
            map[car_from] = 3;
        }
        // 写入小车目标位置
        if (map[car_to] == 0)
        {
            map[car_to] = 5;
        }
        else if (map[car_to] == 3)
        {
            map[car_to] = 8;
        }
    }

    // -------------------------- 视觉定位触发判断 完全原封不动保留 --------------------------
    if ((car_to_to / 16) == (car_to / 16))
    {
        if (car_to < car_to_to)
        {
            for (uint8_t i = car_to; i <= car_to_to + 1; i++)
            {
                if (i - 16 > 0)
                {
                    uint8_t type = map[i - 16];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                if (i + 16 < 192)
                {
                    uint8_t type = map[i + 16];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                uint8_t type = map[i];
                if (type == 2 || type == 4 || type == 6 || type == 7)
                    return 1;
            }
        }
        else
        {
            for (uint8_t i = car_to; i >= car_to_to - 1; i--)
            {
                if (i - 16 > 0)
                {
                    uint8_t type = map[i - 16];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                if (i + 16 < 192)
                {
                    uint8_t type = map[i + 16];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                uint8_t type = map[i];
                if (type == 2 || type == 4 || type == 6 || type == 7)
                    return 1;
            }
        }
    }
    else if ((car_to_to % 16) == (car_to % 16))
    {
        if (car_to < car_to_to)
        {
            for (uint8_t i = car_to; i <= car_to_to + 16; i += 16)
            {
                if (i - 1 > 0)
                {
                    uint8_t type = map[i - 1];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                if (i + 1 < 192)
                {
                    uint8_t type = map[i + 1];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                uint8_t type = map[i];
                if (type == 2 || type == 4 || type == 6 || type == 7)
                    return 1;
            }
        }
        else
        {
            for (uint8_t i = car_to; i >= car_to_to - 16; i -= 16)
            {
                if (i - 1 > 0)
                {
                    uint8_t type = map[i - 1];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                if (i + 1 < 192)
                {
                    uint8_t type = map[i + 1];
                    if (type == 2 || type == 4 || type == 6 || type == 7)
                        return 1;
                }
                uint8_t type = map[i];
                if (type == 2 || type == 4 || type == 6 || type == 7)
                    return 1;
            }
        }
    }

    return 0;
}

// /**
//  * @brief 实时障碍物检查函数（供运控避�?/侧向补偿调用�?
//  * @param ctx 引擎上下文指�?
//  * @param grid_index 待检查的网格索引 (0~191)
//  * @return uint8_t 1: 有障�?(墙、箱子、炸�?)  0: 空地或纯目标�?
//  */
// uint8_t check_obstacle(SokobanContext *ctx, uint8_t grid_index)
// {
//     // 0. 越界保护
//     if (grid_index >= MAP_SIZE)
//         return 1;

//     // 1. WALL_NORMAL、WALL_SEPARATOR、WALL_DEADLOCK 均视为障碍。
//     // 说明：engine_init �? generate_path 都会实时更新 initial_walls�?
//     // 墙被炸掉后值为 0，所以这里只�? >= 1 就是墙�?
//     if (ctx->initial_walls[grid_index] >= 1)

//         return 1;

//     // 2. 检查现存的动态箱�?
//     for (uint8_t i = 0; i < ctx->initial_state.box_count; i++)
//     {
//         if (ctx->initial_state.boxes[i].pos == grid_index)
//             return 1;
//     }

//     // 3. 检查现存的动态炸�?
//     for (uint8_t i = 0; i < ctx->initial_state.bomb_count; i++)
//     {
//         if (ctx->initial_state.bombs[i] == grid_index)
//             return 1;
//     }

//     return 0;
// }
