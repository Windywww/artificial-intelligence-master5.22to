#include "sokoban_engine.h"
#include "sokoban_lut.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
#include "myUart.h"
#include "move_control.h"
#include "WIFI2SPI.h"

#define SOKOBAN_EMBEDDED 1

#define DEBUG_RECON 0
#define MAX_ID 12 // id 可能的取值个数
#ifndef MAX_ALLOWABLE_NODES
#define MAX_ALLOWABLE_NODES 700000 // 限制搜索节点总数
#endif
#ifndef SOKOBAN_CURRENT_WEIGHT
#define SOKOBAN_CURRENT_WEIGHT 3.0f
#endif
#ifndef SOKOBAN_MIN_WEIGHT
#define SOKOBAN_MIN_WEIGHT 3.0f
#endif
#ifndef MOVE_PENALTY
#define MOVE_PENALTY 10
#endif
#define UNKNOWN 11
#define ERROR 0.05f
// 节点上限必须作为独立状态逐层传播，不能与无解或下一阈值混淆。
#define RES_NODE_LIMIT -2.0f
// 保持置换表总容量不变，每个集合容纳四个相同低位索引的状态。
#define HASH_WAYS 4
#define HASH_SET_COUNT (HASH_TABLE_SIZE / HASH_WAYS)
#define HASH_SET_MASK (HASH_SET_COUNT - 1)

int angle = 0;

typedef struct
{
    State next_state;
    MacroAction action;
    uint16_t next_g;
    uint16_t next_h;
} ChildNode;

typedef struct
{
    float f;
    uint16_t g;
    uint16_t h;
} SearchRes;

// 哈希表结构与全局内存分配
static uint8_t current_hash_version = 0;

// 分离存储避免 HashEntry 的5字节对齐填充：8 MiB + 2 MiB + 1 MiB。
__attribute__((section(".bss.sdram"))) static uint64_t transposition_signatures[HASH_TABLE_SIZE];
__attribute__((section(".bss.sdram"))) static uint16_t transposition_g_scores[HASH_TABLE_SIZE];
__attribute__((section(".bss.sdram"))) static uint8_t transposition_versions[HASH_TABLE_SIZE];

// ChildNode 为 88 字节，池总计 1,196,800 字节（约 1.141 MiB）。
//__attribute__((section(".ocram_data")))
__attribute__((section(".bss.sdram"))) static ChildNode all_children_pool[MAX_STEPS][MAX_BRANCHES];

// 关键内部接口。
static void get_smooth_path(SokobanContext *ctx, const WaypointPath *grid_path, const uint8_t *obstacles, WaypointPath *out_smooth_path);
static uint8_t is_deadlock(SokobanContext *ctx, uint8_t idx, State *state, bool is_bomb, const uint8_t *walls);
// 按当前墙布局和剩余炸弹数，构建各目标点的反向推动距离表。
static void get_maze_distances(SokobanContext *ctx, const uint8_t *current_walls, uint8_t bomb_count);

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

static void precalc_explosion_masks(SokobanContext *ctx)
{

    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            uint8_t center_idx = y * WIDTH + x;
            uint8_t valid_target_count = 0;

            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT)
                        continue;
                    uint8_t n_idx = ny * WIDTH + nx;

                    // 地图最外圈上的墙是不可破坏边界墙，不加入爆炸范围。
                    if (!ctx->boundary_walls[n_idx])
                    {
                        ctx->explosion_areas[center_idx][valid_target_count] = n_idx;
                        valid_target_count++;
                    }
                }
            }

            ctx->explosion_area_count[center_idx] = valid_target_count;
        }
    }
}
static void hash_table_clear(void)
{
    current_hash_version++;

    if (current_hash_version == 0)
    {
        memset(transposition_versions, 0, sizeof(transposition_versions));
        current_hash_version = 1;
    }
}

// ==========================================
static uint64_t zobrist_car[MAP_SIZE];
static uint64_t zobrist_box[MAX_ID][MAP_SIZE];
static uint64_t zobrist_bomb[MAP_SIZE];
static uint64_t zobrist_wall[MAP_SIZE];
static uint64_t zobrist_goal_mask[MAX_GOALS];

static uint64_t xorshift64(uint64_t *state)
{
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    return *state;
}

static void init_zobrist()
{
    uint64_t seed = 0x123456789ABCDEF0ULL;
    for (int i = 0; i < MAP_SIZE; i++)
    {
        zobrist_car[i] = xorshift64(&seed);
        zobrist_bomb[i] = xorshift64(&seed);
        zobrist_wall[i] = xorshift64(&seed);
        for (int j = 0; j < MAX_ID; j++)
            zobrist_box[j][i] = xorshift64(&seed);
    }
    for (int i = 0; i < MAX_GOALS; i++)
        zobrist_goal_mask[i] = xorshift64(&seed);
}

// 对小车、箱子、炸弹、墙和未完成目标生成完整 Zobrist 状态签名。
static uint64_t compute_initial_base_hash(const State *state, const uint8_t *walls)
{
    uint64_t h = 0;
    for (int i = 0; i < state->box_count; i++)
        h ^= zobrist_box[state->boxes[i].id][state->boxes[i].pos];
    for (int i = 0; i < state->bomb_count; i++)
        h ^= zobrist_bomb[state->bombs[i]];
    for (int i = 0; i < MAP_SIZE; i++)
        if (walls[i])
            h ^= zobrist_wall[i];
    for (int i = 0; i < MAX_GOALS; i++)
        if (state->active_goals_mask & (1U << i))
            h ^= zobrist_goal_mask[i];
    h ^= zobrist_car[state->car_pos];
    return h;
}

// 查询四路组相联置换表；已存在不劣状态时返回 true，否则写入当前 g。
static bool hash_table_insert_or_check(const State *state, int g_score, int tolerance)
{
    uint64_t sig = state->base_hash;
    uint32_t set_start = (uint32_t)(sig & HASH_SET_MASK) * HASH_WAYS;
    uint32_t replacement = set_start;
    uint16_t worst_g = 0;
    // 优先使用空槽；集合已满时替换 g 值最大的条目。
    for (uint32_t way = 0; way < HASH_WAYS; way++)
    {
        uint32_t idx = set_start + way;
        if (transposition_versions[idx] != current_hash_version)
        {
            replacement = idx;
            worst_g = UINT16_MAX;
            break;
        }

        if (transposition_signatures[idx] == sig)
        {
            if (transposition_g_scores[idx] <= g_score + tolerance)
                return true;

            transposition_g_scores[idx] = (uint16_t)g_score;
            return false;
        }

        if (transposition_g_scores[idx] >= worst_g)
        {
            worst_g = transposition_g_scores[idx];
            replacement = idx;
        }
    }

    transposition_versions[replacement] = current_hash_version;
    transposition_signatures[replacement] = sig;
    transposition_g_scores[replacement] = (uint16_t)g_score;
    return false;
}

// 引爆动作保留基础推动代价，此处只返回墙类型对应的额外代价。
static inline uint16_t wall_action_penalty(uint8_t wall_type)
{
    if (wall_type == WALL_NORMAL)
        return BOMB_PENALTY;
    if (wall_type == WALL_SEPARATOR)
        return BOMB_PENALTY / 2;
    return 0;
}

// 距离启发式中的普通墙、隔离墙、死锁墙代价依次为 20、0、0。
static inline uint16_t wall_heuristic_penalty(uint8_t wall_type)
{
    if (wall_type == WALL_NORMAL)
        return VIRTUAL_WALL_COST;
    return 0;
}

static void engine_init(SokobanContext *ctx, const uint8_t *raw_map)
{

    init_zobrist();

    ctx->goal_count = 0;
    ctx->map_valid = true;

    ctx->dir_offsets[0] = -WIDTH; // UP
    ctx->dir_offsets[1] = WIDTH;  // DOWN
    ctx->dir_offsets[2] = -1;     // LEFT
    ctx->dir_offsets[3] = 1;      // RIGHT

    memset(ctx->boundary_walls, 0, sizeof(ctx->boundary_walls));
    memset(ctx->goal_type_map, 255, sizeof(ctx->goal_type_map));
    memset(ctx->goal_mask_map, -1, sizeof(ctx->goal_mask_map));
    State *init_state = &ctx->initial_state;
    init_state->box_count = 0;
    init_state->bomb_count = 0;
    memset(ctx->initial_walls, 0, sizeof(ctx->initial_walls));
    ctx->cache_valid = false;
    ctx->current_weight = SOKOBAN_CURRENT_WEIGHT;
    ctx->min_weight = SOKOBAN_MIN_WEIGHT;

    // 4. ������ͼ
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            int idx = y * WIDTH + x;
            uint8_t val = raw_map[idx];
            if (val == 1)
            {
                ctx->initial_walls[idx] = WALL_NORMAL;
                if (x == 0 || x == WIDTH - 1 || y == 0 || y == HEIGHT - 1)
                {
                    ctx->boundary_walls[idx] = 1;
                }
            }
            else if (val == 2)
            { // 箱子 (BOX)
                if (init_state->box_count < MAX_BOXES)
                {
                    init_state->boxes[init_state->box_count].pos = idx;
                    init_state->boxes[init_state->box_count].id = UNKNOWN;
                    init_state->box_count++;
                }
                else
                {
                    ctx->map_valid = false;
                }
            }
            else if (val == 3)
            {
                if (ctx->goal_count < MAX_GOALS)
                {
                    ctx->goal_type_map[idx] = UNKNOWN;
                    ctx->goals[ctx->goal_count].pos = idx;
                    ctx->goals[ctx->goal_count].id = UNKNOWN;
                    ctx->goal_mask_map[idx] = ctx->goal_count;
                    ctx->goal_count++;
                }
                else
                {
                    ctx->map_valid = false;
                }
            }
            else if (val == 4)
            { // ը�� (BOMB)
                if (init_state->bomb_count < MAX_BOMBS)
                {
                    init_state->bombs[init_state->bomb_count++] = idx;
                }
                else
                {
                }
            }
            else if (val == 5)
            { // С�� (CAR)
                init_state->car_pos = idx;
            }
        }
    }

    if (ctx->goal_count > 0)
    {
        init_state->active_goals_mask = (UINT32_C(1) << ctx->goal_count) - 1;
    }
    else
    {
        init_state->active_goals_mask = 0;
    }
    precalc_explosion_masks(ctx);

    if (init_state->bomb_count > 0)
    {
        uint8_t region_map[MAP_SIZE];
        memset(region_map, 0, sizeof(region_map));
        uint8_t current_region = 1;

        uint8_t queue[MAP_SIZE];
        int head = 0, tail = 0;

        for (int i = 0; i < MAP_SIZE; i++)
        {

            if (ctx->initial_walls[i] == 0 && region_map[i] == 0)
            {
                head = 0;
                tail = 0;
                queue[tail++] = i;
                region_map[i] = current_region;

                while (head < tail)
                {
                    uint8_t curr = queue[head++];
                    for (int d = 0; d < 4; d++)
                    {
                        int n_idx = neighbor_index(curr, d);
                        if (n_idx >= 0 && ctx->initial_walls[n_idx] == 0 && region_map[n_idx] == 0)
                        {
                            region_map[n_idx] = current_region;
                            queue[tail++] = n_idx;
                        }
                    }
                }
                current_region++;
            }
        }

        for (int i = 0; i < MAP_SIZE; i++)
        {
            if (ctx->initial_walls[i] == WALL_NORMAL && !ctx->boundary_walls[i])
            {
                uint8_t flag = 0;
                int wall_x = i % WIDTH;
                int wall_y = i / WIDTH;
                bool done = false;
                for (int dy = -1; dy <= 1 && !done; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dx == 0 && dy == 0)
                            continue;
                        int nx = wall_x + dx;
                        int ny = wall_y + dy;
                        if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT)
                        {
                            int n_idx = ny * WIDTH + nx;
                            uint8_t r = region_map[n_idx];
                            if (r > 0)
                            {
                                if (flag == 0)
                                {
                                    flag = r;
                                }
                                else if (flag != r)
                                {
                                    ctx->initial_walls[i] = WALL_SEPARATOR;
                                    done = true;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        for (uint8_t i = 0; i < init_state->box_count; i++)
        {
            uint8_t box_idx = init_state->boxes[i].pos;
            if (is_deadlock(ctx, box_idx, init_state, false, ctx->initial_walls))
            {
                for (uint8_t j = 0; j < ctx->explosion_area_count[box_idx]; j++)
                {
                    uint8_t n_idx = ctx->explosion_areas[box_idx][j];

                    if (ctx->initial_walls[n_idx] != WALL_NONE && !ctx->boundary_walls[n_idx])
                    {
                        ctx->initial_walls[n_idx] = WALL_DEADLOCK;
                    }
                }
            }
        }
    }
    get_maze_distances(ctx, ctx->initial_walls, init_state->bomb_count);
}

// ==========================================

// 用于 Dijkstra 的最小堆节点
typedef struct
{
    uint16_t dist;
    uint8_t pos;
} HeapNode;

typedef struct
{
    HeapNode nodes[MAP_SIZE * 4];
    uint16_t size;
} MinHeap;
static void heap_push(MinHeap *h, uint16_t dist, uint8_t pos)
{
    int i = h->size++;
    while (i > 0)
    {
        int p = (i - 1) / 2;
        if (h->nodes[p].dist <= dist)
            break;
        h->nodes[i] = h->nodes[p];
        i = p;
    }
    h->nodes[i].dist = dist;
    h->nodes[i].pos = pos;
}
static HeapNode heap_pop(MinHeap *h)
{
    HeapNode ret = h->nodes[0];
    HeapNode last = h->nodes[--h->size];
    int i = 0;
    while (i * 2 + 1 < h->size)
    {
        int child = i * 2 + 1;
        if (child + 1 < h->size && h->nodes[child + 1].dist < h->nodes[child].dist)
        {
            child++;
        }
        if (last.dist <= h->nodes[child].dist)
            break;
        h->nodes[i] = h->nodes[child];
        i = child;
    }
    h->nodes[i] = last;
    return ret;
}

// 使用反向 Dijkstra 计算“箱子从每格推到各目标”的下界距离。
// 缓存键包含墙布局和剩余炸弹数；无炸弹时不可穿墙，有炸弹时按墙类型计启发式代价。
static void get_maze_distances(SokobanContext *ctx, const uint8_t *current_walls, uint8_t bomb_count)
{
    if (ctx->cache_valid && ctx->cached_bomb_count == bomb_count &&
        memcmp(ctx->cached_walls, current_walls, MAP_SIZE) == 0)
    {
        return;
    }

    for (int g = 0; g < ctx->goal_count; g++)
    {
        for (int i = 0; i < MAP_SIZE; i++)
        {
            ctx->cached_dist_table[g][i] = INF_DIST;
        }
    }
    bool has_bombs = (bomb_count > 0);
    int dx_arr[4] = {0, 0, -1, 1};
    int dy_arr[4] = {-1, 1, 0, 0};

    for (int g = 0; g < ctx->goal_count; g++)
    {
        uint8_t goal_idx = ctx->goals[g].pos;
        ctx->cached_dist_table[g][goal_idx] = 0;
        MinHeap pq;
        pq.size = 0;
        heap_push(&pq, 0, goal_idx);
        while (pq.size > 0)
        {
            HeapNode curr_node = heap_pop(&pq);
            uint16_t dist = curr_node.dist;
            uint16_t curr = curr_node.pos;
            if (dist > ctx->cached_dist_table[g][curr])
                continue;
            int cx = curr % WIDTH;
            int cy = curr / WIDTH;
            for (int i = 0; i < 4; i++)
            {
                int dx = dx_arr[i];
                int dy = dy_arr[i];

                int px = cx - dx;
                int py = cy - dy;
                int ppx = px - dx;
                int ppy = py - dy;
                // Խ�����
                if (px < 0 || px >= WIDTH || py < 0 || py >= HEIGHT)
                    continue;
                if (ppx < 0 || ppx >= WIDTH || ppy < 0 || ppy >= HEIGHT)
                    continue;
                uint16_t p_idx = py * WIDTH + px;
                uint16_t pp_idx = ppy * WIDTH + ppx;
                if (ctx->boundary_walls[p_idx] || ctx->boundary_walls[pp_idx])
                    continue;
                uint16_t step_cost = 1;
                bool is_p_wall = current_walls[p_idx];
                bool is_pp_wall = current_walls[pp_idx];
                if (is_p_wall || is_pp_wall)
                {
                    if (!has_bombs)
                        continue;

                    if (is_p_wall)
                        step_cost += wall_heuristic_penalty(current_walls[p_idx]);
                    if (is_pp_wall)
                        step_cost += wall_heuristic_penalty(current_walls[pp_idx]);
                }
                uint16_t new_dist = dist + step_cost;
                if (new_dist < ctx->cached_dist_table[g][p_idx])
                {
                    ctx->cached_dist_table[g][p_idx] = new_dist;
                    heap_push(&pq, new_dist, p_idx);
                }
            }
        }
    }
    memcpy(ctx->cached_walls, current_walls, MAP_SIZE);
    ctx->cached_bomb_count = bomb_count;
    ctx->cache_valid = true;
}

static void build_car_dist_map(uint8_t start_pos, const uint8_t *obstacles, uint8_t *dist_map)
{

    for (uint8_t i = 0; i < MAP_SIZE; i++)
    {
        dist_map[i] = UINT8_MAX;
    }
    dist_map[start_pos] = 0;

    uint8_t queue[MAP_SIZE];
    uint8_t head = 0, tail = 0;
    queue[tail++] = start_pos;
    // BFS
    while (head < tail)
    {
        uint8_t curr = queue[head++];
        uint8_t current_d = dist_map[curr];
        for (uint8_t i = 0; i < 4; i++)
        {
            int n_idx = neighbor_index(curr, i);

            if (n_idx >= 0 && !obstacles[n_idx] && dist_map[n_idx] == UINT8_MAX)
            {
                dist_map[n_idx] = current_d + 1;
                queue[tail++] = n_idx;
            }
        }
    }
}

static void solve_assignment_km(int cost_matrix[MAX_BOXES][MAX_GOALS], int num_items,
                                int best_assignment[MAX_BOXES], int *best_cost)
{
    int u[MAX_BOXES + 1] = {0};
    int v[MAX_GOALS + 1] = {0};
    int p[MAX_GOALS + 1] = {0}; // 记录右侧顶点匹配到的左侧顶点
    int way[MAX_GOALS + 1] = {0};

    for (int i = 1; i <= num_items; i++)
    {
        p[0] = i;
        int j0 = 0;
        int minv[MAX_GOALS + 1];
        bool used[MAX_GOALS + 1] = {false};
        for (int k = 0; k <= num_items; k++)
            minv[k] = INF_DIST;

        do
        {
            used[j0] = true;
            int i0 = p[j0], delta = INF_DIST, j1 = 0;

            for (int j = 1; j <= num_items; j++)
            {
                if (!used[j])
                {

                    int cur_cost = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur_cost < minv[j])
                    {
                        minv[j] = cur_cost;
                        way[j] = j0;
                    }
                    if (minv[j] < delta)
                    {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (int j = 0; j <= num_items; j++)
            {
                if (used[j])
                {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else
                {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do
        {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    *best_cost = -v[0];

    for (int j = 1; j <= num_items; j++)
    {
        if (p[j] > 0)
        {
            best_assignment[p[j] - 1] = j - 1;
        }
    }
}

// 用 KM 最小权匹配汇总所有箱子到兼容目标的距离下界。
static int calc_heuristic(SokobanContext *ctx, State *state, const uint8_t *walls)
{
    // �Ѿ�ʤ��������Ϊ 0
    if (state->box_count == 0)
        return 0;

    get_maze_distances(ctx, walls, state->bomb_count);

    // 2. �������۾��� (Cost Matrix)
    int cost_matrix[MAX_BOXES][MAX_GOALS];
    uint8_t active_goal_indices[MAX_GOALS];
    uint8_t active_goal_count = 0;
    for (int g = 0; g < ctx->goal_count; g++)
    {
        if (state->active_goals_mask & (1U << g))
        {
            active_goal_indices[active_goal_count++] = (uint8_t)g;
        }
    }
    if (active_goal_count != state->box_count)
        return INF_DIST;

    for (int b = 0; b < state->box_count; b++)
    {
        for (int col = 0; col < active_goal_count; col++)
        {
            int g = active_goal_indices[col];
            if (state->boxes[b].id != ctx->goals[g].id)
            {
                cost_matrix[b][col] = INF_DIST;
            }
            else
            {
                cost_matrix[b][col] = ctx->cached_dist_table[g][state->boxes[b].pos];
            }
        }
    }

    int best_cost = INF_DIST;
    int best_assignment[MAX_BOXES] = {0};
    solve_assignment_km(cost_matrix, state->box_count, best_assignment, &best_cost);

    if (best_cost >= INF_DIST)
        return INF_DIST;
    int base_h = best_cost;

    int conflict_penalty = 0;

    for (int i = 0; i < state->box_count; i++)
    {
        for (int j = i + 1; j < state->box_count; j++)
        {
            uint8_t b1_idx = state->boxes[i].pos;
            uint8_t g1_idx = ctx->goals[active_goal_indices[best_assignment[i]]].pos;
            uint8_t b2_idx = state->boxes[j].pos;
            uint8_t g2_idx = ctx->goals[active_goal_indices[best_assignment[j]]].pos;

            int b1_x = b1_idx % WIDTH;
            int b1_y = b1_idx / WIDTH;
            int g1_x = g1_idx % WIDTH;
            int g1_y = g1_idx / WIDTH;

            int b2_x = b2_idx % WIDTH;
            int b2_y = b2_idx / WIDTH;
            int g2_x = g2_idx % WIDTH;
            int g2_y = g2_idx / WIDTH;

            if (b1_y == b2_y && g1_y == g2_y && b1_y == g1_y)
            {

                if ((b1_x - b2_x) * (g1_x - g2_x) < 0)
                {
                    conflict_penalty += 4;
                }
            }

            else if (b1_x == b2_x && g1_x == g2_x && b1_x == g1_x)
            {

                if ((b1_y - b2_y) * (g1_y - g2_y) < 0)
                {
                    conflict_penalty += 4;
                }
            }
        }
    }
    return base_h + conflict_penalty;
}

static inline uint8_t get_cell_state(SokobanContext *ctx, State *state, const uint8_t *walls, int idx)
{
    if (idx < 0 || idx >= MAP_SIZE)
        return 2;
    if (ctx->boundary_walls[idx])
        return 2;
    if (walls[idx])
        return 1;
    for (int i = 0; i < state->box_count; i++)
    {
        if (state->boxes[i].pos == idx)
            return 2;
    }
    for (int i = 0; i < state->bomb_count; i++)
    {
        if (state->bombs[i] == idx)
            return 3;
    }
    return 0;
}

static inline uint8_t get_relative_cell_state(SokobanContext *ctx, State *state,
                                              const uint8_t *walls, uint8_t center,
                                              int dx, int dy)
{
    int x = center % WIDTH + dx;
    int y = center / WIDTH + dy;
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
        return 2;
    return get_cell_state(ctx, state, walls, y * WIDTH + x);
}
// O(1) 死锁判定函数
static uint8_t is_deadlock(SokobanContext *ctx, uint8_t idx, State *state, bool is_bomb, const uint8_t *walls)
{
    uint16_t env = 0;
    env |= (get_relative_cell_state(ctx, state, walls, idx, -1, -1) << 0);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 0, -1) << 2);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 1, -1) << 4);
    env |= (get_relative_cell_state(ctx, state, walls, idx, -1, 0) << 6);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 1, 0) << 8);
    env |= (get_relative_cell_state(ctx, state, walls, idx, -1, 1) << 10);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 0, 1) << 12);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 1, 1) << 14);

    uint8_t required_tnt = is_bomb ? DEADLOCK_LUT_BOMB[env] : DEADLOCK_LUT_BOX[env];
    return required_tnt;
}

static inline float child_f_score(const ChildNode *child, float weight)
{
    return child->next_g + weight * child->next_h;
}

static inline bool child_after(const ChildNode *left, const ChildNode *right, float weight)
{
    float left_f = child_f_score(left, weight);
    float right_f = child_f_score(right, weight);
    return left_f > right_f || (left_f == right_f && left->next_g < right->next_g);
}

// 子节点只存 g/h，排序时即时计算 f，避免每个节点保存一个 float。
static inline void sort_children(const ChildNode *children, uint8_t count, uint8_t *indices, float weight)
{
    for (int i = 1; i < count; i++)
    {
        if (child_after(&children[indices[i - 1]], &children[indices[i]], weight))
        {
            // ��������
            uint8_t tmp_idx = indices[i];
            indices[i] = indices[i - 1];
            int j;
            for (j = i - 2; j >= 0 && child_after(&children[indices[j]], &children[tmp_idx], weight); j--)
                indices[j + 1] = indices[j];
            indices[j + 1] = tmp_idx;
        }
    }
}

// 在单个加权 IDA* 阈值内深搜，并返回成功、节点上限或下一最小 f。
static SearchRes dfs_ida(SokobanContext *ctx, State *current_state, const uint8_t *current_walls, uint16_t current_g, int current_h, float threshold, MacroAction *acts, uint8_t act_len)
{
    // 在计入当前节点前检查累计预算，保证计数永不越过上限。
    if (ctx->total_explored_nodes >= MAX_ALLOWABLE_NODES)
        return (SearchRes){RES_NODE_LIMIT, 0, 0};
    ctx->total_explored_nodes++;

    if (act_len >= MAX_STEPS)
    {
        return (SearchRes){RES_INF, 0, 0};
    }
    if (current_h == -1)
    {
        current_h = calc_heuristic(ctx, current_state, current_walls);
    }
    float f_score = current_g + ctx->current_weight * current_h;
    if (act_len > 150)
    {
        return (SearchRes){RES_INF, 0, 0};
    }
    if (f_score > threshold)
    {
        return (SearchRes){f_score, current_g, current_h};
    }

    if (current_state->box_count == 0)
    {

        ctx->solution_actions_len = act_len;
        for (int i = 0; i < act_len; i++)
        {
            ctx->solution_actions[i] = acts[i];
        }
        return (SearchRes){RES_SUCCESS, current_g, current_h};
    }

    SearchRes min_node_data = {RES_INF, 0, 0};
    ChildNode *children = all_children_pool[act_len];
    uint8_t child_count = 0;

    get_maze_distances(ctx, current_walls, current_state->bomb_count);

    uint8_t obstacles[MAP_SIZE];
    memcpy(obstacles, current_walls, MAP_SIZE);

    uint8_t all_items[MAX_BOXES + MAX_BOMBS];
    uint8_t total_items = 0;
    for (int i = 0; i < current_state->box_count; i++)
    {
        obstacles[current_state->boxes[i].pos] = 1;
        all_items[total_items++] = current_state->boxes[i].pos;
    }
    for (int i = 0; i < current_state->bomb_count; i++)
    {
        obstacles[current_state->bombs[i]] = 1;
        all_items[total_items++] = current_state->bombs[i];
    }

    uint8_t car_dist_map[MAP_SIZE];
    build_car_dist_map(current_state->car_pos, obstacles, car_dist_map);

    for (int i = 0; i < total_items; i++)
    {
        uint8_t item_idx = all_items[i];
        bool is_bomb = false;
        uint8_t current_box_type = 0;
        if (i >= current_state->box_count)
        {
            is_bomb = true;
        }
        else
        {
            current_box_type = current_state->boxes[i].id;
        }
        for (int d = 0; d < 4; d++)
        {
            int next_item_idx = neighbor_index(item_idx, d);
            int push_stand_idx = neighbor_index(item_idx, d ^ 1);
            if (next_item_idx < 0 || push_stand_idx < 0)
                continue;
            bool exploded = false;
            bool consumed = false;

            if (obstacles[next_item_idx] && !current_walls[next_item_idx])
                continue;
            if (obstacles[push_stand_idx] && push_stand_idx != current_state->car_pos)
                continue;

            if (current_walls[next_item_idx])
            {
                if (is_bomb)
                {
                    if (ctx->boundary_walls[next_item_idx])
                        continue;
                    exploded = true;
                }
                else
                {
                    continue;
                }
            }
            else if (!is_bomb)
            {

                int8_t goal_i = ctx->goal_mask_map[next_item_idx];
                if (goal_i != -1 && (current_state->active_goals_mask & (1U << goal_i)) &&
                    ctx->goal_type_map[next_item_idx] == current_box_type)
                {
                    consumed = true; // ��������
                }
                else
                {

                    bool is_safe = false;
                    for (int g = 0; g < ctx->goal_count; g++)
                    {
                        if (current_state->active_goals_mask & (1U << g))
                        {

                            if (ctx->goals[g].id == current_box_type || current_box_type == NO_CLS)
                            {

                                if (ctx->cached_dist_table[g][next_item_idx] < INF_DIST)
                                {
                                    is_safe = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (!is_safe)
                        continue;
                }
            }

            int car_dist = car_dist_map[push_stand_idx];
            if (car_dist == UINT8_MAX)
                continue;

            int step_cost = car_dist + 1;

            if (exploded)
                step_cost += wall_action_penalty(current_walls[next_item_idx]);
            int next_g = current_g + step_cost;

            // 创建新state
            State next_state = *current_state;

            next_state.base_hash ^= zobrist_car[current_state->car_pos];
            next_state.base_hash ^= zobrist_car[item_idx];

            next_state.car_pos = item_idx; // С������

            const uint8_t *walls_for_eval = current_walls;
            uint8_t temp_walls[MAP_SIZE];
            if (exploded)
            {
                next_state.base_hash ^= zobrist_bomb[item_idx];
                // �Ƴ�ը��
                for (int k = 0; k < next_state.bomb_count; k++)
                {
                    if (next_state.bombs[k] == item_idx)
                    {
                        next_state.bombs[k] = next_state.bombs[--next_state.bomb_count];
                        break;
                    }
                }

                memcpy(temp_walls, current_walls, MAP_SIZE);
                int exp_count = ctx->explosion_area_count[next_item_idx];
                for (int e = 0; e < exp_count; e++)
                {
                    uint8_t w_idx = ctx->explosion_areas[next_item_idx][e];
                    if (temp_walls[w_idx])
                    {
                        temp_walls[w_idx] = 0;
                        next_state.base_hash ^= zobrist_wall[w_idx];
                    }
                }
                walls_for_eval = temp_walls;
            }
            else if (consumed)
            {

                next_state.base_hash ^= zobrist_box[current_box_type][item_idx];
                next_state.base_hash ^= zobrist_goal_mask[ctx->goal_mask_map[next_item_idx]];

                for (int k = 0; k < next_state.box_count; k++)
                {
                    if (next_state.boxes[k].pos == item_idx)
                    {
                        next_state.boxes[k] = next_state.boxes[--next_state.box_count];
                        break;
                    }
                }
                next_state.active_goals_mask &= ~(1U << ctx->goal_mask_map[next_item_idx]);
            }
            else
            {

                if (is_bomb)
                {

                    next_state.base_hash ^= zobrist_bomb[item_idx];
                    next_state.base_hash ^= zobrist_bomb[next_item_idx];
                    for (int k = 0; k < next_state.bomb_count; k++)
                    {
                        if (next_state.bombs[k] == item_idx)
                        {
                            next_state.bombs[k] = next_item_idx;
                            break;
                        }
                    }
                }
                else
                {
                    next_state.base_hash ^= zobrist_box[current_box_type][item_idx];
                    next_state.base_hash ^= zobrist_box[current_box_type][next_item_idx];
                    for (int k = 0; k < next_state.box_count; k++)
                    {
                        if (next_state.boxes[k].pos == item_idx)
                        {
                            next_state.boxes[k].pos = next_item_idx;
                            break;
                        }
                    }
                }
                if (is_deadlock(ctx, next_item_idx, &next_state, is_bomb, current_walls) > next_state.bomb_count)
                {
                    continue;
                }
            }

            if (hash_table_insert_or_check(&next_state, next_g, 0))
            {
                continue;
            }

            int next_h = calc_heuristic(ctx, &next_state, walls_for_eval);
            if (next_h >= INF_DIST)
                continue;
            if (child_count >= MAX_BRANCHES)
            {
                continue;
            }
            children[child_count].next_h = (uint16_t)next_h;
            children[child_count].next_g = (uint16_t)next_g;
            children[child_count].next_state = next_state;
            children[child_count].action = (MacroAction){push_stand_idx, item_idx, exploded, consumed};
            child_count++;
        }
    }

    // �ڵ�����
    uint8_t indices[MAX_BRANCHES];
    for (uint8_t i = 0; i < child_count; i++)
    {
        indices[i] = i;
    }
    sort_children(children, child_count, indices, ctx->current_weight);
    // 展开各child
    for (uint8_t i = 0; i < child_count; i++)
    {
        ChildNode *sorted_child = &children[indices[i]];
        float sorted_f = child_f_score(sorted_child, ctx->current_weight);

        if (sorted_f > threshold)
        {
            if (sorted_f < min_node_data.f)
            {
                min_node_data.f = sorted_f;
                min_node_data.g = sorted_child->next_g;
                min_node_data.h = sorted_child->next_h;
            }
            break;
        }
        acts[act_len] = sorted_child->action;

        const uint8_t *walls_to_pass = current_walls;
        uint8_t recurse_walls[MAP_SIZE];

        if (sorted_child->action.is_explode)
        {

            memcpy(recurse_walls, current_walls, MAP_SIZE);

            // ���ݱ�ը���
            int push_dir = sorted_child->action.push_to - sorted_child->action.move_to;
            uint8_t next_pos = sorted_child->action.push_to + push_dir;

            uint8_t exp_count = ctx->explosion_area_count[next_pos];
            for (uint8_t e = 0; e < exp_count; e++)
            {
                recurse_walls[ctx->explosion_areas[next_pos][e]] = 0;
            }
            walls_to_pass = recurse_walls;
        }
        SearchRes res = dfs_ida(ctx, &sorted_child->next_state, walls_to_pass, sorted_child->next_g, sorted_child->next_h, threshold, acts, act_len + 1);

        if (res.f == RES_SUCCESS || res.f == RES_NODE_LIMIT)
            return res;
        if (res.f < min_node_data.f)
        {
            min_node_data = res;
        }
    }
    return min_node_data;
}
// =====================================================================

static bool try_infer_identities(SokobanContext *ctx, State *current_state)
{
    bool inferred_something = false;

    uint8_t box_counts[MAX_ID] = {0};
    uint8_t goal_counts[MAX_ID] = {0};
    uint8_t unk_box_count = 0;
    uint8_t unk_goal_count = 0;

    for (int i = 0; i < current_state->box_count; i++)
    {
        if (current_state->boxes[i].id == UNKNOWN)
            unk_box_count++;
        else
            box_counts[current_state->boxes[i].id]++;
    }
    for (int i = 0; i < ctx->goal_count; i++)
    {
        if (ctx->goals[i].id == UNKNOWN)
            unk_goal_count++;
        else
            goal_counts[ctx->goals[i].id]++;
    }

    if (unk_box_count > 0)
    {
        uint8_t total_box_deficit = 0;
        uint8_t deficit_id = 0;
        uint8_t distinct_deficit_ids = 0;

        for (uint8_t id = 0; id < MAX_ID; id++)
        {
            if (id == UNKNOWN)
                continue;

            if (goal_counts[id] > box_counts[id])
            {
                total_box_deficit += (goal_counts[id] - box_counts[id]);
                deficit_id = id;
                distinct_deficit_ids++;
            }
        }

        if (total_box_deficit == unk_box_count && distinct_deficit_ids == 1)
        {
            for (uint8_t i = 0; i < current_state->box_count; i++)
            {
                if (current_state->boxes[i].id == UNKNOWN)
                {
                    current_state->boxes[i].id = deficit_id;
                    inferred_something = true;
                }
            }
        }
    }

    if (unk_goal_count > 0)
    {
        uint8_t total_goal_deficit = 0;
        uint8_t deficit_id = 0;
        uint8_t distinct_deficit_ids = 0;
        for (uint8_t id = 0; id < MAX_ID; id++)
        {
            if (id == UNKNOWN)
                continue;
            if (box_counts[id] > goal_counts[id])
            {
                total_goal_deficit += (box_counts[id] - goal_counts[id]);
                deficit_id = id;
                distinct_deficit_ids++;
            }
        }
        if (total_goal_deficit == unk_goal_count && distinct_deficit_ids == 1)
        {
            for (uint8_t i = 0; i < ctx->goal_count; i++)
            {
                if (ctx->goals[i].id == UNKNOWN)
                {
                    ctx->goals[i].id = deficit_id;
                    ctx->goal_type_map[ctx->goals[i].pos] = deficit_id;
                    inferred_something = true;
                }
            }
        }
    }
    return inferred_something;
}

static bool get_nearest_path(uint8_t start_pos, const bool *obs_points, const uint8_t *obstacles, WaypointPath *out_path)
{
    if (obs_points[start_pos])
    {
        out_path->points[0] = start_pos;
        out_path->length = 1;
        return true;
    }

    uint8_t queue[MAP_SIZE];
    uint8_t parent[MAP_SIZE];
    bool visited[MAP_SIZE] = {false};
    int head = 0, tail = 0;

    queue[tail++] = start_pos;
    visited[start_pos] = true;
    parent[start_pos] = start_pos;

    uint8_t found_target = 255;
    while (head < tail)
    {
        uint8_t curr = queue[head++];
        if (obs_points[curr])
        {
            found_target = curr;
            break;
        }
        for (int i = 0; i < 4; i++)
        {
            int n_idx = neighbor_index(curr, i);
            if (n_idx < 0 || n_idx >= MAP_SIZE)
                continue;
            if (!obstacles[n_idx] && !visited[n_idx])
            {
                visited[n_idx] = true;
                parent[n_idx] = curr;
                queue[tail++] = n_idx;
            }
        }
    }
    if (found_target == 255)
        return false;

    uint8_t temp_path[MAP_SIZE];
    int count = 0;
    uint8_t curr = found_target;
    while (curr != start_pos)
    {
        temp_path[count++] = curr;
        curr = parent[curr];
    }
    temp_path[count++] = start_pos;
    out_path->length = count;
    for (int i = 0; i < count; i++)
        out_path->points[i] = temp_path[count - 1 - i];
    return true;
}

// 估计小车到最近可用识别视点的破障/绕行代价。
static int calc_recon_heuristic(SokobanContext *ctx, State *state, const bool *obs_points, const uint8_t *walls)
{
    MinHeap pq;
    pq.size = 0;
    uint8_t movable[MAP_SIZE];
    memset(movable, 0, MAP_SIZE);
    for (int i = 0; i < state->box_count; i++)
    {
        movable[state->boxes[i].pos] = 1;
    }
    for (int i = 0; i < state->bomb_count; i++)
    {
        movable[state->bombs[i]] = 1;
    }
    uint16_t dist[MAP_SIZE];
    for (int i = 0; i < MAP_SIZE; i++)
    {
        dist[i] = INF_DIST;
    }

    dist[state->car_pos] = 0;
    heap_push(&pq, 0, state->car_pos);
    while (pq.size > 0)
    {
        HeapNode node = heap_pop(&pq);
        uint8_t curr = node.pos;
        if (node.dist > dist[curr])
            continue;
        if (obs_points[curr])
            return node.dist;
        for (uint8_t i = 0; i < 4; i++)
        {
            int n_idx = neighbor_index(curr, i);
            if (n_idx < 0 || n_idx >= MAP_SIZE)
                continue;
            if (ctx->boundary_walls[n_idx])
                continue;
            uint16_t step_cost = 1;
            if (walls[n_idx])
            {
                if (state->bomb_count == 0)
                    continue;
                step_cost += wall_action_penalty(walls[n_idx]);
            }
            else if (movable[n_idx] == 1)
                step_cost = MOVE_PENALTY; // �ƶ��������
            if (dist[curr] + step_cost < dist[n_idx])
            {
                dist[n_idx] = dist[curr] + step_cost;
                heap_push(&pq, dist[n_idx], n_idx);
            }
        }
    }
    return INF_DIST;
}
// ida*ʶͼѰ·

// 为识别阶段搜索一条可到达视点的破障动作序列。
static SearchRes dfs_ida_recon(SokobanContext *ctx, State *current_state, const uint8_t *current_walls, uint16_t current_g, int current_h,
                               float threshold, MacroAction *acts, uint8_t act_len, const bool *obs_points, const bool *virtual_obs_points)
{
    // 识别搜索与正式求解共用同一累计节点预算语义。
    if (ctx->total_explored_nodes >= MAX_ALLOWABLE_NODES)
        return (SearchRes){RES_NODE_LIMIT, 0, 0};
    ctx->total_explored_nodes++;
    if (act_len >= MAX_STEPS)
        return (SearchRes){RES_INF, 0, 0};

    if (current_h == -1)
        current_h = calc_recon_heuristic(ctx, current_state, virtual_obs_points, current_walls);
    float f_score = current_g + current_h;
    if (f_score > threshold)
        return (SearchRes){f_score, current_g, current_h};

    get_maze_distances(ctx, current_walls, current_state->bomb_count);
    uint8_t obstacles[MAP_SIZE];
    memcpy(obstacles, current_walls, MAP_SIZE);
    for (int i = 0; i < current_state->box_count; i++)
        obstacles[current_state->boxes[i].pos] = 1;
    for (int i = 0; i < current_state->bomb_count; i++)
        obstacles[current_state->bombs[i]] = 1;

    WaypointPath temp_path;
    if (get_nearest_path(current_state->car_pos, obs_points, obstacles, &temp_path))
    {
        ctx->solution_actions_len = act_len;
        for (int i = 0; i < act_len; i++)
            ctx->solution_actions[i] = acts[i];
        return (SearchRes){RES_SUCCESS, 0, 0};
    }

    SearchRes min_node_data = {RES_INF, 0, 0};
    uint8_t all_items[MAX_BOXES + MAX_BOMBS];
    uint8_t total_items = 0;
    for (int i = 0; i < current_state->box_count; i++)
        all_items[total_items++] = current_state->boxes[i].pos;
    for (int i = 0; i < current_state->bomb_count; i++)
        all_items[total_items++] = current_state->bombs[i];

    uint8_t car_dist_map[MAP_SIZE];
    build_car_dist_map(current_state->car_pos, obstacles, car_dist_map);

    for (int i = 0; i < total_items; i++)
    {
        uint8_t item_idx = all_items[i];
        bool is_bomb = (i >= current_state->box_count);
        uint8_t current_box_type = is_bomb ? 0 : current_state->boxes[i].id;

        for (int d = 0; d < 4; d++)
        {
            int next_item_idx = neighbor_index(item_idx, d);
            int push_stand_idx = neighbor_index(item_idx, d ^ 1);
            if (next_item_idx < 0 || push_stand_idx < 0)
                continue;


            bool exploded = false, consumed = false;

            if (obstacles[next_item_idx] && !current_walls[next_item_idx])
            {
                continue;
            }
            if (obstacles[push_stand_idx] && push_stand_idx != current_state->car_pos)
            {
                continue;
            }

            if (current_walls[next_item_idx])
            {
                if (is_bomb && !ctx->boundary_walls[next_item_idx])
                    exploded = true;
                else
                {
                    continue;
                }
            }
            else if (!is_bomb)
            {
                int8_t goal_i = ctx->goal_mask_map[next_item_idx];
                if (goal_i != -1 && (current_state->active_goals_mask & (1U << goal_i)) && ctx->goal_type_map[next_item_idx] == current_box_type)
                {
                    consumed = true;
                }
                else
                {
                    bool is_safe = false;
                    for (int g = 0; g < ctx->goal_count; g++)
                    {
                        if (current_state->active_goals_mask & (1U << g))
                        {

                            if (ctx->goals[g].id == current_box_type || current_box_type == NO_CLS || current_box_type == UNKNOWN || ctx->goals[g].id == NO_CLS || ctx->goals[g].id == UNKNOWN)
                            {
                                if (ctx->cached_dist_table[g][next_item_idx] < INF_DIST)
                                {
                                    is_safe = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (!is_safe)
                    {
                        continue;
                    }
                }
            }

            int car_dist = car_dist_map[push_stand_idx];
            if (car_dist == UINT8_MAX)
            {
                continue;
            }

            int step_cost = car_dist + 1;
            if (exploded)
                step_cost += wall_action_penalty(current_walls[next_item_idx]);
            int next_g = current_g + step_cost;

            State next_state = *current_state;

            next_state.base_hash ^= zobrist_car[current_state->car_pos];
            next_state.base_hash ^= zobrist_car[item_idx];
            next_state.car_pos = item_idx;

            const uint8_t *walls_for_eval = current_walls;
            uint8_t temp_walls[MAP_SIZE];

            bool new_obs_points[MAP_SIZE];
            const bool *obs_points_to_pass = obs_points;
            bool obs_changed = false;

            if (exploded)
            {

                next_state.base_hash ^= zobrist_bomb[item_idx];
                for (int k = 0; k < next_state.bomb_count; k++)
                {
                    if (next_state.bombs[k] == item_idx)
                    {
                        next_state.bombs[k] = next_state.bombs[--next_state.bomb_count];
                        break;
                    }
                }

                memcpy(temp_walls, current_walls, MAP_SIZE);
                int exp_count = ctx->explosion_area_count[next_item_idx];
                for (int e = 0; e < exp_count; e++)
                {
                    uint8_t w_idx = ctx->explosion_areas[next_item_idx][e];
                    if (temp_walls[w_idx])
                    {
                        temp_walls[w_idx] = 0;

                        next_state.base_hash ^= zobrist_wall[w_idx];
                    }
                    if (virtual_obs_points[w_idx] && !obs_points[w_idx])
                    {
                        if (!obs_changed)
                        {
                            memcpy(new_obs_points, obs_points, MAP_SIZE);
                            obs_changed = true;
                        }
                        new_obs_points[w_idx] = true;
                    }
                }
                walls_for_eval = temp_walls;
            }
            else if (consumed)
            {

                next_state.base_hash ^= zobrist_box[current_box_type][item_idx];
                next_state.base_hash ^= zobrist_goal_mask[ctx->goal_mask_map[next_item_idx]];
                for (int k = 0; k < next_state.box_count; k++)
                {
                    if (next_state.boxes[k].pos == item_idx)
                    {
                        next_state.boxes[k] = next_state.boxes[--next_state.box_count];
                        break;
                    }
                }
                next_state.active_goals_mask &= ~(1U << ctx->goal_mask_map[next_item_idx]);
            }
            else
            {
                if (is_bomb)
                {

                    next_state.base_hash ^= zobrist_bomb[item_idx];
                    next_state.base_hash ^= zobrist_bomb[next_item_idx];
                    for (int k = 0; k < next_state.bomb_count; k++)
                    {
                        if (next_state.bombs[k] == item_idx)
                        {
                            next_state.bombs[k] = next_item_idx;
                            break;
                        }
                    }
                }
                else
                {

                    next_state.base_hash ^= zobrist_box[current_box_type][item_idx];
                    next_state.base_hash ^= zobrist_box[current_box_type][next_item_idx];
                    for (int k = 0; k < next_state.box_count; k++)
                    {
                        if (next_state.boxes[k].pos == item_idx)
                        {
                            next_state.boxes[k].pos = next_item_idx;
                            break;
                        }
                    }
                }
                if (is_deadlock(ctx, next_item_idx, &next_state, is_bomb, walls_for_eval) > next_state.bomb_count)
                {
                    continue;
                }
            }

            if (virtual_obs_points[item_idx] && !obs_points[item_idx])
            {
                if (!obs_changed)
                {
                    memcpy(new_obs_points, obs_points, MAP_SIZE);
                    obs_changed = true;
                }
                new_obs_points[item_idx] = true;
            }
            if (obs_changed)
                obs_points_to_pass = new_obs_points;

            if (hash_table_insert_or_check(&next_state, next_g, 0))
            {
                continue;
            }

            int next_h = calc_recon_heuristic(ctx, &next_state, virtual_obs_points, walls_for_eval);
            float next_f = next_g + next_h;

            if (next_f > threshold)
            {
                if (next_f < min_node_data.f)
                {
                    min_node_data.f = next_f;
                    min_node_data.g = next_g;
                    min_node_data.h = next_h;
                }
                continue;
            }

            acts[act_len] = (MacroAction){push_stand_idx, item_idx, exploded, consumed};
            SearchRes res = dfs_ida_recon(ctx, &next_state, walls_for_eval, next_g, next_h, threshold, acts, act_len + 1, obs_points_to_pass, virtual_obs_points);

            if (res.f == RES_SUCCESS || res.f == RES_NODE_LIMIT)
                return res;
            if (res.f < min_node_data.f)
                min_node_data = res;
        }
    }
    return min_node_data;
}

// 迭代提升识别搜索阈值，直到抵达视点、证明失败或达到节点上限。
static bool solve_recon_ida(SokobanContext *ctx, State *start_state, const bool *obs_points, const bool *virtual_obs_points)
{
    ctx->total_explored_nodes = 0;
    ctx->solution_actions_len = 0;

    ctx->initial_state.base_hash = compute_initial_base_hash(&ctx->initial_state, ctx->initial_walls);

    int initial_h = calc_recon_heuristic(ctx, start_state, virtual_obs_points, ctx->initial_walls);
    if (initial_h >= INF_DIST)
    {
        return false;
    }
    float threshold = (float)initial_h;
    MacroAction current_act[MAX_STEPS];
    while (true)
    {
        hash_table_clear();
        hash_table_insert_or_check(start_state, 0, 0);
        SearchRes res = dfs_ida_recon(ctx, start_state, ctx->initial_walls, 0, initial_h, threshold, current_act, 0, obs_points, virtual_obs_points);
        if (res.f == RES_SUCCESS)
            return true;
        if (res.f == RES_NODE_LIMIT)
        {
            return false;
        }
        if (res.f >= RES_INF)
        {
            return false;
        }
        threshold = res.f;
    }
}

void build_map_info(SokobanContext *ctx, const uint8_t *raw_map, uint8_t cls)
{
    hash_table_clear();
    engine_init(ctx, raw_map);
    if (!ctx->map_valid)
        return;
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
        return;
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
        uint8_t target_map[MAP_SIZE];
        memset(target_map, 0, sizeof(target_map));
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
                        if (n >= 0 && !the_goals[n] &&
                            !(failed[n] & (1U << (d ^ 1))))
                        {
                            virtual_obs_points[n] = true; // �����ӵ�, ��IDA*�õ�
                            if (!obstacles[n])
                            {
                                observation_points[n] = true;
                                target_map[n] = (0 << 7) | i; // Type 0: Goal
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
                        if (n >= 0 && !the_goals[n] &&
                            !(failed[n] & (1U << (d ^ 1))))
                        {
                            virtual_obs_points[n] = true; // �����ӵ�, ��IDA*�õ�
                            if (!obstacles[n])
                            {
                                observation_points[n] = true;
                                target_map[n] = (1 << 7) | i; // Type 1: Box
                            }
                        }
                    }
                }
            }
        }

        if (get_nearest_path(current_state->car_pos, observation_points, obstacles, &path))
        {
            uint8_t final_pos = path.points[path.length - 1];
            uint8_t target_info = target_map[final_pos];
            bool is_box = (target_info >> 7) == 1;
            uint8_t index = target_info & 0b01111111;
            uint8_t entity_pos = is_box ? current_state->boxes[index].pos : ctx->goals[index].pos;
            get_smooth_path(ctx, &path, obstacles, &smooth_path);
            current_state->car_pos = final_pos;

            int8_t target_delta = entity_pos - final_pos;
            uint8_t target_direction = 0;
            if (target_delta == WIDTH)
                target_direction = 1;
            else if (target_delta == -1)
                target_direction = 2;
            else if (target_delta == 1)
                target_direction = 3;

            //--��Ϊ�˲��ߵ����һ���㣬���һ������������
            smooth_path.length--;
            car_move(&smooth_path, angle, 0);
            while (navigate_flag)
            {
            }
            uint8_t final_pos_X = final_pos % 16;
            uint8_t final_pos_Y = final_pos / 16;
            uint8_t entity_pos_X = entity_pos % 16;
            uint8_t entity_pos_Y = entity_pos / 16;

            int8_t dx = entity_pos_X - final_pos_X;
            int8_t dy = entity_pos_Y - final_pos_Y;

            float final_actual_x = final_pos_X * 0.2 + 0.1;
            float final_actual_y = 2.4f - final_pos_Y * 0.2 - 0.1;
            // back_error��С������ʶ����ľ���
            float back_error = 0.025;
            // ȷ�����һ��λ������
            if (dx > 0)
                final_actual_x -= back_error;
            else if (dx < 0)
                final_actual_x += back_error;
            else if (dy > 0)
                final_actual_y += back_error;
            else if (dy < 0)
                final_actual_y -= back_error;

            car_move_point(final_actual_x, final_actual_y, angle, 0);
            while (navigate_flag)

            if (dx > 0)
                angle = -90;
            else if (dx < 0)
                angle = 90;
            else if (dy > 0)
                angle = 180;
            else if (dy < 0)
                angle = 0;

            system_delay_ms(200);
            car_turn(angle);
            while (!yaw_arrived_flag)

            // UNKNOWN is a valid result; UINT8_MAX means the request is pending.
            final_image_index = UINT8_MAX;
            check_image(3 - is_box, 1);
            vision_angle_switch = 0;
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
                    if (final_actual_x <= final_pos_X * 0.2 + 0.1)
                    {
                        final_actual_x += 0.005f;
                    }
                }
                else if (dx < 0)
                {
                    if (final_actual_x >= final_pos_X * 0.2 + 0.1)
                    {
                        final_actual_x -= 0.005f;
                    }
                }
                else if (dy > 0)
                {
                    if (final_actual_y >= 2.4f - final_pos_Y * 0.2 - 0.1)
                    {
                        final_actual_y -= 0.005f;
                    }
                }
                else if (dy < 0)
                {
                    if (final_actual_y <= 2.4f - final_pos_Y * 0.2 - 0.1)
                    {
                        final_actual_y += 0.005f;
                    }
                }

                car_move_point(final_actual_x, final_actual_y, angle, 0);
                while (navigate_flag){}
            }
            // ��ʶ������Ȼδ֪�����������ʶ�𣨿����ǵ�һ�ζ�׼����׼ȷ��
            // ʶ��ʱ����carmove�����Ӿ��Ƕ�У��
            vision_angle_switch = 0;
            // system_delay_ms(700);
            system_delay_ms(200);
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
                return;
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

            while (try_infer_identities(ctx, current_state))
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
            if (solve_recon_ida(ctx, current_state, observation_points, virtual_obs_points))
            {
                generate_path(ctx, &smooth_path);
                current_state = &ctx->initial_state;
                car_move(&smooth_path, angle, 0);
                while (navigate_flag)
                continue;
            }
            else
            {
                return;
            }
        }
    }

    uint8_t final_box_counts[MAX_ID] = {0};
    uint8_t final_goal_counts[MAX_ID] = {0};

    for (int i = 0; i < current_state->box_count; i++)
    {
        if (current_state->boxes[i].id != UNKNOWN)
        {
            final_box_counts[current_state->boxes[i].id]++;
        }
    }
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
    if (box_surplus_count == 1 && goal_surplus_count == 1 &&
        final_box_counts[err_box_id] == 1 && final_goal_counts[err_goal_id] == 1)
    {

        for (int i = 0; i < current_state->box_count; i++)
        {
            if (current_state->boxes[i].id == err_box_id)
            {
                current_state->boxes[i].id = NO_CLS;
                break;
            }
        }

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
}
// 引擎主入口：执行加权 IDA* 搜索。
// 返回 true 表示找到可行解；返回 false 表示地图无效、无解或达到节点上限。
bool solve(SokobanContext *ctx)
{
    if (!ctx->map_valid || ctx->initial_state.box_count != ctx->goal_count)
    {
        return false;
    }

    ctx->total_explored_nodes = 0;
    ctx->solution_actions_len = 0;

    ctx->initial_state.base_hash = compute_initial_base_hash(&ctx->initial_state, ctx->initial_walls);
    if (ctx->current_weight <= 0.0f)
        ctx->current_weight = SOKOBAN_CURRENT_WEIGHT;
    if (ctx->min_weight <= 0.0f)
        ctx->min_weight = SOKOBAN_MIN_WEIGHT;
    if (ctx->min_weight > ctx->current_weight)
        ctx->min_weight = ctx->current_weight;
    int patience_limit = 2;

    int initial_h = calc_heuristic(ctx, &ctx->initial_state, ctx->initial_walls);
    if (initial_h >= INF_DIST)
    {
        return false;
    }
    uint8_t iteration = 0;

    float threshold = initial_h * ctx->current_weight;
    MacroAction acts[MAX_STEPS];

    while (true)
    {
        iteration += 1;
        hash_table_clear();
        // ����ʼ״̬�����ϣ��
        hash_table_insert_or_check(&ctx->initial_state, 0, 0);

        SearchRes res = dfs_ida(ctx, &ctx->initial_state, ctx->initial_walls, 0, initial_h, threshold, acts, 0);
        if (res.f == RES_SUCCESS)
        {
            return true;
        }
        if (res.f == RES_NODE_LIMIT)
        {
            return false;
        }
        if (res.f >= RES_INF)
        {
            return false;
        }
        float min_f = res.f;
        int min_g = res.g;
        int min_h = res.h;
        // ˥���ж�����������˥����
        if (iteration % patience_limit == 0 && ctx->current_weight > ctx->min_weight)
        {

            ctx->current_weight = ctx->min_weight + (ctx->current_weight - ctx->min_weight) * 0.5f;

            if (ctx->current_weight - ctx->min_weight < 0.2f)
            {
                ctx->current_weight = ctx->min_weight;
            }
            // 根据新的权重重新计算当前边界
            threshold = (float)min_g + ctx->current_weight * (float)min_h;
            patience_limit += patience_limit;
            iteration = 0;
            continue;
        }

        // 标准 IDA* 使用本轮所有越界节点的最小 f 作为下一阈值。
        threshold = min_f;
    }
    return false;
}

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

    uint8_t queue[MAP_SIZE];
    uint8_t parent[MAP_SIZE];
    bool visited[MAP_SIZE];
    memset(visited, 0, sizeof(visited));

    int head = 0;
    int tail = 0;
    // ������
    queue[tail++] = start_pos;
    visited[start_pos] = true;
    parent[start_pos] = start_pos;
    bool found = false;

    while (head < tail)
    {
        uint8_t curr = queue[head++];
        if (curr == target_pos)
        {
            found = true;
            break;
        }

        for (int i = 0; i < 4; i++)
        {

            int n_idx = neighbor_index(curr, i);
            if (n_idx < 0 || n_idx >= MAP_SIZE)
                continue;

            if (!obstacles[n_idx] && !visited[n_idx])
            {
                visited[n_idx] = true;
                parent[n_idx] = curr;
                queue[tail++] = n_idx;
            }
        }
    }
 
    if (!found)
        return false;

    uint8_t temp_path[MAP_SIZE];
    int count = 0;
    uint8_t curr = target_pos;
    while (curr != start_pos)
    {
        temp_path[count++] = curr;
        curr = parent[curr];
    }
    temp_path[count++] = start_pos;

    out_path->length = count;
    for (int i = 0; i < count; i++)
    {
        out_path->points[i] = temp_path[count - 1 - i];
    }
    return true;
}

static bool pass(uint8_t startpoint, uint8_t endpoint, float error, const uint8_t *obstacles)
{

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
    return 0;
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
            float plus_xy[4][2] = {{0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}};
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
    return 1;
}
// 节点平滑
static void get_smooth_path(SokobanContext *ctx, const WaypointPath *grid_path, const uint8_t *obstacles, WaypointPath *out_smooth_path)
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

static void get_final_path(SokobanContext *ctx, WaypointPath *path)
{
    if (path->length <= 2)
    {
        return;
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
    if (unique_len <= 2)
    {
        path->length = unique_len;
        for (int i = 0; i < unique_len; i++)
            path->points[i] = unique_points[i];
        return;
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
    // д��ԭ�ṹ��
    path->length = new_len;
    for (int i = 0; i < new_len; i++)
    {
        path->points[i] = new_points[i];
    }
}

void generate_path(SokobanContext *ctx, WaypointPath *out_full_path)
{
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
            return;
        }
        get_smooth_path(ctx, &micro_path, obstacles, &smooth_path);
        out_full_path->length += smooth_path.length;
        for (int p = 0; p < smooth_path.length; p++)
        {
            out_full_path->points[out_full_path->length - smooth_path.length + p] = smooth_path.points[p];
        }
        out_full_path->points[out_full_path->length++] = act.push_to;
        if (act.is_explode)
        {
            out_full_path->points[out_full_path->length++] = 255;   //延时特殊标记符号
        }
        // ===========================================

        // ������һ֡��ͼ
        int push_dir = act.push_to - act.move_to;
        uint8_t next_pos = act.push_to + push_dir;

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
    // 更新初始状态为最终状�?
    ctx->initial_state = sim_state;
    memcpy(ctx->initial_walls, sim_walls, MAP_SIZE);
    get_final_path(ctx, out_full_path); // 对整条路径进行最终的优化处理
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
