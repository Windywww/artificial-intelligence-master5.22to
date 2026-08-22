#include "sokoban_engine.h"
#include "sokoban_solver_internal.h"
#include "sokoban_lut.h"
#include <string.h>

#define SOKOBAN_EMBEDDED 1
#ifndef ENTER_GOAL
#define ENTER_GOAL 1 // 识别时能否进入目标点 1=能
#endif
#define DEBUG_RECON 0
#define MAX_ID 12 // id 可能的取值个数
#ifndef MAX_ALLOWABLE_NODES
#define MAX_ALLOWABLE_NODES 200000 // 限制搜索节点总数
#endif
#ifndef SOKOBAN_CURRENT_WEIGHT
#define SOKOBAN_CURRENT_WEIGHT 3.5f
#endif
#ifndef SOKOBAN_MIN_WEIGHT
#define SOKOBAN_MIN_WEIGHT 3.5f
#endif
#ifndef MOVE_PENALTY
#define MOVE_PENALTY 10
#endif
#define UNKNOWN 11
#define ERROR 99.0f
// 节点上限必须作为独立状态逐层传播，不能与无解或下一阈值混淆。
#define RES_NODE_LIMIT -2.0f
// 保持置换表总容量不变，每个集合容纳四个相同低位索引的状态。
#define HASH_WAYS 4
#define HASH_SET_COUNT (HASH_TABLE_SIZE / HASH_WAYS)
#define HASH_SET_MASK (HASH_SET_COUNT - 1)
#ifndef RECON_PATH_SLACK
#define RECON_PATH_SLACK 2U
#endif
#define RECON_ROUTE_LIMIT 18U
#define RECON_MAX_ROUTE_LENGTH (RECON_ROUTE_LIMIT - RECON_PATH_SLACK)

#define MAX_RECON_CANDIDATES ((MAX_BOXES + MAX_GOALS) * 4)

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
static uint8_t is_deadlock(SokobanContext *ctx, uint8_t idx, State *state, bool is_bomb, const uint8_t *walls);
// 按当前墙布局和剩余炸弹数，构建各目标点的反向推动距离表。
static void get_maze_distances(SokobanContext *ctx, const uint8_t *current_walls, uint8_t bomb_count);

int sokoban_neighbor_index(int idx, int direction)
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

// 判断箱子从 next_pos 是否仍能到达一个兼容的未完成目标。
// UNKNOWN 目标可作为分类尚未完成时的后续归宿；NO_CLS 箱子沿用无分类规则，
// 可匹配任意未完成目标。距离表必须已按当前墙布局和剩余炸弹数更新。
static bool can_reach_compatible_goal(const SokobanContext *ctx, const State *state,
                                      uint8_t box_type, uint8_t next_pos)
{
    for (int g = 0; g < ctx->goal_count; g++)
    {
        if (!(state->active_goals_mask & (1U << g)))
            continue;

        uint8_t goal_type = ctx->goals[g].id;
        bool compatible = (goal_type == UNKNOWN || goal_type == box_type);
        if (compatible && ctx->cached_dist_table[g][next_pos] < INF_DIST)
            return true;
    }
    return false;
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

static void init_zobrist(void)
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
    uint8_t car_count = 0;
    init_state->car_pos = 0;
    init_state->box_count = 0;
    init_state->bomb_count = 0;
    ctx->initial_tnt_count = 0;
    ctx->deadlock_required_tnt = 0;
    ctx->redundant_tnt = 0;
    ctx->has_absolute_deadlock = false;
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
                    ctx->map_valid = false;
                }
            }
            else if (val == 5)
            { // С�� (CAR)
                init_state->car_pos = idx;
                car_count++;
                if (car_count > 1U)
                {
                    ctx->map_valid = false;
                }
            }
            else if (val != 0)
            {
                ctx->map_valid = false;
            }
        }
    }

    if (car_count != 1U || ctx->goal_count != ctx->initial_state.box_count)
    {
        ctx->map_valid = false;
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
    ctx->initial_tnt_count = init_state->bomb_count;

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
                        int n_idx = sokoban_neighbor_index(curr, d);
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

    }

    uint16_t required_tnt_total = 0;
    for (uint8_t i = 0; i < init_state->box_count; i++)
    {
        uint8_t box_idx = init_state->boxes[i].pos;
        uint8_t required_tnt = is_deadlock(ctx, box_idx, init_state, false, ctx->initial_walls);
        if (required_tnt == UINT8_MAX)
            ctx->has_absolute_deadlock = true;
        else
            required_tnt_total += required_tnt;
        if (required_tnt > 0)
        {
            for (uint8_t j = 0; j < ctx->explosion_area_count[box_idx]; j++)
            {
                uint8_t n_idx = ctx->explosion_areas[box_idx][j];
                if (ctx->initial_walls[n_idx] != WALL_NONE && !ctx->boundary_walls[n_idx])
                    ctx->initial_walls[n_idx] = WALL_DEADLOCK;
            }
        }
    }
    ctx->deadlock_required_tnt = (required_tnt_total > UINT8_MAX) ? UINT8_MAX : (uint8_t)required_tnt_total;
    if (!ctx->has_absolute_deadlock && ctx->initial_tnt_count > ctx->deadlock_required_tnt)
        ctx->redundant_tnt = (uint8_t)(ctx->initial_tnt_count - ctx->deadlock_required_tnt);
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
            int n_idx = sokoban_neighbor_index(curr, i);

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
            int next_item_idx = sokoban_neighbor_index(item_idx, d);
            int push_stand_idx = sokoban_neighbor_index(item_idx, d ^ 1);
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
                bool active_goal = goal_i != -1 && (current_state->active_goals_mask & (1U << goal_i));
                bool temporary_goal = false;
                if (active_goal &&
                    can_consume_goal(current_box_type, ctx->goal_type_map[next_item_idx]))
                {
                    consumed = true;
                }
                else
                {
                    if (active_goal &&
                        current_box_type != UNKNOWN && ctx->goal_type_map[next_item_idx] != UNKNOWN &&
                        current_box_type != ctx->goal_type_map[next_item_idx])
                    {
                        temporary_goal = true;
                    }

                    // 空地不受类别约束；未完成目标只能被已知异类箱子临时占位。
                    if (active_goal && (!temporary_goal || (current_state->bomb_count == 0 &&
                                                            !can_reach_compatible_goal(ctx, current_state, current_box_type, (uint8_t)next_item_idx))))
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
            int n_idx = sokoban_neighbor_index(curr, i);
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

static int recon_direction_to_angle(uint8_t direction)
{
    // 与 sokoban_neighbor_index() 的方向编号保持一致：上、下、左、右分别对应 0、180、90、-90 度。
    static const int direction_angles[4] = {0, 180, 90, -90};
    return direction_angles[direction];
}

static uint8_t recon_angle_to_direction(int current_angle)
{
    // angle 可能经过多次累加，先规整到 [-180, 180] 再转成识别方向编号。
    while (current_angle > 180)
        current_angle -= 360;
    while (current_angle <= -180)
        current_angle += 360;

    if (current_angle == 180)
        return 1;
    if (current_angle == 90)
        return 2;
    if (current_angle == -90)
        return 3;
    return 0;
}

static bool select_recon_candidate(uint8_t start_pos, uint8_t current_direction,
                                   const ReconCandidate *candidates, uint8_t candidate_count,
                                   const uint8_t *obstacles, ReconCandidate *selected)
{
    // 第一层先用 BFS 求当前到每个观察候选的最短距离，只保留相对最近视点至多多走两格的候选。
    uint8_t current_distances[MAP_SIZE];
    build_car_dist_map(start_pos, obstacles, current_distances);

    uint8_t minimum_distance = UINT8_MAX;
    for (uint8_t i = 0; i < candidate_count; i++)
    {
        uint8_t distance = current_distances[candidates[i].pos];
        if (distance < minimum_distance)
            minimum_distance = distance;
    }
    if (minimum_distance == UINT8_MAX)
        return false;

    uint8_t best_rotations = UINT8_MAX;
    uint16_t best_total_distance = UINT16_MAX;
    uint8_t best_current_distance = UINT8_MAX;
    uint8_t best_index = UINT8_MAX;

    for (uint8_t i = 0; i < candidate_count; i++)
    {
        const ReconCandidate *candidate = &candidates[i];
        uint8_t current_distance = current_distances[candidate->pos];
        if (current_distance == UINT8_MAX ||
            current_distance > (uint8_t)(minimum_distance + RECON_PATH_SLACK))
        {
            continue;
        }

        uint8_t next_distances[MAP_SIZE];
        build_car_dist_map(candidate->pos, obstacles, next_distances);

        // 两步前瞻只假设当前目标识别成功；真实识别失败、推断或 UNKNOWN 后会回到外层重新规划。
        uint8_t next_minimum_distance = UINT8_MAX;
        for (uint8_t j = 0; j < candidate_count; j++)
        {
            if (candidates[j].target_info == candidate->target_info)
                continue;
            uint8_t distance = next_distances[candidates[j].pos];
            if (distance < next_minimum_distance)
                next_minimum_distance = distance;
        }

        uint8_t next_rotations = 0;
        uint8_t next_distance = 0;
        if (next_minimum_distance != UINT8_MAX)
        {
            // 下一步同样限定在最近距离 + RECON_PATH_SLACK 内，再挑转向次数最少的候选。
            next_rotations = UINT8_MAX;
            next_distance = UINT8_MAX;
            uint8_t next_position = UINT8_MAX;
            uint8_t next_target_info = UINT8_MAX;
            uint8_t next_direction = UINT8_MAX;

            for (uint8_t j = 0; j < candidate_count; j++)
            {
                const ReconCandidate *next_candidate = &candidates[j];
                if (next_candidate->target_info == candidate->target_info)
                    continue;

                uint8_t distance = next_distances[next_candidate->pos];
                if (distance == UINT8_MAX ||
                    distance > (uint8_t)(next_minimum_distance + RECON_PATH_SLACK))
                {
                    continue;
                }

                uint8_t rotations = (next_candidate->direction != candidate->direction) ? 1U : 0U;
                if (rotations < next_rotations ||
                    (rotations == next_rotations && distance < next_distance) ||
                    (rotations == next_rotations && distance == next_distance && next_candidate->pos < next_position) ||
                    (rotations == next_rotations && distance == next_distance && next_candidate->pos == next_position &&
                     next_candidate->target_info < next_target_info) ||
                    (rotations == next_rotations && distance == next_distance && next_candidate->pos == next_position &&
                     next_candidate->target_info == next_target_info && next_candidate->direction < next_direction))
                {
                    next_rotations = rotations;
                    next_distance = distance;
                    next_position = next_candidate->pos;
                    next_target_info = next_candidate->target_info;
                    next_direction = next_candidate->direction;
                }
            }
        }

        uint8_t total_rotations = (candidate->direction != current_direction ? 1U : 0U) + next_rotations;
        uint16_t total_distance = (uint16_t)current_distance + next_distance;
        // 排序优先级：两次识别的原地旋转次数、两段预测距离、当前距离、观察位置、目标索引、方向。
        if (total_rotations < best_rotations ||
            (total_rotations == best_rotations && total_distance < best_total_distance) ||
            (total_rotations == best_rotations && total_distance == best_total_distance && current_distance < best_current_distance) ||
            (total_rotations == best_rotations && total_distance == best_total_distance && current_distance == best_current_distance &&
             (best_index == UINT8_MAX || candidate->pos < candidates[best_index].pos)) ||
            (total_rotations == best_rotations && total_distance == best_total_distance && current_distance == best_current_distance &&
             best_index != UINT8_MAX && candidate->pos == candidates[best_index].pos &&
             candidate->target_info < candidates[best_index].target_info) ||
            (total_rotations == best_rotations && total_distance == best_total_distance && current_distance == best_current_distance &&
             best_index != UINT8_MAX && candidate->pos == candidates[best_index].pos &&
             candidate->target_info == candidates[best_index].target_info && candidate->direction < candidates[best_index].direction))
        {
            best_rotations = total_rotations;
            best_total_distance = total_distance;
            best_current_distance = current_distance;
            best_index = i;
        }
    }

    if (best_index == UINT8_MAX)
        return false;

    *selected = candidates[best_index];
    return true;
}

static inline bool recon_can_use_bomb(const SokobanContext *ctx, const State *state)
{
    if (ctx->redundant_tnt == 0 || state->bomb_count > ctx->initial_tnt_count)
        return false;
    uint8_t used_tnt = (uint8_t)(ctx->initial_tnt_count - state->bomb_count);
    return used_tnt < ctx->redundant_tnt;
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
            int n_idx = sokoban_neighbor_index(curr, i);
            if (n_idx < 0 || n_idx >= MAP_SIZE)
                continue;
            if (ctx->boundary_walls[n_idx])
                continue;
            uint16_t step_cost = 1;
            if (walls[n_idx])
            {
                if (!recon_can_use_bomb(ctx, state))
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
    if (get_nearest_path(current_state->car_pos, obs_points, obstacles, &temp_path) &&
        temp_path.length <= RECON_MAX_ROUTE_LENGTH)
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
        if (is_bomb && !recon_can_use_bomb(ctx, current_state))
            continue;

        for (int d = 0; d < 4; d++)
        {
            int next_item_idx = sokoban_neighbor_index(item_idx, d);
            int push_stand_idx = sokoban_neighbor_index(item_idx, d ^ 1);
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
                bool active_goal = goal_i != -1 && (current_state->active_goals_mask & (1U << goal_i));
                bool temporary_goal = false;
                if (active_goal)
                {
                    uint8_t goal_type = ctx->goal_type_map[next_item_idx];
                    if (can_consume_goal(current_box_type, goal_type))
                    {
                        consumed = true;
                    }
                    else if (current_box_type != UNKNOWN && goal_type != UNKNOWN && current_box_type != goal_type)
                    {
                        temporary_goal = true;
                    }
                }
                if (!consumed)
                {
                    // 空地不受类别约束；未完成目标只能被已知异类箱子临时占位，
                    // 且炸弹耗尽后还必须保留到兼容目标的反向推动可达性证明。
                    if (active_goal && (!temporary_goal || (current_state->bomb_count == 0 &&
                                                            !can_reach_compatible_goal(ctx, current_state, current_box_type,
                                                                                       (uint8_t)next_item_idx))))
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

void sokoban_solver_prepare_map(SokobanContext *ctx, const uint8_t *raw_map)
{
    hash_table_clear();
    engine_init(ctx, raw_map);
}

bool sokoban_solver_try_infer_identities(SokobanContext *ctx, State *current_state)
{
    return try_infer_identities(ctx, current_state);
}

bool sokoban_solver_select_recon_candidate(uint8_t start_pos, uint8_t current_direction,
                                           const ReconCandidate *candidates, uint8_t candidate_count,
                                           const uint8_t *obstacles, ReconCandidate *selected)
{
    return select_recon_candidate(start_pos, current_direction, candidates, candidate_count, obstacles, selected);
}

bool sokoban_solver_solve_recon(SokobanContext *ctx, State *start_state,
                                const bool *obs_points, const bool *virtual_obs_points)
{
    return solve_recon_ida(ctx, start_state, obs_points, virtual_obs_points);
}

int sokoban_recon_direction_to_angle(uint8_t direction)
{
    return recon_direction_to_angle(direction);
}

uint8_t sokoban_recon_angle_to_direction(int current_angle)
{
    return recon_angle_to_direction(current_angle);
}
