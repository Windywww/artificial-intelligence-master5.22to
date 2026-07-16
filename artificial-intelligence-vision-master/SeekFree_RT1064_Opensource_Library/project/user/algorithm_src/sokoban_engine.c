#include "sokoban_engine.h"
#include "sokoban_lut.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
#include "myUart.h"
#include "move_control.h"
#include "WIFI2SPI.h"

#define DEBUG_RECON 0              // 设为 0 可关闭海量打�?
#define MAX_ID 12                  // 支持最多id分类�?, 范围0~11
#define MAX_ALLOWABLE_NODES 700000 // 限制搜索节点总数
#ifndef SOKOBAN_CURRENT_WEIGHT
#define SOKOBAN_CURRENT_WEIGHT 2.0f
#endif
#ifndef SOKOBAN_MIN_WEIGHT
#define SOKOBAN_MIN_WEIGHT 2.0f
#endif
#ifndef IDA_THRESHOLD_STEP
#define IDA_THRESHOLD_STEP 16.0f
#endif
#ifndef MOVE_PENALTY
#define MOVE_PENALTY 10
#endif
#define UNKNOWN 11
#define ERROR 0.05f
int angle = 0;

typedef struct
{
    State next_state;
    MacroAction action;
    uint16_t next_g;
    uint16_t next_h;
} ChildNode;

// IDA* 返回的数据包
typedef struct
{
    float f;
    uint16_t g;
    uint16_t h;
} SearchRes;

static uint8_t current_hash_version = 0;

// 分离存储避免结构体对齐填充，总计 11 MiB�?
__attribute__((section(".bss.sdram"))) static uint64_t transposition_signatures[HASH_TABLE_SIZE];
__attribute__((section(".bss.sdram"))) static uint16_t transposition_g_scores[HASH_TABLE_SIZE];
__attribute__((section(".bss.sdram"))) static uint8_t transposition_versions[HASH_TABLE_SIZE];

// ChildNode �? 88 字节，池总计 1,196,800 字节（约 1.141 MiB）�?
//__attribute__((section(".ocram_data")))
__attribute__((section(".bss.sdram"))) static ChildNode all_children_pool[MAX_STEPS][MAX_BRANCHES];

// 声明================
static void get_smooth_path(SokobanContext *ctx, const WaypointPath *grid_path, const uint8_t *obstacles, WaypointPath *out_smooth_path);
static uint8_t is_deadlock(SokobanContext *ctx, uint8_t idx, State *state, bool is_bomb, const uint8_t *walls);
static void get_maze_distances(SokobanContext *ctx, const uint8_t *current_walls);
//==================

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
    // 遍历地图，抠除最外层的一圈死�?
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            uint8_t center_idx = y * WIDTH + x;
            uint8_t valid_target_count = 0;

            // 扫描 3x3 邻域
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT)
                        continue;
                    uint8_t n_idx = ny * WIDTH + nx;
                    // 地图最外圈上的墙是不可破坏边界墙，不加入爆炸范围�?
                    if (!ctx->boundary_walls[n_idx])
                    {
                        ctx->explosion_areas[center_idx][valid_target_count] = n_idx;
                        valid_target_count++;
                    }
                }
            }
            // 存入真实波及格子数，用于运行�? O(1) 控制循环次数
            ctx->explosion_area_count[center_idx] = valid_target_count;
        }
    }
}

// 清空哈希�?
static void hash_table_clear(void)
{
    current_hash_version++;
    // 如果溢出到了 0，才需要做一次彻底清�? (�? 255 轮迭代才发生一�?)
    if (current_hash_version == 0)
    {
        memset(transposition_versions, 0, sizeof(transposition_versions));
        current_hash_version = 1;
    }
}
// Zobrist 增量哈希�?
// ==========================================
static uint64_t zobrist_car[MAP_SIZE];
static uint64_t zobrist_box[MAX_ID][MAP_SIZE]; // 最�?12种箱子类�?
static uint64_t zobrist_bomb[MAP_SIZE];
static uint64_t zobrist_wall[MAP_SIZE];
static uint64_t zobrist_goal_mask[MAX_GOALS]; // 目标位图的每一�?
// 极速伪随机数生成器
static uint64_t xorshift64(uint64_t *state)
{
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    return *state;
}
// 引擎启动时初始化一次即�?
static void init_zobrist()
{
    uint64_t seed = 0x123456789ABCDEF0ULL; // 固定种子保证每次运行一�?
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
// 全量计算一次基�? Hash（仅在引擎初始化或侦察补全ID后调用）
static uint64_t compute_initial_base_hash(const State *state, const uint8_t *walls)
{
    uint64_t h = 0;
    for (int i = 0; i < state->box_count; i++)
        h ^= zobrist_box[state->boxes[i].id][state->boxes[i].pos];
    for (int i = 0; i < state->bomb_count; i++)
        h ^= zobrist_bomb[state->bombs[i]];
    for (int i = 0; i < MAP_SIZE; i++)
        if (walls[i])
            h ^= zobrist_wall[i]; // 改用传入�? walls
    for (int i = 0; i < MAX_GOALS; i++)
        if (state->active_goals_mask & (1U << i))
            h ^= zobrist_goal_mask[i];
    h ^= zobrist_car[state->car_pos];
    return h;
}
// 无锁、无循环�? O(1) 查重函数
static bool hash_table_insert_or_check(const State *state, int g_score, int tolerance)
{
    uint64_t sig = state->base_hash;
    uint32_t idx = sig & HASH_MASK;
    // 直接对位映射 (Direct Mapped)
    if (transposition_versions[idx] == current_hash_version &&
        transposition_signatures[idx] == sig)
    {
        if (transposition_g_scores[idx] <= g_score + tolerance)
        {
            return true; // 剪枝
        }
        else
        {
            transposition_g_scores[idx] = (uint16_t)g_score; // 更新更优代价
            return false;
        }
    }
    else
    {
        // 如果版本号对不上（说明是上一轮或很久以前的残余垃圾数据），直接无情覆盖！
        transposition_versions[idx] = current_hash_version;
        transposition_signatures[idx] = sig;
        transposition_g_scores[idx] = (uint16_t)g_score;
        return false;
    }
}

static void engine_init(SokobanContext *ctx, const uint8_t *raw_map)
{
    // 初始�? Zobrist �?
    init_zobrist();

    // 1. 基础尺寸配置
    ctx->goal_count = 0;
    ctx->map_valid = true;

    // 2. 预计算全局方向偏移�? (UP, DOWN, LEFT, RIGHT)
    // 这样�? BFS 中就可以彻底抛弃 (x, y) 坐标计算
    ctx->dir_offsets[0] = -WIDTH; // UP
    ctx->dir_offsets[1] = WIDTH;  // DOWN
    ctx->dir_offsets[2] = -1;     // LEFT
    ctx->dir_offsets[3] = 1;      // RIGHT

    // 3. 清空上下文缓�?
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

    // 4. 解析地图
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            int idx = y * WIDTH + x;
            uint8_t val = raw_map[idx];
            if (val == 1)
            { // 墙壁 (WALL)
                ctx->initial_walls[idx] = 1;
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
            { // 目标�? (GOAL)
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
            { // 炸弹 (BOMB)
                if (init_state->bomb_count < MAX_BOMBS)
                {
                    init_state->bombs[init_state->bomb_count++] = idx;
                }
                else
                {
                    // printf("[溢出]：地图炸弹数超过 MAX_BOMBS 上限！\n");
                }
            }
            else if (val == 5)
            { // 小车 (CAR)
                init_state->car_pos = idx;
            }
        }
    }
    // 生成初始目标位图 3个目标对�? ..0000111
    if (ctx->goal_count > 0)
    {
        init_state->active_goals_mask = (UINT32_C(1) << ctx->goal_count) - 1;
    }
    else
    {
        init_state->active_goals_mask = 0;
    }
    precalc_explosion_masks(ctx);

    // 标记优先轰炸墙。实测在某些带炸弹地图中会给算法带来质的飞跃�?
    if (init_state->bomb_count > 0)
    {
        uint8_t region_map[MAP_SIZE];
        memset(region_map, 0, sizeof(region_map));
        uint8_t current_region = 1;

        uint8_t queue[MAP_SIZE];
        int head = 0, tail = 0;

        // 阶段 A：划分全图空地连通域
        for (int i = 0; i < MAP_SIZE; i++)
        {
            // 如果是空地且未分配区�?
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
                        if (n_idx >= 0)
                        {
                            if (ctx->initial_walls[n_idx] == 0 && region_map[n_idx] == 0)
                            {
                                region_map[n_idx] = current_region;
                                queue[tail++] = n_idx;
                            }
                        }
                    }
                }
                current_region++; // 准备划分下一个孤立的房间/区域
            }
        }

        // 检测墙�? 3x3 邻域是否触及至少两个不同的连通域�?
        for (int i = 0; i < MAP_SIZE; i++)
        {
            if (ctx->initial_walls[i] == 1 && !ctx->boundary_walls[i])
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
                            uint8_t r = region_map[ny * WIDTH + nx];
                            if (r > 0)
                            {
                                if (flag == 0)
                                {
                                    flag = r;
                                }
                                else if (flag != r)
                                {
                                    ctx->initial_walls[i] = 2;
                                    done = true;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        // 将初始box周围死锁的标记为优先轰炸�?
        for (uint8_t i = 0; i < init_state->box_count; i++)
        {
            uint8_t box_idx = init_state->boxes[i].pos;
            if (is_deadlock(ctx, box_idx, init_state, false, ctx->initial_walls))
            {
                for (uint8_t j = 0; j < ctx->explosion_area_count[box_idx]; j++)
                {
                    uint8_t n_idx = ctx->explosion_areas[box_idx][j];
                    // 将死锁附近的墙标记为优先轰炸�? (2)�?
                    if (ctx->initial_walls[n_idx] == 1 && !ctx->boundary_walls[n_idx])
                    {
                        ctx->initial_walls[n_idx] = 2;
                    }
                }
            }
        }
    }
    get_maze_distances(ctx, ctx->initial_walls);
}

// ==========================================
// 辅助功能：极速静态最小堆 (替代 Python �? heapq)
// 用于 Dijkstra 的最小堆节点
typedef struct
{
    uint16_t dist;
    uint8_t pos;
} HeapNode;
// 静态最小堆结构
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

// 更新当前ctx的拉箱子距离表cached_dist_table �? cached_walls
static void get_maze_distances(SokobanContext *ctx, const uint8_t *current_walls)
{
    // ---------------------------------------------------------
    // 1. LRU Cache 机制�? C 语言实现
    // 因为 walls 只有在炸弹爆炸时才会改变，大部分时间它与上一步完全相�?
    if (ctx->cache_valid && memcmp(ctx->cached_walls, current_walls, MAP_SIZE) == 0)
    {
        return; // 缓存命中，直接返�?
    }
    // ---------------------------------------------------------
    // 2. 初始化距离表 (全部设为 INF)
    for (int g = 0; g < ctx->goal_count; g++)
    {
        for (int i = 0; i < MAP_SIZE; i++)
        {
            ctx->cached_dist_table[g][i] = INF_DIST;
        }
    }
    bool has_bombs = (ctx->initial_state.bomb_count > 0);
    int dx_arr[4] = {0, 0, -1, 1};
    int dy_arr[4] = {-1, 1, 0, 0};
    // --------------------------------------------------------
    // 3. 用优先队列对每一个目标点执行 Dijkstra (逆推物理站位)
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
                continue; // 如果取出的节点距离大于已记录的最短距离，直接丢弃
            int cx = curr % WIDTH;
            int cy = curr / WIDTH;
            for (int i = 0; i < 4; i++)
            {
                int dx = dx_arr[i];
                int dy = dy_arr[i];
                // 假设把箱子从 (px, py) 拉到 (cx, cy)
                int px = cx - dx;
                int py = cy - dy;
                int ppx = px - dx;
                int ppy = py - dy;
                // 越界防线
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
                        continue; // 没有炸弹，禁�?
                    // 穿墙惩罚 (优先轰炸墙免�?)
                    if (is_p_wall && current_walls[p_idx] != 2)
                        step_cost += VIRTUAL_WALL_COST;
                    if (is_pp_wall && current_walls[pp_idx] != 2)
                        step_cost += VIRTUAL_WALL_COST;
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
    ctx->cache_valid = true;
}

// 优化：单�? BFS 构建当前状态下小车的全局距离�?
static void build_car_dist_map(uint8_t start_pos, const uint8_t *obstacles, uint8_t *dist_map)
{
    // 初始化距离场
    for (uint8_t i = 0; i < MAP_SIZE; i++)
    {
        dist_map[i] = UINT8_MAX;
    }
    dist_map[start_pos] = 0; // 起点距离�? 0

    // 静态栈上分配，极速队�?
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
            // 只有当不是障碍物，且从未被访问过（距离仍�? INF）时才入�?
            if (n_idx >= 0 && !obstacles[n_idx] && dist_map[n_idx] == UINT8_MAX)
            {
                dist_map[n_idx] = current_d + 1;
                queue[tail++] = n_idx;
            }
        }
    }
}
// O(N^3) 匈牙利算�? (KM算法求最小权完美匹配)
static void solve_assignment_km(int cost_matrix[MAX_BOXES][MAX_GOALS], int num_items,
                                int best_assignment[MAX_BOXES], int *best_cost)
{
    int u[MAX_BOXES + 1] = {0};   // 左侧顶点的顶�?
    int v[MAX_GOALS + 1] = {0};   // 右侧顶点的顶�?
    int p[MAX_GOALS + 1] = {0};   // 记录右侧顶点匹配到的左侧顶点
    int way[MAX_GOALS + 1] = {0}; // 记录增广�?

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
                    // 注意：算法内部索引从 1 开始，对应 matrix 需�? -1
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

    *best_cost = -v[0]; // 最小总代价就�? v[0] 的相反数

    // 导出最优分配方�?
    for (int j = 1; j <= num_items; j++)
    {
        if (p[j] > 0)
        {
            best_assignment[p[j] - 1] = j - 1;
        }
    }
}

static int calc_heuristic(SokobanContext *ctx, State *state, const uint8_t *walls)
{
    // 已经胜利，代价为 0
    if (state->box_count == 0)
        return 0;
    // 1. 获取物理拉拽距离�? (依赖自带�? LRU 缓存，耗时极低)
    get_maze_distances(ctx, walls);

    // 2. 构建代价矩阵 (Cost Matrix)
    int cost_matrix[MAX_BOXES][MAX_GOALS];
    uint8_t active_goal_indices[MAX_GOALS];
    uint8_t active_goal_count = 0;
    for (int g = 0; g < ctx->goal_count; g++)
    {
        if (state->active_goals_mask & (1U << g))
            active_goal_indices[active_goal_count++] = (uint8_t)g;
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
    // 3. 求解最小权二分图匹�?
    int best_cost = INF_DIST;
    int best_assignment[MAX_BOXES] = {0};
    solve_assignment_km(cost_matrix, state->box_count, best_assignment, &best_cost);

    // 如果算出来的代价是正无穷，说明无论怎么分配，必然有箱子进不了目�? (死局)
    if (best_cost >= INF_DIST)
        return INF_DIST;
    int base_h = best_cost;
    // 线性冲突惩�? (Linear Conflict) - 解决“幽灵箱子”穿透问�?
    int conflict_penalty = 0;
    // 穷举两两箱子的配对关�?
    for (int i = 0; i < state->box_count; i++)
    {
        for (int j = i + 1; j < state->box_count; j++)
        {
            uint8_t b1_idx = state->boxes[i].pos;
            uint8_t g1_idx = ctx->goals[active_goal_indices[best_assignment[i]]].pos;
            uint8_t b2_idx = state->boxes[j].pos;
            uint8_t g2_idx = ctx->goals[active_goal_indices[best_assignment[j]]].pos;
            // 一维转二维
            int b1_x = b1_idx % WIDTH;
            int b1_y = b1_idx / WIDTH;
            int g1_x = g1_idx % WIDTH;
            int g1_y = g1_idx / WIDTH;

            int b2_x = b2_idx % WIDTH;
            int b2_y = b2_idx / WIDTH;
            int g2_x = g2_idx % WIDTH;
            int g2_y = g2_idx / WIDTH;

            // --- 行冲�? (Row Conflict) ---
            if (b1_y == b2_y && g1_y == g2_y && b1_y == g1_y)
            {
                // 判断是否在X轴上交叉
                if ((b1_x - b2_x) * (g1_x - g2_x) < 0)
                {
                    conflict_penalty += 4; // 避让错车开销
                }
            }
            // --- 列冲�? (Column Conflict) ---
            else if (b1_x == b2_x && g1_x == g2_x && b1_x == g1_x)
            {
                // 判断是否在Y轴上交叉
                if ((b1_y - b2_y) * (g1_y - g2_y) < 0)
                {
                    conflict_penalty += 4;
                }
            }
        }
    }
    return base_h + conflict_penalty;
}

// 极速获取单个格子的 2-bit 状�? (0=�?, 1=内墙, 2=box/边界�?, 3=tnt)
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
    // �? 8 个邻居的状态压缩成一�? 16-bit 的环境变�? (Environment Mask)
    // 布局: [BR][B][BL][R][L][TR][T][TL] -> 每个�? 2 bits
    uint16_t env = 0;
    env |= (get_relative_cell_state(ctx, state, walls, idx, -1, -1) << 0);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 0, -1) << 2);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 1, -1) << 4);
    env |= (get_relative_cell_state(ctx, state, walls, idx, -1, 0) << 6);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 1, 0) << 8);
    env |= (get_relative_cell_state(ctx, state, walls, idx, -1, 1) << 10);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 0, 1) << 12);
    env |= (get_relative_cell_state(ctx, state, walls, idx, 1, 1) << 14);

    // LUT查表：获取打破当前环境所需的最�? TNT 数量
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

// 子节点仅保存 g/h，排序时即时计算 f，避免每个节点额外保存一�? float�?
static inline void sort_children(const ChildNode *children, uint8_t count, uint8_t *indices, float weight)
{
    for (int i = 1; i < count; i++)
    {
        if (child_after(&children[indices[i - 1]], &children[indices[i]], weight))
        {
            // 交换索引
            uint8_t tmp_idx = indices[i];
            indices[i] = indices[i - 1];
            int j;
            for (j = i - 2; j >= 0 && child_after(&children[indices[j]], &children[tmp_idx], weight); j--)
                indices[j + 1] = indices[j];
            indices[j + 1] = tmp_idx;
        }
    }
}

static SearchRes dfs_ida(SokobanContext *ctx, State *current_state, const uint8_t *current_walls, uint16_t current_g, int current_h, float threshold, MacroAction *acts, uint8_t act_len)
{
    ctx->total_explored_nodes++; // 性能统计
    // 安全防线：防止内存池越界�?
    if (act_len >= MAX_STEPS)
    {
        return (SearchRes){RES_INF, 0, 0};
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
    // 成功判断 (所有箱子被消除/归位)
    if (current_state->box_count == 0)
    {
        // 保存最优路径到 ctx->solution_actions
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
    // 更新当前ctx拉箱子safe_mask和距离表
    get_maze_distances(ctx, current_walls);
    // 在进入循环前，一次性构建全图的障碍�? Bitmap
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

    // ? 核心优化：在当前节点，只执行一次全�? BFS，获取小车到所有点的真实距�?
    uint8_t car_dist_map[MAP_SIZE];
    build_car_dist_map(current_state->car_pos, obstacles, car_dist_map);

    // 展开所有可推物�?
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
            // 碰撞剪枝
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
                    continue; // 箱子撞墙，非�?
                }
            }
            else if (!is_bomb)
            {
                // 验证目标消除匹配
                int8_t goal_i = ctx->goal_mask_map[next_item_idx];
                if (goal_i != -1 && (current_state->active_goals_mask & (1U << goal_i)) &&
                    ctx->goal_type_map[next_item_idx] == current_box_type)
                {
                    consumed = true; // 触发消除
                }
                else
                { // 动�? Safe Mask 剪枝 (基于 cached_dist_table)
                    // 检查箱子能否到达至少一个同颜色的终�?
                    bool is_safe = false;
                    for (int g = 0; g < ctx->goal_count; g++)
                    {
                        if (current_state->active_goals_mask & (1U << g))
                        {
                            // 检查颜色是否匹�?
                            if (ctx->goals[g].id == current_box_type || current_box_type == NO_CLS)
                            {
                                // 只要不是正无穷，说明这步棋是活的�?
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
            // ? ============ 修改�? (瞬间 O(1) 获取) ============
            int car_dist = car_dist_map[push_stand_idx];
            if (car_dist == UINT8_MAX)
                continue;

            // 生成新代�?
            int step_cost = car_dist + 1;
            // 若引发了爆炸，且被撞击的墙不是优先轰炸墙(2)，才增加惩罚
            if (exploded && current_walls[next_item_idx] != 2)
            {
                step_cost += BOMB_PENALTY;
            }
            int next_g = current_g + step_cost;

            // 创建新state
            State next_state = *current_state; // C 语言极致性能�?64字节结构体浅拷贝仅需数个周期
            // ?Zobrist 增量：更新小车位置的哈希
            next_state.base_hash ^= zobrist_car[current_state->car_pos]; // 扣除老位置的特征
            next_state.base_hash ^= zobrist_car[item_idx];

            next_state.car_pos = item_idx; // 小车顶上

            const uint8_t *walls_for_eval = current_walls; // 默认零拷�?
            uint8_t temp_walls[MAP_SIZE];                  // 栈上临时数组，阅后即�?
            if (exploded)
            {
                next_state.base_hash ^= zobrist_bomb[item_idx];
                // 移除炸弹
                for (int k = 0; k < next_state.bomb_count; k++)
                {
                    if (next_state.bombs[k] == item_idx)
                    {
                        next_state.bombs[k] = next_state.bombs[--next_state.bomb_count]; // 将最后一个炸弹移到被移除炸弹的位�?
                        break;
                    }
                }
                // COW 阶段 1：只为了计算 h 值的临时拷贝
                memcpy(temp_walls, current_walls, MAP_SIZE);
                int exp_count = ctx->explosion_area_count[next_item_idx];
                for (int e = 0; e < exp_count; e++)
                {
                    uint8_t w_idx = ctx->explosion_areas[next_item_idx][e];
                    if (temp_walls[w_idx])
                    {
                        temp_walls[w_idx] = 0;
                        next_state.base_hash ^= zobrist_wall[w_idx]; // 补上这句，防止不同地形产生相同指�?
                    }
                }
                walls_for_eval = temp_walls; // 让启发函数去读这张新�?
            }
            else if (consumed)
            {
                // 【Zobrist 增量】：扣除被消除的箱子和目标点
                next_state.base_hash ^= zobrist_box[current_box_type][item_idx];
                next_state.base_hash ^= zobrist_goal_mask[ctx->goal_mask_map[next_item_idx]];
                // 移除被消除的箱子
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
                // 常规平移，执行死锁预�?
                // 先更新坐�?
                if (is_bomb)
                {
                    // 【Zobrist 增量】：一出一�?
                    next_state.base_hash ^= zobrist_bomb[item_idx];      // 扣除旧坐�?
                    next_state.base_hash ^= zobrist_bomb[next_item_idx]; // 加上新坐�?
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

            // 6. 哈希查重 (现在已经是彻底的 O(1) 速度了！)
            if (hash_table_insert_or_check(&next_state, next_g, 0))
            {
                continue;
            }

            // 7. 生成启发式与入列
            int next_h = calc_heuristic(ctx, &next_state, walls_for_eval);
            if (next_h >= INF_DIST)
                continue;
            // 【新增】防止由于意外情况导致的子节点数超过池子容量
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

    // 节点排序
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
        // 如果连最优质的节点也超过阈值，直接跳出
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
        // COW 阶段 2：为向下递归准备真正安全的栈上墙�?
        const uint8_t *walls_to_pass = current_walls; // 默认情况：完美共享上一层的墙壁（零拷贝�?
        uint8_t recurse_walls[MAP_SIZE];

        if (sorted_child->action.is_explode)
        {
            // 如果这个子节点是一个爆炸动作，我们就在当前 for 循环的栈空间内开辟新墙壁
            memcpy(recurse_walls, current_walls, MAP_SIZE);

            // 推演爆炸落点
            int push_dir = sorted_child->action.push_to - sorted_child->action.move_to;
            uint8_t next_pos = sorted_child->action.push_to + push_dir;

            // 拆墙
            uint8_t exp_count = ctx->explosion_area_count[next_pos];
            for (uint8_t e = 0; e < exp_count; e++)
            {
                recurse_walls[ctx->explosion_areas[next_pos][e]] = 0;
            }
            walls_to_pass = recurse_walls; // 让下一层递归指向这张新图
        }
        SearchRes res = dfs_ida(ctx, &sorted_child->next_state, walls_to_pass, sorted_child->next_g, sorted_child->next_h, threshold, acts, act_len + 1);

        if (res.f == RES_SUCCESS)
            return res;
        if (res.f < min_node_data.f)
        {
            min_node_data = res;
        }
    }
    return min_node_data;
}
// =====================================================================
// 智能推理引擎：根据已识别的配对关系，推断剩余 NO_CLS 的身�? (O(N) 复杂�?)
// 返回值：本轮是否成功推断出至少一个实�?
static bool try_infer_identities(SokobanContext *ctx, State *current_state)
{
    bool inferred_something = false;

    uint8_t box_counts[MAX_ID] = {0};
    uint8_t goal_counts[MAX_ID] = {0};
    uint8_t unk_box_count = 0;
    uint8_t unk_goal_count = 0;

    // 统计当前所有的已知 ID (直方图统�?)
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

    // 推理未知的箱�?
    if (unk_box_count > 0)
    {
        uint8_t total_box_deficit = 0;
        uint8_t deficit_id = 0;
        uint8_t distinct_deficit_ids = 0;

        for (uint8_t id = 0; id < MAX_ID; id++)
        {
            if (id == UNKNOWN)
                continue;
            // 目标需要这�? ID，但箱子不够，说明差额就在未知箱子里�?
            if (goal_counts[id] > box_counts[id])
            {
                total_box_deficit += (goal_counts[id] - box_counts[id]);
                deficit_id = id;
                distinct_deficit_ids++;
            }
        }
        // 核心破案条件：goal比box多的数量 刚好等于 地图上未知箱子的总数，并且只缺失同一种编�?!!
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
    // 推理未知目的�?
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
// 侦察专用 BFS：寻找离小车最近的任意一个“视点�?
static bool get_nearest_path(SokobanContext *ctx, uint8_t start_pos, const bool *obs_points, const uint8_t *obstacles, WaypointPath *out_path)
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
    // 回溯路径
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

// ida*识图启发函数：用 Dijkstra估算从当前位置到达任意视点的代价
static int calc_recon_heuristic(SokobanContext *ctx, State *state, const bool *obs_points, const uint8_t *walls)
{
    MinHeap pq;
    pq.size = 0;
    uint8_t movable[MAP_SIZE]; // 可移动物�?
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
    // 从小车当前位置向外扩�?
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
                if (walls[n_idx] != 2)
                {
                    step_cost = BOMB_PENALTY; // 非优先墙才使用炸弹代�?
                }
            }
            else if (movable[n_idx] == 1)
                step_cost = MOVE_PENALTY; // 移动物体代价
            if (dist[curr] + step_cost < dist[n_idx])
            {
                dist[n_idx] = dist[curr] + step_cost;
                heap_push(&pq, dist[n_idx], n_idx);
            }
        }
    }
    return INF_DIST; // 彻底死局，视点被死墙封死
}
// ida*ʶͼѰ·

static SearchRes dfs_ida_recon(SokobanContext *ctx, State *current_state, const uint8_t *current_walls, uint16_t current_g, int current_h,
                               float threshold, MacroAction *acts, uint8_t act_len, const bool *obs_points, const bool *virtual_obs_points)
{
    ctx->total_explored_nodes++;
    if (act_len >= MAX_STEPS)
        return (SearchRes){RES_INF, 0, 0};
    float f_score = current_g + current_h;
    if (f_score > threshold)
        return (SearchRes){f_score, current_g, current_h};

    get_maze_distances(ctx, current_walls);
    uint8_t obstacles[MAP_SIZE];
    memcpy(obstacles, current_walls, MAP_SIZE);
    for (int i = 0; i < current_state->box_count; i++)
        obstacles[current_state->boxes[i].pos] = 1;
    for (int i = 0; i < current_state->bomb_count; i++)
        obstacles[current_state->bombs[i]] = 1;

    WaypointPath temp_path;
    if (get_nearest_path(ctx, current_state->car_pos, obs_points, obstacles, &temp_path))
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
                continue;
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
                            // ? 修复点应用：
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
            if (exploded && current_walls[next_item_idx] != 2)
                step_cost += BOMB_PENALTY;
            int next_g = current_g + step_cost;

            // ================= 物理推演与哈希同步更�? =================
            State next_state = *current_state;

            // ? 修复1：小车位移的哈希同步
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
                // ? 修复2：炸弹引爆的哈希同步 (移除炸弹)
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
                        // ? 修复3：墙壁被炸毁的哈希同�? (极其关键，防止不同地形产生相同哈�?)
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
                // ? 修复4：箱子消除的哈希同步 (移除箱子 + 修改目标掩码)
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
                    // ? 修复5：炸弹平移的哈希同步
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
                    // ? 修复6：箱子平移的哈希同步
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

            // 视点激活逻辑与后续代码保持不�?
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
            if (next_h >= INF_DIST)
                continue;
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

            if (res.f == RES_SUCCESS)
                return res;
            if (res.f < min_node_data.f)
                min_node_data = res;
        }
    }
    return min_node_data;
}

// 封装ida*识图寻路启动�?
static bool solve_recon_ida(SokobanContext *ctx, State *start_state, const bool *obs_points, const bool *virtual_obs_points)
{
    ctx->total_explored_nodes = 0;
    ctx->solution_actions_len = 0;
    // ? 进入搜索前，计算初始绝对特征�?
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
        if (res.f >= RES_INF)
        {
            // printf("识别 IDA* 阈值达�? RES_INF, no solution found.\n");
            return false;
        }
        threshold = res.f;
        if (ctx->total_explored_nodes > MAX_ALLOWABLE_NODES)
        {
            // printf("识别 IDA* exceeded node limit, aborting search.\n");
            return false;
        }
    }
}
// 获取地图所有信�?
// cls :0=无分�?  1=分类 2=未知
void build_map_info(SokobanContext *ctx, const uint8_t *raw_map, uint8_t cls)
{
    hash_table_clear();
    engine_init(ctx, raw_map); // 解析地图，初始化状态，构建边界墙等预处�?
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
    uint8_t failed[MAP_SIZE];      // 记录识别失败的位�?
    memset(failed, 255, MAP_SIZE); // 255 在补码中就是 -1，完美覆盖每一个字�?
    // 核心状态机循环
    // 逻辑：先找完所有能到的未知实体，剩下不连通的实体用ida*破障寻路
    bool is_first = (cls == 2) ? true : false;

    while (unid_boxes > 0 || unid_goals > 0)
    {
        // 构建当前障碍物位�?
        uint8_t obstacles[MAP_SIZE];
        uint8_t the_goals[MAP_SIZE]; // 仅用于判定视点不能在goal�?
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

        // 标定当前所有需要被侦察的“视点�? (紧挨着未知目标�? 4 个格�?)
        bool observation_points[MAP_SIZE] = {false};
        bool virtual_obs_points[MAP_SIZE] = {false}; // 虚拟视点, 给IDA*用的
        uint8_t target_map[MAP_SIZE];                // 记录某个视点对应的是哪个实体 (最高位存类型，�?7位存索引)
        memset(target_map, 0, sizeof(target_map));
        // 标定未知终点
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
                        if (n >= 0 && !the_goals[n])
                        {
                            virtual_obs_points[n] = true; // 虚拟视点, 给IDA*用的
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
        // 标定未知箱子
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
                        if (n >= 0 && !the_goals[n])
                        {
                            virtual_obs_points[n] = true; // 虚拟视点, 给IDA*用的
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

        WaypointPath path;
        WaypointPath smooth_path;
        // 常规寻路：寻找最近视�?
        if (get_nearest_path(ctx, current_state->car_pos, observation_points, obstacles, &path))
        {
            uint8_t final_pos = path.points[path.length - 1];
            uint8_t target_info = target_map[final_pos];
            bool is_box = (target_info >> 7) == 1;
            uint8_t index = target_info & 0b01111111;
            uint8_t entity_pos = is_box ? current_state->boxes[index].pos : ctx->goals[index].pos;
            get_smooth_path(ctx, &path, obstacles, &smooth_path); // 对路径平滑处�?
            current_state->car_pos = final_pos;
            // 上下左右 
            int8_t dd = entity_pos - final_pos;
            if (dd == -16)
                dd = 0;
            else if (dd == 1)
                dd = 1;
            else if (dd == 16)
                dd = 2;
            else if (dd == -1)
                dd = 3; // �??
            smooth_path.length--;
            car_move(&smooth_path, angle, 0);
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

            float final_actual_x = final_pos_X * 0.2 + 0.1;
            float final_actual_y = 2.4f - final_pos_Y * 0.2 - 0.1;

            float back_error = 0.04;
            if (dx > 0)
            {
                final_actual_x -= back_error;
            }
            else if (dx < 0)
            {
                final_actual_x += back_error;
            }
            else if (dy > 0)
            {
                final_actual_y += back_error;
            }
            else if (dy < 0)
            {
                final_actual_y -= back_error;
            }
            car_move_point(final_actual_x, final_actual_y, angle, 0);
            while (navigate_flag)
            {
                wifi_task();
            }
            // ?????????
            if (dx > 0)
            {
                angle = -90;
            }
            else if (dx < 0)
            {
                angle = 90;
            }
            else if (dy > 0)
            {
                angle = 180;
            }
            else if (dy < 0)
            {
                angle = 0;
            }
            system_delay_ms(200);
            car_turn(angle);
            while (!yaw_arrived_flag)
            {
                wifi_task();
            }

            check_image(3 - is_box, 1);
            final_image_index = UNKNOWN;

            // ????????carmove??????????��??
            vision_angle_switch = 0;
            while (final_image_index == UNKNOWN)
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
                while (navigate_flag)
                {
                    wifi_task();
                }
            } 

            vision_angle_switch = 0;
            // system_delay_ms(700);
            system_delay_ms(200);

            uint8_t recognized_id = NO_CLS;
            recognized_id = final_image_index;

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
                failed[final_pos] = entity_pos; // 后续不在final_pos识别entity_pos
                continue;
            }
            if (is_first)
            {
                is_first = false;
            }
            // 更新状�?
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
            // 每次更新id后尝试推断剩余未知实体的身份
            // while (try_infer_identities(ctx, current_state))
            // {
            //     // 每推断出一个，更新未知计数
            //     unid_boxes = 0;
            //     unid_goals = 0;
            //     for (int k = 0; k < current_state->box_count; k++)
            //     {
            //         if (current_state->boxes[k].id == UNKNOWN)
            //             unid_boxes++;
            //     }
            //     for (int k = 0; k < ctx->goal_count; k++)
            //     {
            //         if (ctx->goals[k].id == UNKNOWN)
            //             unid_goals++;
            //     }
            // }
        }
        else
        {
            if (solve_recon_ida(ctx, current_state, observation_points, virtual_obs_points))
            {
                generate_path(ctx, &smooth_path);
                current_state = &ctx->initial_state;
                //硬件接口预留：将 smooth_path 发送给电机
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
    // 1. 直方图统计所有已识别�? ID 频次
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
    // 2. 核心比对：从 1 开始查（跳�? NO_CLS(0) �? UNKNOWN(11)�?
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
        // printf("\n[视觉纠错系统]: 发现单点独立偏移！箱子孤�? ID=%d，终点孤�? ID=%d\n", err_box_id, err_goal_id);
        // printf(" -> 自动将二者退化为 NO_CLS (0) 万能匹配模式！\n");
        //  修正出错的那个箱�?
        for (int i = 0; i < current_state->box_count; i++)
        {
            if (current_state->boxes[i].id == err_box_id)
            {
                current_state->boxes[i].id = NO_CLS;
                break;
            }
        }
        // 修正出错的那个目标点
        for (int i = 0; i < ctx->goal_count; i++)
        {
            if (ctx->goals[i].id == err_goal_id)
            {
                ctx->goals[i].id = NO_CLS;
                ctx->goal_type_map[ctx->goals[i].pos] = NO_CLS; // 同步 O(1) 映射�?
                break;
            }
        }
    }
}
// 引擎主入口：执行 IDA* 搜索
// 返回值：true 表示找到最优解，false 表示无解或超�?/超节点限�?
bool solve(SokobanContext *ctx)
{
    if (!ctx->map_valid || ctx->initial_state.box_count != ctx->goal_count)
        return false;
    // 引擎运行状态与性能统计初始�?
    ctx->total_explored_nodes = 0;
    ctx->solution_actions_len = 0;
    // ? 进入搜索前，计算初始绝对特征�?
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
    // 设置 IDA* 初始阈�?
    float threshold = initial_h * ctx->current_weight;
    MacroAction acts[MAX_STEPS];
    // 核心外层循环
    while (true)
    {
        iteration += 1;
        hash_table_clear();
        // 将初始状态加入哈希表
        hash_table_insert_or_check(&ctx->initial_state, 0, 0);

        SearchRes res = dfs_ida(ctx, &ctx->initial_state, ctx->initial_walls, 0, initial_h, threshold, acts, 0);
        if (res.f == RES_SUCCESS)
        {
            return true;
        }
        if (res.f >= RES_INF)
        {
            return false;
        }
        float min_f = res.f;
        int min_g = res.g;
        int min_h = res.h;
        // 衰减判定：渐进二分衰减法
        if (iteration % patience_limit == 0 && ctx->current_weight > ctx->min_weight)
        {
            // 每次砍掉当前值与保底值之间一半的差距
            ctx->current_weight = ctx->min_weight + (ctx->current_weight - ctx->min_weight) * 0.5f;

            // 如果因为精度问题导致逼近不动了，强制贴底
            if (ctx->current_weight - ctx->min_weight < 0.2f)
            {
                ctx->current_weight = ctx->min_weight;
            }
            // 根据新的权重重新计算当前边界
            threshold = (float)min_g + ctx->current_weight * (float)min_h;
            patience_limit += patience_limit; // 耐心值翻�?
            iteration = 0;                    // 重置迭代计数�?
            continue;
        }

        threshold = MAX(min_f, threshold + IDA_THRESHOLD_STEP);
        // 限制搜索节点总数
        if (ctx->total_explored_nodes > MAX_ALLOWABLE_NODES)
        {
            return false;
        }
    }
    return false; // 理论上不会走到这�?
}
static bool get_micro_path(SokobanContext *ctx, uint8_t start_pos, uint8_t target_pos, const uint8_t *obstacles, WaypointPath *out_path)
{
    // 提早退出：如果小车已经在发力点，直接原地待�?
    if (start_pos == target_pos)
    {
        out_path->points[0] = start_pos;
        out_path->length = 1;
        return true;
    }
    // 终点被挡
    if (obstacles[target_pos])
    {
        return false;
    }
    // 静态栈内存分配
    uint8_t queue[MAP_SIZE];
    uint8_t parent[MAP_SIZE];
    bool visited[MAP_SIZE];
    memset(visited, 0, sizeof(visited)); // 必须清零，因为栈内存复用会有垃圾数据�?

    int head = 0;
    int tail = 0;
    // 起点入队
    queue[tail++] = start_pos;
    visited[start_pos] = true;
    parent[start_pos] = start_pos; // 起点的父节点是自�?

    bool found = false;
    // BFS 洪水填充
    while (head < tail)
    {
        uint8_t curr = queue[head++];
        if (curr == target_pos)
        {
            found = true;
            break;
        }
        // 展开 4 个方�?
        for (int i = 0; i < 4; i++)
        {
            // 致命级防雷：必须强转�? (int)，防止第一行网上偏�? (-16) �? uint8 下溢变成 240 越界�?
            int n_idx = neighbor_index(curr, i);
            if (n_idx < 0 || n_idx >= MAP_SIZE)
                continue;
            // 障碍物和访问判重
            if (!obstacles[n_idx] && !visited[n_idx])
            {
                visited[n_idx] = true;
                parent[n_idx] = curr; // 记录父节点以便回�?
                queue[tail++] = n_idx;
            }
        }
    }
    // 如果把图搜遍了都没找到终点，说明路被封死�?
    if (!found)
        return false;
    // 路径回溯
    uint8_t temp_path[MAP_SIZE];
    int count = 0;
    uint8_t curr = target_pos;
    while (curr != start_pos)
    {
        temp_path[count++] = curr;
        curr = parent[curr];
    }
    temp_path[count++] = start_pos; // 最后把起点也塞进去
    // 翻转路径为正�?
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
        // 长度小于等于2，不需要平�?
        *out_smooth_path = *grid_path;
        return;
    }
    out_smooth_path->points[0] = grid_path->points[0];
    out_smooth_path->length = 1;
    int current_idx = 0; // 当前锚点在原始路径中的索�?

    while (current_idx < grid_path->length - 1)
    {
        int furthest_visible = current_idx + 1;
        // 贪心算法：从终点开始往回找，找到第一个能直线看到当前点的节点
        for (int next = grid_path->length - 1; next > current_idx; next--)
        {
            if (pass(grid_path->points[current_idx], grid_path->points[next], ERROR, obstacles))
            {
                furthest_visible = next;
                break;
            }
        }
        // 把找到的最远节点加入平滑路径，并将锚点移动到该节点
        out_smooth_path->points[out_smooth_path->length++] = grid_path->points[furthest_visible];
        current_idx = furthest_visible;
    }
}
// 最终总路径后处理
static void get_final_path(SokobanContext *ctx, WaypointPath *path)
{
    if (path->length <= 2)
    {
        return;
    }
    // 第一步：数据净�? 剔除重复�?
    uint8_t unique_points[MAP_SIZE];
    int unique_len = 0;
    unique_points[unique_len++] = path->points[0];
    for (int i = 1; i < path->length; i++)
    {
        // 只有当前点与上一个写入的点不同，才允许写�?
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

    // 第二步：轨迹压缩  通过向量剔除同一直线冗余�?
    uint8_t new_points[MAP_SIZE];
    int new_len = 0;
    new_points[new_len++] = unique_points[0];
    // 遍历中间�?
    for (int i = 1; i < unique_len - 1; i++)
    {
        int p = unique_points[i - 1]; // 前一个点
        int c = unique_points[i];     // 当前�?
        int n = unique_points[i + 1]; // 后一个点

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
        // 判断前后两段线段是否同向且共�?
        bool is_horizontal = (dy1 == 0 && dy2 == 0 && (dx1 * dx2 > 0));
        bool is_vertical = (dx1 == 0 && dx2 == 0 && (dy1 * dy2 > 0));
        if (!is_horizontal && !is_vertical)
        {
            new_points[new_len++] = c;
        }
    }
    new_points[new_len++] = unique_points[unique_len - 1];
    // 写回原结构体
    path->length = new_len;
    for (int i = 0; i < new_len; i++)
    {
        path->points[i] = new_points[i];
    }
}
// 由宏动作推演轨迹 并更新ctx->initial_state和ctx->initial_walls为最终状�?
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
        // 步骤 A：构建当前时刻的障碍物位�?
        memset(obstacles, 0, sizeof(obstacles));
        // 填入�?
        for (int k = 0; k < MAP_SIZE; k++)
        {
            if (sim_walls[k])
                obstacles[k] = 1;
        }
        // 填入箱子
        for (int k = 0; k < sim_state.box_count; k++)
        {
            obstacles[sim_state.boxes[k].pos] = 1;
        }
        // 填入炸弹
        for (int k = 0; k < sim_state.bomb_count; k++)
        {
            obstacles[sim_state.bombs[k]] = 1;
        }
        // 步骤 B：执行底盘寻路BFS
        // 注意：小车的当前位置�? sim_state.car_pos，目标是 act.move_to
        if (!get_micro_path(ctx, sim_state.car_pos, act.move_to, obstacles, &micro_path))
        {
            return;
        }
        get_smooth_path(ctx, &micro_path, obstacles, &smooth_path);
        out_full_path->length += smooth_path.length;
        for (int p = 0; p < smooth_path.length; p++)
        {
            out_full_path->points[out_full_path->length - smooth_path.length + p] = smooth_path.points[p];
        }
        out_full_path->points[out_full_path->length++] = act.push_to; // 确保终点也在路径�?

        // 更新下一帧地�?
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
            // 常规平移更新坐标
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

    ctx->initial_state = sim_state;
    memcpy(ctx->initial_walls, sim_walls, MAP_SIZE);
    get_final_path(ctx, out_full_path); // 对整条路径进行最终的优化处理
}

uint8_t check_obstacle(SokobanContext *ctx, uint8_t grid_index)
{
    // 0. 越界保护
    if (grid_index >= MAP_SIZE)
    {
        return 1;
    }

    if (ctx->initial_walls[grid_index] >= 1)
    {
        return 1;
    }

    for (int i = 0; i < ctx->initial_state.box_count; i++)
    {
        if (ctx->initial_state.boxes[i].pos == grid_index)
        {
            return 1;
        }
    }

    for (int i = 0; i < ctx->initial_state.bomb_count; i++)
    {
        if (ctx->initial_state.bombs[i] == grid_index)
        {
            return 1;
        }
    }

    return 0;
}
