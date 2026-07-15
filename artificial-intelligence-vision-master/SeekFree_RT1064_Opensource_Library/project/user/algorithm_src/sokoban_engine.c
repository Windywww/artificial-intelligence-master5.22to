#include "sokoban_engine.h"
#include "sokoban_lut.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
#include "myUart.h"
#include "move_control.h"
#include "WIFI2SPI.h"
#define MAX_ALLOWABLE_NODES 700000 // 限制搜索节点总数
#define MOVE_PENALTY 10

int angel = 0;

typedef struct
{
    float next_f;
    uint16_t next_g;
    uint16_t next_h;
    State next_state;
    MacroAction action;
} ChildNode;

// IDA* 返回的数据包
typedef struct
{
    float f;
    uint16_t g;
    uint16_t h;
} SearchRes;

// 哈希表结构与全局内存分配
typedef struct
{
    uint64_t signature; // 64位状态特征码
    uint16_t g_score;   // 走到该状态花费的实际代价
    uint8_t version;    // 记录这个数据是第几轮迭代存入的
} HashEntry;            // 16 bytes
// 预分配全局hash表
static uint8_t current_hash_version = 0;

//__attribute__((section(".bss.sdram")))
__attribute__((section(".bss.sdram"))) static HashEntry transposition_table[HASH_TABLE_SIZE]; // 16MB

// 1 ChildNode = 232B
//__attribute__((section(".ocram_data")))
__attribute__((section(".bss.sdram"))) static ChildNode all_children_pool[MAX_STEPS][MAX_BRANCHES]; // 188kb

// 声明================
static void get_smooth_path(SokobanContext *ctx, const WaypointPath *grid_path, const uint8_t *obstacles, WaypointPath *out_smooth_path);
static uint8_t is_deadlock(SokobanContext *ctx, uint8_t idx, State *state, bool is_bomb, const uint8_t *walls);
static void get_maze_distances(SokobanContext *ctx, const uint8_t *current_walls);
//==================

static void precalc_explosion_masks(SokobanContext *ctx)
{
    // 遍历地图，抠除最外层的一圈死墙
    for (int y = 1; y < HEIGHT - 1; y++)
    {
        for (int x = 1; x < WIDTH - 1; x++)
        {
            uint8_t center_idx = y * WIDTH + x;
            int valid_target_count = 0;

            // 扫描 3x3 邻域
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = x + dx;
                    int ny = y + dy;
                    uint8_t n_idx = ny * WIDTH + nx;
                    // 炸弹无法摧毁边界墙
                    if (!ctx->boundary_walls[n_idx])
                    {
                        // 记录下会被波及的有效坐标索引
                        ctx->explosion_areas[center_idx][valid_target_count] = n_idx;
                        valid_target_count++;
                    }
                }
            }
            // 存入真实波及格子数，用于运行时 O(1) 控制循环次数
            ctx->explosion_area_count[center_idx] = valid_target_count;
        }
    }
}

// 清空哈希表
static void hash_table_clear(void)
{
    current_hash_version++;
    // 如果溢出到了 0，才需要做一次彻底清空 (每 255 轮迭代才发生一次)
    if (current_hash_version == 0)
    {
        memset(transposition_table, 0, sizeof(transposition_table));
        current_hash_version = 1;
    }
}
// 🌟 Zobrist 增量哈希表
// ==========================================
static uint64_t zobrist_car[MAP_SIZE];
static uint64_t zobrist_box[11][MAP_SIZE]; // 最多11种箱子类型
static uint64_t zobrist_bomb[MAP_SIZE];
static uint64_t zobrist_wall[MAP_SIZE];
static uint64_t zobrist_goal_mask[MAX_GOALS]; // 目标位图的每一位
// 极速伪随机数生成器
static uint64_t xorshift64(uint64_t *state)
{
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    return *state;
}
// 引擎启动时初始化一次即可
static void init_zobrist()
{
    uint64_t seed = 0x123456789ABCDEF0ULL; // 固定种子保证每次运行一致
    for (int i = 0; i < MAP_SIZE; i++)
    {
        zobrist_car[i] = xorshift64(&seed);
        zobrist_bomb[i] = xorshift64(&seed);
        zobrist_wall[i] = xorshift64(&seed);
        for (int j = 0; j < 11; j++)
            zobrist_box[j][i] = xorshift64(&seed);
    }
    for (int i = 0; i < MAX_GOALS; i++)
        zobrist_goal_mask[i] = xorshift64(&seed);
}
// 全量计算一次基准 Hash（仅在引擎初始化或侦察补全ID后调用）
static uint64_t compute_initial_base_hash(const State *state, const uint8_t *walls)
{
    uint64_t h = 0;
    for (int i = 0; i < state->box_count; i++)
        h ^= zobrist_box[state->boxes[i].id][state->boxes[i].pos];
    for (int i = 0; i < state->bomb_count; i++)
        h ^= zobrist_bomb[state->bombs[i]];
    for (int i = 0; i < MAP_SIZE; i++)
        if (walls[i])
            h ^= zobrist_wall[i]; // 改用传入的 walls
    for (int i = 0; i < MAX_GOALS; i++)
        if (state->active_goals_mask & (1U << i))
            h ^= zobrist_goal_mask[i];
    h ^= zobrist_car[state->car_pos];
    return h;
}
// 无锁、无循环的 O(1) 查重函数
static bool hash_table_insert_or_check(const State *state, int g_score, int tolerance)
{
    uint64_t sig = state->base_hash;
    uint32_t idx = sig & HASH_MASK;
    // 直接对位映射 (Direct Mapped)
    if (transposition_table[idx].version == current_hash_version &&
        transposition_table[idx].signature == sig)
    {
        if (transposition_table[idx].g_score <= g_score + tolerance)
        {
            return true; // 剪枝
        }
        else
        {
            transposition_table[idx].g_score = g_score; // 更新更优代价
            return false;
        }
    }
    else
    {
        // 如果版本号对不上（说明是上一轮或很久以前的残余垃圾数据），直接无情覆盖！
        transposition_table[idx].version = current_hash_version;
        transposition_table[idx].signature = sig;
        transposition_table[idx].g_score = g_score;
        return false;
    }
}

static void engine_init(SokobanContext *ctx, const uint8_t *raw_map)
{
    // 🌟 初始化 Zobrist 表
    static bool z_init = false;
    if (!z_init)
    {
        init_zobrist();
        z_init = true;
    }
    // 1. 基础尺寸配置
    ctx->goal_count = 0;

    // 2. 预计算全局方向偏移量 (UP, DOWN, LEFT, RIGHT)
    // 这样在 BFS 中就可以彻底抛弃 (x, y) 坐标计算
    ctx->dir_offsets[0] = -WIDTH; // UP
    ctx->dir_offsets[1] = WIDTH;  // DOWN
    ctx->dir_offsets[2] = -1;     // LEFT
    ctx->dir_offsets[3] = 1;      // RIGHT

    // 3. 清空上下文缓存
    memset(ctx->boundary_walls, 0, sizeof(ctx->boundary_walls));
    memset(ctx->goal_type_map, 255, sizeof(ctx->goal_type_map));
    memset(ctx->goal_mask_map, -1, sizeof(ctx->goal_mask_map));
    State *init_state = &ctx->initial_state;
    init_state->box_count = 0;
    init_state->bomb_count = 0;
    memset(ctx->initial_walls, 0, sizeof(ctx->initial_walls));

    // 4. 解析地图
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            int idx = y * WIDTH + x;
            uint8_t val = raw_map[idx];
            if (x == 0 || x == WIDTH - 1 || y == 0 || y == HEIGHT - 1)
            {
                val = 1; // 强制最外层为墙壁
            }
            if (val == 1)
            { // 墙壁 (WALL)
                ctx->initial_walls[idx] = 1;
                // 标记不可破坏的边界墙
                if (x == 0 || x == WIDTH - 1 || y == 0 || y == HEIGHT - 1)
                {
                    ctx->boundary_walls[idx] = 1;
                }
            }
            else if (val == 2)
            { // 箱子 (BOX)
                init_state->boxes[init_state->box_count].pos = idx;
                init_state->boxes[init_state->box_count].id = NO_CLS;
                init_state->box_count++;
            }
            else if (val == 3)
            { // 目标点 (GOAL)
                ctx->goal_type_map[idx] = NO_CLS;
                ctx->goals[ctx->goal_count].pos = idx;
                ctx->goals[ctx->goal_count].id = NO_CLS;
                ctx->goal_mask_map[idx] = ctx->goal_count;
                ctx->goal_count++;
            }
            else if (val == 4)
            { // 炸弹 (BOMB)
                init_state->bombs[init_state->bomb_count++] = idx;
            }
            else if (val == 5)
            { // 小车 (CAR)
                init_state->car_pos = idx;
            }
        }
    }
    // 生成初始目标位图 3个目标对应 ..0000111
    if (ctx->goal_count > 0)
    {
        init_state->active_goals_mask = (1U << ctx->goal_count) - 1;
    }
    else
    {
        init_state->active_goals_mask = 0;
    }
    precalc_explosion_masks(ctx);
    get_maze_distances(ctx, ctx->initial_walls);

    // 标记优先轰炸墙
    if (init_state->bomb_count == 0)
        return;
    for (uint8_t i = 0; i < init_state->box_count; i++)
    {
        uint8_t box_idx = init_state->boxes[i].pos;
        if (is_deadlock(ctx, box_idx, init_state, false, ctx->initial_walls))
        {
            for (uint8_t j = 0; j < ctx->explosion_area_count[box_idx]; j++)
            {
                uint8_t n_idx = ctx->explosion_areas[box_idx][j];
                // 如果这个格子是墙且不是边界墙，就标记为优先轰炸墙 (2)
                if (ctx->initial_walls[n_idx] == 1 && !ctx->boundary_walls[n_idx])
                {
                    ctx->initial_walls[n_idx] = 2;
                }
            }
        }
    }
}

// ==========================================
// 辅助功能：极速静态最小堆 (替代 Python 的 heapq)
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
    int size;
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

// 更新当前ctx的拉箱子距离表cached_dist_table
static void get_maze_distances(SokobanContext *ctx, const uint8_t *current_walls)
{
    // ---------------------------------------------------------
    // 1. LRU Cache 机制的 C 语言实现
    // 因为 walls 只有在炸弹爆炸时才会改变，大部分时间它与上一步完全相同
    if (ctx->cache_valid && memcmp(ctx->cached_walls, current_walls, MAP_SIZE) == 0)
    {
        return; // 缓存命中，直接返回
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
                // 边界墙剪枝
                if (ctx->boundary_walls[p_idx] || ctx->boundary_walls[pp_idx])
                    continue;

                uint16_t step_cost = 1;
                bool is_p_wall = current_walls[p_idx];
                bool is_pp_wall = current_walls[pp_idx];
                if (is_p_wall || is_pp_wall)
                {
                    if (!has_bombs)
                        continue; // 没有炸弹，禁穿
                    // 穿墙惩罚 (优先轰炸墙免罚)
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

// 优化：单次 BFS 构建当前状态下小车的全局距离场
static void build_car_dist_map(SokobanContext *ctx, uint8_t start_pos, const uint8_t *obstacles, uint16_t *dist_map)
{
    // 初始化距离场
    for (uint8_t i = 0; i < MAP_SIZE; i++)
    {
        dist_map[i] = INF_DIST;
    }
    dist_map[start_pos] = 0; // 起点距离为 0

    // 静态栈上分配，极速队列
    uint8_t queue[MAP_SIZE];
    uint8_t head = 0, tail = 0;
    queue[tail++] = start_pos;
    // BFS
    while (head < tail)
    {
        uint8_t curr = queue[head++];
        uint16_t current_d = dist_map[curr];
        for (uint8_t i = 0; i < 4; i++)
        {
            uint8_t n_idx = curr + ctx->dir_offsets[i];
            // 只有当不是障碍物，且从未被访问过（距离仍是 INF）时才入队
            if (!obstacles[n_idx] && dist_map[n_idx] == INF_DIST)
            {
                dist_map[n_idx] = current_d + 1;
                queue[tail++] = n_idx;
            }
        }
    }
}

// O(N^3) 匈牙利算法 (KM算法求最小权完美匹配)
static void solve_assignment_km(int cost_matrix[MAX_BOXES][MAX_GOALS], int num_items,
                                int best_assignment[MAX_BOXES], int *best_cost)
{
    int u[MAX_BOXES + 1] = {0};   // 左侧顶点的顶标
    int v[MAX_GOALS + 1] = {0};   // 右侧顶点的顶标
    int p[MAX_GOALS + 1] = {0};   // 记录右侧顶点匹配到的左侧顶点
    int way[MAX_GOALS + 1] = {0}; // 记录增广路

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
                    // 注意：算法内部索引从 1 开始，对应 matrix 需要 -1
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

    *best_cost = -v[0]; // 最小总代价就是 v[0] 的相反数

    // 导出最优分配方案
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
    // 1. 获取物理拉拽距离场 (依赖自带的 LRU 缓存，耗时极低)
    get_maze_distances(ctx, walls);

    // 2. 构建代价矩阵 (Cost Matrix)
    int cost_matrix[MAX_BOXES][MAX_GOALS];
    for (int b = 0; b < state->box_count; b++)
    {
        for (int g = 0; g < ctx->goal_count; g++)
        {
            if (!(state->active_goals_mask & (1U << g)))
            {
                cost_matrix[b][g] = INF_DIST;
                continue;
            }
            if (state->boxes[b].id != ctx->goals[g].id)
            {
                cost_matrix[b][g] = INF_DIST;
            }
            else
            {
                cost_matrix[b][g] = ctx->cached_dist_table[g][state->boxes[b].pos];
            }
        }
    }
    // 3. 求解最小权二分图匹配
    int best_cost = INF_DIST;
    int best_assignment[MAX_BOXES] = {0};
    int current_assignment[MAX_BOXES] = {0};
    solve_assignment_km(cost_matrix, state->box_count, best_assignment, &best_cost);

    // 如果算出来的代价是正无穷，说明无论怎么分配，必然有箱子进不了目标 (死局)
    if (best_cost >= INF_DIST)
        return INF_DIST;
    int base_h = best_cost;
    // 线性冲突惩罚 (Linear Conflict) - 解决“幽灵箱子”穿透问题
    int conflict_penalty = 0;
    // 穷举两两箱子的配对关系
    for (int i = 0; i < state->box_count; i++)
    {
        for (int j = i + 1; j < state->box_count; j++)
        {
            uint8_t b1_idx = state->boxes[i].pos;
            uint8_t g1_idx = ctx->goals[best_assignment[i]].pos;
            uint8_t b2_idx = state->boxes[j].pos;
            uint8_t g2_idx = ctx->goals[best_assignment[j]].pos;
            // 一维转二维
            int b1_x = b1_idx % WIDTH;
            int b1_y = b1_idx / WIDTH;
            int g1_x = g1_idx % WIDTH;
            int g1_y = g1_idx / WIDTH;

            int b2_x = b2_idx % WIDTH;
            int b2_y = b2_idx / WIDTH;
            int g2_x = g2_idx % WIDTH;
            int g2_y = g2_idx / WIDTH;

            // --- 行冲突 (Row Conflict) ---
            if (b1_y == b2_y && g1_y == g2_y && b1_y == g1_y)
            {
                // 判断是否在X轴上交叉
                if ((b1_x - b2_x) * (g1_x - g2_x) < 0)
                {
                    conflict_penalty += 4; // 避让错车开销
                }
            }
            // --- 列冲突 (Column Conflict) ---
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

// 极速获取单个格子的 2-bit 状态 (0=空, 1=内墙, 2=box/边界墙, 3=tnt)
static inline uint8_t get_cell_state(SokobanContext *ctx, State *state, const uint8_t *walls, uint8_t idx)
{
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
// O(1) 死锁判定函数
static uint8_t is_deadlock(SokobanContext *ctx, uint8_t idx, State *state, bool is_bomb, const uint8_t *walls)
{
    uint8_t UP = -WIDTH;
    uint8_t DOWN = WIDTH;
    uint8_t LEFT = -1;
    uint8_t RIGHT = 1;
    // 将 8 个邻居的状态压缩成一个 16-bit 的环境变量 (Environment Mask)
    // 布局: [BR][B][BL][R][L][TR][T][TL] -> 每个占 2 bits
    uint16_t env = 0;
    env |= (get_cell_state(ctx, state, walls, idx + UP + LEFT) << 0);
    env |= (get_cell_state(ctx, state, walls, idx + UP) << 2);
    env |= (get_cell_state(ctx, state, walls, idx + UP + RIGHT) << 4);
    env |= (get_cell_state(ctx, state, walls, idx + LEFT) << 6);
    env |= (get_cell_state(ctx, state, walls, idx + RIGHT) << 8);
    env |= (get_cell_state(ctx, state, walls, idx + DOWN + LEFT) << 10);
    env |= (get_cell_state(ctx, state, walls, idx + DOWN) << 12);
    env |= (get_cell_state(ctx, state, walls, idx + DOWN + RIGHT) << 14);

    // LUT查表：获取打破当前环境所需的最小 TNT 数量
    uint8_t required_tnt = is_bomb ? DEADLOCK_LUT_BOMB[env] : DEADLOCK_LUT_BOX[env];
    return required_tnt;
}

// 辅助内联函数：对纯坐标数组(如 bombs)进行极速排序
static inline void sort_bombs(uint8_t *arr, int count)
{
    for (int i = 1; i < count; i++)
    {
        if (arr[i - 1] > arr[i])
        {
            uint16_t temp = arr[i];
            arr[i] = arr[i - 1];
            int j;
            for (j = i - 2; j >= 0 && arr[j] > temp; j--)
            {
                arr[j + 1] = arr[j];
            }
            arr[j + 1] = temp;
        }
    }
}
// 辅助函数：对box插入排序 (保证 Hash 一致性)
static inline void sort_boxes(EntityData *arr, int count)
{
    for (int i = 1; i < count; i++)
    {
        if (arr[i - 1].id > arr[i].id ||
            (arr[i - 1].id == arr[i].id && arr[i - 1].pos > arr[i].pos))
        {
            EntityData temp = arr[i];
            arr[i] = arr[i - 1];
            int j;
            for (j = i - 2; j >= 0 && (arr[j].id > temp.id || (arr[j].id == temp.id && arr[j].pos > temp.pos)); j--)
            {
                arr[j + 1] = arr[j];
            }
            arr[j + 1] = temp;
        }
    }
}
// 辅助函数：子节点索引插入排序
static inline void sort_children(const ChildNode *children, int count, uint8_t *indices)
{
    for (int i = 1; i < count; i++)
    {
        if (children[indices[i - 1]].next_f > children[indices[i]].next_f || (children[indices[i - 1]].next_f == children[indices[i]].next_f && children[indices[i - 1]].next_g < children[indices[i]].next_g))
        {
            // 交换索引
            uint8_t tmp_idx = indices[i];
            indices[i] = indices[i - 1];
            int j;
            for (j = i - 2; j >= 0 && (children[indices[j]].next_f > children[tmp_idx].next_f || (children[indices[j]].next_f == children[tmp_idx].next_f && children[indices[j]].next_g < children[tmp_idx].next_g)); j--)
                indices[j + 1] = indices[j];
            indices[j + 1] = tmp_idx;
        }
    }
}

static SearchRes dfs_ida(SokobanContext *ctx, State *current_state, const uint8_t *current_walls, uint16_t current_g, uint16_t current_h, float threshold, MacroAction *acts, uint8_t act_len)
{
    ctx->total_explored_nodes++; // 性能统计
    // 安全防线：防止内存池越界！
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
    int child_count = 0;
    // 更新当前ctx拉箱子safe_mask和距离表
    get_maze_distances(ctx, current_walls);
    // 在进入循环前，一次性构建全图的障碍物 Bitmap
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

    // 🌟 核心优化：在当前节点，只执行一次全域 BFS，获取小车到所有点的真实距离
    uint16_t car_dist_map[MAP_SIZE];
    build_car_dist_map(ctx, current_state->car_pos, obstacles, car_dist_map);

    // 展开所有可推物体
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
            int8_t dir_offset = ctx->dir_offsets[d];
            int next_item_idx = (int)item_idx + dir_offset;
            int push_stand_idx = (int)item_idx - dir_offset;
            if (next_item_idx < 0 || next_item_idx >= MAP_SIZE)
                continue;
            if (push_stand_idx < 0 || push_stand_idx >= MAP_SIZE)
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
                        continue;    // 无法炸掉边界墙
                    exploded = true; // 炸弹炸毁内墙
                }
                else
                {
                    continue; // 箱子撞墙，非法
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
                { // 动态 Safe Mask 剪枝 (基于 cached_dist_table)
                    // 检查箱子能否到达至少一个同颜色的终点
                    bool is_safe = false;
                    for (int g = 0; g < ctx->goal_count; g++)
                    {
                        if (current_state->active_goals_mask & (1U << g))
                        {
                            // 检查颜色是否匹配
                            if (ctx->goals[g].id == current_box_type || current_box_type == NO_CLS)
                            {
                                // 只要不是正无穷，说明这步棋是活的！
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
            // 🌟 ============ 修改后 (瞬间 O(1) 获取) ============
            int car_dist = car_dist_map[push_stand_idx];
            if (car_dist >= INF_DIST)
                continue;

            // 生成新代价
            int step_cost = car_dist + 1;
            // 若引发了爆炸，且被撞击的墙不是优先轰炸墙(2)，才增加惩罚
            if (exploded && current_walls[next_item_idx] != 2)
            {
                step_cost += BOMB_PENALTY;
            }
            int next_g = current_g + step_cost;

            // 创建新state
            State next_state = *current_state; // C 语言极致性能：64字节结构体浅拷贝仅需数个周期
            // 🌟Zobrist 增量：更新小车位置的哈希
            next_state.base_hash ^= zobrist_car[current_state->car_pos]; // 扣除老位置的特征
            next_state.base_hash ^= zobrist_car[item_idx];

            next_state.car_pos = item_idx; // 小车顶上

            const uint8_t *walls_for_eval = current_walls; // 默认零拷贝
            uint8_t temp_walls[MAP_SIZE];                  // 栈上临时数组，阅后即焚
            if (exploded)
            {
                next_state.base_hash ^= zobrist_bomb[item_idx];
                // 移除炸弹
                for (int k = 0; k < next_state.bomb_count; k++)
                {
                    if (next_state.bombs[k] == item_idx)
                    {
                        next_state.bombs[k] = next_state.bombs[--next_state.bomb_count]; // 将最后一个炸弹移到被移除炸弹的位置
                        break;
                    }
                }
                sort_bombs(next_state.bombs, next_state.bomb_count); // 保证 Hash 一致性
                // 🌟 COW 阶段 1：只为了计算 h 值的临时拷贝
                memcpy(temp_walls, current_walls, MAP_SIZE);
                int exp_count = ctx->explosion_area_count[next_item_idx];
                for (int e = 0; e < exp_count; e++)
                {
                    temp_walls[ctx->explosion_areas[next_item_idx][e]] = 0;
                }
                walls_for_eval = temp_walls; // 让启发函数去读这张新图
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
                sort_boxes(next_state.boxes, next_state.box_count);
                next_state.active_goals_mask &= ~(1U << ctx->goal_mask_map[next_item_idx]);
            }
            else
            {
                // 常规平移，执行死锁预测
                // 先更新坐标
                if (is_bomb)
                {
                    // 【Zobrist 增量】：一出一进
                    next_state.base_hash ^= zobrist_bomb[item_idx];      // 扣除旧坐标
                    next_state.base_hash ^= zobrist_bomb[next_item_idx]; // 加上新坐标
                    for (int k = 0; k < next_state.bomb_count; k++)
                    {
                        if (next_state.bombs[k] == item_idx)
                        {
                            next_state.bombs[k] = next_item_idx;
                            break;
                        }
                    }
                    sort_bombs(next_state.bombs, next_state.bomb_count);
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
                    sort_boxes(next_state.boxes, next_state.box_count);
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
            float next_f = next_g + ctx->current_weight * next_h;

            // 【新增】防止由于意外情况导致的子节点数超过池子容量
            if (child_count >= MAX_BRANCHES)
            {
                continue;
            }
            children[child_count].next_f = next_f;
            children[child_count].next_h = next_h;
            children[child_count].next_g = next_g;
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
    sort_children(children, child_count, indices);
    // 展开各child
    for (uint8_t i = 0; i < child_count; i++)
    {
        ChildNode *sorted_child = &children[indices[i]];
        // 如果连最优质的节点也超过阈值，直接跳出
        if (sorted_child->next_f > threshold)
        {
            if (sorted_child->next_f < min_node_data.f)
            {
                min_node_data.f = sorted_child->next_f;
                min_node_data.g = sorted_child->next_g;
                min_node_data.h = sorted_child->next_h;
            }
            break;
        }
        acts[act_len] = sorted_child->action;
        // 🌟 COW 阶段 2：为向下递归准备真正安全的栈上墙壁
        const uint8_t *walls_to_pass = current_walls; // 默认情况：完美共享上一层的墙壁（零拷贝）
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
// 智能推理引擎：根据已识别的配对关系，推断剩余 NO_CLS 的身份 (O(N) 复杂度)
// 返回值：本轮是否成功推断出至少一个实体
static bool try_infer_identities(SokobanContext *ctx, State *current_state)
{
    bool inferred_something = false;

    uint8_t box_counts[11] = {0};
    uint8_t goal_counts[11] = {0};
    uint8_t unk_box_count = 0;
    uint8_t unk_goal_count = 0;

    // 统计当前所有的已知 ID (直方图统计)
    for (int i = 0; i < current_state->box_count; i++)
    {
        if (current_state->boxes[i].id == NO_CLS)
            unk_box_count++;
        else
            box_counts[current_state->boxes[i].id]++;
    }
    for (int i = 0; i < ctx->goal_count; i++)
    {
        if (ctx->goals[i].id == NO_CLS)
            unk_goal_count++;
        else
            goal_counts[ctx->goals[i].id]++;
    }

    // 推理未知的箱子
    if (unk_box_count > 0)
    {
        uint8_t total_box_deficit = 0;
        uint8_t deficit_id = 0;
        uint8_t distinct_deficit_ids = 0;

        for (uint8_t id = 0; id < 11; id++)
        {
            if (id == NO_CLS)
                continue;
            // 目标需要这个 ID，但箱子不够，说明差额就在未知箱子里！
            if (goal_counts[id] > box_counts[id])
            {
                total_box_deficit += (goal_counts[id] - box_counts[id]);
                deficit_id = id;
                distinct_deficit_ids++;
            }
        }
        // 核心破案条件：goal比box多的数量 刚好等于 地图上未知箱子的总数，并且只缺失同一种编号!!
        if (total_box_deficit == unk_box_count && distinct_deficit_ids == 1)
        {
            for (uint8_t i = 0; i < current_state->box_count; i++)
            {
                if (current_state->boxes[i].id == NO_CLS)
                {
                    current_state->boxes[i].id = deficit_id;
                    inferred_something = true;
                }
            }
        }
    }
    // 推理未知目的地
    if (unk_goal_count > 0)
    {
        uint8_t total_goal_deficit = 0;
        uint8_t deficit_id = 0;
        uint8_t distinct_deficit_ids = 0;
        for (uint8_t id = 0; id < 11; id++)
        {
            if (id == NO_CLS)
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
                if (ctx->goals[i].id == NO_CLS)
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
// 侦察专用 BFS：寻找离小车最近的任意一个“视点”
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
            int n_idx = (int)curr + ctx->dir_offsets[i];
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
    uint8_t movable[MAP_SIZE]; // 可移动物体
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
        dist[i] = INF_DIST;
    // 从小车当前位置向外扩散
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
            uint8_t n_idx = curr + ctx->dir_offsets[i];
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
                    step_cost = BOMB_PENALTY; // 非优先墙才使用炸弹代价
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
// ida*识图寻路
static SearchRes dfs_ida_recon(SokobanContext *ctx, State *current_state, const uint8_t *current_walls, uint16_t current_g, uint16_t current_h,
                               float threshold, MacroAction *acts, uint8_t act_len, const bool *obs_points)
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
    // 成功判定
    WaypointPath temp_path;
    if (get_nearest_path(ctx, current_state->car_pos, obs_points, obstacles, &temp_path))
    {
        ctx->solution_actions_len = act_len;
        for (int i = 0; i < act_len; i++)
            ctx->solution_actions[i] = acts[i];
        return (SearchRes){RES_SUCCESS, 0, 0};
    }

    // 以下展开宏动作 (与普通 dfs_ida 完全一致)
    SearchRes min_node_data = {RES_INF, 0, 0};
    ChildNode *children = all_children_pool[act_len];
    int child_count = 0;

    uint8_t all_items[MAX_BOXES + MAX_BOMBS];
    uint8_t total_items = 0;
    for (int i = 0; i < current_state->box_count; i++)
        all_items[total_items++] = current_state->boxes[i].pos;
    for (int i = 0; i < current_state->bomb_count; i++)
        all_items[total_items++] = current_state->bombs[i];
    uint16_t car_dist_map[MAP_SIZE];
    build_car_dist_map(ctx, current_state->car_pos, obstacles, car_dist_map);
    for (int i = 0; i < total_items; i++)
    {
        uint8_t item_idx = all_items[i];
        bool is_bomb = (i >= current_state->box_count);
        uint8_t current_box_type = is_bomb ? 0 : current_state->boxes[i].id;
        for (int d = 0; d < 4; d++)
        {
            int dir_offset = ctx->dir_offsets[d];
            int next_item_idx = (int)item_idx + dir_offset;
            int push_stand_idx = (int)item_idx - dir_offset;
            if (next_item_idx < 0 || next_item_idx >= MAP_SIZE)
                continue;
            if (push_stand_idx < 0 || push_stand_idx >= MAP_SIZE)
                continue;
            bool exploded = false, consumed = false;

            if (obstacles[next_item_idx] && !current_walls[next_item_idx])
                continue;
            if (obstacles[push_stand_idx] && push_stand_idx != current_state->car_pos)
                continue;

            if (current_walls[next_item_idx])
            {
                if (is_bomb && !ctx->boundary_walls[next_item_idx])
                    exploded = true;
                else
                    continue;
            }
            else if (!is_bomb)
            {
                int8_t goal_i = ctx->goal_mask_map[next_item_idx];
                if (goal_i != -1 && (current_state->active_goals_mask & (1U << goal_i)) && ctx->goal_type_map[next_item_idx] == current_box_type)
                {
                    consumed = true;
                }
                else
                { // 动态 Safe Mask 剪枝 (基于 cached_dist_table)
                    // 检查箱子能否到达至少一个同颜色的终点
                    bool is_safe = false;
                    for (int g = 0; g < ctx->goal_count; g++)
                    {
                        if (current_state->active_goals_mask & (1U << g))
                        {
                            // 检查颜色是否匹配
                            if (ctx->goals[g].id == current_box_type || current_box_type == NO_CLS)
                            {
                                // 只要不是正无穷，说明这步棋是活的！
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
            if (car_dist >= INF_DIST)
                continue;

            int step_cost = car_dist + 1;
            if (exploded && current_walls[next_item_idx] != 2)
            {
                step_cost += BOMB_PENALTY;
            }
            int next_g = current_g + step_cost;

            // 物理推演
            State next_state = *current_state;
            next_state.car_pos = item_idx;
            const uint8_t *walls_for_eval = current_walls; // 默认零拷贝
            uint8_t temp_walls[MAP_SIZE];
            if (exploded)
            {
                for (int k = 0; k < next_state.bomb_count; k++)
                {
                    if (next_state.bombs[k] == item_idx)
                    {
                        next_state.bombs[k] = next_state.bombs[--next_state.bomb_count];
                        break;
                    }
                }
                sort_bombs(next_state.bombs, next_state.bomb_count);
                // 🌟 COW 阶段 1：只为了计算 h 值的临时拷贝
                memcpy(temp_walls, current_walls, MAP_SIZE);
                int exp_count = ctx->explosion_area_count[next_item_idx];
                for (int e = 0; e < exp_count; e++)
                {
                    temp_walls[ctx->explosion_areas[next_item_idx][e]] = 0;
                }
                walls_for_eval = temp_walls; // 让启发函数去读这张新图
            }
            else if (consumed)
            {
                for (int k = 0; k < next_state.box_count; k++)
                {
                    if (next_state.boxes[k].pos == item_idx)
                    {
                        next_state.boxes[k] = next_state.boxes[--next_state.box_count];
                        break;
                    }
                }
                sort_boxes(next_state.boxes, next_state.box_count);
                next_state.active_goals_mask &= ~(1U << ctx->goal_mask_map[next_item_idx]);
            }
            else
            {
                if (is_bomb)
                {
                    for (int k = 0; k < next_state.bomb_count; k++)
                    {
                        if (next_state.bombs[k] == item_idx)
                        {
                            next_state.bombs[k] = next_item_idx;
                            break;
                        }
                    }
                    sort_bombs(next_state.bombs, next_state.bomb_count);
                }
                else
                {
                    for (int k = 0; k < next_state.box_count; k++)
                    {
                        if (next_state.boxes[k].pos == item_idx)
                        {
                            next_state.boxes[k].pos = next_item_idx;
                            break;
                        }
                    }
                    sort_boxes(next_state.boxes, next_state.box_count);
                }
                if (is_deadlock(ctx, next_item_idx, &next_state, is_bomb, walls_for_eval) > next_state.bomb_count)
                    continue;
            }

            if (hash_table_insert_or_check(&next_state, next_g, 0))
                continue;

            // 使用 Recon 专用启发函数
            int next_h = calc_recon_heuristic(ctx, &next_state, obs_points, current_walls);
            float next_f = next_g + next_h; // Recon模式下无需权重，1:1即可

            if (child_count >= MAX_BRANCHES)
            {
                continue;
            }

            children[child_count].next_f = next_f;
            children[child_count].next_h = next_h;
            children[child_count].next_g = next_g;
            children[child_count].next_state = next_state;
            children[child_count].action = (MacroAction){push_stand_idx, item_idx, exploded, consumed};
            child_count++;
        }
    }

    uint8_t indices[MAX_BRANCHES];
    for (uint8_t i = 0; i < child_count; i++)
        indices[i] = i;
    sort_children(children, child_count, indices);

    for (uint8_t i = 0; i < child_count; i++)
    {
        ChildNode *sorted_child = &children[indices[i]];
        if (sorted_child->next_f > threshold)
        {
            if (sorted_child->next_f < min_node_data.f)
            {
                min_node_data.f = sorted_child->next_f;
                min_node_data.g = sorted_child->next_g;
                min_node_data.h = sorted_child->next_h;
            }
            break;
        }
        acts[act_len] = sorted_child->action;
        // 🌟 COW 阶段 2：为向下递归准备真正安全的栈上墙壁
        const uint8_t *walls_to_pass = current_walls; // 默认情况：完美共享上一层的墙壁（零拷贝）
        uint8_t recurse_walls[MAP_SIZE];

        if (sorted_child->action.is_explode)
        {
            // 如果这个子节点是一个爆炸动作，我们就在当前 for 循环的栈空间内开辟新墙壁
            memcpy(recurse_walls, current_walls, MAP_SIZE);

            // 推演爆炸落点
            int push_dir = sorted_child->action.push_to - sorted_child->action.move_to;
            uint8_t next_pos = sorted_child->action.push_to + push_dir;

            // 拆墙
            int exp_count = ctx->explosion_area_count[next_pos];
            for (int e = 0; e < exp_count; e++)
            {
                recurse_walls[ctx->explosion_areas[next_pos][e]] = 0;
            }
            walls_to_pass = recurse_walls; // 让下一层递归指向这张新图
        }
        SearchRes res = dfs_ida_recon(ctx, &sorted_child->next_state, walls_to_pass, sorted_child->next_g, sorted_child->next_h, threshold, acts, act_len + 1, obs_points);

        if (res.f == RES_SUCCESS)
            return res;
        if (res.f < min_node_data.f)
            min_node_data = res;
    }
    return min_node_data;
}
// 辅助函数：快速统计 mask 中 1 的个数 (即未知实体的数量)
static inline int count_set_bits(uint16_t n)
{
    int count = 0;
    while (n)
    {
        n &= (n - 1); // 清除最低位的 1
        count++;
    }
    return count;
}
// 封装ida*识图寻路启动器
static bool solve_recon_ida(SokobanContext *ctx, State *start_state, const bool *obs_points)
{
    ctx->total_explored_nodes = 0;
    ctx->solution_actions_len = 0;
    // 🌟 进入搜索前，计算初始绝对特征码
    ctx->initial_state.base_hash = compute_initial_base_hash(&ctx->initial_state, ctx->initial_walls);

    int initial_h = calc_recon_heuristic(ctx, start_state, obs_points, ctx->initial_walls);
    if (initial_h >= INF_DIST)
        return false;
    float threshold = (float)initial_h;
    MacroAction current_act[MAX_STEPS];
    while (true)
    {
        hash_table_clear();
        hash_table_insert_or_check(start_state, 0, 0);
        SearchRes res = dfs_ida_recon(ctx, start_state, ctx->initial_walls, 0, initial_h, threshold, current_act, 0, obs_points);
        if (res.f == RES_SUCCESS)
            return true;
        if (res.f >= RES_INF)
            return false;
        threshold = res.f;
        if (ctx->total_explored_nodes > MAX_ALLOWABLE_NODES)
            return false;
    }
}

void build_map_info(SokobanContext *ctx, uint8_t *raw_map)
{
    hash_table_clear();
    engine_init(ctx, raw_map); // 解析地图，初始化状态，构建边界墙等预处理
    State *current_state = &ctx->initial_state;
    uint16_t unidentified_boxes_mask = (1 << current_state->box_count) - 1;
    uint16_t unidentified_goals_mask = (1 << ctx->goal_count) - 1;
    uint8_t failed[MAP_SIZE] = {-1}; // 记录识别失败的位置
    // 核心状态机循环
    // 逻辑：先找完所有能到的未知实体，剩下不连通的实体用ida*破障寻路
    while (unidentified_boxes_mask > 0 || unidentified_goals_mask > 0)
    {
        // 构建当前障碍物位图
        uint8_t obstacles[MAP_SIZE];
        uint8_t the_goals[MAP_SIZE] = {0};
        memcpy(obstacles, ctx->initial_walls, MAP_SIZE);
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
        // --- 优化逻辑：统计当前未知数量 ---
        int unk_box_num = count_set_bits(unidentified_boxes_mask);
        int unk_goal_num = count_set_bits(unidentified_goals_mask);

        // 标定当前所有需要被侦察的“视点” (紧挨着未知目标的 4 个格子)
        bool observation_points[MAP_SIZE] = {false};
        uint8_t target_map[MAP_SIZE]; // 记录某个视点对应的是哪个实体 (最高位存类型 1=box 0=goal，低7位存索引)
        memset(target_map, 0, sizeof(target_map));
        // 标定未知箱子视点
        bool skip_boxes = (unk_box_num == 1 && unk_goal_num > 1);
        if (!skip_boxes)
        {
            for (int i = 0; i < current_state->box_count; i++)
            {
                if (unidentified_boxes_mask & (1 << i))
                {
                    uint8_t b_pos = current_state->boxes[i].pos;
                    for (int d = 0; d < 4; d++)
                    {
                        int n = b_pos + ctx->dir_offsets[d];
                        if (n >= 0 && n < MAP_SIZE && !obstacles[n] && !the_goals[n] && failed[n] != b_pos)
                        {
                            observation_points[n] = true;
                            target_map[n] = (1 << 7) | i; // Type 1: Box
                        }
                    }
                }
            }
        }
        // 标定未知终点视点
        bool skip_goals = (unk_goal_num == 1 && unk_box_num > 1);
        if (!skip_goals)
        {
            for (int i = 0; i < ctx->goal_count; i++)
            {
                if (unidentified_goals_mask & (1 << i))
                {
                    uint8_t g_pos = ctx->goals[i].pos;
                    for (int d = 0; d < 4; d++)
                    {
                        int n = g_pos + ctx->dir_offsets[d];
                        if (n >= 0 && n < MAP_SIZE && !obstacles[n] && !the_goals[n] && failed[n] != g_pos)
                        {
                            observation_points[n] = true;
                            target_map[n] = (0 << 7) | i;
                        }
                    }
                }
            }
        }
        WaypointPath path;
        WaypointPath smooth_path;
        // 常规寻路：寻找最近视点
        if (get_nearest_path(ctx, current_state->car_pos, observation_points, obstacles, &path))
        {
            uint8_t final_pos = path.points[path.length - 1];
            uint8_t target_info = target_map[final_pos];
            bool is_box = (target_info >> 7) == 1;
            uint8_t index = target_info & 0b01111111;
            uint8_t entity_pos = is_box ? current_state->boxes[index].pos : ctx->goals[index].pos;
            get_smooth_path(ctx, &path, obstacles, &smooth_path); // 对路径平滑处理
            current_state->car_pos = final_pos;
            // printf("[行驶]: 侦察最近的未知%s (坐标 %d)\n", is_box ? "箱子" : "终点", entity_pos);

            // 上下左右
            int8_t dd = entity_pos - final_pos;
            if (dd == -16)
                dd = 0; // 上
            else if (dd == 1)
                dd = 1; // 右
            else if (dd == 16)
                dd = 2; // 下
            else if (dd == -1)
                dd = 3; // 左

            WaypointPath straight_path;
            straight_path.length = 0;
            for (int i = 0; i < smooth_path.length - 1; i++)
            {
                straight_path.length++;
                uint8_t current_point = smooth_path.points[i];
                uint8_t next_point = smooth_path.points[i + 1];
                uint8_t current_x = current_point % 16;
                uint8_t current_y = current_point / 16;
                uint8_t next_x = next_point % 16;
                uint8_t next_y = next_point / 16;
                straight_path.points[straight_path.length - 1] = current_point;
                if (current_x != next_x && current_y != next_y)
                {
                    straight_path.length++;
                    straight_path.points[straight_path.length - 1] = next_x + current_y * 16;
                }
            }
            straight_path.length++;
            straight_path.points[straight_path.length - 1] = smooth_path.points[smooth_path.length - 1];

            // 🌟 硬件接口预留：将 smooth_path 发送给电机，等待小车走到 final_pos，并转向 entity_pos
            car_move(&straight_path, angel, 0);
            while (navigate_flag)
            {
                wifi_task();
            }

            uint8_t final_pos_X = final_pos % 16;
            uint8_t final_pos_Y = final_pos / 16;
            uint8_t entity_pos_X = entity_pos % 16;
            uint8_t entity_pos_Y = entity_pos / 16;

            uint8_t dx = entity_pos_X - final_pos_X;
            uint8_t dy = entity_pos_Y - final_pos_Y;

            WaypointPath turn_path;
            turn_path.length = 1;
            turn_path.points[0] = final_pos;
            // x,y分别取1,0；-1,0；0，1；0，-1时对应的函数值分别为-90,90,180,0

            // system_delay_ms(2000);
            while (current_step >= target_step)
            {
                wifi_task();
            }
            current_step++;
            angel = 90 * dy * dy - 90 * dx + 90 * dy;
            car_turn(angel);
            // car_move(&turn_path, (float)angel, 0);
            while (!yaw_arrived_flag)
            {
                wifi_task();
            }

            // system_delay_ms(700);

            // 🌟 硬件接口预留：开启摄像头识别
            do
            {
                check_image(3 - is_box, 1);
                while (image_rx_state == 1)
                {
                    wifi_task();
                    check_image(3 - is_box, 0);
                }
                car_move(&turn_path, angel, 0); 
            }while(final_image_index == 13); // 等待识别结果，直到收到有效 ID

            // system_delay_ms(700);

            angel = 0;
            // car_move(&turn_path, angel, 0);
            car_turn(angel);
            while (!yaw_arrived_flag)
            {
                wifi_task();
            }
            while (current_step >= target_step)
            {
                wifi_task();
            }
            current_step++;
            uint8_t recognized_id = NO_CLS; // 假设摄像头传回的 ID
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

            if (recognized_id == NO_CLS)
            {
                return;
            }

            // 更新状态
            // printf("[识别]: 成功！坐标 %d 的 ID 为 %d\n", entity_pos, recognized_id);
            if (is_box)
            {
                current_state->boxes[index].id = recognized_id;
                unidentified_boxes_mask &= ~(1 << index);
            }
            else
            {
                ctx->goals[index].id = recognized_id;
                ctx->goal_type_map[entity_pos] = recognized_id;
                unidentified_goals_mask &= ~(1 << index);
            }
            // 每次更新id后尝试推断剩余未知实体的身份
            while (try_infer_identities(ctx, current_state))
            {
                // 每推断出一个，同步更新 Mask
                for (int k = 0; k < current_state->box_count; k++)
                {
                    if (current_state->boxes[k].id != NO_CLS)
                        unidentified_boxes_mask &= ~(1 << k);
                }
                for (int k = 0; k < ctx->goal_count; k++)
                {
                    if (ctx->goals[k].id != NO_CLS)
                        unidentified_goals_mask &= ~(1 << k);
                }
            }
        }
        else
        {
            // printf("\n[路径受阻]: 不连通, 启用 IDA* 破障...\n");
            if (solve_recon_ida(ctx, current_state, observation_points))
            {
                // printf("[破障成功]: 计算出一条 %d 步的宏观破障动作流！\n", ctx->solution_actions_len);
                generate_path(ctx, &smooth_path);
                current_state = &ctx->initial_state;
                // 🌟 硬件接口预留：将 smooth_path 发送给电机
                car_move(&smooth_path, 0, 0);
                while (navigate_flag)
                {
                }
                continue;
            }
            else
            {
                // printf("[彻底死局]: 用 IDA* 也无法为小车开辟出通向视点的路，拓荒被迫终止。\n");
                return;
            }
        }
    }
    // printf("侦察全部完成！地图全貌已解锁\n\n");
}

// 引擎主入口：执行 IDA* 搜索
// 返回值：true 表示找到最优解，false 表示无解或超时/超节点限制
bool solve(SokobanContext *ctx)
{
    // 引擎运行状态与性能统计初始化
    ctx->total_explored_nodes = 0;
    ctx->solution_actions_len = 0;
    ctx->cache_valid = false; // 清空 LRU 距离场缓存
    // 🌟 进入搜索前，计算初始绝对特征码
    ctx->initial_state.base_hash = compute_initial_base_hash(&ctx->initial_state, ctx->initial_walls);
    // 给一个默认的贪婪权重 (可根据需要调整)
    ctx->current_weight = 2.0f;
    ctx->min_weight = 2.0f;
    int patience_limit = 2;

    int initial_h = calc_heuristic(ctx, &ctx->initial_state, ctx->initial_walls);
    if (initial_h >= INF_DIST)
    {
        return false;
    }
    uint8_t iteration = 0;
    // 设置 IDA* 初始阈值
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
            patience_limit += patience_limit; // 耐心值翻倍
            iteration = 0;                    // 重置迭代计数器
            continue;
        }

        threshold = MAX(min_f, threshold + 4.0f);
        // 限制搜索节点总数
        if (ctx->total_explored_nodes > MAX_ALLOWABLE_NODES)
        {
            return false;
        }
    }
    return false; // 理论上不会走到这里
}

static bool get_micro_path(SokobanContext *ctx, uint8_t start_pos, uint8_t target_pos, const uint8_t *obstacles, WaypointPath *out_path)
{
    // 提早退出：如果小车已经在发力点，直接原地待命
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
    memset(visited, 0, sizeof(visited)); // 必须清零，因为栈内存复用会有垃圾数据！

    int head = 0;
    int tail = 0;
    // 起点入队
    queue[tail++] = start_pos;
    visited[start_pos] = true;
    parent[start_pos] = start_pos; // 起点的父节点是自己

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
        // 展开 4 个方向
        for (int i = 0; i < 4; i++)
        {
            // 致命级防雷：必须强转为 (int)，防止第一行网上偏移 (-16) 时 uint8 下溢变成 240 越界！
            int n_idx = (int)curr + ctx->dir_offsets[i];
            if (n_idx < 0 || n_idx >= MAP_SIZE)
                continue;
            // 障碍物和访问判重
            if (!obstacles[n_idx] && !visited[n_idx])
            {
                visited[n_idx] = true;
                parent[n_idx] = curr; // 记录父节点以便回溯
                queue[tail++] = n_idx;
            }
        }
    }
    // 如果把图搜遍了都没找到终点，说明路被封死了
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
    // 翻转路径为正序
    out_path->length = count;
    for (int i = 0; i < count; i++)
    {
        out_path->points[i] = temp_path[count - 1 - i];
    }
    return true;
}
// Bresenham 算法判断两点间是否有视线遮挡
static bool has_sight(SokobanContext *ctx, uint8_t p1, uint8_t p2, const uint8_t *obstacles)
{
    int x0 = p1 % WIDTH;
    int y0 = p1 / WIDTH;
    int x1 = p2 % WIDTH;
    int y1 = p2 / WIDTH;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true)
    {
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        bool step_x = false;
        bool step_y = false;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
            step_x = true;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
            step_y = true;
        }
        // 遍历主轴线上的网格是否有障碍
        if (obstacles[y0 * WIDTH + x0])
            return false;
        // 若走对角，多检测两个侧边网格
        if (step_x && step_y)
        {
            // 两个侧边网格分别是 (旧X, 新Y) 和 (新X, 旧Y)
            int side1_x = x0 - sx;
            int side1_y = y0;
            int side2_x = x0;
            int side2_y = y0 - sy;
            if (obstacles[side1_y * WIDTH + side1_x])
                return false;
            if (obstacles[side2_y * WIDTH + side2_x])
                return false;
        }
    }
    return true;
}
// 节点平滑
static void get_smooth_path(SokobanContext *ctx, const WaypointPath *grid_path, const uint8_t *obstacles, WaypointPath *out_smooth_path)
{
    if (grid_path->length <= 2)
    {
        // 长度小于等于2，不需要平滑
        *out_smooth_path = *grid_path;
        return;
    }
    out_smooth_path->points[0] = grid_path->points[0];
    out_smooth_path->length = 1;
    int current_idx = 0; // 当前锚点在原始路径中的索引

    while (current_idx < grid_path->length - 1)
    {
        int furthest_visible = current_idx + 1;
        // 贪心算法：从终点开始往回找，找到第一个能直线看到当前点的节点
        for (int next = grid_path->length - 1; next > current_idx; next--)
        {
            if (has_sight(ctx, grid_path->points[current_idx], grid_path->points[next], obstacles))
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
    // 第一步：数据净化 剔除重复点
    uint8_t unique_points[MAP_SIZE];
    int unique_len = 0;
    unique_points[unique_len++] = path->points[0];
    for (int i = 1; i < path->length; i++)
    {
        // 只有当前点与上一个写入的点不同，才允许写入
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

    // 第二步：轨迹压缩  通过向量剔除同一直线冗余点
    uint8_t new_points[MAP_SIZE];
    int new_len = 0;
    new_points[new_len++] = unique_points[0];
    // 遍历中间点
    for (int i = 1; i < unique_len - 1; i++)
    {
        int p = unique_points[i - 1]; // 前一个点
        int c = unique_points[i];     // 当前点
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
        // 判断前后两段线段是否同向且共线
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
// 由宏动作推演轨迹 并更新ctx->initial_state和ctx->initial_walls为最终状态
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
        // 步骤 A：构建当前时刻的障碍物位图
        memset(obstacles, 0, sizeof(obstacles));
        // 填入墙
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
        // 注意：小车的当前位置是 sim_state.car_pos，目标是 act.move_to
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
        out_full_path->points[out_full_path->length++] = act.push_to; // 确保终点也在路径里

        // ===========================================
        // --- 调试————打印底盘行驶指令 ---
        for (int p = 0; p < smooth_path.length; p++)
        {
            int px = smooth_path.points[p] % WIDTH;
            int py = smooth_path.points[p] / WIDTH;
        }
        int push_cx = act.push_to % WIDTH;
        int push_cy = act.push_to / WIDTH;

        // ===========================================
        // 更新下一帧地图
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
    // 更新初始状态为最终状态
    ctx->initial_state = sim_state;
    memcpy(ctx->initial_walls, sim_walls, MAP_SIZE);
    get_final_path(ctx, out_full_path); // 对整条路径进行最终的优化处理
}

/**
 * @brief 实时障碍物检查函数（供运控避障/侧向补偿调用）
 * @param ctx 引擎上下文指针
 * @param grid_index 待检查的网格索引 (0~191)
 * @return uint8_t 1: 有障碍(墙、箱子、炸弹)  0: 空地或纯目标点
 */
uint8_t check_obstacle(SokobanContext *ctx, uint8_t grid_index)
{
    // 0. 越界保护
    if (grid_index >= MAP_SIZE)
    {
        return 1;
    }

    // 1. 检查所有的墙体（边界死墙=1，普通内墙=1，优先轰炸墙=2）
    // 说明：engine_init 和 generate_path 都会实时更新 initial_walls，
    // 墙被炸掉后值为 0，所以这里只要 >= 1 就是墙。
    if (ctx->initial_walls[grid_index] >= 1)
    {
        return 1;
    }

    // 2. 检查现存的动态箱子
    for (int i = 0; i < ctx->initial_state.box_count; i++)
    {
        if (ctx->initial_state.boxes[i].pos == grid_index)
        {
            return 1;
        }
    }

    // 3. 检查现存的动态炸弹
    for (int i = 0; i < ctx->initial_state.bomb_count; i++)
    {
        if (ctx->initial_state.bombs[i] == grid_index)
        {
            return 1;
        }
    }

    // 4. 既不是墙，也不是箱子炸弹，说明是绝对安全的空地/目标点
    return 0;
}