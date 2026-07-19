#ifndef SOKOBAN_ENGINE_H
#define SOKOBAN_ENGINE_H

#include <stdbool.h>
#include <stdint.h>

// 地图与实体容量。
#define WIDTH 16
#define HEIGHT 12
#define MAP_SIZE (WIDTH * HEIGHT)
#define MAX_BOXES 30
#define MAX_GOALS 30
#define MAX_BOMBS 4

#define VIRTUAL_WALL_COST 20
#define BOMB_PENALTY 10
#define RES_SUCCESS -1.0f
#define RES_INF 9999.0f
#define MAX_STEPS 100
#define INF_DIST 9999
#define NO_CLS 0
#define WAYPOINT_CONTROL_PERIOD_MS 10U
#define BOMB_EXPLOSION_DELAY_MS 800U
#define MAX_PATH_SEGMENT_CELLS 6U
// 设为 1 时，将最终路径中超过 6 格的水平/竖直线段均匀拆分；设为 0 时不处理。
#ifndef ENABLE_LONG_PATH_SEGMENT_SPLIT
#define ENABLE_LONG_PATH_SEGMENT_SPLIT 1
#endif

// 置换表采用分离数组：签名 8 MiB、g 值 2 MiB、版本号 1 MiB。
#define HASH_TABLE_SIZE 1048576
#define HASH_MASK (HASH_TABLE_SIZE - 1)

// 每个箱子或炸弹最多产生四个方向的推动分支。
#define MAX_BRANCHES ((MAX_BOXES + MAX_BOMBS) * 4)

typedef struct
{
    uint8_t pos;
    uint8_t id;
} EntityData;

typedef struct
{
    uint8_t car_pos;
    uint8_t box_count;
    EntityData boxes[MAX_BOXES];
    uint8_t bomb_count;
    uint8_t bombs[MAX_BOMBS];
    // 第 i 位为 1 表示 ctx->goals[i] 尚未被抵消。
    uint32_t active_goals_mask;
    uint64_t base_hash;
} State;

// 一次宏动作：小车到达发力点后推动一个实体。
typedef struct
{
    uint8_t move_to;
    uint8_t push_to;
    bool is_explode;
    bool is_consume;
} MacroAction;

typedef struct
{
    uint8_t points[MAP_SIZE];
    // Delay after arriving at the corresponding point, in milliseconds.
    uint16_t wait_after_ms[MAP_SIZE];
    uint16_t length;
} WaypointPath;

typedef struct
{
    // 地图边缘上实际存在的墙是不可引爆的边界墙；边缘空地仍可通行。
    uint8_t boundary_walls[MAP_SIZE];
    bool map_valid;
    State initial_state;
    int8_t dir_offsets[4];

    // 每个爆炸中心最多影响其 3x3 邻域内的九个格子。
    uint8_t explosion_areas[MAP_SIZE][9];
    uint8_t explosion_area_count[MAP_SIZE];

    EntityData goals[MAX_GOALS];
    uint8_t goal_count;
    uint8_t goal_type_map[MAP_SIZE];
    int8_t goal_mask_map[MAP_SIZE];

    bool cache_valid;
    uint8_t cached_walls[MAP_SIZE];
    uint8_t initial_walls[MAP_SIZE];
    uint16_t cached_dist_table[MAX_GOALS][MAP_SIZE];

    uint32_t total_explored_nodes;

    // build_map_info() 写入默认值，可在 solve() 前覆盖以平衡速度与路径质量。
    float current_weight;
    float min_weight;

    MacroAction solution_actions[MAX_STEPS];
    uint8_t solution_actions_len;
} SokobanContext;

// 执行 Weighted IDA* 搜索。
bool solve(SokobanContext *ctx);

// 将宏动作展开为底盘可执行的路径点。
void generate_path(SokobanContext *ctx, WaypointPath *out_full_path);

// cls：0=无分类，1=已分类，2=未知类别。
void build_map_info(SokobanContext *ctx, const uint8_t *raw_map, uint8_t cls);

// RT1064 运动控制使用的实时状态接口。
uint8_t check_obstacle(SokobanContext *ctx, uint8_t grid_index);
extern int angle;

#endif // SOKOBAN_ENGINE_H
