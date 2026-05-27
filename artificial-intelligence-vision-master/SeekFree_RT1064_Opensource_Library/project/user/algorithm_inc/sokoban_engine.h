#ifndef SOKOBAN_ENGINE_H
#define SOKOBAN_ENGINE_H

#include <stdint.h>
#include <stdbool.h>


// ========================================
// 1. 常量与内存宏定义
// ==========================================
#define WIDTH  16
#define HEIGHT 12
#define MAP_SIZE (WIDTH * HEIGHT)
#define MAX_BOXES 6
#define MAX_GOALS 6    //由于state中用uint16_t的位图来记录目标状态，最多支持16个目标!!!
#define MAX_BOMBS 4 

#define VIRTUAL_WALL_COST 20
#define BOMB_PENALTY 2  //含义：可能是等推炸弹耗时大概相当于car走2格（g += 2）  但实测0更好？
#define RES_SUCCESS -1.0f
#define RES_INF     9999.0f
#define MAX_STEPS 100   //最大步数，用于初始化action数组
#define INF_DIST  9999
#define NO_CLS 0   //无分类模式箱子默认id


#define HASH_TABLE_SIZE 1048576     // 1MB *16(HashEntry) = 16MB
#define HASH_MASK (HASH_TABLE_SIZE - 1)
// 【新增】最大分支数 (单层展开的子节点上限)
// 如6个箱子4个炸弹 = 10*4 = 40
#define MAX_BRANCHES 40
#define FAIL 13   //识别失败回传id

// ==========================================
//状态数据结构 (State)
// 采用紧凑型结构体，方便进行 Zobrist Hashing 或 Memcmp
//将坐标和类型打包
typedef struct {
    uint8_t pos;
    uint8_t  id; 
} EntityData;

typedef struct {
    uint8_t car_pos;
    uint8_t  box_count;
    EntityData boxes[MAX_BOXES]; 
    uint8_t  bomb_count;
    uint8_t bombs[MAX_BOMBS];
    //uint8_t  walls[MAP_SIZE];  废弃！
    // 目标点存活位图，第 i 位为 1 表示 ctx->goals[i] 尚未被抵消
    uint16_t active_goals_mask;
    uint64_t base_hash;  // 🌟 新增：Zobrist 增量哈希基值
} State;    //32 bytes

// 宏动作记录
typedef struct {
    uint8_t move_to; // 小车发力点
    uint8_t push_to; //最终落脚点
    bool is_explode;  // 是否伴随炸弹爆炸
    bool is_consume;  // 是否消除目标
} MacroAction;

// 轨迹数据结构 (用于 M 车模电机控制)
typedef struct {
    uint8_t points[MAP_SIZE]; // 路径点序列 (包含起点和终点)
    uint16_t length;               // 路径总长度
} WaypointPath;
// ==========================================
// 核心引擎上下文 (SokobanContext)
typedef struct {
    uint8_t boundary_walls[MAP_SIZE]; 
    State initial_state;    //记录上一次执行generate_path后的状态！
    int8_t dir_offsets[4];
    uint8_t explosion_areas[MAP_SIZE][9]; // 存储每个点爆炸能波及的格子索引
    uint8_t  explosion_area_count[MAP_SIZE]; // 记录波及格子的实际数量
    // 用于启发式匹配的目的地格式
    EntityData goals[MAX_GOALS];
    uint8_t goal_count;
    // 用于在ida*中 O(1)查询目的地信息
    uint8_t goal_type_map[MAP_SIZE];  //初始化255表示无目标
    int8_t goal_mask_map[MAP_SIZE];   //记录某个坐标对应位图第几位的映射 没有目标存 -1

    bool cache_valid;
    uint8_t  cached_walls[MAP_SIZE];  //用于判断是否需更新cached_dist_table
    uint8_t initial_walls[MAP_SIZE]; //记录初始墙体分布
    uint16_t cached_dist_table[MAX_GOALS][MAP_SIZE];
  
    // 性能统计
    uint32_t total_explored_nodes;
    
    // 权重配置
    float current_weight;
    float min_weight;
    //结果保存
    MacroAction solution_actions[MAX_STEPS];
    uint8_t solution_actions_len;
} SokobanContext;   //9.54 KB

// 引擎主入口：执行 IDA* 搜索
// 返回值：true 表示找到最优解，false 表示无解或超时/超节点限制
bool solve(SokobanContext* ctx);

// 由宏动作推演轨迹
// 参数 1: ctx - 引擎上下文
// 参数 2: out_full_path - 外部分配好的轨迹数组，长度应至少为 MAX_STEPS
void generate_path(SokobanContext* ctx, WaypointPath* out_full_path);
// 获取地图所有信息
void build_map_info(SokobanContext* ctx, uint8_t* raw_map);

extern int angel;
#endif // SOKOBAN_ENGINE_H