#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#include "sokoban_engine.h"

/* 节点动作标志可组合；共线普通宏节点可省略，视觉和等待节点必须停车。 */
typedef enum
{
    MOTION_STOP = 1U << 0,
    MOTION_MACRO_END = 1U << 1,
    MOTION_VISION_FIX = 1U << 2,
    MOTION_DWELL = 1U << 3
} MotionFlags;

/*
 * 运动节点以地图格索引保存位置，速度统一使用 mm/s，等待时间使用 ms，
 * 避免在主循环与 10 ms 中断之间传递浮点配置结果。
 */
typedef struct
{
    uint8_t grid_index;
    uint8_t flags;
    uint16_t segment_limit_mmps;
    uint16_t node_speed_mmps;
    uint16_t dwell_ms;
} MotionNode;

/* 规划结果容量与地图格数一致，防止求解器生成路径时动态分配内存。 */
typedef struct
{
    MotionNode nodes[MAP_SIZE];
    uint16_t length;
} MotionPlan;

/* 中断侧累计的运行统计，主循环在 navigate_flag 清零后通过访问器读取。 */
typedef struct
{
    uint32_t elapsed_ms;
    uint32_t vision_wait_ms;
    uint16_t stop_count;
    uint16_t max_cross_track_mm;
    uint8_t vision_timeout_count;
    uint8_t recovery_count;
    uint8_t aborted;
} MotionRuntimeStats;

/* 路径规划参数使用 SI 单位，最终节点速度会转换为 mm/s 存入 MotionNode。 */
typedef struct
{
    float max_cruise_speed_mps;
    float segment_base_speed_mps;
    float segment_length_gain;
    float corner_speed_mps;
    float max_accel_mps2;
    float max_decel_mps2;
    float vision_distance_m;
    uint8_t vision_turn_count;
    uint16_t explosion_dwell_ms;
} MotionPlanConfig;

extern const MotionPlanConfig motion_plan_default_config;

/* 将纯几何 WaypointPath（兼容旧 255 标记）转换为可执行运动计划。 */
bool build_motion_plan(const WaypointPath *path,
                       const MotionPlanConfig *config,
                       MotionPlan *out_plan);

/* 求解器专用入口：空间节点之外，由并行数组携带节点动作和等待时间。 */
bool motion_plan_build_annotated(const WaypointPath *path,
                                 const uint8_t *node_flags,
                                 const uint16_t *node_dwell_ms,
                                 const MotionPlanConfig *config,
                                 MotionPlan *out_plan);

/* 从求解器宏动作直接生成带动作语义的计划，避免将动作编码成伪坐标。 */
bool generate_motion_plan(SokobanContext *ctx,
                          const MotionPlanConfig *config,
                          MotionPlan *out_plan);

/* 启动新状态机；旧 car_move() 仍保留用于实车 A/B 回退。 */
bool car_move_plan(const MotionPlan *plan, float yaw, uint8_t mode);
const MotionRuntimeStats *motion_plan_stats(void);

#endif
