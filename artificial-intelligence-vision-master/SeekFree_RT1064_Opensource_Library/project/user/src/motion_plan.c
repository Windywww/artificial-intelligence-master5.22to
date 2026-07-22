#include "motion_plan.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GRID_SIZE_M 0.2f
#define MM_PER_M 1000.0f
#define MOTION_ALL_FLAGS (MOTION_STOP | MOTION_MACRO_END | MOTION_VISION_FIX | MOTION_DWELL)

/* 默认参数对应 0.2 m 网格和当前底盘的实车初始调参值。 */
const MotionPlanConfig motion_plan_default_config = {
    0.60f,
    0.15f,
    0.75f,
    0.08f,
    1.00f,
    1.20f,
    0.80f,
    4U,
    1310U};

/* 浮点速度只在规划阶段使用，执行阶段使用定点化后的 mm/s。 */
static uint16_t speed_to_mmps(float speed_mps)
{
    float scaled = speed_mps * MM_PER_M + 0.5f;
    if (scaled <= 0.0f)
        return 0U;
    if (scaled >= 65535.0f)
        return 65535U;
    return (uint16_t)scaled;
}

/* 正交路径的格间距离等于曼哈顿距离乘单格边长。 */
static float grid_distance_m(uint8_t from, uint8_t to)
{
    int from_x = from % WIDTH;
    int from_y = from / WIDTH;
    int to_x = to % WIDTH;
    int to_y = to / WIDTH;
    return (float)(abs(to_x - from_x) + abs(to_y - from_y)) * GRID_SIZE_M;
}

/* 相邻点必须同行或同列。 */
static bool same_axis(uint8_t from, uint8_t to)
{
    return (from % WIDTH == to % WIDTH) || (from / WIDTH == to / WIDTH);
}

/* 只把方向相同的共线运动视为直行，反向运动属于 180 度转折。 */
static bool is_straight_through(uint8_t previous, uint8_t current, uint8_t next)
{
    int px = previous % WIDTH;
    int py = previous / WIDTH;
    int cx = current % WIDTH;
    int cy = current / WIDTH;
    int nx = next % WIDTH;
    int ny = next / WIDTH;
    int dx1 = cx - px;
    int dy1 = cy - py;
    int dx2 = nx - cx;
    int dy2 = ny - cy;

    return (dy1 == 0 && dy2 == 0 && dx1 * dx2 > 0) ||
           (dx1 == 0 && dx2 == 0 && dy1 * dy2 > 0);
}

/* 检测同一坐标轴上的反向运动，用于强制掉头停车。 */
static bool is_reverse_turn(uint8_t previous, uint8_t current, uint8_t next)
{
    int dx1 = (int)(current % WIDTH) - (int)(previous % WIDTH);
    int dy1 = (int)(current / WIDTH) - (int)(previous / WIDTH);
    int dx2 = (int)(next % WIDTH) - (int)(current % WIDTH);
    int dy2 = (int)(next / WIDTH) - (int)(current / WIDTH);

    return (dy1 == 0 && dy2 == 0 && dx1 * dx2 < 0) ||
           (dx1 == 0 && dx2 == 0 && dy1 * dy2 < 0);
}

/* 向归一化计划追加一个节点，并就地合并连续重复坐标。 */
static bool append_normalized(MotionPlan *plan,
                              uint8_t grid_index,
                              uint8_t flags,
                              uint16_t dwell_ms)
{
    MotionNode *last;

    if (grid_index >= MAP_SIZE)
        return false;

    if (plan->length > 0U)
    {
        last = &plan->nodes[plan->length - 1U];
        if (last->grid_index == grid_index)
        {
            /* 重复坐标合并，但动作语义和最长等待时间必须保留。 */
            last->flags |= flags;
            if (dwell_ms > last->dwell_ms)
                last->dwell_ms = dwell_ms;
            return true;
        }
    }

    if (plan->length >= MAP_SIZE)
        return false;

    last = &plan->nodes[plan->length++];
    last->grid_index = grid_index;
    last->flags = flags;
    last->segment_limit_mmps = 0U;
    last->node_speed_mmps = 0U;
    last->dwell_ms = dwell_ms;
    return true;
}

/* 合并重复点并删除无意义直行点，输出仍暂不计算速度。 */
static bool normalize_annotated_path(const WaypointPath *path,
                                     const uint8_t *node_flags,
                                     const uint16_t *node_dwell_ms,
                                     MotionPlan *out_plan)
{
    uint16_t i;

    out_plan->length = 0U;
    for (i = 0U; i < path->length; i++)
    {
        uint8_t flags = node_flags != NULL ? node_flags[i] : 0U;
        uint16_t dwell_ms = node_dwell_ms != NULL ? node_dwell_ms[i] : 0U;
        if ((flags & (uint8_t)~MOTION_ALL_FLAGS) != 0U ||
            (dwell_ms != 0U && (flags & MOTION_DWELL) == 0U))
        {
            return false;
        }
        if ((flags & (MOTION_MACRO_END | MOTION_VISION_FIX | MOTION_DWELL)) != 0U)
            flags |= MOTION_STOP;
        if (!append_normalized(out_plan, path->points[i], flags, dwell_ms))
            return false;
    }

    if (out_plan->length == 0U)
        return false;

    i = 1U;
    while (i + 1U < out_plan->length)
    {
        MotionNode *node = &out_plan->nodes[i];
        bool removable_macro = node->flags == (MOTION_STOP | MOTION_MACRO_END);
        /* 无动作点和仅表示宏结束的同向共线点都可删除，动作转角仍保留。 */
        if ((node->flags == 0U || removable_macro) &&
            is_straight_through(out_plan->nodes[i - 1U].grid_index,
                                node->grid_index,
                                out_plan->nodes[i + 1U].grid_index))
        {
            memmove(node,
                    node + 1,
                    (out_plan->length - i - 1U) * sizeof(MotionNode));
            out_plan->length--;
            continue;
        }
        i++;
    }

    return true;
}

static bool validate_geometry(const MotionPlan *plan)
{
    /* 新控制器只接受横平竖直且相邻节点不同的路径。 */
    uint16_t i;
    for (i = 1U; i < plan->length; i++)
    {
        if (plan->nodes[i - 1U].grid_index == plan->nodes[i].grid_index ||
            !same_axis(plan->nodes[i - 1U].grid_index, plan->nodes[i].grid_index))
        {
            return false;
        }
    }
    return true;
}

static void mark_stops_and_vision(MotionPlan *plan, const MotionPlanConfig *config)
{
    float distance_since_vision = 0.0f;
    uint8_t turns_since_vision = 0U;
    uint16_t i;

    /* 终点无条件停车并校正，途中按累计里程或转角数插入风险锚点。 */
    plan->nodes[plan->length - 1U].flags |= MOTION_STOP | MOTION_VISION_FIX;

    for (i = 1U; i < plan->length; i++)
    {
        MotionNode *node = &plan->nodes[i];
        distance_since_vision += grid_distance_m(plan->nodes[i - 1U].grid_index,
                                                 node->grid_index);

        if (i + 1U < plan->length)
        {
            if (is_reverse_turn(plan->nodes[i - 1U].grid_index,
                                node->grid_index,
                                plan->nodes[i + 1U].grid_index))
            {
                node->flags |= MOTION_STOP;
            }
            else if (!is_straight_through(plan->nodes[i - 1U].grid_index,
                                          node->grid_index,
                                          plan->nodes[i + 1U].grid_index))
            {
                turns_since_vision++;
            }
        }

        if ((node->flags & MOTION_VISION_FIX) != 0U)
        {
            distance_since_vision = 0.0f;
            turns_since_vision = 0U;
        }
        else if (distance_since_vision + 0.0001f >= config->vision_distance_m ||
                 turns_since_vision >= config->vision_turn_count)
        {
            node->flags |= MOTION_STOP | MOTION_VISION_FIX;
            distance_since_vision = 0.0f;
            turns_since_vision = 0U;
        }
    }
}

static void calculate_speed_plan(MotionPlan *plan, const MotionPlanConfig *config)
{
    float node_speeds[MAP_SIZE];
    uint16_t i;

    node_speeds[0] = 0.0f;
    plan->nodes[0].node_speed_mmps = 0U;
    plan->nodes[0].segment_limit_mmps = 0U;

    for (i = 1U; i < plan->length; i++)
    {
        /* 短段自动降速：min(最大巡航速度, 基础速度 + 长度增益)。 */
        float length_m = grid_distance_m(plan->nodes[i - 1U].grid_index,
                                         plan->nodes[i].grid_index);
        float segment_limit = config->segment_base_speed_mps +
                              config->segment_length_gain * length_m;
        if (segment_limit > config->max_cruise_speed_mps)
            segment_limit = config->max_cruise_speed_mps;
        plan->nodes[i].segment_limit_mmps = speed_to_mmps(segment_limit);
        node_speeds[i] = (plan->nodes[i].flags & MOTION_STOP) != 0U
                             ? 0.0f
                             : config->corner_speed_mps;
    }

    /* 前向扫描约束从上一节点出发后能够达到的速度。 */
    for (i = 1U; i < plan->length; i++)
    {
        float length_m = grid_distance_m(plan->nodes[i - 1U].grid_index,
                                         plan->nodes[i].grid_index);
        float reachable = sqrtf(node_speeds[i - 1U] * node_speeds[i - 1U] +
                                2.0f * config->max_accel_mps2 * length_m);
        if (node_speeds[i] > reachable)
            node_speeds[i] = reachable;
    }

    /* 后向扫描预留到下一停车点或转角速度所需的制动距离。 */
    for (i = plan->length - 1U; i > 0U; i--)
    {
        float length_m = grid_distance_m(plan->nodes[i - 1U].grid_index,
                                         plan->nodes[i].grid_index);
        float reachable = sqrtf(node_speeds[i] * node_speeds[i] +
                                2.0f * config->max_decel_mps2 * length_m);
        if (node_speeds[i - 1U] > reachable)
            node_speeds[i - 1U] = reachable;
    }

    for (i = 0U; i < plan->length; i++)
        plan->nodes[i].node_speed_mmps = speed_to_mmps(node_speeds[i]);
}

/* 公共构建核心，供求解器动作路径和旧 WaypointPath 解析共同调用。 */
bool motion_plan_build_annotated(const WaypointPath *path,
                                 const uint8_t *node_flags,
                                 const uint16_t *node_dwell_ms,
                                 const MotionPlanConfig *config,
                                 MotionPlan *out_plan)
{
    /* 失败时清空输出，保证主流程不会误执行上一次残留计划。 */
    if (out_plan != NULL)
        out_plan->length = 0U;
    if (path == NULL || config == NULL || out_plan == NULL ||
        path->length == 0U || path->length > MAP_SIZE ||
        config->max_cruise_speed_mps <= 0.0f ||
        config->max_accel_mps2 <= 0.0f ||
        config->max_decel_mps2 <= 0.0f ||
        config->vision_turn_count == 0U)
    {
        return false;
    }

    if (!normalize_annotated_path(path, node_flags, node_dwell_ms, out_plan) ||
        !validate_geometry(out_plan))
    {
        out_plan->length = 0U;
        return false;
    }

    mark_stops_and_vision(out_plan, config);
    calculate_speed_plan(out_plan, config);
    return true;
}

/* 解析几何路径和旧 255 爆破标记，再交给统一构建核心。 */
bool build_motion_plan(const WaypointPath *path,
                       const MotionPlanConfig *config,
                       MotionPlan *out_plan)
{
    WaypointPath spatial_path;
    uint8_t flags[MAP_SIZE];
    uint16_t dwell_ms[MAP_SIZE];
    uint16_t i;
    bool previous_was_marker = false;

    if (out_plan != NULL)
        out_plan->length = 0U;
    if (path == NULL || config == NULL || out_plan == NULL ||
        path->length == 0U || path->length > MAP_SIZE)
        return false;

    spatial_path.length = 0U;
    memset(flags, 0, sizeof(flags));
    memset(dwell_ms, 0, sizeof(dwell_ms));

    for (i = 0U; i < path->length; i++)
    {
        uint8_t point = path->points[i];
        if (point == 255U)
        {
            /* 旧 255 紧跟在爆破点之后，转换为该节点的动作而非空间坐标。 */
            uint16_t previous;
            if (spatial_path.length == 0U || previous_was_marker)
                return false;
            previous = spatial_path.length - 1U;
            flags[previous] |= MOTION_STOP | MOTION_VISION_FIX | MOTION_DWELL;
            dwell_ms[previous] = config->explosion_dwell_ms;
            previous_was_marker = true;
            continue;
        }
        if (point >= MAP_SIZE || spatial_path.length >= MAP_SIZE)
            return false;
        spatial_path.points[spatial_path.length++] = point;
        previous_was_marker = false;
    }

    if (previous_was_marker && spatial_path.length == 0U)
        return false;

    return motion_plan_build_annotated(&spatial_path,
                                       flags,
                                       dwell_ms,
                                       config,
                                       out_plan);
}
