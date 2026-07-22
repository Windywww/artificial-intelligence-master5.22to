#include "motion_plan.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 纯规划测试不依赖底盘硬件，用于验证路径语义和速度约束。 */
static WaypointPath make_path(const uint8_t *points, uint16_t length)
{
    WaypointPath path;
    memset(&path, 0, sizeof(path));
    path.length = length;
    if (length <= MAP_SIZE)
        memcpy(path.points, points, length);
    return path;
}

static void test_straight_path_is_simplified(void)
{
    /* 无动作的同向共线中间点应被删除。 */
    const uint8_t points[] = {0U, 1U, 3U};
    WaypointPath path = make_path(points, 3U);
    MotionPlan plan;

    assert(build_motion_plan(&path, &motion_plan_default_config, &plan));
    assert(plan.length == 2U);
    assert(plan.nodes[0].grid_index == 0U);
    assert(plan.nodes[1].grid_index == 3U);
    assert(plan.nodes[1].segment_limit_mmps == 600U);
    assert((plan.nodes[1].flags & (MOTION_STOP | MOTION_VISION_FIX)) ==
           (MOTION_STOP | MOTION_VISION_FIX));
}

static void test_short_zigzag_gets_risk_anchor(void)
{
    /* 连续四个短转角应触发停车视觉风险锚点。 */
    const uint8_t points[] = {0U, 1U, 17U, 18U, 34U, 35U};
    WaypointPath path = make_path(points, 6U);
    MotionPlan plan;

    assert(build_motion_plan(&path, &motion_plan_default_config, &plan));
    assert(plan.length == 6U);
    assert(plan.nodes[1].segment_limit_mmps == 300U);
    assert(plan.nodes[1].node_speed_mmps == 80U);
    assert(plan.nodes[2].node_speed_mmps == 80U);
    assert(plan.nodes[3].node_speed_mmps == 80U);
    assert((plan.nodes[4].flags & (MOTION_STOP | MOTION_VISION_FIX)) ==
           (MOTION_STOP | MOTION_VISION_FIX));
    assert(plan.nodes[4].node_speed_mmps == 0U);
}

static void test_mixed_segment_limits_and_speed_passes(void)
{
    /* 长短混合路径分别验证线段限速和前后向加减速度扫描。 */
    const uint8_t points[] = {0U, 3U, 19U, 23U};
    const uint8_t constrained_points[] = {0U, 1U, 17U, 18U};
    WaypointPath path = make_path(points, 4U);
    WaypointPath constrained_path = make_path(constrained_points, 4U);
    MotionPlanConfig config = motion_plan_default_config;
    MotionPlan plan;

    assert(build_motion_plan(&path, &config, &plan));
    assert(plan.nodes[1].segment_limit_mmps == 600U);
    assert(plan.nodes[2].segment_limit_mmps == 300U);
    assert(plan.nodes[3].segment_limit_mmps == 600U);

    config.corner_speed_mps = 0.60f;
    config.max_accel_mps2 = 0.01f;
    config.max_decel_mps2 = 0.01f;
    assert(build_motion_plan(&constrained_path, &config, &plan));
    assert(plan.nodes[1].node_speed_mmps <= 64U);
    assert(plan.nodes[2].node_speed_mmps <= 64U);
}

static void test_legacy_explosion_marker(void)
{
    /* 共线爆破点不能随普通宏节点删除，仍需保留 1310 ms 等待。 */
    const uint8_t points[] = {0U, 1U, 255U, 2U};
    WaypointPath path = make_path(points, 4U);
    MotionPlan plan;

    assert(build_motion_plan(&path, &motion_plan_default_config, &plan));
    assert(plan.length == 3U);
    assert((plan.nodes[1].flags &
            (MOTION_STOP | MOTION_VISION_FIX | MOTION_DWELL)) ==
           (MOTION_STOP | MOTION_VISION_FIX | MOTION_DWELL));
    assert(plan.nodes[1].dwell_ms == 1310U);
}

static void test_collinear_macro_point_is_removed(void)
{
    /* 仅表示宏结束的同向共线节点应删除，避免直线中多余停车。 */
    const uint8_t points[] = {0U, 1U, 2U};
    uint8_t flags[MAP_SIZE] = {0U};
    uint16_t dwell[MAP_SIZE] = {0U};
    WaypointPath path = make_path(points, 3U);
    MotionPlan plan;

    flags[1] = MOTION_MACRO_END;
    assert(motion_plan_build_annotated(&path,
                                       flags,
                                       dwell,
                                       &motion_plan_default_config,
                                       &plan));
    assert(plan.length == 2U);
    assert(plan.nodes[0].grid_index == 0U);
    assert(plan.nodes[1].grid_index == 2U);
}

static void test_duplicate_points_merge_annotations(void)
{
    /* 重复坐标合并时不得丢失后一个节点携带的动作标志。 */
    const uint8_t points[] = {0U, 1U, 1U, 17U};
    uint8_t flags[MAP_SIZE] = {0U};
    uint16_t dwell[MAP_SIZE] = {0U};
    WaypointPath path = make_path(points, 4U);
    MotionPlan plan;

    flags[2] = MOTION_MACRO_END;
    assert(motion_plan_build_annotated(&path,
                                       flags,
                                       dwell,
                                       &motion_plan_default_config,
                                       &plan));
    assert(plan.length == 3U);
    assert(plan.nodes[1].grid_index == 1U);
    assert((plan.nodes[1].flags & (MOTION_MACRO_END | MOTION_STOP)) ==
           (MOTION_MACRO_END | MOTION_STOP));
}

static void test_reverse_turn_stops(void)
{
    /* 180 度掉头节点速度必须为零。 */
    const uint8_t points[] = {0U, 2U, 1U};
    WaypointPath path = make_path(points, 3U);
    MotionPlan plan;

    assert(build_motion_plan(&path, &motion_plan_default_config, &plan));
    assert((plan.nodes[1].flags & MOTION_STOP) != 0U);
    assert(plan.nodes[1].node_speed_mmps == 0U);
}

static void test_invalid_inputs_are_rejected(void)
{
    /* 斜线、错误 255、容量溢出、未知标志和错误等待值均应失败。 */
    const uint8_t diagonal[] = {0U, 17U};
    const uint8_t leading_marker[] = {255U, 0U};
    const uint8_t repeated_marker[] = {0U, 255U, 255U};
    WaypointPath diagonal_path = make_path(diagonal, 2U);
    WaypointPath marker_path = make_path(leading_marker, 2U);
    WaypointPath repeated_marker_path = make_path(repeated_marker, 3U);
    WaypointPath oversized = make_path(diagonal, MAP_SIZE + 1U);
    WaypointPath annotated_path = make_path(diagonal, 2U);
    uint8_t flags[MAP_SIZE] = {0U};
    uint16_t dwell[MAP_SIZE] = {0U};
    MotionPlan plan;

    assert(!build_motion_plan(&diagonal_path, &motion_plan_default_config, &plan));
    assert(plan.length == 0U);
    assert(!build_motion_plan(&marker_path, &motion_plan_default_config, &plan));
    assert(!build_motion_plan(&repeated_marker_path,
                              &motion_plan_default_config,
                              &plan));
    assert(!build_motion_plan(&oversized, &motion_plan_default_config, &plan));

    flags[0] = 0x80U;
    assert(!motion_plan_build_annotated(&annotated_path,
                                        flags,
                                        dwell,
                                        &motion_plan_default_config,
                                        &plan));
    assert(plan.length == 0U);

    flags[0] = 0U;
    dwell[0] = 10U;
    assert(!motion_plan_build_annotated(&annotated_path,
                                        flags,
                                        dwell,
                                        &motion_plan_default_config,
                                        &plan));
}

int main(void)
{
    test_straight_path_is_simplified();
    test_short_zigzag_gets_risk_anchor();
    test_mixed_segment_limits_and_speed_passes();
    test_legacy_explosion_marker();
    test_collinear_macro_point_is_removed();
    test_duplicate_points_merge_annotations();
    test_reverse_turn_stops();
    test_invalid_inputs_are_rejected();
    puts("motion_plan_test: PASS");
    return 0;
}
