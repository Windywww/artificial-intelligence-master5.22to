#include "motion_plan.h"
#include "pid.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* 以 10 ms 理想底盘模型闭环执行真实状态机，并替代硬件和视觉接口。 */
volatile uint8_t global_infor_type = 5U;
uint8_t vision_correct_flag = 0U;
float time_line = 0.0f;
float car_location[2] = {0.0f, 0.0f};
float car_angel = 90.0f;
float imu660rb_acc_x = 0.0f;
float imu660rb_acc_y = 0.0f;
SokobanContext engine_ctx;
static uint32_t vision_request_count = 0U;
static float max_command_delta = 0.0f;

extern volatile uint8_t navigate_flag;
extern float global_x;
extern float global_y;
extern float actual_yaw;
extern float target_vx;
extern float target_vy;
extern float local_encoder_vx;
extern float local_encoder_vy;
extern uint8_t walk_mode;

void navigation_update(void);

void want_global_infor(char type)
{
    if (type == 0)
        vision_request_count++;
    if (global_infor_type == 5U)
        global_infor_type = (uint8_t)type;
}

int16_t encoder_get_count(uint32_t port)
{
    (void)port;
    return 0;
}

void encoder_clear_count(uint32_t port)
{
    (void)port;
}

void motor_set_pwm_RF(float value) { (void)value; }
void motor_set_pwm_LF(float value) { (void)value; }
void motor_set_pwm_LB(float value) { (void)value; }
void motor_set_pwm_RB(float value) { (void)value; }

void pid_init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    (void)pid;
    (void)kp;
    (void)ki;
    (void)kd;
}

float pid_calculate(PID_TypeDef *pid, float target, float actual)
{
    (void)pid;
    (void)actual;
    return target;
}

static void provide_vision_frame(void)
{
    /* 用当前仿真位姿生成与 OpenART 协议一致的归一化坐标。 */
    if (global_infor_type == 0U)
    {
        car_location[0] = global_x / 3.2f;
        car_location[1] = (2.4f - global_y) / 2.4f;
        global_infor_type = 5U;
    }
}

static void simulate_tick(void)
{
    float previous_vx = target_vx;
    float previous_vy = target_vy;
    float delta;

    provide_vision_frame();
    navigation_update();
    /* 记录每周期矢量速度变化，用于验证 1.0/1.2 m/s^2 限制。 */
    delta = sqrtf((target_vx - previous_vx) * (target_vx - previous_vx) +
                  (target_vy - previous_vy) * (target_vy - previous_vy));
    if (delta > max_command_delta)
        max_command_delta = delta;
    global_x += target_vx * 0.01f;
    global_y += target_vy * 0.01f;
    local_encoder_vx = target_vx;
    local_encoder_vy = target_vy;
    time_line += 0.01f;
}

static void start_plan(const uint8_t *points, uint16_t length, MotionPlan *plan)
{
    /* 从几何点构建默认参数计划。 */
    WaypointPath path = {{0U}, 0U};

    path.length = length;
    for (uint16_t i = 0U; i < length; i++)
        path.points[i] = points[i];
    assert(build_motion_plan(&path, &motion_plan_default_config, plan));
}

static void start_prebuilt_plan(const MotionPlan *plan)
{
    /* 每个场景从地图 0 号格中心和静止状态开始。 */
    global_x = 0.1f;
    global_y = 2.3f;
    actual_yaw = 0.0f;
    local_encoder_vx = 0.0f;
    local_encoder_vy = 0.0f;
    target_vx = 0.0f;
    target_vy = 0.0f;
    time_line = 0.0f;
    global_infor_type = 5U;
    max_command_delta = 0.0f;
    assert(car_move_plan(plan, 0.0f, 0U));
}

static uint32_t run_until_finished(uint32_t tick_limit)
{
    uint32_t ticks = 0U;

    while (navigate_flag && ticks < tick_limit)
    {
        simulate_tick();
        ticks++;
    }
    return ticks;
}

static void test_short_turns_keep_moving(void)
{
    /* 六段连续短折线的普通转角不得停车或触发恢复。 */
    const uint8_t points[] = {0U, 1U, 17U, 18U, 34U, 35U};
    MotionPlan plan;
    const MotionRuntimeStats *stats;
    uint32_t ticks;

    vision_correct_flag = 0U;
    vision_request_count = 0U;
    start_plan(points, 6U, &plan);
    start_prebuilt_plan(&plan);
    ticks = run_until_finished(5000U);
    stats = motion_plan_stats();
    assert(!navigate_flag);
    assert(!stats->aborted);
    assert(stats->max_cross_track_mm <= 15U);
    assert(stats->stop_count == 2U);
    assert(stats->recovery_count == 0U);
    assert(walk_mode == 3U);
    assert(ticks < 5000U);
    assert(max_command_delta <= 0.0121f);
}

static void test_cross_track_violation_recovers_to_line(void)
{
    /* 注入 20 mm 横偏后应停车、回到投影线并继续到终点。 */
    const uint8_t points[] = {0U, 3U};
    MotionPlan plan;
    const MotionRuntimeStats *stats;
    uint32_t ticks = 0U;

    vision_correct_flag = 0U;
    vision_request_count = 0U;
    start_plan(points, 2U, &plan);
    start_prebuilt_plan(&plan);
    while (navigate_flag && ticks < 5000U)
    {
        if (ticks == 20U)
            global_y += 0.020f;
        simulate_tick();
        ticks++;
    }

    stats = motion_plan_stats();
    assert(!navigate_flag);
    assert(!stats->aborted);
    assert(stats->max_cross_track_mm >= 20U);
    assert(stats->recovery_count == 1U);
    assert(fabsf(global_y - 2.3f) <= 0.015f);
    assert(max_command_delta <= 0.0121f);
}

static void test_node_vision_waits_for_periodic_request(void)
{
    /* 定时视觉未释放时，节点视觉不得发送竞争请求。 */
    const uint8_t points[] = {0U, 1U};
    MotionPlan plan;
    const MotionRuntimeStats *stats;

    vision_correct_flag = 1U;
    vision_request_count = 0U;
    start_plan(points, 2U, &plan);
    start_prebuilt_plan(&plan);
    assert(run_until_finished(5000U) < 5000U);

    stats = motion_plan_stats();
    assert(!stats->aborted);
    assert(stats->vision_timeout_count == 1U);
    assert(vision_request_count == 0U);
    assert(max_command_delta <= 0.0121f);
    vision_correct_flag = 0U;
}

static void test_collinear_macro_endpoint_is_skipped(void)
{
    /* 共线宏动作节点应被规划器省略，整段只在最终节点停车。 */
    const uint8_t points[] = {0U, 1U, 2U};
    WaypointPath path = {{0U}, 3U};
    uint8_t flags[MAP_SIZE] = {0U};
    uint16_t dwell[MAP_SIZE] = {0U};
    MotionPlan plan;
    const MotionRuntimeStats *stats;

    for (uint16_t i = 0U; i < path.length; i++)
        path.points[i] = points[i];
    flags[1] = MOTION_MACRO_END;
    assert(motion_plan_build_annotated(&path,
                                       flags,
                                       dwell,
                                       &motion_plan_default_config,
                                       &plan));
    assert(plan.length == 2U);
    assert(plan.nodes[1].grid_index == 2U);

    vision_correct_flag = 0U;
    vision_request_count = 0U;
    start_prebuilt_plan(&plan);
    assert(run_until_finished(5000U) < 5000U);
    stats = motion_plan_stats();
    assert(!stats->aborted);
    assert(stats->stop_count == 1U);
}

int main(void)
{
    test_short_turns_keep_moving();
    test_cross_track_violation_recovers_to_line();
    test_node_vision_waits_for_periodic_request();
    test_collinear_macro_endpoint_is_skipped();
    puts("motion_control_plan_test: PASS");
    return 0;
}
