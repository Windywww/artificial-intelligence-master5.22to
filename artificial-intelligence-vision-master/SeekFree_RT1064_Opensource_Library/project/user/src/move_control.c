#include "move_control.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "Set_Follow.h"
#include <math.h>
#include <string.h>
#include "sokoban_engine.h"
#include "motion_plan.h"
uint32_t encoder_ports[4] = {ENCODER_1, ENCODER_2, ENCODER_3, ENCODER_4};
int16_t encoder_data[4] = {0, 0, 0, 0};

float ax = 0;
float ay = 0;
float az = 0;

float imu_x = 0;
float imu_y = 0;

float actual_v[4] = {0.01, 0.01, 0.01, 0.01}; // 实际线速度 单位 m/s
float target_v[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 目标线速度 单位 m/s
float out_duty[4] = {0, 0, 0, 0};             // 输出占空比 -100~100
float vision_weight = 1;

float actual_yaw = 0.0f;       // 实际航向角 单位度
float target_yaw = 0.0f;       // 瞬时目标航向角 单位度
float final_target_yaw = 0.0f; // 最终目标航向角 单位度

float target_vx = 0.0f; // 车模坐标系下横移速度 单位 m/s
float target_vy = 0.0f; // 车模坐标系下前进速度 单位 m/s

float Kp_yaw = 0.06f; // 航向角 P 参数
float Kd_yaw = 0.1f;  // 航向角 D 参数

float global_x = 0.3f; // 车模全局 x 坐标 单位 m
float global_y = 1.2f; // 车模全局 y 坐标 单位 m

uint8_t move_flag = 0;      // 1表示车子在移动 0 表示车子在停止
uint8_t mode = 0;           // 两种运动模式
float target_x = 0.3f;      // 目标 x 坐标 单位 m
float target_y = 1.2f;      // 目标 y 坐标 单位 m
float min_distance = 0.01f; // 距离小于这个值就认为到达目标点了 单位 m
float kp_position_x = 4.0f;
float kp_position_y = 4.0f;
float kd_position_x = 0.0f;
float kd_position_y = 0.0f;

float path_queue_x[100];
float path_queue_y[100];
uint16_t path_length = 0;  // 路径总点数
uint16_t current_path = 0; // 当前正在追第几个点
volatile uint8_t navigate_flag = 0; // 1: 正在追路径 0: 没有路径需要追
uint8_t yaw_arrived_flag = 0;
uint8_t stop_flag = 0; // 1: 手刹 0: 不手刹
uint8_t vision_xy_update_flag_first = 0;

PID_TypeDef pid[4];
uint8_t vision_angle_switch = 0;
// SecondOrder_Set_Follow_t planner_x; // x 轴 s 曲线跟随规划器
// SecondOrder_Set_Follow_t planner_y; // y 轴 s 曲线跟随规划器
uint8_t wrong_time = 0;
// 小车在横着走竖着走还是斜着走,0是横，1是竖；2是斜3是不走,4等待炸弹延时
uint8_t walk_mode = 3;
extern uint8_t vision_correct_flag;
/* 旧路径视觉超时逻辑使用主程序维护的系统运行时间。 */
extern float time_line;
uint8_t wrong_over_time = 0;
void move_control_init()
{
    for (int i = 0; i < 4; i++)
    {
        pid_init(&pid[i], 40.0f, 2.0f, 0.0f);
    }

    // 初始化 s 曲线跟随规划器 参数分别是：最大速度、最大加速度、计算周期,加速度原来是1.5
    // SecondOrder_Set_Follow_init(&planner_x, v_x, a_x, 0.02f);
    // SecondOrder_Set_Follow_init(&planner_y, v_y, a_y, 0.02f);

    // 将规划器的当前位置设置为全局坐标系下的当前位置 这样规划器就不会一上来就算一个很大的误差了
    // planner_x.p = global_x;
    // planner_y.p = global_y;
}
/**
 * @brief 航向角 PID 计算函数
 *
 * @return float vz
 */
float yaw_pid_calculate(void)
{
    static float error0_yaw = 0.0f, error1_yaw = 0.0f; // 上次误差 和 本次误差

    error0_yaw = error1_yaw;              // 上次误差
    error1_yaw = target_yaw - actual_yaw; // 本次误差

    // 保证误差永远在 -180 到 +180 之间
    while (error1_yaw > 180.0f)
        error1_yaw -= 360.0f;
    while (error1_yaw < -180.0f)
        error1_yaw += 360.0f;

    // 计算外环 PD 输出
    float vz = Kp_yaw * error1_yaw + Kd_yaw * (error1_yaw - error0_yaw);

    if (vz > 0.6f)
        vz = 0.6f;
    if (vz < -0.6f)
        vz = -0.6f;

    if (error1_yaw < 1.0f && error1_yaw > -1.0f)
    {
        vz = 0.0f;
        yaw_arrived_flag = 1;
    }
    else
    {
        yaw_arrived_flag = 0;
    }

    return vz;
}
/**
 * @brief 计算各轮速度
 *
 * @param vx
 * @param vy
 * @param vz
 */
void wheel_speed_calculate(float vx, float vy, float vz)
{
    // 运动学逆解算
    target_v[LF] = target_vy + target_vx - vz;
    target_v[LB] = target_vy - target_vx - vz;
    target_v[RF] = target_vy - target_vx + vz;
    target_v[RB] = target_vy + target_vx + vz;

    for (int i = 0; i < 4; i++)
    {
        encoder_data[i] = encoder_get_count(encoder_ports[i]);
        encoder_clear_count(encoder_ports[i]);

        float raw_v = (float)encoder_data[i] * SPEED_COEFFICIENT;
        if (i == RF || i == RB)
        {
            raw_v = -raw_v;
        }
        actual_v[i] = raw_v;
        // actual_v[i] = 0.3f*raw_v+0.7f*actual_v[i]; // 速度低通滤波

        out_duty[i] = pid_calculate(&pid[i], target_v[i], actual_v[i]);

        switch (i)
        {
        case RF:
            motor_set_pwm_RF(-out_duty[i]);
            break;
        case LF:
            motor_set_pwm_LF(-out_duty[i]);
            break;
        case LB:
            motor_set_pwm_LB(out_duty[i]);
            break;
        case RB:
            motor_set_pwm_RB(out_duty[i]);
            break;

        default:
            break;
        }
    }
}

float local_encoder_vx = 0.0f;
float local_encoder_vy = 0.0f;
float local_imu_vx = 0.0f;
float local_imu_vy = 0.0f;

float vx_encoder_index = 0.925f;
float vy_encoder_index = 1.0f;
/**
 * @brief 里程计更新
 *
 */
void odometry_update()
{
    // 车模坐标系下的速度
    local_encoder_vx = (actual_v[LF] + actual_v[RB] - actual_v[LB] - actual_v[RF]) / 4.0f*vx_encoder_index;
    local_encoder_vy = (actual_v[LF] + actual_v[LB] + actual_v[RF] + actual_v[RB]) / 4.0f*vy_encoder_index;

    // imu积分推算出的速度
    // static float local_imu_vx = 0.0f;
    // static float local_imu_vy = 0.0f;

    float dt = 0.01f; // 10ms

    // 速度积分
    // local_imu_vx += imu660rb_acc_x * dt;
    // local_imu_vy += imu660rb_acc_y * dt;
    // if (fabs(target_vx) == 0.0f && fabs(target_vy) == 0.0f &&
    //     fabs(local_encoder_vx) < 0.02f && fabs(local_encoder_vy) < 0.02f)
    // {
    //     local_imu_vx = 0.0f;
    //     local_imu_vy = 0.0f;
    //     local_encoder_vx = 0.0f; // 彻底锁死
    //     local_encoder_vy = 0.0f;
    // }

    float encoder_weight_x = 0.8f; // 编码器权重 80%
    float encoder_weight_y = 0.8f; // 编码器权重 80%

    static float last_yaw = 0.0f;
    float yaw_change = actual_yaw - last_yaw;
    last_yaw = actual_yaw;

    // 如果小车正在剧烈旋转（20ms内转动超过 1.5度）
    if (yaw_change > 1.5f || yaw_change < -1.5f)
    {
        // 把编码器算出来的虚假平移速度削弱掉 90%,???
        // local_encoder_vx *= 0.1f;
        // local_encoder_vy *= 0.1f;
    }

    // 角度换成弧度
    float yaw_rad = actual_yaw * 3.1415926f / 180.0f;

    // 计算全局坐标系下的速度
    float global_actual_vx = -local_encoder_vy * sinf(yaw_rad) + local_encoder_vx * cosf(yaw_rad);
    float global_actual_vy = local_encoder_vy * cosf(yaw_rad) + local_encoder_vx * sinf(yaw_rad);

    // if(final_target_yaw-actual_yaw>=9||final_target_yaw-actual_yaw<=-9){
    //     return;
    // }
    // 速度积分得到位置
    global_x += global_actual_vx * 0.01f;
    global_y += global_actual_vy * 0.01f;
}

/**
 * @brief 全局导航
 *
 *
 *
 */

// 记录视觉传来的信息连续相同的次数
uint8_t loac_test = 0;
// 标志位，表示此刻串口是否正在使用
uint8_t wait_for_loc = 0;
// 记录小车跑过的节点个数是否应该让视觉矫正
uint8_t vision_point_num = 0;
// 从一个节点到另一个节点的角度信息，以及走的状态(横向，纵向，斜向)
float speed_angle = 0.0f;           //(弧度制)
float last_global_target_vx = 0.0f; // 全局坐标系下的目标速度
float last_global_target_vy = 0.0f; // 全局坐标系下
float amax = 2.0f;                  // 最大加速度 m/s^2

// 分别在最后一个点与其它节点起到延时作用
uint8_t count_A = 0;
volatile uint8_t count = 0;

uint8_t got_angle = 0;
uint8_t angle_test = 0;
// 是为了小车到节点根据视觉传来的坐标再矫正一次
uint8_t first_time_fix = 1;

// 视觉传来的坐标,角度
float vision_x = -1;
float vision_y = -1;
float vision_angle = 999;

// 接收视觉接收判定
float time_vision = 0;

/* 新路径状态机由 10 ms PIT 中断驱动，以下距离单位为 m、速度单位为 m/s。 */
#define MOTION_CONTROL_DT_S 0.01f
#define MOTION_PASS_PLANE_M 0.002f
#define MOTION_POSITION_TOLERANCE_M 0.015f
#define MOTION_START_ANCHOR_TOLERANCE_M 0.05f
#define MOTION_STOP_SPEED_MPS 0.03f
#define MOTION_STOP_STABLE_TICKS 3U
#define MOTION_STOP_TIMEOUT_TICKS 50U
#define MOTION_VISION_TIMEOUT_TICKS 50U
#define MOTION_CROSS_TRACK_GAIN 2.0f
#define MOTION_CROSS_CORRECTION_LIMIT_MPS 0.10f
#define MOTION_RECOVERY_SPEED_MPS 0.10f
#define MOTION_START_RECOVERY_SPEED_MPS 0.15f
#define MOTION_NODE_SPEED_TOLERANCE_MPS 0.005f
#define MOTION_RECOVERY_POSITION_GAIN 2.0f
#define MOTION_RECOVERY_MIN_SPEED_MPS 0.02f
#define MOTION_VISION_STABLE_NORMALIZED_TOLERANCE 0.002f
#define MOTION_VISION_STABLE_FRAMES 3U

typedef enum
{
    MOTION_STATE_TRACK = 0,          /* 沿当前线段连续跟踪 */
    MOTION_STATE_SETTLE,            /* 减速并连续确认停稳 */
    MOTION_STATE_WAIT_VISION_IDLE,  /* 等待 50 ms 定时纠偏释放通信 */
    MOTION_STATE_VISION,            /* 节点三帧视觉校正 */
    MOTION_STATE_DWELL              /* 爆破等动作的定时等待 */
} MotionControlState;

typedef enum
{
    MOTION_RECOVERY_NONE = 0,       /* 正常线段跟踪 */
    MOTION_RECOVERY_NODE,           /* 对齐首锚点或动作节点 */
    MOTION_RECOVERY_CROSS_TRACK     /* 回到当前线段的正交投影点 */
} MotionRecoveryKind;

/* car_move_plan() 在主循环配置完成后最后置 active，中断侧随后独占更新运行状态。 */
static MotionPlan active_motion_plan;
static MotionRuntimeStats active_motion_stats;
static volatile uint8_t motion_plan_active = 0U;
static MotionControlState motion_control_state = MOTION_STATE_TRACK;
static uint16_t motion_node_index = 0U;
static uint16_t motion_state_ticks = 0U;
static uint8_t motion_stable_ticks = 0U;
static uint8_t motion_vision_stable_frames = 0U;
static uint8_t motion_node_vision_done = 0U;
static uint8_t motion_node_dwell_done = 0U;
static uint8_t motion_stop_counted = 0U;
static MotionRecoveryKind motion_recovery_kind = MOTION_RECOVERY_NONE;
static uint8_t motion_recovery_started = 0U;
static float motion_recovery_x = 0.0f;
static float motion_recovery_y = 0.0f;
static float motion_last_global_vx = 0.0f;
static float motion_last_global_vy = 0.0f;
static float motion_last_vision_x = 0.0f;
static float motion_last_vision_y = 0.0f;

/* 地图格中心转换为赛道世界坐标。 */
static void motion_grid_to_world(uint8_t grid_index, float *x, float *y)
{
    *x = (float)(grid_index % WIDTH) * 0.2f + 0.1f;
    *y = 2.4f - (float)(grid_index / WIDTH) * 0.2f - 0.1f;
}

/* 控制器内部统一使用的上下限裁剪。 */
static float motion_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

/* 计算里程计位置到当前计划节点中心的欧氏距离。 */
static float motion_distance_to_current_node(void)
{
    float node_x;
    float node_y;
    float dx;
    float dy;

    motion_grid_to_world(active_motion_plan.nodes[motion_node_index].grid_index,
                         &node_x,
                         &node_y);
    dx = node_x - global_x;
    dy = node_y - global_y;
    return sqrtf(dx * dx + dy * dy);
}

static void motion_set_walk_mode_for_segment(void)
{
    uint8_t previous;
    uint8_t current;

    if (motion_node_index == 0U)
    {
        walk_mode = 2U;
        return;
    }

    previous = active_motion_plan.nodes[motion_node_index - 1U].grid_index;
    current = active_motion_plan.nodes[motion_node_index].grid_index;
    /* 保持 0=横移、1=纵移，使原 50 ms 纠偏继续修正对应横向坐标。 */
    walk_mode = previous / WIDTH == current / WIDTH ? 0U : 1U;
}

/* 在全局坐标中做矢量限加速度，再转换为车体坐标交给现有轮速 PID。 */
static void motion_apply_global_velocity(float desired_vx, float desired_vy)
{
    float delta_x = desired_vx - motion_last_global_vx;
    float delta_y = desired_vy - motion_last_global_vy;
    float delta_norm = sqrtf(delta_x * delta_x + delta_y * delta_y);
    float old_norm = sqrtf(motion_last_global_vx * motion_last_global_vx +
                           motion_last_global_vy * motion_last_global_vy);
    float desired_norm = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy);
    float accel_limit = desired_norm + 0.0001f < old_norm
                            ? motion_plan_default_config.max_decel_mps2
                            : motion_plan_default_config.max_accel_mps2;
    float max_delta = accel_limit * MOTION_CONTROL_DT_S;
    float yaw_rad;

    if (delta_norm > max_delta && delta_norm > 0.000001f)
    {
        float scale = max_delta / delta_norm;
        delta_x *= scale;
        delta_y *= scale;
    }

    motion_last_global_vx += delta_x;
    motion_last_global_vy += delta_y;

    yaw_rad = actual_yaw * 3.1415926f / 180.0f;
    target_vx = motion_last_global_vx * cosf(yaw_rad) +
                motion_last_global_vy * sinf(yaw_rad);
    target_vy = -motion_last_global_vx * sinf(yaw_rad) +
                motion_last_global_vy * cosf(yaw_rad);
}

static void motion_stop_command(void)
{
    /* 停车过程仍经过减速度限制；爆破节点使用原有 walk_mode=4。 */
    motion_apply_global_velocity(0.0f, 0.0f);
    walk_mode = (active_motion_plan.nodes[motion_node_index].flags & MOTION_DWELL) != 0U
                    ? 4U
                    : 3U;
}

static void motion_abort_navigation(void)
{
    /* 安全中止立即清零外环指令，并通过统计量向主循环报告失败。 */
    target_vx = 0.0f;
    target_vy = 0.0f;
    motion_last_global_vx = 0.0f;
    motion_last_global_vy = 0.0f;
    walk_mode = 3U;
    active_motion_stats.aborted = 1U;
    motion_plan_active = 0U;
    navigate_flag = 0U;
}

static void motion_finish_navigation(void)
{
    /* 正常结束时路径已在终点完成停稳，因此可安全清零速度历史。 */
    target_vx = 0.0f;
    target_vy = 0.0f;
    motion_last_global_vx = 0.0f;
    motion_last_global_vy = 0.0f;
    walk_mode = 3U;
    motion_plan_active = 0U;
    navigate_flag = 0U;
}

static void motion_complete_current_node(void)
{
    /* 普通转角在此直接切段，不清零 motion_last_global_v*，从而保持连续运动。 */
    if (motion_node_index + 1U >= active_motion_plan.length)
    {
        motion_finish_navigation();
        return;
    }

    motion_node_index++;
    motion_control_state = MOTION_STATE_TRACK;
    motion_state_ticks = 0U;
    motion_stable_ticks = 0U;
    motion_vision_stable_frames = 0U;
    motion_node_vision_done = 0U;
    motion_node_dwell_done = 0U;
    motion_stop_counted = 0U;
    motion_recovery_kind = MOTION_RECOVERY_NONE;
    motion_recovery_started = 0U;
    motion_grid_to_world(active_motion_plan.nodes[motion_node_index].grid_index,
                         &target_x,
                         &target_y);
    motion_set_walk_mode_for_segment();
}

static void motion_continue_after_vision(void)
{
    MotionNode *node = &active_motion_plan.nodes[motion_node_index];

    motion_node_vision_done = 1U;
    if (motion_distance_to_current_node() > MOTION_POSITION_TOLERANCE_M)
    {
        /* 视觉更新只修正累计位姿；偏离节点后仍由里程计控制低速重新对点。 */
        active_motion_stats.recovery_count++;
        motion_recovery_kind = MOTION_RECOVERY_NODE;
        motion_recovery_started = 1U;
        motion_control_state = MOTION_STATE_TRACK;
        motion_state_ticks = 0U;
        motion_stable_ticks = 0U;
        return;
    }

    if ((node->flags & MOTION_DWELL) != 0U && !motion_node_dwell_done)
    {
        motion_control_state = MOTION_STATE_DWELL;
        motion_state_ticks = 0U;
        return;
    }
    motion_complete_current_node();
}

static void motion_continue_after_settle(void)
{
    MotionNode *node = &active_motion_plan.nodes[motion_node_index];

    if ((node->flags & MOTION_VISION_FIX) != 0U && !motion_node_vision_done)
    {
        /* 先进入等待态，让同一 PIT 周期后的 run_vision_correct() 自行复位。 */
        motion_control_state = MOTION_STATE_WAIT_VISION_IDLE;
        motion_state_ticks = 0U;
        motion_vision_stable_frames = 0U;
        return;
    }
    if ((node->flags & MOTION_DWELL) != 0U && !motion_node_dwell_done)
    {
        motion_control_state = MOTION_STATE_DWELL;
        motion_state_ticks = 0U;
        return;
    }
    motion_complete_current_node();
}

static void motion_enter_settle(void)
{
    /* 一个节点的恢复过程可能多次进入 SETTLE，但停车次数只统计一次。 */
    motion_control_state = MOTION_STATE_SETTLE;
    motion_state_ticks = 0U;
    motion_stable_ticks = 0U;
    if (!motion_stop_counted)
    {
        active_motion_stats.stop_count++;
        motion_stop_counted = 1U;
    }
}

static void motion_track_recovery_target(float speed_limit)
{
    float recovery_x;
    float recovery_y;
    float dx;
    float dy;
    float distance;
    float speed;

    if (motion_recovery_kind == MOTION_RECOVERY_CROSS_TRACK)
    {
        recovery_x = motion_recovery_x;
        recovery_y = motion_recovery_y;
    }
    else
    {
        motion_grid_to_world(active_motion_plan.nodes[motion_node_index].grid_index,
                             &recovery_x,
                             &recovery_y);
    }
    target_x = recovery_x;
    target_y = recovery_y;
    dx = recovery_x - global_x;
    dy = recovery_y - global_y;
    distance = sqrtf(dx * dx + dy * dy);

    if (distance <= MOTION_PASS_PLANE_M)
    {
        motion_enter_settle();
        motion_stop_command();
        return;
    }

    speed = motion_clamp(MOTION_RECOVERY_POSITION_GAIN * distance +
                             MOTION_RECOVERY_MIN_SPEED_MPS,
                         MOTION_RECOVERY_MIN_SPEED_MPS,
                         speed_limit);
    motion_apply_global_velocity(dx / distance * speed, dy / distance * speed);
    /* 回线运动同样标明实际横纵方向，保留定时视觉纠偏的坐标语义。 */
    walk_mode = fabsf(dx) >= fabsf(dy) ? 0U : 1U;
}

/* 横偏超限后先停车，停稳后只沿法向回到当前线段，不斜向追逐终点。 */
static void motion_schedule_cross_track_recovery(float projection_x,
                                                 float projection_y)
{
    active_motion_stats.recovery_count++;
    motion_recovery_kind = MOTION_RECOVERY_CROSS_TRACK;
    motion_recovery_started = 0U;
    motion_recovery_x = projection_x;
    motion_recovery_y = projection_y;
    motion_enter_settle();
    motion_stop_command();
}

static void motion_track_current_segment(void)
{
    MotionNode *node = &active_motion_plan.nodes[motion_node_index];
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    float direction_x;
    float direction_y;
    float segment_length;
    float progress;
    float remaining;
    float cross_error;
    float cross_correction;
    float exit_speed;
    float decel_speed;
    float desired_speed;
    float command_along_speed;
    float braking_remaining;
    float cross_abs;
    uint16_t cross_mm;

    if (motion_node_index == 0U || motion_recovery_kind != MOTION_RECOVERY_NONE)
    {
        motion_track_recovery_target(motion_node_index == 0U
                                         ? MOTION_START_RECOVERY_SPEED_MPS
                                         : MOTION_RECOVERY_SPEED_MPS);
        return;
    }

    motion_grid_to_world(active_motion_plan.nodes[motion_node_index - 1U].grid_index,
                         &start_x,
                         &start_y);
    motion_grid_to_world(node->grid_index, &end_x, &end_y);
    direction_x = end_x - start_x;
    direction_y = end_y - start_y;
    segment_length = sqrtf(direction_x * direction_x + direction_y * direction_y);
    if (segment_length <= 0.000001f)
    {
        motion_abort_navigation();
        return;
    }
    direction_x /= segment_length;
    direction_y /= segment_length;

    /* 将当前位置分解为沿线进度和横向误差。 */
    progress = (global_x - start_x) * direction_x +
               (global_y - start_y) * direction_y;
    remaining = motion_clamp(segment_length - progress, 0.0f, segment_length);
    cross_error = direction_x != 0.0f ? global_y - start_y : global_x - start_x;
    cross_abs = fabsf(cross_error);
    cross_mm = (uint16_t)motion_clamp(cross_abs * 1000.0f + 0.5f, 0.0f, 65535.0f);
    if (cross_mm > active_motion_stats.max_cross_track_mm)
        active_motion_stats.max_cross_track_mm = cross_mm;

    if (cross_abs > MOTION_POSITION_TOLERANCE_M)
    {
        /* 投影限制在线段范围内，避免越过端点后恢复到无效延长线。 */
        float recovery_progress = motion_clamp(progress, 0.0f, segment_length);
        motion_schedule_cross_track_recovery(start_x + direction_x * recovery_progress,
                                             start_y + direction_y * recovery_progress);
        return;
    }

    exit_speed = (float)node->node_speed_mmps / 1000.0f;
    command_along_speed = motion_last_global_vx * direction_x +
                          motion_last_global_vy * direction_y;

    if (progress + MOTION_PASS_PLANE_M >= segment_length)
    {
        /* 普通转角必须同时满足位置、横偏和节点速度，才允许无停车切段。 */
        if ((node->flags & MOTION_STOP) == 0U &&
            cross_abs <= MOTION_POSITION_TOLERANCE_M &&
            command_along_speed <= exit_speed + MOTION_NODE_SPEED_TOLERANCE_MPS)
        {
            motion_complete_current_node();
            return;
        }

        motion_enter_settle();
        motion_stop_command();
        return;
    }

    /* 扣除通过窗口和一个控制周期位移，补偿离散限减速度带来的制动滞后。 */
    braking_remaining = remaining - MOTION_PASS_PLANE_M -
                        fmaxf(command_along_speed, 0.0f) * MOTION_CONTROL_DT_S;
    if (braking_remaining < 0.0f)
        braking_remaining = 0.0f;
    decel_speed = sqrtf(exit_speed * exit_speed +
                        2.0f * motion_plan_default_config.max_decel_mps2 * braking_remaining);
    desired_speed = (float)node->segment_limit_mmps / 1000.0f;
    if (desired_speed > decel_speed)
        desired_speed = decel_speed;

    /* 最终指令由沿线前馈和限幅后的横向误差修正组成。 */
    cross_correction = motion_clamp(-MOTION_CROSS_TRACK_GAIN * cross_error,
                                    -MOTION_CROSS_CORRECTION_LIMIT_MPS,
                                    MOTION_CROSS_CORRECTION_LIMIT_MPS);
    if (direction_x != 0.0f)
    {
        motion_apply_global_velocity(direction_x * desired_speed, cross_correction);
        walk_mode = 0U;
    }
    else
    {
        motion_apply_global_velocity(cross_correction, direction_y * desired_speed);
        walk_mode = 1U;
    }
}

static void motion_update_settle(void)
{
    float actual_speed = sqrtf(local_encoder_vx * local_encoder_vx +
                               local_encoder_vy * local_encoder_vy);

    motion_stop_command();
    motion_state_ticks++;
    if (actual_speed < MOTION_STOP_SPEED_MPS)
        motion_stable_ticks++;
    else
        motion_stable_ticks = 0U;

    /* 编码器速度连续三个周期低于阈值后，再检查位置并决定恢复或继续。 */
    if (motion_stable_ticks >= MOTION_STOP_STABLE_TICKS)
    {
        if (motion_recovery_kind == MOTION_RECOVERY_CROSS_TRACK)
        {
            if (!motion_recovery_started)
            {
                motion_recovery_started = 1U;
                motion_control_state = MOTION_STATE_TRACK;
                motion_state_ticks = 0U;
                motion_stable_ticks = 0U;
                return;
            }

            motion_recovery_kind = MOTION_RECOVERY_NONE;
            motion_recovery_started = 0U;
            motion_control_state = MOTION_STATE_TRACK;
            motion_state_ticks = 0U;
            motion_stable_ticks = 0U;
            motion_set_walk_mode_for_segment();
            return;
        }

        if (motion_distance_to_current_node() > MOTION_POSITION_TOLERANCE_M)
        {
            if (motion_recovery_kind != MOTION_RECOVERY_NODE)
                active_motion_stats.recovery_count++;
            motion_recovery_kind = MOTION_RECOVERY_NODE;
            motion_recovery_started = 1U;
            motion_control_state = MOTION_STATE_TRACK;
            motion_state_ticks = 0U;
            motion_stable_ticks = 0U;
            return;
        }
        motion_recovery_kind = MOTION_RECOVERY_NONE;
        motion_recovery_started = 0U;
        motion_continue_after_settle();
        return;
    }

    if (motion_state_ticks >= MOTION_STOP_TIMEOUT_TICKS)
        /* 500 ms 内无法停稳视为控制异常，禁止继续执行动作。 */
        motion_abort_navigation();
}

/* 只在定时纠偏标志和全局通信都空闲时，节点状态机才拥有视觉请求。 */
static void motion_update_wait_vision_idle(void)
{
    motion_stop_command();
    active_motion_stats.vision_wait_ms += 10U;
    motion_state_ticks++;

    if (vision_correct_flag == 0U && global_infor_type == 5U)
    {
        want_global_infor(0);
        motion_control_state = MOTION_STATE_VISION;
        motion_state_ticks = 0U;
        motion_vision_stable_frames = 0U;
        return;
    }

    if (motion_state_ticks >= MOTION_VISION_TIMEOUT_TICKS)
    {
        /* 等待超时不改写通信状态，避免取消仍在进行的定时纠偏请求。 */
        active_motion_stats.vision_timeout_count++;
        wrong_over_time++;
        motion_continue_after_vision();
    }
}

static void motion_update_vision(void)
{
    motion_stop_command();
    active_motion_stats.vision_wait_ms += 10U;
    motion_state_ticks++;

    if (global_infor_type == 5U)
    {
        if (motion_vision_stable_frames == 0U)
        {
            motion_last_vision_x = car_location[0];
            motion_last_vision_y = car_location[1];
            motion_vision_stable_frames = 1U;
        }
        else if (fabsf(car_location[0] - motion_last_vision_x) <=
                     MOTION_VISION_STABLE_NORMALIZED_TOLERANCE &&
                 fabsf(car_location[1] - motion_last_vision_y) <=
                     MOTION_VISION_STABLE_NORMALIZED_TOLERANCE)
        {
            motion_vision_stable_frames++;
        }
        else
        {
            motion_last_vision_x = car_location[0];
            motion_last_vision_y = car_location[1];
            motion_vision_stable_frames = 1U;
        }

        /* 连续三帧归一化坐标变化不超过阈值后，才接受节点视觉坐标。 */
        if (motion_vision_stable_frames >= MOTION_VISION_STABLE_FRAMES)
        {
            global_x = 3.2f * (car_location[0] + motion_last_vision_x) * 0.5f;
            global_y = 2.4f - 2.4f * (car_location[1] + motion_last_vision_y) * 0.5f;
            motion_continue_after_vision();
            return;
        }
        want_global_infor(0);
    }

    if (motion_state_ticks >= MOTION_VISION_TIMEOUT_TICKS)
    {
        /* 此状态中的请求由节点状态机发起，因此超时后由它负责释放通信。 */
        active_motion_stats.vision_timeout_count++;
        wrong_over_time++;
        global_infor_type = 5U;
        want_global_infor(5);
        motion_continue_after_vision();
    }
}

static void motion_update_dwell(void)
{
    /* 等待期间持续输出停车命令，爆破默认等待 1310 ms。 */
    MotionNode *node = &active_motion_plan.nodes[motion_node_index];
    motion_stop_command();
    motion_state_ticks++;
    if ((uint32_t)motion_state_ticks * 10U >= node->dwell_ms)
    {
        motion_node_dwell_done = 1U;
        motion_complete_current_node();
    }
}

static void motion_plan_navigation_update(void)
{
    /* 新状态机与旧 navigation_update() 共用 10 ms 调度入口。 */
    if (!motion_plan_active || motion_node_index >= active_motion_plan.length)
    {
        motion_abort_navigation();
        return;
    }

    active_motion_stats.elapsed_ms += 10U;
    switch (motion_control_state)
    {
    case MOTION_STATE_TRACK:
        motion_track_current_segment();
        break;
    case MOTION_STATE_SETTLE:
        motion_update_settle();
        break;
    case MOTION_STATE_WAIT_VISION_IDLE:
        motion_update_wait_vision_idle();
        break;
    case MOTION_STATE_VISION:
        motion_update_vision();
        break;
    case MOTION_STATE_DWELL:
        motion_update_dwell();
        break;
    default:
        motion_abort_navigation();
        break;
    }
}

const MotionRuntimeStats *motion_plan_stats(void)
{
    /* 主循环应在 navigate_flag 清零后读取，以获得一条完整路径的统计。 */
    return &active_motion_stats;
}

void navigation_update(void)
{
    if (motion_plan_active)
    {
        /* 新计划有效时优先执行；否则完整保留旧路径控制用于 A/B。 */
        motion_plan_navigation_update();
        return;
    }

    // 参数：&对象, 目标坐标, 到达目标时的末速度
    // SecondOrder_Set_Follow_Cal(&planner_x, target_x, 0.0f);
    // SecondOrder_Set_Follow_Cal(&planner_y, target_y, 0.0f);

    static float last_error_x = 0.0f;
    static float last_error_y = 0.0f;

    float error_x = target_x - global_x;
    float error_y = target_y - global_y;

    float global_target_vx = kp_position_x * (target_x - global_x) + kd_position_x * (error_x - last_error_x);
    float global_target_vy = kp_position_y * (target_y - global_y) + kd_position_y * (error_y - last_error_y);

    last_error_x = error_x;
    last_error_y = error_y;

    // float global_target_vx = planner_x.v + kp_position_x * (planner_x.p - global_x);
    // float global_target_vy = planner_y.v + kp_position_y * (planner_y.p - global_y);

    float max_speed = 1.0f;
    if (global_target_vx > max_speed)
        global_target_vx = max_speed;
    if (global_target_vx < -max_speed)
        global_target_vx = -max_speed;
    if (global_target_vy > max_speed)
        global_target_vy = max_speed;
    if (global_target_vy < -max_speed)
        global_target_vy = -max_speed;
    // 加速度限制
    if (last_global_target_vx >= amax * 0.01f)
    {
        if (global_target_vx >= last_global_target_vx + amax * 0.01f)
        {
            global_target_vx = last_global_target_vx + amax * 0.01f;
        }
    }
    else if (last_global_target_vx <= -amax * 0.01f)
    {
        if (global_target_vx <= last_global_target_vx - amax * 0.01f)
        {
            global_target_vx = last_global_target_vx - amax * 0.01f;
        }
    }
    else
    {
        if (global_target_vx >= amax * 0.02f)
        {
            global_target_vx = amax * 0.02f;
        }
        else if (global_target_vx <= -amax * 0.02f)
        {
            global_target_vx = -amax * 0.02f;
        }
    }

    if (last_global_target_vy >= amax * 0.01f)
    {
        if (global_target_vy >= last_global_target_vy + amax * 0.01f)
        {
            global_target_vy = last_global_target_vy + amax * 0.01f;
        }
    }
    else if (last_global_target_vy <= -amax * 0.01f)
    {
        if (global_target_vy <= last_global_target_vy - amax * 0.01f)
        {
            global_target_vy = last_global_target_vy - amax * 0.01f;
        }
    }
    else
    {
        if (global_target_vy >= amax * 0.02f)
        {
            global_target_vy = amax * 0.02f;
        }
        else if (global_target_vy <= -amax * 0.02f)
        {
            global_target_vy = -amax * 0.02f;
        }
    }

    // 记忆目标速度赋值，为了求加速度
    last_global_target_vx = global_target_vx;
    last_global_target_vy = global_target_vy;
    // float speed_mix = sqrtf(global_target_vx * global_target_vx + global_target_vy * global_target_vy);
    // speed_angle = atan2f(target_y - global_y, target_x - global_x);
    // global_target_vx = speed_mix * cosf(speed_angle);
    // global_target_vy = speed_mix * sinf(speed_angle);

    float dx = target_x - global_x;
    float dy = target_y - global_y;
    float distance = sqrtf(dx * dx + dy * dy);

    if (navigate_flag == 1)
    {

        uint8_t is_last_point = (current_path == path_length - 1) ? 1 : 0; // 判断是否是最后一个路径点

        if (is_last_point)
        {
            // if (stop_flag == 0 && distance <= 0.015f)
            // {
            //     stop_flag = 1; // 开启手刹
            // }
            // else if (stop_flag == 1 && distance > 0.015f)
            // {
            //     stop_flag = 0; // 关闭手刹
            // }
            if (distance <= 0.015f && stop_flag == 0)
            {
                stop_flag = 1; // 开启手刹
                if (walk_mode != 4)
                {
                    walk_mode = 3;
                }
            }
            if (stop_flag == 1)
            {
                target_vx = 0.0f;
                target_vy = 0.0f;
                if (walk_mode == 4)
                {
                    if (count_A <= 130)
                    {
                        count_A++;
                        return;
                    }
                }
                else
                {
                    if (count_A <= 5)
                    {
                        count_A++;
                        return;
                    }

                    if (first_time_fix == 1)
                    {
                        if (wait_for_loc == 0)
                        {
                            if (global_infor_type != 5)
                            {
                                return;
                            }
                            want_global_infor(0);
                            time_vision = time_line;
                            wait_for_loc = 1;
                        }
                        uint8_t if_longtime = 0;
                        if (wait_for_loc == 1)
                        {
                            if (time_line - time_vision >= 0.5f)
                            {
                                wrong_over_time++;
                                if_longtime = 1;
                                wait_for_loc = 0;
                                global_infor_type = 5;
                                want_global_infor(5);
                            }
                            else
                            {
                                if (global_infor_type == 5)
                                {
                                    wait_for_loc = 0;
                                }
                                else
                                {
                                    return;
                                }
                            }
                        }

                        if (!if_longtime)
                        {
                            if (car_location[0] - vision_x >= -0.002f && car_location[0] - vision_x <= 0.002f &&
                                car_location[1] - vision_y >= -0.002f &&
                                car_location[1] - vision_y <= 0.002f)
                            {
                                loac_test++;
                            }
                            else
                            {
                                loac_test = 0;
                                vision_x = car_location[0];
                                vision_y = car_location[1];
                            }

                            if (loac_test >= 3)
                            {
                                float dx = global_x - 3.2f * car_location[0];
                                float dy = global_y - (2.4f - 2.4f * car_location[1]);

                                global_x = 3.2f * (car_location[0] + vision_x) * 0.5f;
                                global_y = 2.4f - 2.4f * (car_location[1] + vision_y) * 0.5f;
                            }
                            else
                            {
                                return;
                            }
                        }
                        vision_x = -1;
                        vision_y = -1;
                        loac_test = 0;
                        first_time_fix = 0;
                        stop_flag = 0;
                        return;
                    }
                }

                navigate_flag = 0;
                walk_mode = 3;
                stop_flag = 0;
                count_A = 0;
                got_angle = 0;
                first_time_fix = 1;
                last_global_target_vx = 0;
                last_global_target_vy = 0;
                for (int k = 0; k < 4; k++)
                {
                    pid[k].duty_out = 0.0f;   // 清空已经累加的 PWM 输出
                    pid[k].error_last = 0.0f; // 清空历史误差
                    pid[k].error_acc = 0.0f;  // 清空更早的历史误差
                }

                return; // 开启手刹了 就不继续往下算了 等下个周期再算新的目标点
            }
            else
            {
                float yaw_rad = actual_yaw * 3.1415926f / 180.0f;
                target_vx = global_target_vx * cosf(yaw_rad) + global_target_vy * sinf(yaw_rad);
                target_vy = -global_target_vx * sinf(yaw_rad) + global_target_vy * cosf(yaw_rad);
            }
        }
        else
        {
            if (distance <= 0.015f && stop_flag == 0)
            {
                if (walk_mode != 4)
                {
                    walk_mode = 3;
                }
                stop_flag = 1; // 开启手刹
            }
            if (stop_flag == 1)
            {
                target_vx = 0.0f;
                target_vy = 0.0f;

                if (walk_mode == 4)
                {
                    if (count <= 130)
                    {
                        count++;
                        return;
                    }
                }
                else
                {
                    if (count <= 5)
                    {
                        count++;
                        return;
                    }
                    if (first_time_fix == 1)
                    {
                        if (vision_point_num == 0)
                        {
                            if (wait_for_loc == 0)
                            {
                                if (global_infor_type != 5)
                                {
                                    return;
                                }
                                want_global_infor(0);
                                time_vision = time_line;
                                wait_for_loc = 1;
                            }
                            uint8_t if_longtime = 0;
                            if (wait_for_loc == 1)
                            {
                                if (time_line - time_vision >= 0.5f)
                                {
                                    wrong_over_time++;
                                    if_longtime = 1;
                                    global_infor_type = 5;
                                    wait_for_loc = 0;
                                    want_global_infor(5);
                                }
                                else
                                {
                                    if (global_infor_type == 5)
                                    {
                                        wait_for_loc = 0;
                                    }
                                    else
                                    {
                                        return;
                                    }
                                }
                            }
                            if (!if_longtime)
                            {
                                if (car_location[0] - vision_x >= -0.002f && car_location[0] - vision_x <= 0.002f &&
                                    car_location[1] - vision_y >= -0.002f && car_location[1] - vision_y <= 0.002f)
                                {
                                    loac_test++;
                                }
                                else
                                {
                                    loac_test = 0;
                                    vision_x = car_location[0];
                                    vision_y = car_location[1];
                                }
                                if (loac_test >= 3)
                                {
                                    float dx = global_x - 3.2f * car_location[0];
                                    float dy = global_y - (2.4f - 2.4f * car_location[1]);
                                    // if (sqrtf(dx * dx + dy * dy) >= 0.25f)
                                    // {
                                    //     // 如果视觉坐标和里程计坐标差距超过 25cm 就不修正了
                                    // }
                                    // else
                                    // {
                                    global_x = 3.2f * (car_location[0] + vision_x) * 0.5f;
                                    global_y = 2.4f - 2.4f * (car_location[1] + vision_y) * 0.5f;
                                    // actual_yaw = car_angel - 90;
                                    // }
                                }
                                else
                                {
                                    return;
                                }
                            }
                        }
                        loac_test = 0;
                        vision_x = -1;
                        vision_y = -1;
                        //判定该节点是否需要运动矫正一下再前往下一个点
                        if (fabs(path_queue_x[current_path + 1] - 3.1) <= 0.001f && fabs(path_queue_y[current_path + 1] + 0.7) <= 0.001f)
                        {
                            if (fabs(path_queue_x[current_path + 2] - path_queue_x[current_path]) <= 0.001f)
                            {
                                if (fabs(path_queue_x[current_path] - global_x) >= 0.015f)
                                {
                                    first_time_fix = 0;
                                    stop_flag = 0;
                                    return;
                                }
                            }
                            else if (fabs(path_queue_y[current_path + 2] - path_queue_y[current_path]) <= 0.001f)
                            {
                                if (fabs(path_queue_y[current_path] - global_y) >= 0.015f)
                                {
                                    first_time_fix = 0;
                                    stop_flag = 0;
                                    return;
                                }
                            }
                        }
                        else
                        {
                            if (fabs(path_queue_x[current_path + 1] - path_queue_x[current_path]) <= 0.001f)
                            {
                                if (fabs(path_queue_x[current_path] - global_x) >= 0.015f)
                                {
                                    first_time_fix = 0;
                                    stop_flag = 0;
                                    return;
                                }
                            }
                            else if (fabs(path_queue_y[current_path + 1] - path_queue_y[current_path]) <= 0.001f)
                            {
                                if (fabs(path_queue_y[current_path] - global_y) >= 0.015f)
                                {
                                    first_time_fix = 0;
                                    stop_flag = 0;
                                    return;
                                }
                            }
                        }
                    }
                }

                count = 0;
                stop_flag = 0;
                current_path++;
                first_time_fix = 1;
                got_angle = 0;
                if (fabs(path_queue_x[current_path] - 3.1) <= 0.001f && fabs(path_queue_y[current_path] + 0.7) <= 0.001f)
                {
                    target_x = path_queue_x[current_path - 1];
                    target_y = path_queue_y[current_path - 1];
                    walk_mode = 4;
                }
                else
                {
                    target_x = path_queue_x[current_path];
                    target_y = path_queue_y[current_path];
                    float d_point_x = target_x - path_queue_x[current_path - 1];
                    float d_point_y = target_y - path_queue_y[current_path - 1];
                    if (d_point_x != 0 && d_point_y != 0)
                    {
                        walk_mode = 2;
                    }
                    else if (d_point_x != 0 && d_point_y == 0)
                    {
                        walk_mode = 0;
                    }
                    else if (d_point_x == 0 && d_point_y != 0)
                    {
                        walk_mode = 1;
                    }
                    else
                    {
                        walk_mode = 3;
                    }

                    speed_angle = atan2f(d_point_y, d_point_x); // 计算角度

                    // 下面是统计小车跑过的节点个数，从而判断是否应该让视觉矫正(n个节点矫正一次)
                    vision_point_num = (vision_point_num + 1) % VISION_CORRECT_T;
                }
                last_global_target_vx = 0;
                last_global_target_vy = 0;
                // 下面是更改小车从一个节点走到另一个节点走的状态(横向，纵向，斜向)以及角度信息

                return; // 如果到达了当前目标点了 就不继续往下算了 等下个周期再算新的目标点
            }
            else
            {
                float yaw_rad = actual_yaw * 3.1415926f / 180.0f;
                target_vx = global_target_vx * cosf(yaw_rad) + global_target_vy * sinf(yaw_rad);
                target_vy = -global_target_vx * sinf(yaw_rad) + global_target_vy * cosf(yaw_rad);
            }
        }
    }
    // float global_target_vx = dynamic_speed * dx / distance;
    // float global_target_vy = dynamic_speed * dy / distance;
}
/**
 * @brief 在中断调用这个函数来不断计算
 *
 */
void move_control_task(void)
{
    odometry_update();   // 更新里程计
    navigation_update(); // 更新导航
    // vision_xy_update_task();                         // 更新视觉坐标

    float max_yaw_step = 10.0f;

    while (final_target_yaw > 180.0f)
        final_target_yaw -= 360.0f;
    while (final_target_yaw < -180.0f)
        final_target_yaw += 360.0f;
    if (target_yaw < final_target_yaw - max_yaw_step)
    {
        target_yaw += max_yaw_step;
    }
    else if (target_yaw > final_target_yaw + max_yaw_step)
    {
        target_yaw -= max_yaw_step;
    }
    else
    {
        target_yaw = final_target_yaw; // 误差极小时，直接锁定目标
    }

    float vz = yaw_pid_calculate();                  // 计算航向角控制输出
    wheel_speed_calculate(target_vx, target_vy, vz); // 计算轮子速度并输出
}
// /**
//  * @brief 控制小车移动

//  *
//  * @param x 目标 x 坐标 单位 m
//  * @param y 目标 y 坐标 单位 m
//  * @param yaw 目标航向角 单位 °
//  * @param m 模式 0: 只移动不改变航向角 1: 移动并且让航向角朝向目标点
//  */
// void car_move(float x, float y,float yaw, uint8_t m){
//     target_x = x;
//     target_y = y;
//     target_yaw = yaw;
//     mode = m;

//     move_flag = 1;
// }
extern SokobanContext engine_ctx;

uint8_t if_check = 0;
/**
 * @brief
 *
 * @param path 传入的路径点数组
 * @param yaw 目标航向角
 * @param m 模式
 */
void car_move(WaypointPath *path, float yaw, uint8_t m)
{
    /* 显式退出新状态机，确保旧接口可作为独立回退基线运行。 */
    motion_plan_active = 0U;
    if (path->length == 0 || path->length > 100)
        return;

    for (int i = 0; i < path->length; i++)
    {
        path_queue_x[i] = (path->points[i] % 16) * 0.2f + 0.1f;
        path_queue_y[i] = 2.4 - (path->points[i] / 16) * 0.2f - 0.1f;
    }

    path_length = path->length;
    current_path = 0;
    mode = m;

    target_x = path_queue_x[0];
    target_y = path_queue_y[0];

    // 下面是更改小车从一个节点走到另一个节点走的角度信息
    float d_point_x = target_x - global_x;
    float d_point_y = target_y - global_y;
    speed_angle = atan2f(d_point_y, d_point_x); // 计算角度
    final_target_yaw = yaw;

    yaw_arrived_flag = 0;
    navigate_flag = 1;
    stop_flag = 0;
}

bool car_move_plan(const MotionPlan *plan, float yaw, uint8_t m)
{
    float anchor_x;
    float anchor_y;
    float anchor_dx;
    float anchor_dy;
    float anchor_distance;
    uint16_t i;

    /* 即使计划由外部构造，也在启动前再次校验容量、动作标志和正交几何。 */
    if (plan == NULL || plan->length == 0U || plan->length > MAP_SIZE)
        return false;
    for (i = 0U; i < plan->length; i++)
    {
        uint8_t flags = plan->nodes[i].flags;
        if (plan->nodes[i].grid_index >= MAP_SIZE ||
            (flags & (uint8_t)~(MOTION_STOP | MOTION_MACRO_END |
                                MOTION_VISION_FIX | MOTION_DWELL)) != 0U ||
            ((flags & (MOTION_MACRO_END | MOTION_VISION_FIX | MOTION_DWELL)) != 0U &&
             (flags & MOTION_STOP) == 0U) ||
            (plan->nodes[i].dwell_ms != 0U && (flags & MOTION_DWELL) == 0U))
        {
            return false;
        }
        if (i > 0U)
        {
            uint8_t previous = plan->nodes[i - 1U].grid_index;
            uint8_t current = plan->nodes[i].grid_index;
            if (previous == current ||
                (previous / WIDTH != current / WIDTH &&
                 previous % WIDTH != current % WIDTH))
            {
                return false;
            }
        }
    }

    active_motion_plan = *plan;
    memset(&active_motion_stats, 0, sizeof(active_motion_stats));
    motion_grid_to_world(active_motion_plan.nodes[0].grid_index,
                         &anchor_x,
                         &anchor_y);
    anchor_dx = anchor_x - global_x;
    anchor_dy = anchor_y - global_y;
    anchor_distance = sqrtf(anchor_dx * anchor_dx + anchor_dy * anchor_dy);

    /* 首锚点在 5 cm 内直接跳过，否则以 0.15 m/s 先对齐锚点。 */
    motion_node_index = plan->length > 1U &&
                                anchor_distance <= MOTION_START_ANCHOR_TOLERANCE_M
                            ? 1U
                            : 0U;
    motion_control_state = MOTION_STATE_TRACK;
    motion_state_ticks = 0U;
    motion_stable_ticks = 0U;
    motion_vision_stable_frames = 0U;
    motion_node_vision_done = 0U;
    motion_node_dwell_done = 0U;
    motion_stop_counted = 0U;
    motion_recovery_kind = motion_node_index == 0U
                               ? MOTION_RECOVERY_NODE
                               : MOTION_RECOVERY_NONE;
    motion_recovery_started = motion_node_index == 0U ? 1U : 0U;
    motion_recovery_x = anchor_x;
    motion_recovery_y = anchor_y;
    motion_last_global_vx = 0.0f;
    motion_last_global_vy = 0.0f;
    target_vx = 0.0f;
    target_vy = 0.0f;
    mode = m;
    final_target_yaw = yaw;
    yaw_arrived_flag = 0U;
    stop_flag = 0U;
    motion_grid_to_world(active_motion_plan.nodes[motion_node_index].grid_index,
                         &target_x,
                         &target_y);
    motion_set_walk_mode_for_segment();
    /* active 最后置位，避免 PIT 读取到尚未初始化完成的状态。 */
    motion_plan_active = 1U;
    navigate_flag = 1U;
    return true;
}

void car_turn(float yaw)
{
    final_target_yaw = yaw;
    yaw_arrived_flag = 0;
}

void car_move_point(float x, float y, float yaw, uint8_t m)
{
    /* 点到点接口继续使用旧控制器，调用时先退出计划状态机。 */
    motion_plan_active = 0U;
    path_length = 1;
    current_path = 0;
    mode = m;
    path_queue_x[0] = x;
    path_queue_y[0] = y;

    target_x = x;
    target_y = y;

    // 下面是更改小车从一个节点走到另一个节点走的角度信息
    float d_point_x = target_x - global_x;
    float d_point_y = target_y - global_y;
    speed_angle = atan2f(d_point_y, d_point_x); // 计算角度
    final_target_yaw = yaw;

    yaw_arrived_flag = 0;
    navigate_flag = 1;
    stop_flag = 0;
}
/**
 * @brief 停车
 *
 */
void car_stop()
{
    target_vx = 0.0f;
    target_vy = 0.0f;
    /* 同步清理新控制器速度历史，防止下一次导航从旧速度起步。 */
    motion_last_global_vx = 0.0f;
    motion_last_global_vy = 0.0f;
    motion_plan_active = 0U;
    walk_mode = 3U;
    navigate_flag = 0;

    // 同步一下虚拟规划器，防止切回导航时暴冲
    // planner_x.p = global_x;
    // planner_y.p = global_y;
    // planner_x.v = 0.0f;
    // planner_y.v = 0.0f;
}

// if (vision_angle_switch)
// {
//     if (got_angle == 0)
//     {
//         if (global_infor_type == 5)
//         {
//             want_global_infor(2);
//             got_angle = 1;
//         }
//         else
//         {
//             return;
//         }
//     }
//     if (got_angle == 1)
//     {
//         if (global_infor_type == 5)
//         {
//             got_angle = 0;
//         }
//         else
//         {
//             uart_write_byte(UART_GLOBAL_INDEX, 0xFE);
//             return;
//         }
//     }
//     if (got_angle != 2)
//     {
//         if (car_angel - vision_angle <= 2 && car_angel - vision_angle >= -2)
//         {
//             angle_test++;
//         }
//         else
//         {
//             angle_test = 0;
//             vision_angle = car_angel;
//         }

//         if (angle_test >= 3)
//         {
//             actual_yaw = car_angel - 90;
//             while (actual_yaw > 180.0f)
//                 actual_yaw -= 360.0f;
//             while (actual_yaw < -180.0f)
//                 actual_yaw += 360.0f;
//             vision_angle = 999;
//             angle_test = 0;
//             got_angle = 2;
//         }
//         else
//         {
//             return;
//         }
//     }
// }
