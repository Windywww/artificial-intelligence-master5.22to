#include "move_control.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "Set_Follow.h"
#include <math.h>
#include "sokoban_engine.h"

uint32_t encoder_ports[4] = {ENCODER_1, ENCODER_2, ENCODER_3, ENCODER_4};
int16_t encoder_data[4] = {0, 0, 0, 0};
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
float Kd_yaw = 0.3f;  // 航向角 D 参数

float global_x = 0.3f; // 车模全局 x 坐标 单位 m
float global_y = 0.3f; // 车模全局 y 坐标 单位 m

// 积分系数
float k_x = 1.0f;
float k_y = 1.0f;

uint8_t move_flag = 0;      // 1表示车子在移动 0 表示车子在停止
uint8_t mode = 0;           // 两种运动模式
float target_x = 0.1f;      // 目标 x 坐标 单位 m
float target_y = 1.2f;      // 目标 y 坐标 单位 m
float min_distance = 0.01f; // 距离小于这个值就认为到达目标点了 单位 m
float kp_position_x = 3.5f;
float kp_position_y = 3.0f;
float kd_position_x = 0.0f;
float kd_position_y = 0.0f;

float path_queue_x[100];
float path_queue_y[100];
uint16_t path_length = 0;  // 路径总点数
uint16_t current_path = 0; // 当前正在追第几个点
uint8_t navigate_flag = 0; // 1: 正在追路径 0: 没有路径需要追
uint8_t yaw_arrived_flag = 0;
uint8_t stop_flag = 0; // 1: 手刹 0: 不手刹
uint8_t vision_xy_update_flag_first = 0;

float v_x = 0.6f;
float v_y = 0.6f;
float a_x = 0.25f;
float a_y = 0.25f;

PID_TypeDef pid[4];

SecondOrder_Set_Follow_t planner_x; // x 轴 s 曲线跟随规划器
SecondOrder_Set_Follow_t planner_y; // y 轴 s 曲线跟随规划器

void move_control_init()
{
    for (int i = 0; i < 4; i++)
    {
        pid_init(&pid[i], 40.0f, 2.0f, 0.0f);
    }

    // 初始化 s 曲线跟随规划器 参数分别是：最大速度、最大加速度、计算周期,加速度原来是1.5
    SecondOrder_Set_Follow_init(&planner_x, v_x, a_x, 0.02f);
    SecondOrder_Set_Follow_init(&planner_y, v_y, a_y, 0.02f);

    // 将规划器的当前位置设置为全局坐标系下的当前位置 这样规划器就不会一上来就算一个很大的误差了
    planner_x.p = global_x;
    planner_y.p = global_y;
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

void lhn_odometry_update()
{
    global_x = car_location[0];
    global_y = car_location[1];
}

float local_encoder_vx = 0.0f;
float local_encoder_vy = 0.0f;
float local_imu_vx = 0.0f;
float local_imu_vy = 0.0f;
/**
 * @brief 里程计更新
 *
 */
void odometry_update()
{
    // 车模坐标系下的速度
    local_encoder_vx = (actual_v[LF] + actual_v[RB] - actual_v[LB] - actual_v[RF]) / 4.0f;
    local_encoder_vy = (actual_v[LF] + actual_v[LB] + actual_v[RF] + actual_v[RB]) / 4.0f;

    // imu积分推算出的速度
    // static float local_imu_vx = 0.0f;
    // static float local_imu_vy = 0.0f;

    float dt = 0.02f; // 20ms

    // 速度积分
    local_imu_vx += imu660rb_acc_x * dt;
    local_imu_vy += imu660rb_acc_y * dt;

    if (fabs(target_vx) == 0.0f && fabs(target_vy) == 0.0f &&
        fabs(local_encoder_vx) < 0.02f && fabs(local_encoder_vy) < 0.02f)
    {
        local_imu_vx = 0.0f;
        local_imu_vy = 0.0f;
        local_encoder_vx = 0.0f; // 彻底锁死
        local_encoder_vy = 0.0f;
    }

    float encoder_weight_x = 0.8f; // 编码器权重 80%
    float encoder_weight_y = 0.8f; // 编码器权重 80%

    static float last_yaw = 0.0f;
    float yaw_change = actual_yaw - last_yaw;
    last_yaw = actual_yaw;

    // 如果小车正在剧烈旋转（20ms内转动超过 1.5度）
    if (yaw_change > 1.5f || yaw_change < -1.5f)
    {
        // 把编码器算出来的虚假平移速度削弱掉 90%
        local_encoder_vx *= 0.1f;
        local_encoder_vy *= 0.1f;
    }

    // 角度换成弧度
    float yaw_rad = actual_yaw * 3.1415926f / 180.0f;

    // 计算全局坐标系下的速度
    float global_actual_vx = -local_encoder_vy * sinf(yaw_rad) + local_encoder_vx * cosf(yaw_rad);
    float global_actual_vy = local_encoder_vy * cosf(yaw_rad) + local_encoder_vx * sinf(yaw_rad);

    // 速度积分得到位置
    global_x += global_actual_vx * 0.02f * k_x;
    global_y += global_actual_vy * 0.02f * k_y;
}

float vision_x = 0.0f;
float vision_y = 0.0f;
float vision_yaw = 0.0f;

uint8_t vision_xy_update_flag = 0;  // 视觉数据更新标志位：1表示有新数据，0表示已处理
uint8_t vision_yaw_update_flag = 0; // 视觉航向角更新标志位：1表示有新数据，0表示已处理

/**
 * @brief 更新视觉坐标
 *
 */
void vision_xy_update_task(void)
{
    if (vision_xy_update_flag == 1)
    {
        float odom_weight = 1.0f - vision_weight;

        global_x = global_x * odom_weight + vision_x * vision_weight;
        global_y = global_y * odom_weight + vision_y * vision_weight;

        planner_x.p = planner_x.p * odom_weight + vision_x * vision_weight;
        planner_y.p = planner_y.p * odom_weight + vision_y * vision_weight;

        vision_xy_update_flag = 0; // 清除标志位
    }
}
/**
 * @brief 更新视觉航向角
 *
 */
void vision_yaw_update_task(void)
{
    if (vision_yaw_update_flag == 1)
    {
        float vision_weight = 0.2f;               // 视觉权重 20%
        float odom_weight = 1.0f - vision_weight; // 陀螺仪权重 80%

        actual_yaw = actual_yaw * odom_weight + vision_yaw * vision_weight;

        while (actual_yaw > 180.0f)
            actual_yaw -= 360.0f;
        while (actual_yaw < -180.0f)
            actual_yaw += 360.0f;

        vision_yaw_update_flag = 0; // 清除标志位
    }
}
/**
 * @brief 全局导航
 *
 */
void navigation_update(void)
{
    if (move_flag == 1) // 车在上班
    {
        // 参数：&对象, 目标坐标, 到达目标时的末速度
        SecondOrder_Set_Follow_Cal(&planner_x, target_x, 0.0f);
        SecondOrder_Set_Follow_Cal(&planner_y, target_y, 0.0f);

        static float last_error_x = 0.0f;
        static float last_error_y = 0.0f;

        float error_x = planner_x.p - global_x;
        float error_y = planner_y.p - global_y;

        float global_target_vx = planner_x.v + kp_position_x * (planner_x.p - global_x) + kd_position_x * (error_x - last_error_x);
        float global_target_vy = planner_y.v + kp_position_y * (planner_y.p - global_y) + kd_position_y * (error_y - last_error_y);

        last_error_x = error_x;
        last_error_y = error_y;

        // float global_target_vx = planner_x.v + kp_position_x * (planner_x.p - global_x);
        // float global_target_vy = planner_y.v + kp_position_y * (planner_y.p - global_y);

        float max_speed = 0.8f;
        if (global_target_vx > max_speed)
            global_target_vx = max_speed;
        if (global_target_vx < -max_speed)
            global_target_vx = -max_speed;
        if (global_target_vy > max_speed)
            global_target_vy = max_speed;
        if (global_target_vy < -max_speed)
            global_target_vy = -max_speed;

        float dx = target_x - global_x;
        float dy = target_y - global_y;
        float distance = sqrtf(dx * dx + dy * dy);

        if (navigate_flag == 1)
        {
            uint8_t is_last_point = (current_path == path_length - 1) ? 1 : 0; // 判断是否是最后一个路径点

            if (is_last_point)
            {

                if (stop_flag == 0 && distance <= 0.04f)
                {
                    stop_flag = 1; // 开启手刹
                }
                else if (stop_flag == 1 && distance > 0.05f)
                {
                    stop_flag = 0; // 关闭手刹
                }

                if (stop_flag == 1)
                {
                    target_vx = 0.0f;
                    target_vy = 0.0f;

                    last_error_x = planner_x.p - global_x;
                    last_error_y = planner_y.p - global_y;

                    static uint8_t count = 0;

                    if (count <= 9)
                    {
                        count++;
                        return;
                    }
                    count = 0;

                    // want_global_infor(0);
                    // while (global_infor_type != 5)
                    // {
                    // }

                    // global_x = 3.2f * car_location[0];
                    // global_y = 2.4f - 2.4f * car_location[1];
                    // target_x = global_x;
                    // target_y = global_y;
                    navigate_flag = 0;
                    for (int k = 0; k < 4; k++)
                    {
                        pid[k].duty_out = 0.0f;   // 清空已经累加的 PWM 输出
                        pid[k].error_last = 0.0f; // 清空历史误差
                        pid[k].error_prev = 0.0f; // 清空更早的历史误差
                    }

                    return; // 开启手刹了 就不继续往下算了 等下个周期再算新的目标点
                }
            }
            else
            {
                if (distance <= 0.04f)
                {
                    target_vx = 0.0f;
                    target_vy = 0.0f;

                    static uint8_t count = 0;

                    if (count <= 9)
                    {
                        count++;
                        return;
                    }
                    count = 0;

                    want_global_infor(0);
                    while (global_infor_type != 5)
                    {
                    }

                    global_x = vision_x;
                    global_y = vision_y;
                    target_x = global_x;
                    target_y = global_y;

                    current_path++;
                    target_x = path_queue_x[current_path];
                    target_y = path_queue_y[current_path];
                    return; // 如果到达了当前目标点了 就不继续往下算了 等下个周期再算新的目标点
                }
            }
        }

        float yaw_rad = actual_yaw * 3.1415926f / 180.0f;
        // float global_target_vx = dynamic_speed * dx / distance;
        // float global_target_vy = dynamic_speed * dy / distance;

        target_vx = global_target_vx * cosf(yaw_rad) + global_target_vy * sinf(yaw_rad);
        target_vy = -global_target_vx * sinf(yaw_rad) + global_target_vy * cosf(yaw_rad);

        if (mode == 0)
        {
            // target_yaw = 0;
        }
        else if (mode == 1)
        {
            target_yaw = -atan2f(dx, dy) * 180.0f / 3.1415926f;

            while (target_yaw > 180.0f)
                target_yaw -= 360.0f;
            while (target_yaw < -180.0f)
                target_yaw += 360.0f;
        }
        //}
    }
    else // 车在下班
    {
        car_stop();
    }
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
    vision_yaw_update_task(); // 更新视觉航向角

    float max_yaw_step = 10.0f;

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

    while (final_target_yaw > 180.0f)
        final_target_yaw -= 360.0f;
    while (final_target_yaw < -180.0f)
        final_target_yaw += 360.0f;
        
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
    if (path->length == 0 || path->length > 100)
        return;

    // for (int i = 0; i < path->length; i++)
    // {
    //     path_queue_x[i] = (path->points[i] % 16) * 0.2f + 0.1f;
    //     path_queue_y[i] = 2.4 - (path->points[i] / 16) * 0.2f - 0.1f;
    // }

    for (int i = 0; i < path->length; i++)
    {
        uint8_t curr_pos = path->points[i];

        // 1. 计算出原始的网格正中心物理坐标
        float px = (curr_pos % 16) * 0.2f + 0.1f;
        float py = (curr_pos / 16) * 0.2f + 0.1f;

        // 2. 判断小车当前的行驶方向 (dx, dy)
        int dx = 0, dy = 0;
        if (i < path->length - 1)
        { // 还没到终点，看下一个点确定方向
            dx = (path->points[i + 1] % 16) - (curr_pos % 16);
            dy = (path->points[i + 1] / 16) - (curr_pos / 16);
        }
        else if (i > 0)
        { // 到了终点，看上一个点确定方向
            dx = (curr_pos % 16) - (path->points[i - 1] % 16);
            dy = (curr_pos / 16) - (path->points[i - 1] / 16);
        }

        // ==========================================
        // 🌟 魔法 1：侧向防刮蹭偏移 (左右/上下躲避 5cm)
        // ==========================================
        // 注意这里调用了队友写好的 check_obstacle 函数！
        if (dx != 0 && dy == 0) // 小车正在X轴(左右)移动，检查上下两边有没有东西
        {
            if (curr_pos >= 16 && check_obstacle(&engine_ctx, curr_pos - 16))
                py += 0.05f; // 上方有危险，向下躲
            if (curr_pos < MAP_SIZE - 16 && check_obstacle(&engine_ctx, curr_pos + 16))
                py -= 0.05f; // 下方有危险，向上躲
        }
        else if (dy != 0 && dx == 0) // 小车正在Y轴(上下)移动，检查左右两边有没有东西
        {
            if (curr_pos % 16 != 0 && check_obstacle(&engine_ctx, curr_pos - 1))
                px += 0.05f; // 左方有危险，向右躲
            if (curr_pos % 16 != 15 && check_obstacle(&engine_ctx, curr_pos + 1))
                px -= 0.05f; // 右方有危险，向左躲
        }

        // ==========================================
        // 🌟 魔法 2：推箱子“过推补偿” (前后延伸 5cm)
        // ==========================================
        // 只有在 m == 2 (推箱子模式)，且走到最后一个网格时，才发动延伸！
        if (m == 2 && i == path->length - 1 && path->length > 1)
        {
            if (dx > 0)
                px += 0.05f; // 正在往右推，终点往右多怼 5cm
            else if (dx < 0)
                px -= 0.05f; // 正在往左推，终点往左多怼 5cm

            if (dy > 0)
                py += 0.05f; // 正在往下推，终点往下多怼 5cm
            else if (dy < 0)
                py -= 0.05f; // 正在往上推，终点往上多怼 5cm
        }

        // 3. 将最终计算好的、绝对安全的物理坐标存入你的底层队列
        py = 2.4f - py;
        path_queue_x[i] = px;
        path_queue_y[i] = py;
    }

    path_length = path->length;
    current_path = 0;
    mode = m;

    target_x = path_queue_x[0];
    target_y = path_queue_y[0];

    final_target_yaw = yaw;

    yaw_arrived_flag = 0;
    move_flag = 1;
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

    move_flag = 0;
    navigate_flag = 0;

    // 同步一下虚拟规划器，防止切回导航时暴冲
    planner_x.p = global_x;
    planner_y.p = global_y;
    planner_x.v = 0.0f;
    planner_y.v = 0.0f;
}
