#include "move_control.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "Set_Follow.h"
#include <math.h>
#include "sokoban_engine.h"
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

float actual_yaw = 0.0f;       // 实际航向角 单位度
float target_yaw = 0.0f;       // 瞬时目标航向角 单位度
float final_target_yaw = 0.0f; // 最终目标航向角 单位度

float target_vx = 0.0f; // 车模坐标系下横移速度 单位 m/s
float target_vy = 0.0f; // 车模坐标系下前进速度 单位 m/s

float Kp_yaw = 0.06f; // 航向角 P 参数
float Kd_yaw = 0.1f;  // 航向角 D 参数

float global_x = 0.25f;      // 车模全局 x 坐标 单位 m
float global_y = 1.2f;      // 车模全局 y 坐标 单位 m
uint8_t move_flag = 0;      // 1表示车子在移动 0 表示车子在停止
uint8_t mode = 0;           // 两种运动模式
float target_x = 0.3f;      // 目标 x 坐标 单位 m
float target_y = 1.2f;      // 目标 y 坐标 单位 m
float min_distance = 0.01f; // 距离小于这个值就认为到达目标点了 单位 m
float kp_position_x = 4.0f;
float kp_position_y = 4.0f;
float kd_position_x = 0.0f;
float kd_position_y = 0.0f;

float path_queue_x[MOTION_PATH_CAPACITY];
float path_queue_y[MOTION_PATH_CAPACITY];
uint16_t path_length = 0;  // 路径总点数
uint16_t current_path = 0; // 当前正在追第几个点
volatile uint8_t navigate_flag = 0; // 1: 正在追路径 0: 没有路径需要追
volatile uint8_t yaw_arrived_flag = 0;
uint8_t stop_flag = 0; // 1: 手刹 0: 不手刹

PID_TypeDef pid[4];
uint8_t vision_angle_switch = 0;
uint8_t wrong_time = 0;
// 小车在横着走竖着走还是斜着走,0是横，1是竖；2是斜3是不走,4等待炸弹延时
uint8_t walk_mode = 3;
extern uint8_t vision_correct_flag;
uint8_t wrong_over_time = 0;

uint8_t ban_map_check_ifgetVisionLoc = 1;
uint8_t ban_last_vision_correct = 0;
void move_control_init(void)
{
    for (int i = 0; i < 4; i++)
    {
        pid_init(&pid[i], 40.0f, 2.0f, 0.0f);
    }
}
// 浮点数转整数四舍五入，返回int类型结果
int round_int(float num)
{
    return (int)(num + 0.5f);
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

    if (vz > 0.9f)
        vz = 0.9f;
    if (vz < -0.9f)
        vz = -0.9f;

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

float vx_encoder_index = 0.962f;
float vy_encoder_index = 0.96f;
/**
 * @brief 里程计更新
 *
 */
void odometry_update(void)
{
    // 车模坐标系下的速度
    local_encoder_vx = (actual_v[LF] + actual_v[RB] - actual_v[LB] - actual_v[RF]) / 4.0f * vx_encoder_index;
    local_encoder_vy = (actual_v[LF] + actual_v[LB] + actual_v[RF] + actual_v[RB]) / 4.0f * vy_encoder_index;

    // 角度换成弧度
    float yaw_rad = actual_yaw * 3.1415926f / 180.0f;

    // 计算全局坐标系下的速度
    float global_actual_vx = -local_encoder_vy * sinf(yaw_rad) + local_encoder_vx * cosf(yaw_rad);
    float global_actual_vy = local_encoder_vy * cosf(yaw_rad) + local_encoder_vx * sinf(yaw_rad);

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
// 记录小车跑过的格子数是否应该让视觉矫正
float vision_distance_num = 0;
// 从一个节点到另一个节点的角度信息，以及走的状态(横向，纵向，斜向)

float vision_distance_num_plus = 0;
float speed_angle = 0.0f; //(弧度制)

float global_target_vx = 0.0f;
float global_target_vy = 0.0f;
float last_error_x = 0.0f;
float last_error_y = 0.0f;
float last_global_target_vx = 0.0f; // 全局坐标系下的目标速度
float last_global_target_vy = 0.0f; // 全局坐标系下
float amax = 3.2f;                  // 最大加速度 m/s^2
float max_speed = 1.2f;             // 最大速度 m/s
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

/**
 * @brief 转点，时一些全局变量归零
 *
 */
void turnpoint_update(void)
{
    stop_flag = 0;
    count_A = 0;
    count = 0;
    got_angle = 0;
    first_time_fix = 1;
    last_global_target_vx = 0;
    last_global_target_vy = 0;
    global_target_vx = 0;
    global_target_vy = 0;
    last_error_x = 0;
    last_error_y = 0;
    for (int k = 0; k < 4; k++)
    {
        pid[k].duty_out = 0.0f;   // 清空已经累加的 PWM 输出
        pid[k].error_last = 0.0f; // 清空历史误差
        pid[k].error_acc = 0.0f;  // 清空更早的历史误差
    }
}
void navigation_update(void)
{
    if (navigate_flag != 1)
    {
        return;
    }

    float error_x = target_x - global_x;
    float error_y = target_y - global_y;

    global_target_vx = kp_position_x * (target_x - global_x) + kd_position_x * (error_x - last_error_x);
    global_target_vy = kp_position_y * (target_y - global_y) + kd_position_y * (error_y - last_error_y);

    last_error_x = error_x;
    last_error_y = error_y;

    // 速度与加速度限制，仅当加速时有加速度限制，将要到达节点减速时由位置环决定
    speed_limit();
    float v_whole = sqrtf(global_target_vx*global_target_vx+global_target_vy*global_target_vy);
    if(v_whole>=max_speed+0.1f){
        global_target_vx = max_speed*global_target_vx/v_whole;
        global_target_vy = max_speed*global_target_vy/v_whole;
    }
    // 记忆目标速度赋值，为了求加速度
    last_global_target_vx = global_target_vx;
    last_global_target_vy = global_target_vy;

    /*
     float speed_mix = sqrtf(global_target_vx * global_target_vx + global_target_vy * global_target_vy);
     speed_angle = atan2f(target_y - global_y, target_x - global_x);
     global_target_vx = speed_mix * cosf(speed_angle);
     global_target_vy = speed_mix * sinf(speed_angle);
    */
    float dx = target_x - global_x;
    float dy = target_y - global_y;
    float distance = sqrtf(dx * dx + dy * dy);

    if (distance <= 0.017f && stop_flag == 0)
    {
        stop_flag = 1; // 开启手刹
        if (walk_mode != 4)
        {
            walk_mode = 3;
        }
    }
    if (!stop_flag)
    {
        float yaw_rad = actual_yaw * 3.1415926f / 180.0f;
        target_vx = global_target_vx * cosf(yaw_rad) + global_target_vy * sinf(yaw_rad);
        target_vy = -global_target_vx * sinf(yaw_rad) + global_target_vy * cosf(yaw_rad);
        return;
    }

    // 后面是小车进入停止状态
    uint8_t is_last_point = (current_path == path_length - 1) ? 1 : 0; // 判断是否是最后一个路径点
    if (is_last_point)
    {
        target_vx = 0.0f;
        target_vy = 0.0f;
        if (walk_mode == 4)
        {
            if (count_A <= 100)
            {
                count_A++;
                return;
            }
        }
        else
        {
            if (count_A <= 4)
            {
                count_A++;
                return;
            }

            if (first_time_fix == 1)
            {
                if (!ban_last_vision_correct&&CORRECT_MODE>0)
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

                        if (loac_test >= 4)
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
        }

        navigate_flag = 0;
        walk_mode = 3;
        turnpoint_update();
        return; // 开启手刹了 就不继续往下算了 等下个周期再算新的目标点
    }
    else if (!is_last_point)
    {

        target_vx = 0.0f;
        target_vy = 0.0f;

        if (walk_mode == 4)
        {
            if (count <= 100)
            {
                count++;
                return;
            }
        }
        else
        {
            if (count <= 3)
            {
                count++;
                return;
            }
            if (first_time_fix == 1)
            {
                //   path_queue_x[i] = (path->points[i] % 16) * 0.2f + 0.1f;
                // path_queue_y[i] = 2.4 - (path->points[i] / 16) * 0.2f - 0.1f;
                int path_X = round_int((target_x - 0.1f) / 0.2f);
                int path_Y = round_int((2.3f - target_y) / 0.2f);
                int car_to = path_X + path_Y * WIDTH;
                int path_X_to = round_int((path_queue_x[current_path + 1] - 0.1f) / 0.2f);
                int path_Y_to = round_int((2.3f - path_queue_y[current_path + 1]) / 0.2f);
                int car_to_to = path_X_to + path_Y_to * WIDTH;
                if (car_to_to < 0 || car_to_to >= MAP_SIZE)
                { // 说明是炸弹延时特殊点
                    if (current_path + 2U < path_length)
                    {
                        car_to_to = round_int((path_queue_x[current_path + 2U] - 0.1f) / 0.2f) +
                                    round_int((2.3f - path_queue_y[current_path + 2U]) / 0.2f) * WIDTH;
                    }
                    else
                    {
                        car_to_to = car_to;
                    }
                }
                if (car_to >= 0 && car_to < MAP_SIZE && car_to_to >= 0 && car_to_to < MAP_SIZE &&
                    vision_distance_num_plus >= VISION_CORRECT_DISTANCE && CORRECT_MODE == 2)
                {


                    // 节点是否视觉矫正判定的相关参数归零
                    vision_point_num = 0;
                    vision_distance_num = 0;
                    vision_distance_num_plus = 0;
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
                        if (loac_test >= 4)
                        {
                            // if (sqrtf(dx * dx + dy * dy) >= 0.35f)
                            // {
                            //     // 如果视觉坐标和里程计坐标差距超过 35cm 就不修正了
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
                    loac_test = 0;
                    vision_x = -1;
                    vision_y = -1;
                    first_time_fix = 0;
                    // 判定该节点 是否在视觉获取坐标之后，根据结果微调小车
                    // 原则是如果在这个点偏离的方向恰好是下一个点的方向,就不需要矫正,如果偏离的方向和下一个点的方向不一致,就需要矫正
                    if (check_correctOn_vision() == 1)
                    {
                        stop_flag = 0;
                        return;
                    }
                }
            }
        }

        current_path++;
        turnpoint_update();
        // 下面是更改小车从一个节点走到另一个节点走的状态(横向，纵向，斜向)以及角度信息;以及累加vision_point_num与vision_distance_num
        walk_mode_set();
        return; // 如果到达了当前目标点了 就不继续往下算了 等下个周期再算新的目标点
    }
}
// 速度，加速度限制
void speed_limit(void)
{
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
        else if (global_target_vx <= -amax * 0.01f)
        {
            global_target_vx = -amax * 0.01f;
        }
    }
    else if (last_global_target_vx <= -amax * 0.01f)
    {
        if (global_target_vx <= last_global_target_vx - amax * 0.01f)
        {
            global_target_vx = last_global_target_vx - amax * 0.01f;
        }
        else if (global_target_vx >= amax * 0.01f)
        {
            global_target_vx = amax * 0.01f;
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
        else if (global_target_vy <= -amax * 0.01f)
        {
            global_target_vy = -amax * 0.01f;
        }
    }
    else if (last_global_target_vy <= -amax * 0.01f)
    {
        if (global_target_vy <= last_global_target_vy - amax * 0.01f)
        {
            global_target_vy = last_global_target_vy - amax * 0.01f;
        }
        else if (global_target_vy >= amax * 0.01f)
        {
            global_target_vy = amax * 0.01f;
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
}

// 确立到下一个节点的行走模式，累加vision_point_num与vision_distance_num
void walk_mode_set(void)
{
    if (fabsf(path_queue_x[current_path] - 3.1f) <= 0.001f && fabsf(path_queue_y[current_path] + 0.7f) <= 0.001f)
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
            vision_distance_num += 99.0f;
            vision_distance_num_plus += 99.0f;
        }
        else if (d_point_x != 0 && d_point_y == 0)
        {
            walk_mode = 0;
            vision_distance_num += fabsf(d_point_x);
            vision_distance_num_plus += fabsf(d_point_x);
        }
        else if (d_point_x == 0 && d_point_y != 0)
        {
            walk_mode = 1;
            vision_distance_num += fabsf(d_point_y);
            vision_distance_num_plus += fabsf(d_point_y);
        }
        else
        {
            walk_mode = 3;
        }

        speed_angle = atan2f(d_point_y, d_point_x); // 计算角度
        // 下面是统计小车跑过的节点个数，从而判断是否应该让视觉矫正(n个节点矫正一次)
        vision_point_num = (vision_point_num + 1) % VISION_CORRECT_T;
    }
}
/**
 * @brief 是否在视觉获取坐标之后，根据结果微调小车
 *
 */
uint8_t check_correctOn_vision(void)
{
    if (fabsf(path_queue_x[current_path + 1] - 3.1f) <= 0.001f && fabsf(path_queue_y[current_path + 1] + 0.7f) <= 0.001f)
    {
        if (current_path + 2U >= path_length)
            return 0;

        if (fabsf(path_queue_x[current_path + 2] - path_queue_x[current_path]) <= 0.001f)
        {
            if (fabsf(path_queue_x[current_path] - global_x) >= 0.015f)
            {
                first_time_fix = 0;
                stop_flag = 0;
                return 1;
            }
        }
        else if (fabsf(path_queue_y[current_path + 2] - path_queue_y[current_path]) <= 0.001f)
        {
            if (fabsf(path_queue_y[current_path] - global_y) >= 0.015f)
            {
                first_time_fix = 0;
                stop_flag = 0;
                return 1;
            }
        }
    }
    else
    {
        if (fabsf(path_queue_x[current_path + 1] - path_queue_x[current_path]) <= 0.001f)
        {
            if (fabsf(path_queue_x[current_path] - global_x) >= 0.015f)
            {
                first_time_fix = 0;
                stop_flag = 0;
                return 1;
            }
        }
        else if (fabsf(path_queue_y[current_path + 1] - path_queue_y[current_path]) <= 0.001f)
        {
            if (fabsf(path_queue_y[current_path] - global_y) >= 0.015f)
            {
                first_time_fix = 0;
                stop_flag = 0;
                return 1;
            }
        }
    }
    return 0;
}

// 计算目标转速
float get_target_vz(void)
{
    float max_yaw_step = 10.0f;

    while (final_target_yaw > 180.0f)
        final_target_yaw -= 360.0f;
    while (final_target_yaw < -180.0f)
        final_target_yaw += 360.0f;

    if (target_yaw < final_target_yaw - max_yaw_step)
    {
        if (final_target_yaw - target_yaw > 180)
        {
            target_yaw -= max_yaw_step;
        }
        else
        {
            target_yaw += max_yaw_step;
        }
    }
    else if (target_yaw > final_target_yaw + max_yaw_step)
    {
        if (target_yaw - final_target_yaw > 180)
        {
            target_yaw += max_yaw_step;
        }
        else
        {
            target_yaw -= max_yaw_step;
        }
    }
    else
    {
        target_yaw = final_target_yaw; // 误差极小时，直接锁定目标
    }

    while (target_yaw > 180.0f)
        target_yaw -= 360.0f;
    while (target_yaw < -180.0f)
        target_yaw += 360.0f;

    float vz = yaw_pid_calculate(); // 计算航向角控制输出
    return vz;
}
/**
 * @brief 在中断调用这个函数来不断计算
 *
 */
void move_control_task(void)
{
    odometry_update();                                            // 更新里程计
    navigation_update();                                          // 更新导航
    wheel_speed_calculate(target_vx, target_vy, get_target_vz()); // 计算轮子速度并输出
}

/**
 * @brief
 *
 * @param path 传入的路径点数组
 * @param yaw 目标航向角
 * @param m 模式
 */
bool car_move(const WaypointPath *path, float yaw, uint8_t m)
{
    if (path == NULL || path->length == 0 || path->length > MOTION_PATH_CAPACITY)
        return false;

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
    return true;
}

void car_turn(float yaw)
{
    final_target_yaw = yaw;
    yaw_arrived_flag = 0;
}

void car_move_point(float x, float y, float yaw, uint8_t m)
{
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
void car_stop(void)
{
    target_vx = 0.0f;
    target_vy = 0.0f;
    navigate_flag = 0;
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
