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
uint8_t navigate_flag = 0; // 1: 正在追路径 0: 没有路径需要追
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
    local_encoder_vx = (actual_v[LF] + actual_v[RB] - actual_v[LB] - actual_v[RF]) / 4.0f * vx_encoder_index;
    local_encoder_vy = (actual_v[LF] + actual_v[LB] + actual_v[RF] + actual_v[RB]) / 4.0f * vy_encoder_index;

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
float amax = 0.8f;                  // 最大加速度 m/s^2

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
void navigation_update(void)
{
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

    float max_speed = 1.2f;
    if (global_target_vx > max_speed)
        global_target_vx = max_speed;
    if (global_target_vx < -max_speed)
        global_target_vx = -max_speed;
    if (global_target_vy > max_speed)
        global_target_vy = max_speed;
    if (global_target_vy < -max_speed)
        global_target_vy = -max_speed;
    // 加速度限制
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
            if (distance <= 0.02f && stop_flag == 0)
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
                        // 判定该节点是否需要运动矫正一下再前往下一个点
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
void car_stop()
{
    target_vx = 0.0f;
    target_vy = 0.0f;
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