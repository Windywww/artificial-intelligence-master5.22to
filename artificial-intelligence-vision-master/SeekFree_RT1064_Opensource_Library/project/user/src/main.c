/*********************************************************************************************************************
 * RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 RT1064DVL6A 开源库的一部分
 *
 * RT1064DVL6A 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.33
 * 适用平台          RT1064DVL6A
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-21        SeekFree            first version
 ********************************************************************************************************************/
#include "fsl_common.h"

#include "zf_common_headfile.h"
#include "move_control.h"
#include "motor.h"
#include "encoder.h"
#include "myUart.h"
#include "ips.h"
#include "WIFI2SPI.h"
#include <math.h>
#include "sokoban_engine.h"

#define ROUND_COUNT 3U
#define ROUND_CLEAR_WAIT_MS 600U
#define ROUND_MAP_SETTLE_MS 1200U
#define START_ZONE_GRID_INDEX (0U + 6U * WIDTH)
extern void imu_calibrate(void);

volatile float time_line = 0.0f;
SokobanContext engine_ctx;
static void sync_car_position();

static void reset_round_runtime(void)
{
    car_stop();
    got_map_flag = 0;
    first_time_fix = 1;
    image_rx_state = 0;
    final_image_index = 0;
    count_A = 0;
    count = 0;
    got_angle = 0;
    angle_test = 0;
    wait_for_loc = 0;
    loac_test = 0;
    vision_x = -1.0f;
    vision_y = -1.0f;
}

/* 回到发车区获取地图；仍有箱子或未占用目的地时等待视觉地图刷新。 */
static void return_to_start_zone(void)
{
    first_time_fix = 2;
    vision_angle_switch = 0;
    vision_run_correct_switch = 0;
    ban_map_check_ifgetVisionLoc = 1;
    car_move_point(0.3, 1.2, angle, 0);
    while (navigate_flag)
    {
        if (IF_WIFI)
        {
            wifi_task();
        }
    }
    first_time_fix = 2;
    system_delay_ms(50);
    while (global_infor_type != 5)
    {
        if (IF_WIFI)
        {
            wifi_task();
        }
    }
    want_global_infor(1);
    while (global_infor_type != 5)
    {
        switch (global_infor_type)
        {
        case 1:
            uart_write_byte(UART_GLOBAL_INDEX, 0xBB);
            break;

        case 2:
            uart_write_byte(UART_GLOBAL_INDEX, 0xFE);
            break;
        }
        if (IF_WIFI)
        {
            wifi_task();
        }
    }

    uint8_t if_whitemap = 1;
    for (int i = 0; i < 192; i++)
    {
        if (final_map_data[i] == 2 || final_map_data[i] == 3)
        {
            if_whitemap = 0;
            break;
        }
    }
    if (!if_whitemap)
    {
        system_delay_ms(3000);
    }
    system_delay_ms(40);
    sync_car_position();
    car_move_point(0.3, 1.2, angle, 0);
    while (navigate_flag)
    {
        if (IF_WIFI)
        {
            wifi_task();
        }
    }
}

// 等 navigate_flag 变 0
static void wait_navigation(void)
{
    while (navigate_flag)
    {
        if (IF_WIFI)
        {
            wifi_task();
        }
    }
}
// 等 global_infor_type 变 5
static void wait_global_info(void)
{
    while (global_infor_type != 5)
    {
        if (IF_WIFI)
        {
            wifi_task();
        }
    }
}

// 要一次地图
static void request_round_map(void)
{
    got_map_flag = 0;
    wait_global_info();

    want_global_infor(1);
    float this_time = time_line;
    while (global_infor_type != 5)
    {
        if (time_line - this_time >= 5)
        {
            break;
        }
        switch (global_infor_type)
        {
        case 1:
            uart_write_byte(UART_GLOBAL_INDEX, 0xBB);
            break;

        case 2:
            uart_write_byte(UART_GLOBAL_INDEX, 0xFE);
            break;
        }
        if (IF_WIFI)
        {
            wifi_task();
        }
    }
}
// 矫正一次target_x target_y,阻塞式
uint8_t same_time = 0;
float main_vision_position_x = 99.0f;
float main_vision_position_y = 99.0f;

static void sync_car_position(void)
{
    while (same_time <= 5)
    {
        wait_global_info();
        want_global_infor(0);
        while (global_infor_type != 5)
        {
            if (IF_WIFI)
            {
                wifi_task();
            }
        }
        if (fabs(car_location[0] - main_vision_position_x) <= 0.002f && fabs(car_location[1] - main_vision_position_y) <= 0.002f)
        {
            same_time++;
        }
        else
        {
            main_vision_position_x = car_location[0];
            main_vision_position_y = car_location[1];
            same_time = 0;
        }
    }
    same_time = 0;
    main_vision_position_x = 99.0f;
    main_vision_position_y = 99.0f;
    global_x = 3.2f * car_location[0];
    global_y = 2.4f - 2.4f * car_location[1];
    target_x = global_x;
    target_y = global_y;
}
// 矫正一次车角度，阻塞式,多次采样
float main_vision_angle = 999;

static void sync_car_angle(void)
{
    while (same_time <= 2)
    {
        wait_global_info();
        want_global_infor(2);
        while (global_infor_type != 5)
        {
            if (IF_WIFI)
            {
                wifi_task();
            }
            uart_write_byte(UART_GLOBAL_INDEX, 0xFE);
        }
        if (fabs(car_angel - main_vision_angle) <= 2)
        {
            same_time++;
        }
        else
        {
            main_vision_angle = car_angel;
            same_time = 0;
        }
    }
    same_time = 0;
    main_vision_angle = 999;
    if (fabs(actual_yaw - car_angel + 90) >= 5)
    {
        actual_yaw = car_angel - 90;
        while (actual_yaw > 180.0f)
            actual_yaw -= 360.0f;
        while (actual_yaw < -180.0f)
            actual_yaw += 360.0f;
    }
}

static uint8_t if_in_carStart()
{
    if (global_x <= 0.5f && global_y >= 1.2f - 0.33f && global_y <= 1.2f + 0.33f)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// 小车在某一关卡复活的次数
uint8_t resurgence_time = 0;
/**
 * @brief 跑一关
 * @param round_index 0第一关 1第二关 2第三关
 * @return 1 成功 0失败
 */
uint8_t goal_loac[MAX_BOXES];

static uint8_t run_round(uint8_t round_index)
{
    ban_map_check_ifgetVisionLoc = 1;
    WaypointPath path = {0};
    vision_run_correct_switch = 0;
    reset_round_runtime();
    clear_relation_in();

    vision_angle_switch = 0;
    if (if_in_carStart())
    {
        car_move_point(global_x + 0.25f, global_y, angle, 0);
    }
    wait_navigation();
    if (round_index >= 0 && IF_VISION_ANGLE)
    {
        sync_car_angle();
    }

    // 获取地图
    while (1)
    {
        request_round_map();
        if (resurgence_time > 0)
        {
            break;
        }
        uint8_t map_ok = 0;
        uint8_t box_num = 0;
        uint8_t goal_num = 0;
        for (int i = 0; i < 192; i++)
        {
            if (final_map_data[i] == 2)
            {
                box_num++;
            }
            else if (final_map_data[i] == 3)
            {
                goal_loac[goal_num] = i;
                goal_num++;
            }
        }
        if (box_num == goal_num && box_num > 0)
        {
            map_ok = 1;
        }
        if (map_ok)
        {
            break;
        }
    }

    if (!got_map_flag)
    {
        return 0;
    }
    ban_map_check_ifgetVisionLoc = 0;
    vision_run_correct_switch = 0;
    if (!build_map_info(&engine_ctx, final_map_data, round_index == 0U ? 0U : 1U))
    {
        return 0;
    }
    if (!engine_ctx.map_valid)
    {
        return 0;
    }
    goal_box_giveRelation(&engine_ctx);
    lost = 1;
    if (!solve(&engine_ctx))
    {
        build_map_info(&engine_ctx, final_map_data, 0);
        if (!solve(&engine_ctx))
        {
            return 0;
        }
    }
    goal_box_giveRelation(&engine_ctx);
    generate_path(&engine_ctx, &path);
    if (path.length == 0)
    {
        return 0;
    }

    lost = 66;
    car_move(&path, angle, 0);
    wait_navigation();
    if (resurgence_time < CHECK_TIME_MAX)
    {
        resurgence_time++;
        if (run_round(round_index))
        {
            return 1;
        }
    }

    /* 只有重试次数耗尽后才回发车区检查最终地图。 */
    return_to_start_zone();
    return 1;
}
// 停车
static void fault_stop(void)
{
    car_stop();
    while (1)
    {
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    // debug_init();                  // 调试端口初始化
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(600); // 等待主板其他外设上电完成
    if (IF_WIFI)
    {
        myWIFI2SPI_Init();
    }
    encoder_init();
    // key_init(5);
    // uart1_init();

    imu660rb_init();
    // ips_init();
    // map_init();
    myuart_init();
    system_delay_ms(50);
    imu_calibrate();
    motor_init();

    move_control_init();
    system_delay_ms(50);

    pit_ms_init(PIT_CH0, 10);            // 速度闭环和姿态闭环
    pit_ms_init(PIT_CH1, 5);             // 陀螺仪积分
    interrupt_set_priority(PIT_IRQn, 1); // 设置 PIT 中断优先级为 1

    interrupt_global_enable(0);

    system_delay_ms(600);

    if (CORRECT_MODE != 0)
    {
        sync_car_position();
    }
    // 循环跑三关
    for (uint8_t round_index = 0; round_index < ROUND_COUNT; round_index++)
    {
        resurgence_time = 0;
        if (!run_round(round_index))
        {
            return_to_start_zone();
        }
    }
    // 第三关完成后保持停车，同时继续处理通信。
    car_stop();
    while (1)
    {
        if (IF_WIFI)
        {
            wifi_task();
        }
    }
    // NVIC_SystemReset(); // 复位
    return 0;
}

float bias_z = 0;
float bias_x = 0;
float bias_y = 0;
static int calibrated = 0;
float ax_average = 0;
float ay_average = 0;
float az_average = 0;

int16_t imu_gyro_z_max = 0;
int16_t imu_gyro_z_min = 0;
void imu_calibrate()
{
    int sum_z = 0;
    int sum_x = 0;
    int sum_y = 0;

    for (int i = 0; i < 500; i++)
    {
        imu660rb_get_gyro();
        imu660rb_get_acc();
        if (imu660rb_gyro_z > imu_gyro_z_max)
        {
            imu_gyro_z_max = imu660rb_gyro_z;
        }
        if (imu660rb_gyro_z < imu_gyro_z_min)
        {
            imu_gyro_z_min = imu660rb_gyro_z;
        }
        sum_z += imu660rb_gyro_z;
        sum_x += imu660rb_gyro_x;
        sum_y += imu660rb_gyro_y;
        ax_average = sqrtf((ax_average * ax_average * i + imu660rb_acc_x * imu660rb_acc_x) / (i + 1));
        ay_average = sqrtf((ay_average * ay_average * i + imu660rb_acc_y * imu660rb_acc_y) / (i + 1));
        az_average = sqrtf((az_average * az_average * i + imu660rb_acc_z * imu660rb_acc_z) / (i + 1));
        system_delay_ms(2);
    }
    float a_all = sqrtf(ax_average * ax_average + ay_average * ay_average + az_average * az_average);
    ax_average = ax_average / a_all;
    ay_average = ay_average / a_all;
    az_average = az_average / a_all;
    bias_z = (float)sum_z / 500;
    bias_x = (float)sum_x / 500;
    bias_y = (float)sum_y / 500;
    calibrated = 1;
}

float real_yaw_rate = 0;
void pit_ch1_handler(void)
{

    imu660rb_get_gyro(); // 获取陀螺仪测量数值
    if (!calibrated)
    {
        return;
    }
    float gx_deg_s = 0;
    float gy_deg_s = 0;
    if (!IMU_FLAT)
    {
        gx_deg_s = (float)(imu660rb_gyro_x) / imu660rb_transition_factor[1];
        gy_deg_s = (float)(imu660rb_gyro_y) / imu660rb_transition_factor[1];
    }
    float gz_deg_s = 0;
    if (imu660rb_gyro_z < imu_gyro_z_min || imu660rb_gyro_z > imu_gyro_z_max)
    {
        gz_deg_s = (float)(imu660rb_gyro_z) / imu660rb_transition_factor[1];
        gz_deg_s -= bias_z / imu660rb_transition_factor[1];
    }
    if (!IMU_FLAT)
    {
        float vertical_omega = gx_deg_s * ax_average + gy_deg_s * ay_average + gz_deg_s * az_average;
        real_yaw_rate = vertical_omega;
        actual_yaw -= real_yaw_rate * 0.005f;
    }
    else
    {
        actual_yaw -= gz_deg_s * 0.005f;
    }

    while (actual_yaw > 180.0f)
        actual_yaw -= 360.0f;
    while (actual_yaw < -180.0f)
        actual_yaw += 360.0f;
}

float time_for_vision_loac = 0;
uint8_t vision_correct_flag = 0;
uint8_t vision_run_correct_switch = 0;
float time_vision_main = 0;
void run_vision_correct()
{
    if (vision_run_correct_switch == 1 && walk_mode != 3 && walk_mode != 4)
    {
        if (vision_correct_flag == 0)
        {
            time_for_vision_loac += 0.01f;
            if (time_for_vision_loac >= 0.05f)
            {
                vision_correct_flag = 1;
                time_for_vision_loac = 0;
            }
        }
        else if (vision_correct_flag == 1)
        {
            if (global_infor_type != 5)
            {
                return;
            }
            want_global_infor(0);
            time_vision_main = time_line;
            vision_correct_flag = 2;
        }
        else if (vision_correct_flag == 2)
        {
            // 超时判定
            if (time_line - time_vision_main >= 0.1f)
            {
                wrong_over_time++;
                vision_correct_flag = 0;
                global_infor_type = 5;
                want_global_infor(5);
                return;
            }
            if (global_infor_type != 5)
            {
                return;
            }
            else
            {
                if (walk_mode == 0)
                {
                    global_y = 2.4 - 2.4 * car_location[1];
                }
                else if (walk_mode == 1)
                {
                    global_x = 3.2 * car_location[0];
                }
                vision_correct_flag = 0;
            }
        }
    }
    else
    {
        if (vision_correct_flag == 2)
        {
            global_infor_type = 5;
            want_global_infor(5);
        }
        vision_correct_flag = 0;
        time_for_vision_loac = 0;
    }
}

void pit_ch0_handler(void)
{
    myuart_timeout_tick_10ms();
    // 不要删，统计时间点用
    time_line += 0.01f; // 每10ms增加0.01s
    move_control_task();
    if (IF_RUN_CORRECT)
    {
        run_vision_correct();
    }
}
