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
extern uint8_t vision_run_correct_switch;
extern void imu_calibrate(void);

float time_line = 0.0f;
SokobanContext engine_ctx;

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

static void return_to_start_zone(void)
{
    first_time_fix = 2;
    vision_angle_switch = 0;
    vision_run_correct_switch = 0;
    car_move_point(global_x, 1.2, angle, 0);
    while (navigate_flag)
    {
        wifi_task();
    }

    first_time_fix = 2;
    car_move_point(0.3, 1.2, angle, 0);
    while (navigate_flag)
    {
        wifi_task();
    }
    first_time_fix = 2;
}

// 等 navigate_flag 变 0
static void wait_navigation(void)
{
    while (navigate_flag)
    {
        wifi_task();
    }
}
// 等 global_infor_type 变 5
static void wait_global_info(void)
{
    while (global_infor_type != 5)
    {
        wifi_task();
    }
}
// 要一次地图
static void request_round_map(void)
{
    got_map_flag = 0;
    wait_global_info();
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
        wifi_task();
    }
}
// 矫正一次target_x target_y,阻塞式
static void sync_car_position(void)
{
    wait_global_info();
    want_global_infor(0);
    wait_global_info();

    global_x = 3.2f * car_location[0];
    global_y = 2.4f - 2.4f * car_location[1];
    target_x = global_x;
    target_y = global_y;
}
// 矫正一次车角度，阻塞式,多次采样
float main_vision_angle = 999;
uint8_t same_time = 0;
uint8_t received_time = 0;
static void sync_car_angle(void)
{
    while (same_time <= 2)
    {
        wait_global_info();
        want_global_infor(2);
        while (global_infor_type!=5)
        {
            wifi_task();
            uart_write_byte(UART_GLOBAL_INDEX, 0xFE);
        }
        received_time++;
        if (fabs(car_angel - main_vision_angle) <= 5)
        {
            same_time++;
        }
        else
        {
            same_time = 0;
        }
    }
    same_time = 0;
    main_vision_angle = 999;
    actual_yaw = car_angel-90;
    while (actual_yaw > 180.0f)
        actual_yaw -= 360.0f;
    while (actual_yaw < -180.0f)
        actual_yaw += 360.0f;
}
/**
 * @brief 跑一关
 * @param round_index 0第一关 1第二关 2第三关
 * @return 1 成功 0失败
 */
static uint8_t run_round(uint8_t round_index)
{
    WaypointPath path = {0};
    vision_run_correct_switch = 0;
    reset_round_runtime();

    vision_angle_switch = 0;
    car_move_point(global_x + 0.25f, global_y, angle, 0);
    wait_navigation();
    // 测试时加上，防止地图不对
    system_delay_ms(2000);
    sync_car_angle();
    
    // 获取地图
    request_round_map();
    if (!got_map_flag)
    {
        return 0;
    }
    // system_delay_ms(ROUND_MAP_SETTLE_MS);   // 有什么用？

    vision_run_correct_switch = 1;
    build_map_info(&engine_ctx, final_map_data, round_index == 0U ? 0U : 1U);
    if (!engine_ctx.map_valid)
    {
        return 0;
    }

    lost = 1;
    wifi_task();
    if (!solve(&engine_ctx))
    {
        return 0;
    }

    generate_path(&engine_ctx, &path);
    if (path.length == 0)
    {
        return 0;
    }

    lost = 66;
    car_move(&path, angle, 0);
    wait_navigation();

    return_to_start_zone();
    return 1;
}
// 停车
static void fault_stop(void)
{
    car_stop();
    while (1)
    {
        wifi_task();
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    // debug_init();                  // 调试端口初始化
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(600); // 等待主板其他外设上电完成
    myWIFI2SPI_Init();
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

    pit_ms_init(PIT_CH0, 10);            // 速度闭环和姿态闭环
    pit_ms_init(PIT_CH1, 5);             // 陀螺仪积分
    interrupt_set_priority(PIT_IRQn, 1); // 设置 PIT 中断优先级为 1

    interrupt_global_enable(0);

    system_delay_ms(600);
    sync_car_position();
    // 循环跑三关
    for (uint8_t round_index = 0; round_index < ROUND_COUNT; round_index++)
    {
        if (!run_round(round_index))
        {
            return_to_start_zone();
        }
    }
    // 第三关完成后保持停车，同时继续处理通信。
    car_stop();
    while (1)
    {
        wifi_task();
    }
    // NVIC_SystemReset(); // 复位
    return 0;
}

static int16 bias = 0;
static int calibrated = 0;

void imu_calibrate()
{
    int16 sum = 0;
    for (int i = 0; i < 500; i++)
    {
        imu660rb_get_gyro();
        sum += imu660rb_gyro_z;
        system_delay_ms(2);
    }
    bias = sum / 500;
    calibrated = 1;
}

int time = 0;
float ax_Zero = 0;
float ay_Zero = 0;
float imu_vx = 0;
float imu_vy = 0;
void pit_ch1_handler(void)
{

    imu660rb_get_gyro(); // 获取陀螺仪测量数值
    if (!calibrated)
    {
        return;
    }
    imu660rb_gyro_z = (int)((imu660rb_gyro_z - bias) / 10) * 10;

    // if(time<100){
    // }else{
    actual_yaw -= (float)imu660rb_gyro_z / imu660rb_transition_factor[1] * 0.005f;
    // }
    time++;

    while (actual_yaw > 180.0f)
        actual_yaw -= 360.0f;
    while (actual_yaw < -180.0f)
        actual_yaw += 360.0f;
}

float time_for_vision_loac = 0;
uint8_t vision_correct_flag = 0;
uint8_t vision_run_correct_switch = 1;
float time_vision_main = 0;
void run_vision_correct()
{
    if (vision_run_correct_switch == 1 && walk_mode != 3 && walk_mode != 4)
    {
        if (vision_correct_flag == 0)
        {
            time_for_vision_loac += 0.01f;
            if (time_for_vision_loac >= 0.5f)
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
            if (time_line - time_vision_main >= 0.3f)
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
    run_vision_correct();
}
