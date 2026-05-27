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

SokobanContext engine_ctx;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    // debug_init();                  // 调试端口初始化
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(600); // 等待主板其他外设上电完成
    myWIFI2SPI_Init();
    motor_init();
    encoder_init();
    // key_init(5);
    // uart1_init();

    imu660rb_init();
    // ips_init();
    // map_init();
    myuart_init();

    move_control_init();

    pit_ms_init(PIT_CH0, 20);            // 速度闭环和姿态闭环
    pit_ms_init(PIT_CH1, 5);             // 陀螺仪积分
    interrupt_set_priority(PIT_IRQn, 1); // 设置 PIT 中断优先级为 1

    interrupt_global_enable(0);

    // 车坐标初始化
        want_global_infor(0);
        while (global_infor_type != 5)
        {
                wifi_task();
        }

    global_x = 3.2f * car_location[0];
    global_y = 2.4f - 2.4f * car_location[1];
    target_x = global_x;
    target_y = global_y;

    // 走出发车区
    WaypointPath path_move_out;
    path_move_out.length = 1;
    path_move_out.points[0] = 3 + 6 * 16;
    car_move(&path_move_out, angel, 0);
    while (navigate_flag)
    {
            wifi_task();

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
            wifi_task();

    }

    system_delay_ms(1200);
    build_map_info(&engine_ctx, final_map_data);
    WaypointPath path;
    path.length = 0;
    if (solve(&engine_ctx))
    {
        generate_path(&engine_ctx, &path);
    }else{
        while (1)
        {
        }        
    }

    car_move(&path,angel,0);
    while (navigate_flag)
    {
            wifi_task();

    }
    
    // WaypointPath straight_path;
    // straight_path.length = 0;
    // for (int i = 0; i < path.length - 1; i++)
    // {
    //     straight_path.length++;
    //     uint8_t current_point = path.points[i];
    //     uint8_t next_point = path.points[i + 1];
    //     uint8_t current_x = current_point % 16;
    //     uint8_t current_y = current_point / 16;
    //     uint8_t next_x = next_point % 16;
    //     uint8_t next_y = next_point / 16;
    //     straight_path.points[straight_path.length - 1] = current_point;
    //     if (current_x != next_x && current_y != next_y)
    //     {
    //         straight_path.length++;
    //         straight_path.points[straight_path.length - 1] = next_x + current_y * 16;
    //     }
    // }
    // straight_path.length++;
    // straight_path.points[straight_path.length - 1] = path.points[path.length - 1];

     

    WaypointPath path_move_in;
    path_move_in.length = 1;
    path_move_in.points[0] = 0 + 6 * 16;
    car_move(&path_move_in, angel, 0);
    while (navigate_flag)
    {
            wifi_task();

    }
    system_delay_ms(1500);
    // NVIC_SystemReset(); // 复位
}


/** 
int main()
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    // debug_init();                  // 调试端口初始化
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(600); // 等待主板其他外设上电完成
    myWIFI2SPI_Init();
    motor_init();
    encoder_init();
    imu660rb_init();
    myuart_init();

    move_control_init();

    pit_ms_init(PIT_CH0, 20);            // 速度闭环和姿态闭环
    pit_ms_init(PIT_CH1, 5);             // 陀螺仪积分
    interrupt_set_priority(PIT_IRQn, 1); // 设置 PIT 中断优先级为 1

    interrupt_global_enable(0);
    while (1)
    {
        ReceiveData();
        target_x = seekfree_assistant_parameter[0] * 0.2 + 0.1;
        target_y = seekfree_assistant_parameter[1] * 0.2 + 0.1;
        final_target_yaw = seekfree_assistant_parameter[2];
        move_flag = seekfree_assistant_parameter[3];

        seekfree_assistant_oscilloscope_data.data[0] = global_x;
        seekfree_assistant_oscilloscope_data.data[1] = global_y;
        seekfree_assistant_oscilloscope_data.data[2] = actual_yaw;
        seekfree_assistant_oscilloscope_data.data[3] = target_x;
        seekfree_assistant_oscilloscope_data.data[4] = target_y;
        SendDataToAssistant(&seekfree_assistant_oscilloscope_data,5);
    }
}
**/

void pit_ch1_handler(void)
{
    imu660rb_get_gyro(); // 获取陀螺仪测量数值
    imu660rb_get_acc();

    
    imu660rb_gyro_z = (int)((imu660rb_gyro_z - 4) / 10) * 10;

    actual_yaw -= (float)imu660rb_gyro_z / imu660rb_transition_factor[1] * 0.005f;
}

void pit_ch0_handler(void)
{
    // wifi_task();
    move_control_task();
}
