#include "zf_common_headfile.h"
#include "move_control.h"

#define WIFI_SSID_TEST "听风的nova 14 Ultra"
#define WIFI_PASSWORD_TEST "13345789"
#define IPS200_TYPE (IPS200_TYPE_SPI) // 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8

// 示波器要使用时，发送以下结构体,默认最大容量为8个，如果需要更多数据，请查看seekfree_assistant_oscilloscope_struct的定义
seekfree_assistant_oscilloscope_struct SendData;

/**
 * @brief 连wifi，连一次之后上位机软件不要断联，否则需要小车重新上电
 *
 */
void myWIFI2SPI_Init()
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    system_delay_ms(300);

    while (wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        system_delay_ms(100);
    }

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if (0 == WIFI_SPI_AUTO_CONNECT) // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while (wifi_spi_socket_connect( // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                      // 指定使用TCP方式通讯
            WIFI_SPI_TARGET_IP,         // 指定远端的IP地址，填写上位机的IP地址
            WIFI_SPI_TARGET_PORT,       // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_SPI_LOCAL_PORT))       // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位

            system_delay_ms(100); // 建立连接失败 等待 100ms
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}

/**
 * @brief 发送一次数据。
 *
 * @param sendDataStructure发送的结构体指针
 * @param num 发送结构体的前几个数
 */
void SendDataToAssistant(seekfree_assistant_oscilloscope_struct *sendDataStructure, char num)
{
    sendDataStructure->channel_num = num;
    seekfree_assistant_oscilloscope_send(sendDataStructure);
    system_delay_ms(20);
}

/**
 * @brief 接收数据到seekfree_assistant_parameter数组，这个函数放到周期循环中
 *
 */
void ReceiveData()
{
    seekfree_assistant_data_analysis();
}
void wifi_task()
{

    // 解析上位机发送过来的参数，解析后数据会存放在seekfree_assistant_parameter数组中，可以通过在线调试的方式查看数据
    // 例程为了方便因此写在了主循环，实际使用中推荐放到周期中断等位置，需要确保函数能够及时的被调用，调用周期不超过20ms
    ReceiveData();

    // seekfree_assistant_oscilloscope_data.data[0] = seekfree_assistant_parameter[0];
    // seekfree_assistant_oscilloscope_data.data[1] = seekfree_assistant_parameter[1];
    // seekfree_assistant_oscilloscope_data.data[2] = seekfree_assistant_parameter[2];
    // seekfree_assistant_oscilloscope_data.data[3] = seekfree_assistant_parameter[3];

    vision_weight= seekfree_assistant_parameter[0];
    // k_x = seekfree_assistant_parameter[1];
    // k_y = (uint8_t)seekfree_assistant_parameter[2];
    
    // kp_position_x = seekfree_assistant_parameter[4];
    // kp_position_y = seekfree_assistant_parameter[5];
    // Kp_yaw = seekfree_assistant_parameter[6];
    // Kd_yaw = seekfree_assistant_parameter[7];

    // uint8_t curent_flag = seekfree_assistant_parameter[3];
    // static uint8_t last_flag = 0;
    // if (curent_flag != last_flag)
    // {
    //     last_flag = curent_flag;
    //     switch (curent_flag)
    //     {
    //     case 0:
    //         car_stop();
    //         break;
    //     case 1:
    //     {
    //         WaypointPath path_A;
    //         path_A.length = 2;
    //         path_A.points[0] = 98;  // 起点：X=0.5, Y=1.1
    //         path_A.points[1] = 100; // 终点：X=0.9, Y=1.1

    //         // 把当前坐标对齐到起点，防止起步乱窜
    //         global_x = 0.5f;
    //         global_y = 1.1f;

    //         // 同步虚拟规划器状态
    //         planner_x.p = global_x;
    //         planner_y.p = global_y;
    //         planner_x.v = 0.0f;
    //         planner_y.v = 0.0f;

    //         car_move(&path_A, 0.0f, 0);
    //         break;
    //     }
    //     case 2:
    //     {
    //         WaypointPath path_B;
    //         path_B.length = 3;
    //         path_B.points[0] = 98;  // 起点：X=0.5, Y=1.1
    //         path_B.points[1] = 102; // 拐点：X=1.3, Y=1.1
    //         path_B.points[2] = 54;  // 终点：X=1.3, Y=1.7

    //         // 强行对齐起点
    //         global_x = 0.5f;
    //         global_y = 1.1f;

    //         // 同步虚拟规划器状态
    //         planner_x.p = global_x;
    //         planner_y.p = global_y;
    //         planner_x.v = 0.0f;
    //         planner_y.v = 0.0f;

    //         // 发车：目标角度 0 度，全向平移模式 0
    //         car_move(&path_B, 0.0f, 0);
    //         break;
    //     }
    //     case 3:
    //     {
    //         WaypointPath path_C;
    //         path_C.length = 1;
    //         path_C.points[0] = 98; // 终点就是起点

    //         // 强行对齐起点
    //         global_x = 0.5f;
    //         global_y = 1.1f;

    //         actual_yaw = 0.0f;

    //         // 同步虚拟规划器状态
    //         planner_x.p = global_x;
    //         planner_y.p = global_y;
    //         planner_x.v = 0.0f;
    //         planner_y.v = 0.0f;

    //         car_move(&path_C,90, 0);
    //         break;
    //     }
    //     case 4:
    //     {
    //         WaypointPath path_B;
    //         path_B.length = 3;
    //         path_B.points[0] = 54;  // 起点：X=1.3, Y=1.7
    //         path_B.points[1] = 102; // 拐点：X=1.3, Y=1.1
    //         path_B.points[2] = 98;  // 终点：X=0.5, Y=1.1

    //         // 强行对齐起点
    //         global_x = 1.3f;
    //         global_y = 1.7f;

    //         // 同步虚拟规划器状态
    //         planner_x.p = global_x;
    //         planner_y.p = global_y;
    //         planner_x.v = 0.0f;
    //         planner_y.v = 0.0f;

    //         // 发车：目标角度 0 度，全向平移模式 0
    //         car_move(&path_B, 0.0f, 0);
    //         break;
    //     }

    //     default:
    //         break;
    //     }
    // }
    // if(move_flag == 1){

    // }
    // else{
    //     car_stop();
    // }

    seekfree_assistant_oscilloscope_data.data[0] = global_x;
    seekfree_assistant_oscilloscope_data.data[1] = global_y;
    seekfree_assistant_oscilloscope_data.data[2] = actual_yaw;
    seekfree_assistant_oscilloscope_data.data[3] = final_target_yaw;
    seekfree_assistant_oscilloscope_data.data[4]=vision_y;
    // seekfree_assistant_oscilloscope_data.data[3] = kp_position_x;
    // seekfree_assistant_oscilloscope_data.data[4] = k_x;
    // seekfree_assistant_oscilloscope_data.data[5] = k_y;

    SendDataToAssistant(&seekfree_assistant_oscilloscope_data, 5);
    // system_delay_ms(13);
}
