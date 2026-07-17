#include "zf_common_headfile.h"
#include "move_control.h"

#define WIFI_SSID_TEST "听风的nova 14 Ultra"
#define WIFI_PASSWORD_TEST "13345789"
#define IPS200_TYPE (IPS200_TYPE_SPI) // 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8

extern float ax_Zero;
extern float ay_Zero;
extern float imu_vx;
extern float imu_vy;
// 示波器要使用时，发送以下结构体,默认最大容量为8个，如果需要更多数据，请查看seekfree_assistant_oscilloscope_struct的定义
seekfree_assistant_oscilloscope_struct SendData;
extern uint8_t time_for_vision_loac;

uint8_t lost = 0;
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

    seekfree_assistant_oscilloscope_data.data[0] = global_x;
    seekfree_assistant_oscilloscope_data.data[1] = global_y;
    seekfree_assistant_oscilloscope_data.data[2] = target_x;
    seekfree_assistant_oscilloscope_data.data[3] = target_y;
    seekfree_assistant_oscilloscope_data.data[4] = actual_yaw;
    seekfree_assistant_oscilloscope_data.data[5] = global_infor_type; // 0: 无效 1: 只要坐标 2: 只要角度 3: 坐标+角度 4: 坐标+角度+地图 5: 坐标+角度+地图+小车状态
    seekfree_assistant_oscilloscope_data.data[6] = time_line; // 最终目标航向角 单位度
    seekfree_assistant_oscilloscope_data.data[7] = lost; 
    SendDataToAssistant(&seekfree_assistant_oscilloscope_data, 8);
    // system_delay_ms(13);
}
