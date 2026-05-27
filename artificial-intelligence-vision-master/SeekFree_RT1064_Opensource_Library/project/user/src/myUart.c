#include "myUart.h"
#include <string.h> // 必须引入，为了使用 memset 和 memcpy
#include "move_control.h"

#define MAPBEGIN 255
#define ANGELBEGIN 254

// ---------------- 接收缓存与按需状态机控制 ----------------
uint8_t test_rx_buffer_global[80];
uint8_t test_rx_index_global = 0;

// 【按需接收核心标志位】
// 0: 索要小车位置  1: 索要地图  2: 索要角度  5: 挂机态(不处理任何数据)
uint8_t global_infor_type = 5;

// 定长解析状态机专用的控制变量
uint8_t rx_state_global = 0;     // 0: 等待指定的包头, 1: 接收定长数据, 2: 校验包尾与Checksum
uint8_t current_cmd_global = 0;  // 当前正在接收的指令类型 (0xAA, 0xA5, 0x5A)
uint8_t expected_len_global = 0; // 当前包预期要接收的数据长度

// ---------------- 定义全局存储变量 ----------------
uint8_t final_map_data[MAP_LENS]; // 解压后的 192 个地图数据
float car_location[2];
float car_angel = 0;

// ---------------- UART4 相关变量 (原封不动) ----------------
// 视觉组会持续传来每一帧检测结果，只有连续五次数据一样才接取
uint8_t test_rx_local = 0;
uint8_t test_rx_local_same_time = 0;
uint8_t image_rx_state = 0;
uint8_t final_image_index = 0;
uint8_t image_id = 0; // 2箱子，3目的地

void myuart_init(void)
{
    uart_init(UART_GLOBAL_INDEX, UART_GLOBAL_BAUDRATE, UART_GLOBAL_TX_PIN, UART_GLOBAL_RX_PIN);
    uart_rx_interrupt(UART_GLOBAL_INDEX, ZF_ENABLE);
    interrupt_set_priority(UART_GLOBAL_PRIORITY, 0);

    uart_init(UART_LOCAL_INDEX, UART_LOCAL_BAUDRATE, UART_LOCAL_TX_PIN, UART_LOCAL_RX_PIN);
    uart_rx_interrupt(UART_LOCAL_INDEX, ZF_ENABLE);
    interrupt_set_priority(UART_LOCAL_PRIORITY, 0);
}

/**
 * @brief 索要全局摄像头的一些信息 (触发按需接收)
 * @param infor_type 0小车位置 1地图，2小车角度
 */
void want_global_infor(char infor_type)
{

    if (global_infor_type != 5)
    {
        return;
    }

    // 1. 设置标志位，唤醒接收中断
    global_infor_type = infor_type;

    // 2. 清理战场，确保状态机处于干干净净的找包头状态
    rx_state_global = 0;
    test_rx_index_global = 0;
    memset(test_rx_buffer_global, 0, sizeof(test_rx_buffer_global));

    // 3. 按需发送握手信号给视觉模块
    switch (infor_type)
    {
    case 0:
        // 小车坐标是一直发的，不需要发信号索要，设好标志位等中断抓就行了
        break;
    case 1:
        uart_write_byte(UART_GLOBAL_INDEX, 0xBB);
        break;
    case 2:
        uart_write_byte(UART_GLOBAL_INDEX, 0xFE);
        break;
    default:
        // 如果传入异常值，强制切回挂机态
        global_infor_type = 5;
        break;
    }
}

// ---------------- 解包与解压函数 ----------------
void Unpack_Received_Map(uint8_t *thismap)
{
    // test_rx_buffer_global[0] 是长度位，下面跟着 64 字节真实数据
    uint8_t n_compressed = 64;

    // 解压地图数据 (从 index 1 开始读取 64 个数据)
    for (int i = 1; i <= n_compressed; i++)
    {
        uint8_t compressed_val = test_rx_buffer_global[i];

        // 【已修复 BUG】：原来是 i*3，会导致 thismap[0,1,2] 被跳过
        // 现在改为 (i-1)*3，完美对齐 0 索引！
        int map_idx = (i - 1) * 3;
        thismap[map_idx] = compressed_val / 36;
        compressed_val = compressed_val % 36;
        thismap[map_idx + 1] = compressed_val / 6;
        thismap[map_idx + 2] = compressed_val % 6;
    }

    // 注意：旧版的小车坐标 float 提取已删除，因为新协议下坐标不再附着于地图包中

    // 清零缓存区
    memset(test_rx_buffer_global, 0, sizeof(test_rx_buffer_global));
    test_rx_index_global = 0;
}

void Unpack_Received_CarLoc()
{
    memcpy(&car_location[0], &test_rx_buffer_global[0], 4);
    memcpy(&car_location[1], &test_rx_buffer_global[4], 4);

    memset(test_rx_buffer_global, 0, sizeof(test_rx_buffer_global));
    test_rx_index_global = 0;
    
    vision_x = 3.2f * car_location[0];
    vision_y = 2.4 - 2.4 * car_location[1]; 
    float odom_weight = 1.0f - vision_weight; // 里程计权重 40%

    global_x = global_x * odom_weight + vision_x * vision_weight;
    global_y = global_y * odom_weight + vision_y * vision_weight;

    planner_x.p = planner_x.p * odom_weight + vision_x * vision_weight;
    planner_y.p = planner_y.p * odom_weight + vision_y * vision_weight;

}

void Unpack_Received_CarAngel()
{
    memcpy(&car_angel, &test_rx_buffer_global[0], 4);

    memset(test_rx_buffer_global, 0, sizeof(test_rx_buffer_global));
    test_rx_index_global = 0;
    vision_yaw_update_flag = 1; // 设置视觉航向角更新标志位，通知主循环有新航向角数据可以处理了

    vision_yaw = car_angel - 90;
}

// ---------------- 核心升级：具备 Checksum 和边界帧校验的状态机 ----------------
void uart1_rx_interrupt_handler(void)
{
    uint8_t get_data;
    if (!uart_query_byte(UART_GLOBAL_INDEX, &get_data))
    {
        return;
    }

    // 【拦截】：如果是挂机态，直接丢弃数据
    if (global_infor_type == 5)
        return;
    switch (rx_state_global)
    {
    case 0: // 【状态0：睁眼寻找被要求的包头】
        if (global_infor_type == 0 && get_data == 0xAA)
        {
            current_cmd_global = 0xAA;
            expected_len_global = 8; // 2个 float
            test_rx_index_global = 0;
            rx_state_global = 1;
        }
        else if (global_infor_type == 1 && get_data == 0xA5)
        {
            current_cmd_global = 0xA5;
            expected_len_global = 66; // 长度(1) + 地图(64) + Checksum(1) = 66 字节
            test_rx_index_global = 0;
            rx_state_global = 1;
        }
        else if (global_infor_type == 2 && get_data == 0x5A)
        {
            current_cmd_global = 0x5A;
            expected_len_global = 4; // 1个 float
            test_rx_index_global = 0;
            rx_state_global = 1;
        }
        break;

    case 1: // 【状态1：闷头接收定长数据】
        if (test_rx_index_global < sizeof(test_rx_buffer_global))
        {
            test_rx_buffer_global[test_rx_index_global++] = get_data;
        }
        else
        {
            uint8_t type = global_infor_type;
            global_infor_type = 5;
            // 越界严重错位，重新索要数据！
            want_global_infor(type);
            break;
        }

        // 数据收集满足长度后，进入状态 2 等待自然校验尾缀
        if (test_rx_index_global >= expected_len_global)
        {
            rx_state_global = 2;
        }
        break;

    case 2: // 【状态2：严格校验尾缀与 Checksum】
        // 1. 先验证下一帧的包头（天然边界尾缀）是否为 0xAA
        if (get_data == 0xAA)
        {

            uint8_t is_packet_valid = 1; // 数据有效标志位

            // 2. 如果是地图数据，进行加和校验 (Checksum)
            if (current_cmd_global == 0xA5)
            {
                uint16_t checksum_sum = 0;
                // 将前 65 个字节（[0] 到 [64]）加起来
                for (int i = 0; i < 65; i++)
                {
                    checksum_sum += test_rx_buffer_global[i];
                }

                // 取低 8 位与第 66 个字节（[65]）对比
                uint8_t calculated_checksum = (uint8_t)(checksum_sum & 0xFF);
                if (calculated_checksum != test_rx_buffer_global[65])
                {
                    is_packet_valid = 0; // Checksum 错误！标记为无效数据
                }
            }

            // 3. 终极宣判
            if (is_packet_valid)
            {
                // 完美通过所有测试，开始解包！
                if (current_cmd_global == 0xAA)
                {
                    Unpack_Received_CarLoc();
                }
                else if (current_cmd_global == 0xA5)
                {
                    Unpack_Received_Map(final_map_data);
                }
                else if (current_cmd_global == 0x5A)
                {
                    Unpack_Received_CarAngel();
                }

                // 任务结束，闭眼挂机
                global_infor_type = 5;
                rx_state_global = 0;
            }
            else
            {
                // 🚨 校验和不匹配，数据被污染，立刻重新索要！
                want_global_infor(global_infor_type);
            }
        }
        else
        {
            // 🚨 边界位不是 0xAA，说明发生错位丢包，立刻重新索要！
            want_global_infor(global_infor_type);
        }
        break;
    }
}

// ---------------- UART4 相关逻辑 (原封不动) ----------------
void check_image(char obj,char is_firsttime)
{
    image_id = obj;
    
    uart_write_byte(UART_LOCAL_INDEX, 0xA5);
    system_delay_us(10);
    uart_write_byte(UART_LOCAL_INDEX, 0x5A);
    system_delay_us(10);
    switch (obj)
    {
    case 2:
        uart_write_byte(UART_LOCAL_INDEX, 0xBB);
        system_delay_us(10);
        uart_write_byte(UART_LOCAL_INDEX, 0xBB);
        system_delay_us(10);
        if(is_firsttime){
            image_rx_state = 1;
        }
        break;

    case 3:
        uart_write_byte(UART_LOCAL_INDEX, 0xFE);
        system_delay_us(10);
        uart_write_byte(UART_LOCAL_INDEX, 0xFE);
        system_delay_us(10);
        if(is_firsttime){
            image_rx_state = 1;            
        }
        break;
    }
}

void uart4_rx_interrupt_handler()
{
    uint8_t get_data;
    if (!uart_query_byte(UART_LOCAL_INDEX, &get_data))
    {
        return;
    }
    if (image_rx_state == 1)
    {
        if (get_data == test_rx_local)
        {
            test_rx_local_same_time++;
            if (test_rx_local_same_time >= 2)
            {
                test_rx_local_same_time = 0;
                image_rx_state = 0;
                final_image_index = test_rx_local;
            }
            else
            {
                return;
            }
        }
        else
        {
            test_rx_local = get_data;
            test_rx_local_same_time = 0;
        }
    }
}