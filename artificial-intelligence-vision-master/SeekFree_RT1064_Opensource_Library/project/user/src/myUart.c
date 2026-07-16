#include "myUart.h"
#include <string.h> // 必须引入，为了使用 memset 和 memcpy
#include "move_control.h"

#define MAPBEGIN 255
#define ANGELBEGIN 254

// ---------------- 接收缓存与按需状态机控制 ----------------
uint8_t test_rx_buffer_global[80];
uint8_t test_rx_index_global = 0;
volatile uint8_t rx_idle_ticks_global = 0;

// 【按需接收核心标志位】
// 0: 索要小车位置  1: 索要地图  2: 索要角度  5: 挂机态(不处理任何数据)
volatile uint8_t global_infor_type = 5;

// 定长解析状态机专用的控制变量
uint8_t rx_state_global = 0;     // 0: 等待指定的包头, 1: 接收定长数据
uint8_t current_cmd_global = 0;  // 当前正在接收的指令类型 (0xAA, 0xA5, 0x5A)
uint8_t expected_len_global = 0; // 当前包预期要接收的数据长度

// ---------------- 定义全局存储变量 ----------------
uint8_t final_map_data[MAP_LENS]; // 解压后的 192 个地图数据
uint8_t got_map_flag = 0;         // 标志位，表示是否已经成功接收并解压了地图数据
float car_location[2];
float car_angel = 0;

static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8; i++)
    {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void reset_global_receiver(void)
{
    rx_state_global = 0;
    test_rx_index_global = 0;
    rx_idle_ticks_global = 0;
    memset(test_rx_buffer_global, 0, sizeof(test_rx_buffer_global));
}

static void retry_global_request(void)
{
    uint8_t infor_type = global_infor_type;
    reset_global_receiver();
    if (infor_type == 1)
    {
        uart_write_byte(UART_GLOBAL_INDEX, 0xBB);
    }
    else if (infor_type == 2)
    {
        uart_write_byte(UART_GLOBAL_INDEX, 0xFE);
    }
}

static uint8_t global_packet_crc_valid(void)
{
    uint8_t data_len = expected_len_global - 2;
    uint16_t crc = crc16_ccitt_update(0xFFFFU, current_cmd_global);
    for (uint8_t i = 0; i < data_len; i++)
    {
        crc = crc16_ccitt_update(crc, test_rx_buffer_global[i]);
    }

    uint16_t received_crc = ((uint16_t)test_rx_buffer_global[data_len] << 8) |
                            test_rx_buffer_global[data_len + 1];
    return crc == received_crc;
}

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

void myuart_timeout_tick_10ms(void)
{
    if (global_infor_type != 5 && rx_state_global == 1)
    {
        if (++rx_idle_ticks_global >= 3)
        {
            retry_global_request();
        }
    }
    else
    {
        rx_idle_ticks_global = 0;
    }
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
    reset_global_receiver();

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
    got_map_flag = 1; // 设置标志位，表示地图数据已成功接收并解压
}

void Unpack_Received_CarLoc()
{
    memcpy(&car_location[0], &test_rx_buffer_global[0], 4);
    memcpy(&car_location[1], &test_rx_buffer_global[4], 4);

    memset(test_rx_buffer_global, 0, sizeof(test_rx_buffer_global));
    test_rx_index_global = 0;

    // global_x = 3.2f * car_location[0];
    // global_y = 2.4f - 2.4f * car_location[1];
}

void Unpack_Received_CarAngel()
{
    memcpy(&car_angel, &test_rx_buffer_global[0], 4);

    memset(test_rx_buffer_global, 0, sizeof(test_rx_buffer_global));
    test_rx_index_global = 0;
}

// ---------------- 具备 CRC16 校验的定长接收状态机 ----------------
void uart1_rx_interrupt_handler(void)
{
    uint8_t get_data;
    if (!uart_query_byte(UART_GLOBAL_INDEX, &get_data))
    {
        return;
    }

    if (global_infor_type == 5)
    {
        return;
    }

    rx_idle_ticks_global = 0;

    if (rx_state_global == 0)
    {
        if (global_infor_type == 0 && get_data == 0xAA)
        {
            current_cmd_global = 0xAA;
            expected_len_global = 10;
        }
        else if (global_infor_type == 1 && get_data == 0xA5)
        {
            current_cmd_global = 0xA5;
            expected_len_global = 67;
        }
        else if (global_infor_type == 2 && get_data == 0x5A)
        {
            current_cmd_global = 0x5A;
            expected_len_global = 6;
        }
        else
        {
            return;
        }

        test_rx_index_global = 0;
        rx_state_global = 1;
        return;
    }

    if (test_rx_index_global >= sizeof(test_rx_buffer_global))
    {
        retry_global_request();
        return;
    }

    test_rx_buffer_global[test_rx_index_global++] = get_data;
    if (test_rx_index_global < expected_len_global)
    {
        return;
    }

    uint8_t is_packet_valid = global_packet_crc_valid();
    if (current_cmd_global == 0xA5 && test_rx_buffer_global[0] != 64)
    {
        is_packet_valid = 0;
    }

    if (!is_packet_valid)
    {
        retry_global_request();
        return;
    }

    if (current_cmd_global == 0xAA)
    {
        Unpack_Received_CarLoc();
    }
    else if (current_cmd_global == 0xA5)
    {
        Unpack_Received_Map(final_map_data);
    }
    else
    {
        Unpack_Received_CarAngel();
    }

    global_infor_type = 5;
    rx_state_global = 0;
}

// ---------------- UART4 相关逻辑 (原封不动) ----------------
void check_image(char obj, char is_firsttime)
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
        if (is_firsttime)
        {
            image_rx_state = 1;
        }
        break;

    case 3:
        uart_write_byte(UART_LOCAL_INDEX, 0xFE);
        system_delay_us(10);
        uart_write_byte(UART_LOCAL_INDEX, 0xFE);
        system_delay_us(10);
        if (is_firsttime)
        {
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
