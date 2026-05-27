// #include "uart.h"
// #include "pid.h"
// #include "move_control.h"
// #include "stdio.h"

// uint8 uart_get_data[64]; // 串口接收数据缓冲区
// uint8 fifo_get_data[64]; // fifo 输出读出缓冲区

// uint8 get_data = 0;         // 接收数据变量
// uint32 fifo_data_count = 0; // fifo 数据个数

// fifo_struct uart_data_fifo;

// void uart1_init(void)
// {
//     fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64); // 初始化 fifo 挂载缓冲区

//     uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN); // 初始化串口
//     uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                       // 开启 UART_INDEX 的接收中断
//     interrupt_set_priority(UART_PRIORITY, 2);                       // 设置 UART_INDEX 中断优先级为 2
// }

// void uart1_task(void)
// {
//     fifo_data_count = fifo_used(&uart_data_fifo); // 查看 fifo 是否有数据
//     if (fifo_data_count != 0)                     // 读取到数据了
//     {
//         system_delay_ms(10);                                                                     // 等待数据接收完成 以及 可能的串口发送完成 避免串口发送和接收冲突导致数据异常
//         fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); // 将 fifo 中数据读出并清空 fifo 挂载的缓冲

//         if (fifo_data_count > 63)
//             fifo_data_count = 63;              // 安全保护 避免数据过多导致 uart_write_buffer 发送异常
//         fifo_get_data[fifo_data_count] = '\0'; // 添加字符串结束符 方便 uart_write_string 发送

//         float val;

//         // 1. 尝试解析 P 参数 (兼容 kp: 或者 Kp:)
//         if (sscanf((char *)fifo_get_data, "kp:%f", &val) == 1 || sscanf((char *)fifo_get_data, "Kp:%f", &val) == 1)
//         {
//             for (int j = 0; j < 4; j++)
//             {
//                   //pid[j].Kp = val;
//                   Kp_yaw = val;
//             }
//         }
//         // 2. 尝试解析 I 参数 (兼容 ki: 或者 Ki:)
//         else if (sscanf((char *)fifo_get_data, "ki:%f", &val) == 1 || sscanf((char *)fifo_get_data, "Ki:%f", &val) == 1)
//         {
//             for (int j = 0; j < 4; j++)
//             {
//                  //pid[j].Ki = val;
//                 // Ki_yaw = val;
//             }
//         }
//         // 3. 尝试解析 D 参数 (兼容 kd: 或者 Kd:)
//         else if (sscanf((char *)fifo_get_data, "kd:%f", &val) == 1 || sscanf((char *)fifo_get_data, "Kd:%f", &val) == 1)
//         {
//             for (int j = 0; j < 4; j++)
//             {
//                 //pid[j].Kd = val;
//                 Kd_yaw = val;
//             }
//         }
//         //4. 解析目标速度！(支持 target: 或者 v:)
//         else if (sscanf((char *)fifo_get_data, "target:%f", &val) == 1 || sscanf((char *)fifo_get_data, "v:%f", &val) == 1)
//         {
//              //target_v[RF] = val;
//              target_yaw = val;
//         }
//     }

//     // printf("%.3f,%.3f,%.1f,%.1f,%.1f,%.1f\n",
//     //        target_v[RF], actual_v[RF], out_duty[RF], pid[RF].Kp, pid[RF].Ki, pid[RF].Kd);
//      //printf("%.3f,%.3f,%.3f,%.4f,%.4f\n",
//             // target_yaw, actual_yaw, Kp_yaw, Ki_yaw, Kd_yaw);
//     // printf("%.3f,%.3f,%.3f\n", global_x, global_y, actual_yaw);
// }

// void uart1_rx_interrupt_handler(void)
// {
//     if (uart_query_byte(UART_INDEX, &get_data))
//     {
//         fifo_write_buffer(&uart_data_fifo, &get_data, 1); // 将数据写入 fifo 中
//     }
// }
