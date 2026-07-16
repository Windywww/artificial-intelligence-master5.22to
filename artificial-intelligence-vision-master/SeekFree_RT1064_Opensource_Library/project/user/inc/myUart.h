#ifndef __myUart_H
#define __myUart_H

#include "zf_common_headfile.h"


#define UART_GLOBAL_INDEX (UART_1)       // 默认 UART_1
#define UART_GLOBAL_BAUDRATE (115200) // 默认 115200
#define UART_GLOBAL_TX_PIN (UART1_TX_B12)     // 默认 UART1_TX_B12
#define UART_GLOBAL_RX_PIN (UART1_RX_B13)     // 默认 UART1_RX_B13
#define UART_GLOBAL_PRIORITY (LPUART1_IRQn) // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体
#define MAP_LENS 192               // 预期的地图长度


#define UART_LOCAL_INDEX (UART_4)       // 默认 UART_4
#define UART_LOCAL_BAUDRATE (115200) // 默认 115200
#define UART_LOCAL_TX_PIN (UART4_TX_C16)     // 默认 UART4_TX_C16
#define UART_LOCAL_RX_PIN (UART4_RX_C17)     // 默认 UART4_RX_C17
#define UART_LOCAL_PRIORITY (LPUART4_IRQn) // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体

extern volatile uint8_t global_infor_type;     
extern uint8_t image_rx_state;
extern uint8_t final_image_index;
extern uint8_t final_map_data[MAP_LENS]; // 解压后的 192 个地图数据
extern float car_location[2];
extern float car_angel;
extern uint8_t got_map_flag;
extern float car_angel ;

void myuart_init(void);
void want_global_infor(char infor_type);
void check_image(char obj,char is_firsttime);
#endif
