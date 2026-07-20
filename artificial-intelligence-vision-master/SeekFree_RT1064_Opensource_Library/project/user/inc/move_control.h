#ifndef _MOVE_CONTROL_H_
#define _MOVE_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "Set_Follow.h"
#include "myUart.h"
#include "sokoban_engine.h"

// 系数 = (1/1024) * (1/0.02秒) * (3/7减速比) * (2 * PI * 0.0315米)
#define SPEED_COEFFICIENT ((1.0f / 1024.0f) * (1.0f / 0.01f) * (3.0f / 7.0f) * (2.0f * 3.1415926f * 0.0315f))
#define VISION_CORRECT_T 1
typedef enum
{
    RF = 0,
    LF = 1,
    LB = 2,
    RB = 3 
} Wheel_ID;

extern uint32_t encoder_ports[4];
extern int16_t encoder_data[4];
extern float actual_v[4];
extern float target_v[4];
extern float out_duty[4];

extern float actual_yaw;
extern float final_target_yaw; // 最终目标航向角 单位度
extern float target_yaw;
extern float target_vx;
extern float target_vy;

extern float Kp_yaw, Ki_yaw, Kd_yaw;
extern float global_x, global_y;

extern float k_x, k_y;

extern uint8_t move_flag, mode;
extern float target_x, target_y;
extern float min_distance;
extern float kp_position_x, kp_position_y;

extern float vision_x;
extern float vision_y;

extern PID_TypeDef pid[4];

void move_control_init();
float yaw_pid_calculate(void);
void wheel_speed_calculate(float vx, float vy, float vz);
void odometry_update();
void navigation_update(void);
void move_control_task(void);


void car_move(WaypointPath *path, float yaw, uint8_t m);
void car_stop();
void car_turn(float yaw);
void car_move_point(float x, float y, float yaw, uint8_t m);

extern uint8_t check_obstacle(SokobanContext *ctx, uint8_t grid_index);

extern uint8_t yaw_arrived_flag; // 航向角到达标志位：1表示已到达目标航向角，0表示未到达
extern uint8_t navigate_flag; // 1: 正在追路径 0: 没有路径需要追
extern uint8_t vision_xy_update_flag;  // 视觉数据更新标志位：1表示有新数据，0表示已处理
extern uint8_t vision_yaw_update_flag; // 视觉航向角更新标志位：1表示有新数据，0表示已处理

extern float vision_x;
extern float vision_y;
extern float vision_yaw;
extern float vision_weight;              

extern uint8_t if_check;

extern float local_encoder_vx;
extern float local_encoder_vy;
extern float local_imu_vx;
extern float local_imu_vy;

extern float a_x;
extern float a_y;

extern uint8_t loac_test;
extern uint8_t angle_test;
extern uint8_t wait_for_loc;

extern float ax;
extern float ay;
extern float az;

extern float imu_x;
extern float imu_y;

extern float speed_angle;        //(弧度制)
extern uint8_t vision_angle_switch;

extern uint8_t count_A;
extern volatile uint8_t count;

extern uint8_t wait_for_loc;
extern uint8_t got_angle;

extern uint8_t walk_mode;
extern uint8_t first_time_fix;

#endif
