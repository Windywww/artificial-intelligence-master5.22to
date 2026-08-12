#ifndef __PID_H
#define __PID_H

#include "zf_common_headfile.h"

typedef struct
{
    float Kp; // 比例系数
    float Ki; // 积分系数
    float Kd; // 微分系数

    float error_last; // 上一次的误差 e(k-1)
    float error_acc; // 累积误差

    float duty_out; // 累计输出的占空比值
    float duty_max; // 最大占空比值
} PID_TypeDef;

void pid_init(PID_TypeDef *pid, float p, float i, float d);
float pid_calculate(PID_TypeDef *pid, float target_speed, float actual_speed);

#endif
