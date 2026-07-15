#include "pid.h"

void pid_init(PID_TypeDef *pid, float p, float i, float d)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->error_last = 0.0f;
    pid->error_prev = 0.0f;
    pid->duty_out = 0;
    pid->duty_max = 25; // 假设最大占空比为20
}

/**
 * @brief 速度环PID计算函数 增量式PID算法
 * 
 * @param pid 
 * @param target_speed 
 * @param actual_speed 
 * @return int 
 */
float pid_calculate(PID_TypeDef *pid, float target_speed, float actual_speed)
{
    float error = target_speed - actual_speed; // 计算当前误差 e(k)

    // 增量式 PID 核心公式
    float p_out = pid->Kp * (error - pid->error_last);
    float i_out = pid->Ki * error;
    float d_out = pid->Kd * (error - 2.0f * pid->error_last + pid->error_prev);

    float delta_pwm = p_out + i_out + d_out; // 算出本次 PWM 需要增加/减少多少

    pid->duty_out += delta_pwm; // 累加得到当前的实际 PWM

    // 限幅保护 (极其重要，防止电机烧毁)
    if (pid->duty_out > pid->duty_max)
        pid->duty_out = pid->duty_max;
    if (pid->duty_out < -pid->duty_max)
        pid->duty_out = -pid->duty_max;




        

    // 记住历史误差，留给下一次 20ms 后使用
    pid->error_prev = pid->error_last;
    pid->error_last = error;

    return pid->duty_out;
}
