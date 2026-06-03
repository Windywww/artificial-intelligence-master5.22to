#include "motor.h"

int16_t duty_begin[4];

void motor_init(void)
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR3_PWM, 17000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR4_PWM, 17000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0
    duty_begin[0] = 4;
    duty_begin[1] = 4;
    duty_begin[2] = 10;
    duty_begin[3] = 4;

}

void motor_set_pwm_LF(int16_t duty)
{
    int16_t thisduty = 0;
    if (duty >= 0)
    {
        thisduty = duty;
        if(thisduty<=1){
            thisduty = 0;
        }else{
            thisduty+=duty_begin[0];
        }
        gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                 // DIR输出高电平
        pwm_set_duty(MOTOR1_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
    else
    {
        thisduty = -duty;
        if(thisduty<=1){
            thisduty = 0;
        }else{
            thisduty+=duty_begin[0];
        }
        gpio_set_level(MOTOR1_DIR, GPIO_LOW);                     // DIR输出低电平
        pwm_set_duty(MOTOR1_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
}

void motor_set_pwm_RF(int16_t duty)
{
    int16_t thisduty = 0;
    if (duty >= 0)
    {
        thisduty = duty;
        
        gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                 // DIR输出高电平
        pwm_set_duty(MOTOR2_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
    else
    {
        thisduty = -duty;        
        gpio_set_level(MOTOR2_DIR, GPIO_LOW);                     // DIR输出低电平
        pwm_set_duty(MOTOR2_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
}

void motor_set_pwm_LB(int16_t duty)
{
    int16_t thisduty = 0;
    if (duty >= 0)
    {
        thisduty = duty;
        
        gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                 // DIR输出高电平
        pwm_set_duty(MOTOR3_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
    else
    {
        thisduty = -duty;
        gpio_set_level(MOTOR3_DIR, GPIO_LOW);                     // DIR输出低电平
        pwm_set_duty(MOTOR3_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
}

void motor_set_pwm_RB(int16_t duty)
{
    int16_t thisduty = 0;
    if (duty >= 0)
    {
        thisduty = duty;
        gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                 // DIR输出高电平
        pwm_set_duty(MOTOR4_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
    else
    {
        thisduty = -duty;
        gpio_set_level(MOTOR4_DIR, GPIO_LOW);                     // DIR输出低电平
        pwm_set_duty(MOTOR4_PWM, thisduty * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
}