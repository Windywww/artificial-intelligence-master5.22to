#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_common_headfile.h"

#define MOTOR1_DIR (C9)//LF
#define MOTOR1_PWM (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR (C7)//RF
#define MOTOR2_PWM (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR (D2)//LB
#define MOTOR3_PWM (PWM2_MODULE3_CHB_D3)

#define MOTOR4_DIR (C10)//RB
#define MOTOR4_PWM (PWM2_MODULE2_CHB_C11)

void motor_init(void);
void motor_set_pwm_LF(int16_t duty);
void motor_set_pwm_RF(int16_t duty);
void motor_set_pwm_LB(int16_t duty);
void motor_set_pwm_RB(int16_t duty);

#endif