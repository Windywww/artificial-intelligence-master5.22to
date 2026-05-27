#ifndef __SET_FOLLOW_H
#define __SET_FOLLOW_H

typedef struct
{
    float amax;
    float vmax;
    float dt;
    float dt_2;

    float p; // 输出的位置
    float v; // 输出的速度
    float a; // 输出的加速度

    float a_p; // 加速度上限
    float a_n; // 加速度下限
    float k1;
    float k2;
} SecondOrder_Set_Follow_t;

char SecondOrder_Set_Follow_init(SecondOrder_Set_Follow_t *obj, float vmax, float amax, float dt);
void SecondOrder_Set_Follow_Cal(SecondOrder_Set_Follow_t *obj, float ps, float vs);
void SecondOrder_Set_Follow_AccUpdate(SecondOrder_Set_Follow_t *obj);
#endif
