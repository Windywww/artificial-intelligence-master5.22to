#include "Set_Follow.h"
// Set_Follow.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <math.h>
#include "Set_Follow.h"

// 函数用于初始化s曲线跟随的规划参数
// 本规划可以在每一次计算前接受参数的修改
char SecondOrder_Set_Follow_init(SecondOrder_Set_Follow_t *obj, float vmax, float amax, float dt)
{
    if (vmax <= 0 || amax <= 0 || dt <= 0)
    {
        return -1;
    }
    obj->amax = amax;
    obj->vmax = vmax;
    obj->dt = dt;

    obj->dt_2 = dt * dt;
    obj->p = 0;
    obj->v = 0;

    obj->k1 = obj->amax / 24.f;
    obj->k2 = 24.f / obj->vmax;
    // obj->a_p = k1 * (24.f - k2 * O);
    // obj->a_n = -k1 * (24.f + k2 * v);
    obj->a_p = amax;
    obj->a_n = -amax;

    return 0;
}

void SecondOrder_Set_Follow_AccUpdate(SecondOrder_Set_Follow_t *obj)
{
    obj->a_p = obj->k1 * (24.f - obj->k2 * obj->v);
    obj->a_n = -obj->k1 * (24.f + obj->k2 * obj->v);
}

#define absf(x) (((x) < 0) ? (-(x)) : (x))
#define sign(x) (((x) < 0) ? (-1) : (1))
#define max(a, b) ((a) > (b) ? (a) : (b))

void SecondOrder_Set_Follow_Cal(SecondOrder_Set_Follow_t *obj, float ps, float vs)
{
#define p obj->p
#define v obj->v
#define acc obj->a
#define dt obj->dt
#define dt_2 obj->dt_2

    float t1 = 0, t2 = 0;
    float vm = obj->vmax;

    float a_p = obj->a_p;
    float a_n = obj->a_n;
    float a = a_p;

    ps = ps - p;
    float vm_2 = vm * vm;
    float v_2 = v * v;
    float vs_2 = vs * vs;
    float tv = 0;
    tv = (ps + vs * ((vm - v) / a + (vm - vs) / a) - (vm_2 - v_2) / 2 / a - (vm_2 - vs_2) / 2 / a) / (vm - vs);
    if (tv < 0)
    {
        a = -a_n;
        tv = (ps + vs * ((-vm - v) / -a + (-vm - vs) / -a) - (vm_2 - v_2) / 2 / -a - (vm_2 - vs_2) / 2 / -a) / (-vm - vs);
        if (tv >= 0)
        {
            vm = -vm;
            a = -a;
        }
        else
        {
            a = a_p;
        }
    }
    if (tv < 0)
    {
        float temp1 = (v_2 - 2 * v * vs + vs_2 + 2 * a * ps);
        float temp2 = -(-v_2 + 2 * v * vs - vs_2 + 2 * absf(a_n) * ps);
        float temp = 0;
        if (temp1 < temp2)
        {
            temp = temp2;
            a = -a_n;
            t1 = (v - vs + (sqrtf(2 * a * a * temp)) / (2 * a)) / a;
            t2 = (sqrtf(2 * a * a * temp)) / (2 * a * a);
            a = -a;
        }
        else
        {
            temp = temp1;
            t1 = (vs - v + (sqrtf(2 * a * a * temp)) / (2 * a)) / a;
            t2 = (sqrtf(2 * a * a * temp)) / (2 * a * a);
        }

        t1 = t1 < 0 ? 0 : t1;
        t2 = t2 < 0 ? 0 : t2;
        float deltav_2 = (v - vs) * (v - vs);
        if (temp1 >= 0 && vs >= v && (((vs - v) + sqrtf(deltav_2 + 2 * a_p * ps)) / a_p <= (t1 + t2)))
        {
            t1 = ((vs - v) + sqrtf(deltav_2 + 2 * a_p * ps)) / a_p;
            t2 = 0;
            a = a_p;
        }
        
        else if (temp2 >= 0 && vs <= v && (((v - vs) + sqrtf(deltav_2 + 2 * a_n * ps)) / -a_n <= (t1 + t2)))
        {
            t1 = ((v - vs) + sqrtf(deltav_2 + 2 * a_n * ps)) / -a_n;
            t2 = 0;
            a = a_n;
        }

        if (dt <= t1)
        {
            p = p + v * dt + a * dt_2 / 2;
            v = v + a * dt;
            acc = a;
        }
        else if (dt <= t2 + t1)
        {
            float t = dt - t1;
            p = p + v * t1 + a * t1 * t1 / 2 + (v + a * t1) * t - a * t * t / 2;
            v = v + a * t1 - a * t;
            acc = -a;
        }
        else
        {
            v = vs;
            p = p + ps + vs * dt;
            acc = 0;
        }
    }
    else
    {
        t1 = (vm - v) / a;
        t2 = (vm - vs) / a;
        t1 = t1 < 0 ? 0 : t1;
        t2 = t2 < 0 ? 0 : t2;
        if (dt <= t1)
        {
            p = p + v * dt + a * dt_2 / 2;
            v = v + a * dt;
            acc = a;
        }
        else if (dt <= t1 + tv)
        {
            float t = dt - t1;
            p = p + v * t1 + a * t1 * t1 / 2 + vm * t;
            v = vm;
            acc = 0;
        }
        else if (dt <= t1 + tv + t2)
        {
            float t = dt - t1 - tv;
            p = p + (vm_2 - v_2) / 2 / a + tv * vm + vm * t - a * t * t / 2;
            v = v - a * t;
            acc = -a;
        }
        else
        {
            v = vs;
            p = p + ps + vs * dt;
            acc = 0;
        }
    }
    if (v > absf(vm))
    {
        v = absf(vm) * sign(v);
    }

#undef p
#undef v
#undef dt
#undef dt_2
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧:
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
