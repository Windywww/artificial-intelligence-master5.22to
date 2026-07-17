//查表法的死锁检测，使用两个 64KB 的 LUT 来快速判断某个位置是否是死锁点
#ifndef SOKOBAN_LUT_H
#define SOKOBAN_LUT_H

#include <stdint.h>
#include <stdbool.h>

// 一个 64KB 的查表，返回打破死锁所需的最小 TNT 数量/255. 假定当前目标是箱子(是tnt的话+1即可)
// 这些数组的数据由 Python 脚本计算写入
extern const uint8_t DEADLOCK_LUT_BOX[65536];
extern const uint8_t DEADLOCK_LUT_BOMB[65536];
#endif