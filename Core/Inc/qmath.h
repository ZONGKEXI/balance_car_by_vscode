#ifndef __QMATH_H__
#define __QMATH_H__

#include "stm32f1xx_hal.h"
#include <math.h>

// 三角函数使用弧度；查表计算，结果为近似值。
float qsin(float x); // 正弦。
float qcos(float x); // 余弦。
float qtan(float x); // 正切。
float qasin(float x); // 反正弦，输入范围 [-1, 1]。
float qacos(float x); // 反余弦，输入范围 [-1, 1]。
float qatan(float x); // 反正切。
float qatan2(float y, float x); // 坐标 (x, y) 对应的方向角。

#endif
