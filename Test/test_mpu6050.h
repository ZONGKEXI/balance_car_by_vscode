#ifndef TEST_MPU6050_H
#define TEST_MPU6050_H

#include "stm32f1xx_hal.h"


// 循环输出加速度、角速度和温度；阻塞且不返回。
void Test_MPU6050_Init(void);
// 循环更新姿态并输出传感器值和欧拉角；阻塞且不返回。
void Test_EularAngleTest(void);

#endif // TEST_MPU6050_H