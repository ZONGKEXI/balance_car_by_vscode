#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "main.h"

void App_Motor_Init(void);                  // 初始化左右轮速度 PI。
void App_Motor_Proc(void);                  // 在主循环中更新速度环和 PWM。
void App_Motor_SetOmega_L(float omega);     // 设置左轮目标速度，单位 rad/s。
void App_Motor_SetOmega_R(float omega);     // 设置右轮目标速度，单位 rad/s。
float App_Motor_GetOmegaSet_L(void);        // 获取左轮目标速度，单位 rad/s。
float App_Motor_GetOmegaSet_R(void);        // 获取右轮目标速度，单位 rad/s。
void App_Motor_Cmd( uint8_t on);            // 启停并清除旧输出，非零为开启。
#endif
