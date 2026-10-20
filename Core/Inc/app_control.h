#ifndef APP_CONTROL_H
#define APP_CONTROL_H
#include "main.h"

void App_Control_Init(void);  // 初始化两级平衡 PID。
void App_Control_Proc(void);  // 在主循环中计算平衡所需的目标轮速。
void App_Control_Reset(void); // 启停时清除平衡历史和左右轮目标。
void App_Control_SetMoveSpeed(float moveSpeed); // 设置目标直线速度，m/s。
void App_Control_SetTurnSpeed(float turnSpeed); // 设置目标转向角速度，rad/s。

#endif 
