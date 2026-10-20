#ifndef __APP_BUTTON_H
#define __APP_BUTTON_H

#include "main.h"

// 初始化按键电平、消抖时间和使能状态。
void App_Button_Init(void);

// 在主循环中扫描、消抖并处理单击。
void App_Button_Proc(void);

// 获取按键模块保存的电机使能状态：0 关闭，1 开启。
uint8_t App_Button_GetMotorEnable(void);

#endif
