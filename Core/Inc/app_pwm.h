#ifndef __APP_PWM_H
#define __APP_PWM_H

#include "stm32f1xx_hal.h"
#include "main.h"

// 启动左右 PWM 通道，驱动默认待机。
void App_PWM_Init(void);

// 控制共享 STBY；0 为待机，非零为使能。
void App_PWM_Cmd(uint8_t on);
void App_PWM_SetDutyCycle_L(float duty_cycle); // 设置左轮方向和占空比，-100%～100%。
void App_PWM_SetDutyCycle_R(float duty_cycle); // 设置右轮方向和占空比，-100%～100%。

#endif // __APP_PWM_H
