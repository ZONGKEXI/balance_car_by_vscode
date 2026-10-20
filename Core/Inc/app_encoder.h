#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include "main.h"

// 清零左右轮累计位置并复位测速状态。
void App_Encoder_Init(void);

// 读取左轮累计旋转角度（度）。
float App_Encoder_GetPos_L(void);

// 读取右轮累计旋转角度（度）。
float App_Encoder_GetPos_R(void);

// 读取左轮角速度（rad/s），无有效采样时返回零。
float App_Encoder_GetSpeed_L(void);

// 读取右轮角速度（rad/s），无有效采样时返回零。
float App_Encoder_GetSpeed_R(void);



#endif