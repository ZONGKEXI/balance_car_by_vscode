#ifndef __APP_USART2_H
#define __APP_USART2_H

#include "main.h"

// 阻塞发送 str 指向的字符串，不发送结尾空字符。
void App_USART2_SendString(const char *str);

// 输出 BAT: x.xxx V 格式的电池电压估算值。
void App_USART2_PrintBatVoltage(void);

// 输出左右轮累计角度，单位 deg，保留两位小数。
void App_USART2_PrintEncoderCounts(void);

// 输出带字段名的加速度（g）、角速度（deg/s）和温度（摄氏度）。
void App_USART2_PrintMPU6050Data(void);

// 定时输出十列 CSV：ax、ay、az、gx、gy、gz、temp、yaw、pitch、roll。
// 加速度为 g，角速度为 deg/s，温度为摄氏度，姿态角为 deg；保留三位小数。
void App_USART2_PrintMPU6050Data_Proc(void);

// 定时输出左轮实测、右轮实测、右轮目标速度，rad/s，CSV 保留两位小数。
void App_USART2_PrintMotorOmega(void);

#endif
