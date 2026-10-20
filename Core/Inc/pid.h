#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

// 位置式 PID 的参数和运行状态。
typedef struct {
    float kp;                 // P 项比例系数。
    float ki;                 // I 项积分系数。
    float kd;                 // D 项微分系数。
    float setpoint;           // 目标值，与反馈值使用相同单位。
    uint32_t last_update_time; // 上次计算时间，单位为 ms。
    float integral;           // 误差积分；ki * integral 受限幅约束。
    float previous_error;     // 上一次误差。
    float derivative;         // 误差变化率，尚未乘以 kd。
    float output;             // 限幅后的控制输出。
    float previous_integral;  // 上一次误差积分。
    float error;              // 当前误差。

    float Upperlimit;          // 输出上限。
    float Lowerlimit;          // 输出下限。
} PID_TypeDef;

// 初始化参数和状态。
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);

// 修改目标值，保留历史积分和误差。
void PID_SetPoint(PID_TypeDef *pid, float setpoint);

// 获取目标值。
float PID_GetSetPoint(PID_TypeDef *pid);

// 根据反馈值计算控制输出。
float PID_Compute(PID_TypeDef *pid, float feedback);

// 配置总输出及积分项输出的上下限。
void PID_LimitConfig(PID_TypeDef *pid, float lower, float upper);

// 清空历史状态，保留参数、目标值和限幅。
void PID_Reset(PID_TypeDef *pid);

#endif
